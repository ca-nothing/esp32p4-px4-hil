/**
 * @file c6_controller.c
 * @brief ESP32-C6 wireless coprocessor controller (WiFi AP/STA, ESP-NOW, BLE via SDIO/esp_hosted).
 *
 * SDMMC time-sharing: SD card and C6 share one controller (SDMMC_LL_HOST_CTLR_NUMS=1U,
 * IDF #17889), so C6 is suspended before SD mount and resumed after unmount.
 * ---
 * @brief ESP32-C6 kablosuz yardimci-islemci denetleyicisi (SDIO/esp_hosted uzerinden WiFi AP/STA, ESP-NOW, BLE).
 *
 * SDMMC zaman-paylasimi: SD kart ve C6 tek denetleyiciyi paylasir (SDMMC_LL_HOST_CTLR_NUMS=1U,
 * IDF #17889), bu yuzden C6 SD baglama oncesi duraklatilir ve ayirma sonrasi surdurulur.
 */

#include "c6_controller.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

/* ---- esp_hosted / esp_wifi_remote ---- */
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_hosted_misc.h"
#include "esp_wifi.h"
#include "esp_wifi_remote.h"

/* ---- FreeRTOS ---- */
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* esp_bt.h is not available on ESP32-P4; BT is enabled via
   esp_hosted_bt_controller_init() (HCI over SDIO).
   ---
   esp_bt.h ESP32-P4'te mevcut degil; BT, esp_hosted_bt_controller_init()
   ile etkinlestirilir (SDIO uzerinden HCI). */

static const char *TAG = "c6_ctrl";

/* ---- NVS keys (C6 config stored on P4 side) | NVS anahtarlari (C6 yapilandirmasi P4 tarafinda saklanir) ---- */
#define NVS_NAMESPACE_WIFI "wifi_cfg"
#define NVS_NAMESPACE_BT "bt_cfg"
#define NVS_KEY_WIFI_MODE "mode"
#define NVS_KEY_WIFI_SSID "ssid"
#define NVS_KEY_WIFI_PASS "pass"
#define NVS_KEY_WIFI_AP_SSID "ap_ssid"
#define NVS_KEY_WIFI_CHAN "chan"
#define NVS_KEY_WIFI_ESPNOW "espnow"
#define NVS_KEY_BT_ENABLED "enabled"
#define NVS_KEY_BT_NAME "dev_name"

/* ---- Static state | Statik durum ---- */
static c6_status_t s_c6_status = {
    .state = C6_STATE_OFFLINE,
    .wifi_mode = C6_WIFI_MODE_APSTA,
    .wifi_ap_active = false,
    .wifi_sta_connected = false,
    .wifi_ssid = "",
    .wifi_ap_ssid = "ESP32-P4-NANO",
    .bt_enabled = false,
    .bt_device_name = "P4-NANO-FC",
    .espnow_enabled = false,
    .espnow_peer_count = 0,
    .last_event_time_us = 0,
};

/* ---- Forward declarations | Ileri bildirimler ---- */
static void c6_load_wifi_config_from_nvs(void);
static void c6_load_bt_config_from_nvs(void);
static void c6_save_wifi_config_to_nvs(void);
static void c6_save_bt_config_to_nvs(void);
static void c6_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data);

/* ================================================================
 *  NVS load/save | NVS yukleme/kaydetme
 * ================================================================ */

static void c6_load_wifi_config_from_nvs(void) {
  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE_WIFI, NVS_READONLY, &nvs) != ESP_OK) {
    ESP_LOGW(TAG, "No saved WiFi config, using defaults");
    return;
  }

  uint8_t mode = C6_WIFI_MODE_APSTA;
  nvs_get_u8(nvs, NVS_KEY_WIFI_MODE, &mode);
  s_c6_status.wifi_mode = (c6_wifi_mode_t)mode;

  char buf[33] = {0};
  size_t len = sizeof(buf);
  if (nvs_get_str(nvs, NVS_KEY_WIFI_SSID, buf, &len) == ESP_OK) {
    strncpy(s_c6_status.wifi_ssid, buf, sizeof(s_c6_status.wifi_ssid) - 1);
    s_c6_status.wifi_ssid[sizeof(s_c6_status.wifi_ssid) - 1] = '\0';
  }
  len = sizeof(buf);
  memset(buf, 0, sizeof(buf));
  if (nvs_get_str(nvs, NVS_KEY_WIFI_AP_SSID, buf, &len) == ESP_OK) {
    strncpy(s_c6_status.wifi_ap_ssid, buf,
            sizeof(s_c6_status.wifi_ap_ssid) - 1);
    s_c6_status.wifi_ap_ssid[sizeof(s_c6_status.wifi_ap_ssid) - 1] = '\0';
  }

  uint8_t espnow = 0;
  nvs_get_u8(nvs, NVS_KEY_WIFI_ESPNOW, &espnow);
  s_c6_status.espnow_enabled = (espnow != 0);

  nvs_close(nvs);
  ESP_LOGI(TAG, "WiFi config loaded: mode=%d, sta_ssid=%s, ap_ssid=%s", mode,
           s_c6_status.wifi_ssid, s_c6_status.wifi_ap_ssid);
}

static void c6_load_bt_config_from_nvs(void) {
  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE_BT, NVS_READONLY, &nvs) != ESP_OK) {
    return;
  }

  uint8_t enabled = 0;
  nvs_get_u8(nvs, NVS_KEY_BT_ENABLED, &enabled);
  s_c6_status.bt_enabled = (enabled != 0);

  char buf[32] = {0};
  size_t len = sizeof(buf);
  if (nvs_get_str(nvs, NVS_KEY_BT_NAME, buf, &len) == ESP_OK) {
    strncpy(s_c6_status.bt_device_name, buf,
            sizeof(s_c6_status.bt_device_name) - 1);
    s_c6_status.bt_device_name[sizeof(s_c6_status.bt_device_name) - 1] = '\0';
  }

  nvs_close(nvs);
  ESP_LOGI(TAG, "BT config loaded: enabled=%d, name=%s", s_c6_status.bt_enabled,
           s_c6_status.bt_device_name);
}

static void c6_save_wifi_config_to_nvs(void) {
  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE_WIFI, NVS_READWRITE, &nvs) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open wifi_cfg NVS for writing");
    return;
  }

  nvs_set_u8(nvs, NVS_KEY_WIFI_MODE, (uint8_t)s_c6_status.wifi_mode);
  nvs_set_str(nvs, NVS_KEY_WIFI_SSID, s_c6_status.wifi_ssid);
  nvs_set_str(nvs, NVS_KEY_WIFI_AP_SSID, s_c6_status.wifi_ap_ssid);
  nvs_set_u8(nvs, NVS_KEY_WIFI_ESPNOW, s_c6_status.espnow_enabled ? 1 : 0);
  nvs_commit(nvs);
  nvs_close(nvs);
  ESP_LOGI(TAG, "WiFi config saved to NVS");
}

static void c6_save_bt_config_to_nvs(void) {
  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE_BT, NVS_READWRITE, &nvs) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open bt_cfg NVS for writing");
    return;
  }

  nvs_set_u8(nvs, NVS_KEY_BT_ENABLED, s_c6_status.bt_enabled ? 1 : 0);
  nvs_set_str(nvs, NVS_KEY_BT_NAME, s_c6_status.bt_device_name);
  nvs_commit(nvs);
  nvs_close(nvs);
  ESP_LOGI(TAG, "BT config saved to NVS");
}

/* ================================================================
 *  Init | Baslatma
 * ================================================================ */

esp_err_t c6_controller_init(void) {
  ESP_LOGI(TAG, "=== C6 Controller Init ===");

  /* 1. Load WiFi/BT config from NVS | WiFi/BT yapilandirmasini NVS'ten yukle */
  c6_load_wifi_config_from_nvs();
  c6_load_bt_config_from_nvs();

  /* 2. esp_hosted init — connect to C6 over SDIO | esp_hosted baslat — C6'ya SDIO uzerinden baglan */
  esp_err_t ret = esp_hosted_init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG,
             "esp_hosted_init failed: %s (0x%x). "
             "Is C6 powered and firmware flashed?",
             esp_err_to_name(ret), ret);
    s_c6_status.state = C6_STATE_OFFLINE;
    return ret;
  }

  /* 3. Connect to C6 slave | C6 slave'e baglan */
  ret = esp_hosted_connect_to_slave();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG,
             "esp_hosted_connect_to_slave failed: %s (0x%x). "
             "Check C6 firmware and SDIO wiring.",
             esp_err_to_name(ret), ret);
    s_c6_status.state = C6_STATE_OFFLINE;
    return ret;
  }

  s_c6_status.state = C6_STATE_CONNECTED;
  ESP_LOGI(TAG, "C6 connected via SDIO (connected state)");

  /* 4. Register WiFi event handler | WiFi olay isleyicisini kaydet */
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                             &c6_wifi_event_handler, NULL);
  ESP_LOGI(TAG, "WiFi event handler registered");

  /* 5. WiFi init + start | WiFi baslat + calistir */
  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&wifi_cfg);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_init failed: %s (C6 may not have WiFi fw)",
             esp_err_to_name(ret));
  } else {
    wifi_mode_t mode = WIFI_MODE_APSTA;
    if (s_c6_status.wifi_mode == C6_WIFI_MODE_AP)
      mode = WIFI_MODE_AP;
    else if (s_c6_status.wifi_mode == C6_WIFI_MODE_STA)
      mode = WIFI_MODE_STA;

    ret = esp_wifi_set_mode(mode);
    if (ret == ESP_OK) {
      ret = esp_wifi_start();
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi started (mode=%d)", mode);
        /* Start AP immediately | AP'yi hemen baslat */
        c6_wifi_ap_start();
        /* Connect to saved STA if any | Kayitli STA varsa baglan */
        if (s_c6_status.wifi_ssid[0] &&
            (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)) {
          ESP_LOGI(TAG, "Auto-connecting to saved SSID: %s",
                   s_c6_status.wifi_ssid);
          wifi_config_t cfg = {0};
          memcpy(cfg.sta.ssid, s_c6_status.wifi_ssid, sizeof(cfg.sta.ssid) - 1);
          cfg.sta.ssid[sizeof(cfg.sta.ssid) - 1] = '\0';
          /* Note: password would need to be stored in NVS too
             | Not: parolanin da NVS'te saklanmasi gerekir */
          esp_wifi_set_config(WIFI_IF_STA, &cfg);
          esp_wifi_connect();
        }
      } else {
        ESP_LOGW(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
      }
    } else {
      ESP_LOGW(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
    }
  }

  /* 5. BLE init — optional | BLE baslat — istege bagli */
  if (s_c6_status.bt_enabled) {
    ESP_LOGI(TAG, "Starting C6 BLE (device=%s)...", s_c6_status.bt_device_name);
    c6_bt_set_enabled(true);
  }

  /* 6. ESP-NOW — optional | ESP-NOW — istege bagli */
  if (s_c6_status.espnow_enabled) {
    ESP_LOGI(TAG, "Starting C6 ESP-NOW...");
    /* TODO */
  }

  s_c6_status.last_event_time_us = esp_timer_get_time();
  ESP_LOGI(TAG, "C6 Controller init complete");
  return ESP_OK;
}

/* ================================================================
 *  Status | Durum
 * ================================================================ */

esp_err_t c6_controller_get_status(c6_status_t *status) {
  if (!status)
    return ESP_ERR_INVALID_ARG;
  *status = s_c6_status;
  return ESP_OK;
}

/* ================================================================
 *  WiFi Commands | WiFi Komutlari
 * ================================================================ */

/** Event group for scan completion synchronization | Tarama tamamlanma senkronizasyonu icin olay grubu */
#define WIFI_SCAN_DONE_BIT BIT0
static EventGroupHandle_t s_scan_event_group = NULL;

/** Cached scan results | Onbellekli tarama sonuclari */
static c6_wifi_ap_t s_scan_results[C6_WIFI_SCAN_MAX];
static int s_scan_count = 0;

/** WiFi event handler — captures scan done and connection events
 *  | WiFi olay isleyicisi — tarama-bitti ve baglanti olaylarini yakalar */
static void c6_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_SCAN_DONE:
      if (s_scan_event_group) {
        xEventGroupSetBits(s_scan_event_group, WIFI_SCAN_DONE_BIT);
      }
      break;
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "WiFi STA started");
      break;
    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WiFi STA connected");
      s_c6_status.wifi_sta_connected = true;
      s_c6_status.last_event_time_us = esp_timer_get_time();
      break;
    case WIFI_EVENT_STA_DISCONNECTED: {
      wifi_event_sta_disconnected_t *ev =
          (wifi_event_sta_disconnected_t *)event_data;
      ESP_LOGI(TAG, "WiFi STA disconnected (reason=%d)", ev->reason);
      s_c6_status.wifi_sta_connected = false;
      s_c6_status.last_event_time_us = esp_timer_get_time();
      break;
    }
    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "WiFi AP started");
      s_c6_status.wifi_ap_active = true;
      s_c6_status.last_event_time_us = esp_timer_get_time();
      break;
    case WIFI_EVENT_AP_STOP:
      ESP_LOGI(TAG, "WiFi AP stopped");
      s_c6_status.wifi_ap_active = false;
      break;
    default:
      break;
    }
  }
}

esp_err_t c6_wifi_connect(const char *ssid, const char *password) {
  if (!ssid || !password)
    return ESP_ERR_INVALID_ARG;

  if (s_c6_status.state < C6_STATE_CONNECTED) {
    ESP_LOGW(TAG, "C6 not connected, cannot issue WiFi command");
    return ESP_ERR_INVALID_STATE;
  }

  strncpy(s_c6_status.wifi_ssid, ssid, sizeof(s_c6_status.wifi_ssid) - 1);
  s_c6_status.wifi_ssid[sizeof(s_c6_status.wifi_ssid) - 1] = '\0';
  c6_save_wifi_config_to_nvs();

  wifi_config_t cfg = {0};
  memcpy(cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
  cfg.sta.ssid[sizeof(cfg.sta.ssid) - 1] = '\0';
  memcpy(cfg.sta.password, password, sizeof(cfg.sta.password) - 1);
  cfg.sta.password[sizeof(cfg.sta.password) - 1] = '\0';

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_wifi_connect();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "WiFi connecting to %s...", ssid);
  return ESP_OK;
}

esp_err_t c6_wifi_disconnect(void) {
  if (s_c6_status.state < C6_STATE_CONNECTED)
    return ESP_ERR_INVALID_STATE;

  esp_err_t ret = esp_wifi_disconnect();
  s_c6_status.wifi_sta_connected = false;
  ESP_LOGI(TAG, "WiFi disconnected");
  return ret;
}

esp_err_t c6_wifi_get_status(bool *connected, char *ssid, int8_t *rssi) {
  if (!connected)
    return ESP_ERR_INVALID_ARG;

  *connected = s_c6_status.wifi_sta_connected;
  if (ssid) {
    strncpy(ssid, s_c6_status.wifi_ssid, 32);
    ssid[32] = '\0';
  }
  if (rssi) {
    *rssi = 0; /* TODO: get actual RSSI via wifi_ap_record_t | TODO: gercek RSSI'yi wifi_ap_record_t ile al */
  }
  return ESP_OK;
}

esp_err_t c6_wifi_scan(c6_wifi_ap_t *aps, int max_aps, int *out_count) {
  if (!aps || !out_count)
    return ESP_ERR_INVALID_ARG;
  if (s_c6_status.state < C6_STATE_CONNECTED)
    return ESP_ERR_INVALID_STATE;

  *out_count = 0;

  /* Create event group for scan sync (one-time) | Tarama senkronu icin olay grubu olustur (tek-sefer) */
  if (!s_scan_event_group) {
    s_scan_event_group = xEventGroupCreate();
  }
  xEventGroupClearBits(s_scan_event_group, WIFI_SCAN_DONE_BIT);

  wifi_scan_config_t scan_cfg = {
      .ssid = NULL,
      .bssid = NULL,
      .channel = 0,
      .show_hidden = false,
      .scan_type = WIFI_SCAN_TYPE_ACTIVE,
      .scan_time = {.active = {.min = 100, .max = 300}},
  };

  esp_err_t ret = esp_wifi_scan_start(&scan_cfg, false);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_scan_start failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Wait for scan to complete (10 second timeout) | Taramanin bitmesini bekle (10 saniye zaman-asimi) */
  EventBits_t bits = xEventGroupWaitBits(s_scan_event_group, WIFI_SCAN_DONE_BIT,
                                         pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));

  if (!(bits & WIFI_SCAN_DONE_BIT)) {
    ESP_LOGW(TAG, "WiFi scan timed out");
    esp_wifi_scan_stop();
    return ESP_ERR_TIMEOUT;
  }

  /* Retrieve scan results | Tarama sonuclarini al */
  uint16_t num = max_aps;
  wifi_ap_record_t *records = calloc(num, sizeof(wifi_ap_record_t));
  if (!records)
    return ESP_ERR_NO_MEM;

  ret = esp_wifi_scan_get_ap_records(&num, records);
  if (ret != ESP_OK) {
    free(records);
    return ret;
  }

  for (int i = 0; i < num && i < max_aps; i++) {
    strncpy(aps[i].ssid, (char *)records[i].ssid, 32);
    aps[i].ssid[32] = '\0';
    memcpy(aps[i].bssid, records[i].bssid, 6);
    aps[i].rssi = records[i].rssi;
    aps[i].channel = records[i].primary;
    aps[i].authmode = records[i].authmode;
  }

  *out_count = (num < max_aps) ? num : max_aps;
  free(records);

  /* Cache results | Sonuclari onbelleğe al */
  s_scan_count = *out_count;
  memcpy(s_scan_results, aps, s_scan_count * sizeof(c6_wifi_ap_t));

  ESP_LOGI(TAG, "WiFi scan complete: %d APs found", *out_count);
  return ESP_OK;
}

esp_err_t c6_wifi_ap_start(void) {
  if (s_c6_status.state < C6_STATE_CONNECTED) {
    ESP_LOGW(TAG, "C6 not connected, cannot start AP");
    return ESP_ERR_INVALID_STATE;
  }

  wifi_config_t cfg = {
      .ap =
          {
              .ssid_len = strlen(s_c6_status.wifi_ap_ssid),
              .channel = 1,
              .max_connection = 4,
              .authmode = WIFI_AUTH_OPEN,
          },
  };
  strncpy((char *)cfg.ap.ssid, s_c6_status.wifi_ap_ssid, 32);

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "WiFi AP starting: SSID=%s", s_c6_status.wifi_ap_ssid);
  return ESP_OK;
}

esp_err_t c6_wifi_ap_config(const char *ssid, uint8_t channel) {
  if (ssid) {
    strncpy(s_c6_status.wifi_ap_ssid, ssid,
            sizeof(s_c6_status.wifi_ap_ssid) - 1);
    s_c6_status.wifi_ap_ssid[sizeof(s_c6_status.wifi_ap_ssid) - 1] = '\0';
  }
  c6_save_wifi_config_to_nvs();
  ESP_LOGI(TAG, "WiFi AP config: SSID=%s, ch=%d (saved to NVS)",
           s_c6_status.wifi_ap_ssid, channel);
  return ESP_OK;
}

esp_err_t c6_wifi_set_mode(c6_wifi_mode_t mode) {
  s_c6_status.wifi_mode = mode;
  c6_save_wifi_config_to_nvs();

  // Apply mode to C6 via esp_wifi_remote | Modu esp_wifi_remote ile C6'ya uygula
  wifi_mode_t wmode = WIFI_MODE_APSTA;
  if (mode == C6_WIFI_MODE_AP)
    wmode = WIFI_MODE_AP;
  else if (mode == C6_WIFI_MODE_STA)
    wmode = WIFI_MODE_STA;

  if (s_c6_status.state >= C6_STATE_CONNECTED) {
    // Stop WiFi, set mode, restart | WiFi'yi durdur, modu ayarla, yeniden baslat
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t ret = esp_wifi_set_mode(wmode);
    if (ret == ESP_OK) {
      ret = esp_wifi_start();
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi mode changed to: %d (restarted)", mode);
        // Restart AP if mode includes AP | Mod AP iceriyorsa AP'yi yeniden baslat
        if (mode == C6_WIFI_MODE_AP || mode == C6_WIFI_MODE_APSTA) {
          vTaskDelay(pdMS_TO_TICKS(500));
          c6_wifi_ap_start();
        }
      }
    }
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "WiFi mode change failed: %s", esp_err_to_name(ret));
    }
  }
  return ESP_OK;
}

/* ================================================================
 *  BT Commands | BT Komutlari
 * ================================================================ */

esp_err_t c6_bt_set_enabled(bool enable) {
  s_c6_status.bt_enabled = enable;
  c6_save_bt_config_to_nvs();

  if (s_c6_status.state < C6_STATE_CONNECTED) {
    ESP_LOGW(TAG, "C6 not connected, BT setting saved for later");
    return ESP_OK;
  }

  if (enable) {
    esp_err_t ret = esp_hosted_bt_controller_init();
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "esp_hosted_bt_controller_init: %s", esp_err_to_name(ret));
      return ret;
    }
    ret = esp_hosted_bt_controller_enable();
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "esp_hosted_bt_controller_enable: %s",
               esp_err_to_name(ret));
      return ret;
    }
    ESP_LOGI(TAG, "BLE enabled (HCI transport, saved to NVS)");
  } else {
    esp_hosted_bt_controller_disable();
    ESP_LOGI(TAG, "BLE disabled (saved to NVS)");
  }
  return ESP_OK;
}

esp_err_t c6_bt_set_device_name(const char *name) {
  if (!name)
    return ESP_ERR_INVALID_ARG;
  strncpy(s_c6_status.bt_device_name, name,
          sizeof(s_c6_status.bt_device_name) - 1);
  s_c6_status.bt_device_name[sizeof(s_c6_status.bt_device_name) - 1] = '\0';
  c6_save_bt_config_to_nvs();
  ESP_LOGI(TAG, "BT device name: %s (saved to NVS)", name);
  return ESP_OK;
}

/* ================================================================
 *  SDMMC Time-Sharing (Suspend/Resume) | SDMMC Zaman-Paylasimi (Duraklat/Surdur)
 * ================================================================ */

esp_err_t c6_controller_suspend(void) {
  if (s_c6_status.state != C6_STATE_CONNECTED) {
    return ESP_OK; /* Not connected, nothing to do | Bagli degil, yapilacak sey yok */
  }
  ESP_LOGI(TAG, "=== C6 Suspend: Releasing SDMMC for SD card ===");

  /* 1. Stop WiFi | WiFi'yi durdur */
  esp_wifi_stop();
  esp_wifi_deinit();
  ESP_LOGI(TAG, "WiFi stopped and deinitialized");

  /* 2. Unregister WiFi event handler | WiFi olay isleyicisinin kaydini kaldir */
  esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               &c6_wifi_event_handler);

  /* 3. Release SDMMC controller via esp_hosted_deinit | SDMMC denetleyicisini esp_hosted_deinit ile birak */
  int ret = esp_hosted_deinit();
  if (ret != 0) {
    ESP_LOGW(TAG, "esp_hosted_deinit returned %d (non-fatal)", ret);
  }

  s_c6_status.state = C6_STATE_SUSPENDED;
  s_c6_status.wifi_ap_active = false;
  ESP_LOGI(TAG, "C6 suspended — SDMMC controller released for SD card");
  return ESP_OK;
}

esp_err_t c6_controller_resume(void) {
  if (s_c6_status.state != C6_STATE_SUSPENDED) {
    return ESP_OK;
  }
  ESP_LOGI(TAG, "=== C6 Resume: Reconnecting via SDIO ===");

  /* 1. Re-init esp_hosted (re-acquire SDMMC) | esp_hosted'i yeniden baslat (SDMMC'yi yeniden edin) */
  esp_err_t ret = esp_hosted_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_hosted_init failed after resume: %s",
             esp_err_to_name(ret));
    s_c6_status.state = C6_STATE_OFFLINE;
    return ret;
  }

  /* 2. Reconnect to C6 slave | C6 slave'e yeniden baglan */
  ret = esp_hosted_connect_to_slave();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_hosted_connect_to_slave failed after resume: %s",
             esp_err_to_name(ret));
    s_c6_status.state = C6_STATE_OFFLINE;
    return ret;
  }

  s_c6_status.state = C6_STATE_CONNECTED;
  ESP_LOGI(TAG, "C6 reconnected via SDIO");

  /* 3. Re-register WiFi event handler | WiFi olay isleyicisini yeniden kaydet */
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                             &c6_wifi_event_handler, NULL);

  /* 4. WiFi init + start | WiFi baslat + calistir */
  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&wifi_cfg);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_init failed on resume: %s", esp_err_to_name(ret));
    return ret;
  }

  wifi_mode_t mode = WIFI_MODE_APSTA;
  if (s_c6_status.wifi_mode == C6_WIFI_MODE_AP)
    mode = WIFI_MODE_AP;
  else if (s_c6_status.wifi_mode == C6_WIFI_MODE_STA)
    mode = WIFI_MODE_STA;

  ret = esp_wifi_set_mode(mode);
  if (ret == ESP_OK) {
    ret = esp_wifi_start();
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "WiFi restarted (mode=%d)", mode);
      /* Restart AP | AP'yi yeniden baslat */
      c6_wifi_ap_start();
      /* Reconnect STA if was connected | Onceden bagliysa STA'yi yeniden bagla */
      if (s_c6_status.wifi_ssid[0] &&
          (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)) {
        wifi_config_t cfg = {0};
        memcpy(cfg.sta.ssid, s_c6_status.wifi_ssid, sizeof(cfg.sta.ssid) - 1);
        cfg.sta.ssid[sizeof(cfg.sta.ssid) - 1] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &cfg);
        esp_wifi_connect();
      }
    }
  }

  s_c6_status.last_event_time_us = esp_timer_get_time();
  ESP_LOGI(TAG, "C6 resumed — WiFi AP re-enabled");
  return ESP_OK;
}

/* ================================================================
 *  Event Processing (main loop) | Olay Isleme (ana dongu)
 * ================================================================ */

void c6_controller_process_events(void) {
  /* TODO: handle C6 event callbacks here (WiFi state changes,
   * BLE pairing/connection, ESP-NOW messages).
   * ---
   * TODO: C6 olay geri-cagrilarini burada isle (WiFi durum degisimleri,
   * BLE eslesme/baglanti, ESP-NOW mesajlari). */
}

/* ================================================================
 *  ESP-NOW Enable/Disable (NVS-backed) | ESP-NOW Etkinlestir/Devre-Disi (NVS-destekli)
 * ================================================================ */

esp_err_t c6_espnow_set_enabled(bool enable) {
  s_c6_status.espnow_enabled = enable;
  c6_save_wifi_config_to_nvs();
  ESP_LOGI(TAG, "ESP-NOW %s (saved to NVS)", enable ? "enabled" : "disabled");
  return ESP_OK;
}

/* ================================================================
 *  C6 Reboot via GPIO54 | GPIO54 uzerinden C6 Yeniden-Baslatma
 * ================================================================ */

#define C6_RESET_GPIO 54

esp_err_t c6_controller_reboot(void) {
  ESP_LOGI(TAG, "Rebooting C6 via GPIO%d...", C6_RESET_GPIO);

  gpio_config_t cfg = {.pin_bit_mask = BIT64(C6_RESET_GPIO),
                       .mode = GPIO_MODE_OUTPUT,
                       .pull_up_en = GPIO_PULLUP_DISABLE,
                       .pull_down_en = GPIO_PULLDOWN_DISABLE,
                       .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&cfg);

  gpio_set_level(C6_RESET_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(C6_RESET_GPIO, 1);

  s_c6_status.state = C6_STATE_OFFLINE;
  s_c6_status.wifi_ap_active = false;
  s_c6_status.wifi_sta_connected = false;
  ESP_LOGI(TAG,
           "C6 reset pulse sent. Will attempt reconnect after C6 boots...");

  // Wait for C6 to boot (~2s) then reconnect | C6'nin acilmasini bekle (~2s) sonra yeniden baglan
  vTaskDelay(pdMS_TO_TICKS(3000));
  c6_controller_resume();
  return ESP_OK;
}

/* ================================================================
 *  C6 Factory Reset (erase NVS namespaces) | C6 Fabrika Ayarlarina Donus (NVS ad-alanlarini sil)
 * ================================================================ */

esp_err_t c6_controller_factory_reset(void) {
  ESP_LOGI(TAG, "Factory reset: erasing C6 NVS namespaces...");

  // Erase namespaces within the default NVS partition | Varsayilan NVS bolumundeki ad-alanlarini sil
  nvs_handle_t nvs;
  if (nvs_open(NVS_NAMESPACE_WIFI, NVS_READWRITE, &nvs) == ESP_OK) {
    nvs_erase_all(nvs);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
  if (nvs_open(NVS_NAMESPACE_BT, NVS_READWRITE, &nvs) == ESP_OK) {
    nvs_erase_all(nvs);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
  s_c6_status.wifi_ssid[0] = '\0';
  strcpy(s_c6_status.wifi_ap_ssid, "ESP32-P4-NANO");
  s_c6_status.bt_enabled = false;
  strcpy(s_c6_status.bt_device_name, "P4-NANO-FC");
  s_c6_status.espnow_enabled = false;
  s_c6_status.espnow_peer_count = 0;

  ESP_LOGI(TAG, "C6 NVS erased. Rebooting C6...");
  return c6_controller_reboot();
}
