/**
 * @file main.c
 * @brief ESP32-P4 SMP Flight Controller
 *
 * Core 0: FreeRTOS — HTTP, Ethernet, C6, SD Card, LittleFS
 * Core 1: FC task (pinned max-priority, never yields)
 * ---
 * @brief ESP32-P4 SMP Uçuş Kontrolcüsü
 *
 * Core 0: FreeRTOS — HTTP, Ethernet, C6, SD Kart, LittleFS
 * Core 1: FC görevi (sabitlenmiş maks-öncelik, asla yield etmez)
 */

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>

#include "c6_controller.h"
#include "esp_netif.h"
#include "eth.h"
#include "fc_core.h"
#include "http_server.h"
#include "littlefs_fs.h"
#include "logger/fc_logger.h"
#include "fc_px4_modules.h"   /* PX4 module spawn | PX4 modül başlatma */

static const char *TAG = "FC_MAIN";

/* ── HTTP server start (40s delayed) ── | ── HTTP sunucu başlatma (40s gecikmeli) ── */
static void http_start_cb(void *arg) {
  ESP_LOGI(TAG, "[HTTP] Starting server...");
  if (http_server_init() != ESP_OK)
    ESP_LOGE(TAG, "HTTP start FAIL");
}

/* ── Core 0 system task ── | ── Core 0 sistem görevi ── */
static void system_core0_task(void *pv) {
  ESP_LOGI(TAG, "=== Core 0: System init ===");

  littlefs_fs_init();
  eth_init();
  vTaskDelay(pdMS_TO_TICKS(3000));
  eth_load_config_from_nvs();

  /* Static IP fallback: if DHCP didn't give an IP, use 172.16.16.57
     | Statik IP geri-düşüşü: DHCP bir IP vermediyse 172.16.16.57 kullan */
  {
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(eth_get_netif(), &ip_info) != ESP_OK ||
        ip_info.ip.addr == 0) {
      ESP_LOGW(TAG, "DHCP did not provide IP — using static fallback");
      esp_netif_ip_info_t static_ip = {
          .ip = {.addr = ESP_IP4TOADDR(172, 16, 16, 57)},
          .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0)},
          .gw = {.addr = ESP_IP4TOADDR(172, 16, 16, 1)},
      };
      eth_set_static_ip(&static_ip, NULL, NULL);
    } else {
      char ipstr[16];
      esp_ip4addr_ntoa(&ip_info.ip, ipstr, sizeof(ipstr));
      ESP_LOGI(TAG, "DHCP provided IP: %s", ipstr);
    }
  }

  /* HTTP server disabled (not part of PX4; freed for Core0/RAM). Delayed
   * http_start_cb timer never armed -> HTTP never starts.
   * ---
   * HTTP sunucu devre-dışı (PX4'ün parçası değil; Core0/RAM için serbest bırakıldı).
   * Gecikmeli http_start_cb zamanlayıcısı asla kurulmaz -> HTTP asla başlamaz. */
  /* esp_timer_handle_t ht = NULL;
  esp_timer_create_args_t hta = {.callback = http_start_cb,
                                 .dispatch_method = ESP_TIMER_TASK,
                                 .name = "http_delayed"};
  esp_timer_create(&hta, &ht);
  esp_timer_start_once(ht, 40 * 1000 * 1000); */
  (void)http_start_cb;   /* timer commented out -> suppress 'unused static function' warning | zamanlayıcı yorum-satırında -> 'kullanılmayan statik fonksiyon' uyarısını bastır */

  c6_controller_init();

  /* EKF task on Core 0, kept off the 8kHz Core-1 hot-loop (973us EKF won't fit
   * a 125us slot -> jitter). Created before RX task so the first IMU notify isn't lost.
   * ---
   * EKF görevi Core 0'da, 8kHz Core-1 sıcak-döngüsünden uzak tutuldu (973us EKF bir
   * 125us yuvaya sığmaz -> jitter). RX görevinden önce oluşturuldu, böylece ilk IMU
   * bildirimi kaybolmaz. */
  fc_ekf_task_start();

  ESP_LOGI(TAG, "=== Core 0: Ready ===");
  /* Logger: live jitter_max ESP_LOGI output (fc_logger.c). | Kaydedici: canlı jitter_max ESP_LOGI çıktısı (fc_logger.c). */
  fc_logger_start();

  /* Start PX4 modules (commander/navigator/FMM/mc_pos/att/rate/ekf2/mavlink) as
   * low-prio WQ tasks on Core0 (EKF/8kHz Core1 untouched). After logger/ekf/mavlink are up.
   * ---
   * PX4 modüllerini (commander/navigator/FMM/mc_pos/att/rate/ekf2/mavlink) Core0'da
   * düşük-öncelikli WQ görevleri olarak başlat (EKF/8kHz Core1 dokunulmaz).
   * logger/ekf/mavlink ayağa kalktıktan sonra. */
  fc_px4_modules_start();

  /* Measure Core 0 loop duration: a vTaskDelay(5000ms) should take ~5000ms;
   * >6000ms means Core 0 is also stalling (e.g. SDIO/ESP-Hosted, flash/cache disable).
   * ---
   * Core 0 döngü süresini ölç: bir vTaskDelay(5000ms) ~5000ms sürmeli;
   * >6000ms, Core 0'ın da takıldığı anlamına gelir (örn. SDIO/ESP-Hosted, flash/cache devre-dışı). */
  int64_t dbg_last_tick_us = esp_timer_get_time();
  while (1) {
    c6_controller_process_events();
    vTaskDelay(pdMS_TO_TICKS(5000));

    int64_t now_us = esp_timer_get_time();
    int64_t tick_dur_us = now_us - dbg_last_tick_us;
    dbg_last_tick_us = now_us;
    if (tick_dur_us > 6000000) {
      ESP_LOGW(TAG, "core0_tick: delay! tick_dur=%lld us (expected ~5000000)",
               (long long)tick_dur_us);
    }
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "ESP32-P4 SMP FC starting...");
  ESP_LOGI(TAG, "IDF: %s | CPU: %dMHz | Core: %d", IDF_VER,
           CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, xPortGetCoreID());
  ESP_LOGI(TAG, "Free DRAM: %zu KB | PSRAM: %zu KB",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
  ESP_LOGI(TAG, "BUILD CONFIG: FC_EKF_MODE=%d (0=Mahony 1=SimpleKalman 2=PX4FullEKF) | "
                "FC_BENCH_AUTOPILOT=%d | __DATE__=%s __TIME__=%s",
           FC_EKF_MODE, FC_BENCH_AUTOPILOT, __DATE__, __TIME__);
  ESP_LOGI(TAG, "========================================");

  /* Phase 0: Common init | Faz 0: Ortak init */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  esp_netif_init();
  esp_event_loop_create_default();

  /* Phase 1: FC hardware init | Faz 1: FC donanım init */
  ESP_LOGI(TAG, "FC hardware init...");
  fc_core_init();

  /* Init uORB Manager before spawning fc_task (Core1). Core-1 fc_failsafe
   * events::send -> orb_advertise, so uORB must be ready (else get_device_master
   * null-deref panic). Idempotent.
   * ---
   * fc_task'ı (Core1) başlatmadan önce uORB Manager'ı init et. Core-1 fc_failsafe
   * events::send -> orb_advertise, bu yüzden uORB hazır olmalı (yoksa get_device_master
   * null-deref panik). Idempotent (tekrar-çağrıya dayanıklı). */
  fc_uorb_init();

  /* Phase 2: Launch FC on Core 1 (pinned, max priority) | Faz 2: FC'yi Core 1'de başlat (sabitlenmiş, maks öncelik) */
  TaskHandle_t fc_h = NULL;
  xTaskCreatePinnedToCore(fc_task_main, "fc_task", FC_TASK_STACK_SIZE, NULL,
                          configMAX_PRIORITIES - 1, &fc_h, 1);
  if (fc_h)
    ESP_LOGI(TAG, "FC task on Core 1 (prio %d)", configMAX_PRIORITIES - 1);

  /* Phase 3: System on Core 0 | Faz 3: Sistem Core 0'da */
  xTaskCreatePinnedToCore(system_core0_task, "sys_core0", 4096, NULL, 2, NULL,
                          0);

  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Boot complete");
  ESP_LOGI(TAG, "  Core 0: HTTP, ETH, LittleFS, SD, C6");
  ESP_LOGI(TAG, "  Core 1: FC (8kHz, polling SPI)");
  ESP_LOGI(TAG, "========================================");

  vTaskDelete(NULL);
}
