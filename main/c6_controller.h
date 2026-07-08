/**
 * @file c6_controller.h
 * @brief ESP32-C6 wireless coprocessor controller (WiFi AP/STA, ESP-NOW, BLE).
 *
 * C6 connects to P4 over SDIO via esp_hosted RPC. P4 sends config commands;
 * C6 keeps an AP as fallback and manages its own STA connection.
 *
 * SDMMC conflict: one controller, two slots (Slot 0 = SD card, Slot 1 = C6 SDIO);
 * only one active at a time, so C6 SDIO pauses while the SD card is mounted.
 * ---
 * @brief ESP32-C6 kablosuz yardimci-islemci denetleyicisi (WiFi AP/STA, ESP-NOW, BLE).
 *
 * C6, P4'e SDIO uzerinden esp_hosted RPC ile baglanir. P4 yapilandirma komutlari gonderir;
 * C6 yedek olarak bir AP tutar ve kendi STA baglantisini yonetir.
 *
 * SDMMC catismasi: tek denetleyici, iki yuva (Yuva 0 = SD kart, Yuva 1 = C6 SDIO);
 * ayni anda yalnizca biri etkin, bu yuzden SD kart bagliyken C6 SDIO duraklatilir.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  C6 status structs | C6 durum yapilari
 * ================================================================ */

typedef enum {
  C6_STATE_OFFLINE = 0, /**< No communication with C6 | C6 ile iletisim yok */
  C6_STATE_INIT,        /**< SDIO initializing | SDIO baslatiliyor */
  C6_STATE_CONNECTED,   /**< C6 connected, ready | C6 bagli, hazir */
  C6_STATE_SUSPENDED,   /**< SD card active, C6 paused | SD kart etkin, C6 duraklatildi */
} c6_state_t;

typedef enum {
  C6_WIFI_MODE_NONE = 0,
  C6_WIFI_MODE_AP,
  C6_WIFI_MODE_STA,
  C6_WIFI_MODE_APSTA,
} c6_wifi_mode_t;

typedef struct {
  c6_state_t state;
  c6_wifi_mode_t wifi_mode;
  bool wifi_ap_active;
  bool wifi_sta_connected;
  char wifi_ssid[33];
  char wifi_ap_ssid[33];
  bool bt_enabled;
  char bt_device_name[32];
  bool espnow_enabled;
  int espnow_peer_count;
  int64_t last_event_time_us;
} c6_status_t;

/** WiFi scan result (single AP) | WiFi tarama sonucu (tek AP) */
typedef struct {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t authmode; /**< wifi_auth_mode_t */
} c6_wifi_ap_t;

/** Max scan results returned at once | Tek seferde donen azami tarama sonucu */
#define C6_WIFI_SCAN_MAX 20

/* ================================================================
 *  Public API | Genel API
 * ================================================================ */

/** @brief Init C6 subsystem (SDIO + esp_hosted). Call after Ethernet, before HTTP.
 *  | C6 alt-sistemini baslat (SDIO + esp_hosted). Ethernet sonrasi, HTTP oncesi cagir. */
esp_err_t c6_controller_init(void);

/** @brief Return cached C6 status (no polling). | Onbellekli C6 durumunu dondur (yoklama yok). */
esp_err_t c6_controller_get_status(c6_status_t *status);

/** @brief Start WiFi STA connection; SSID/password saved to NVS and sent to C6.
 *  | WiFi STA baglantisini baslat; SSID/parola NVS'e kaydedilir ve C6'ya gonderilir. */
esp_err_t c6_wifi_connect(const char *ssid, const char *password);

/**
 * @brief WiFi scan (synchronous, 10s timeout).
 *
 * @param aps       [out] AP list (caller-allocated)
 * @param max_aps   max entries
 * @param out_count [out] number of APs found
 * ---
 * @brief WiFi taramasi (senkron, 10s zaman-asimi).
 *
 * @param aps       [cikis] AP listesi (cagiran tarafindan ayrilir)
 * @param max_aps   azami giris sayisi
 * @param out_count [cikis] bulunan AP sayisi
 */
esp_err_t c6_wifi_scan(c6_wifi_ap_t *aps, int max_aps, int *out_count);

/** @brief Update WiFi AP config (ssid NULL = keep, channel 0 = auto).
 *  | WiFi AP yapilandirmasini guncelle (ssid NULL = koru, kanal 0 = otomatik). */
esp_err_t c6_wifi_ap_config(const char *ssid, uint8_t channel);

/** @brief Set WiFi mode (AP/STA/APSTA). | WiFi modunu ayarla (AP/STA/APSTA). */
esp_err_t c6_wifi_set_mode(c6_wifi_mode_t mode);

/** @brief Disconnect WiFi STA. | WiFi STA baglantisini kes. */
esp_err_t c6_wifi_disconnect(void);

/**
 * @brief Return WiFi connection status.
 *
 * @param connected [out] STA connected?
 * @param ssid      [out] connected SSID (>= 33 bytes)
 * @param rssi      [out] signal strength (may be NULL)
 * ---
 * @brief WiFi baglanti durumunu dondur.
 *
 * @param connected [cikis] STA bagli mi?
 * @param ssid      [cikis] bagli SSID (>= 33 bayt)
 * @param rssi      [cikis] sinyal gucu (NULL olabilir)
 */
esp_err_t c6_wifi_get_status(bool *connected, char *ssid, int8_t *rssi);

/** @brief Start the always-on fallback WiFi AP (called automatically at boot).
 *  | Her-zaman-acik yedek WiFi AP'yi baslat (acilista otomatik cagrilir). */
esp_err_t c6_wifi_ap_start(void);

/** @brief Enable/disable C6 BLE. | C6 BLE'yi etkinlestir/devre-disi birak. */
esp_err_t c6_bt_set_enabled(bool enable);

/** @brief Set C6 BLE device name (max 31 chars). | C6 BLE cihaz adini ayarla (azami 31 karakter). */
esp_err_t c6_bt_set_device_name(const char *name);

/** @brief Enable/disable ESP-NOW (saved to NVS). | ESP-NOW'u etkinlestir/devre-disi birak (NVS'e kaydedilir). */
esp_err_t c6_espnow_set_enabled(bool enable);

/** @brief Reset C6 via GPIO54 (500ms LOW pulse). | C6'yi GPIO54 uzerinden sifirla (500ms LOW darbe). */
esp_err_t c6_controller_reboot(void);

/** @brief Erase C6 NVS namespaces (wifi_cfg + bt_cfg). | C6 NVS ad-alanlarini sil (wifi_cfg + bt_cfg). */
esp_err_t c6_controller_factory_reset(void);

/** @brief Pause C6 SDIO transport before SD card mount (WiFi stays configured).
 *  | SD kart baglanmadan once C6 SDIO tasimasini duraklat (WiFi yapilandirili kalir). */
esp_err_t c6_controller_suspend(void);

/** @brief Resume C6 SDIO after SD card unmount. | SD kart ayrildiktan sonra C6 SDIO'yu surdur. */
esp_err_t c6_controller_resume(void);

/** @brief Process pending C6 event callbacks (called periodically from main loop).
 *  | Bekleyen C6 olay geri-cagrilarini isle (ana donguden periyodik cagrilir). */
void c6_controller_process_events(void);

#ifdef __cplusplus
}
#endif
