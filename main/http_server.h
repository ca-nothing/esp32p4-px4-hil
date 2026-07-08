#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file http_server.h
 * @brief Minimal HTTP server — serves a single static placeholder page.
 *
 * Serves only a static page from LittleFS (/index.html); no functional API.
 * The flight stack is monitored via QGroundControl (MAVLink), not HTTP.
 * Kept as a stub to be extended later if a web interface is needed.
 * ---
 * @brief Asgari HTTP sunucusu — tek bir statik yer-tutucu sayfa sunar.
 *
 * Yalnizca LittleFS'ten statik bir sayfa sunar (/index.html); islevsel API yok.
 * Ucus yigini HTTP ile degil, QGroundControl (MAVLink) ile izlenir.
 * Ileride bir web arayuzu gerekirse genisletilmek uzere stub olarak tutulur.
 */

esp_err_t http_server_init(void);
esp_err_t http_server_stop(void);

#ifdef __cplusplus
}
#endif
