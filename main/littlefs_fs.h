#pragma once
#include "esp_err.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file littlefs_fs.h
 * @brief LittleFS filesystem for Web UI (HTML/JS/CSS)
 *
 * Mounts the LittleFS partition containing the management web UI.
 * No writes from main app — HTML/JS updated via build-time image generation.
 * Config/settings stored in NVS, NOT here.
 * ---
 * @brief Web UI icin LittleFS dosya-sistemi (HTML/JS/CSS)
 *
 * Yonetim web UI'sini iceren LittleFS bolumunu baglar.
 * Ana uygulamadan yazma yok — HTML/JS derleme-aninda imaj uretimiyle guncellenir.
 * Yapilandirma/ayarlar burada DEGIL, NVS'te saklanir.
 */

/** LittleFS mount point (also used as partition label) | LittleFS baglama noktasi (bolum etiketi olarak da kullanilir) */
#define LITTLEFS_MOUNT_POINT "/littlefs"
#define LITTLEFS_PARTITION   "littlefs"

esp_err_t littlefs_fs_init(void);
esp_err_t littlefs_fs_get_info(size_t *total, size_t *used);

#ifdef __cplusplus
}
#endif
