/**
 * @file fc_logger.h
 * @brief Serial telemetry logger — Core 0, low-priority, no FC impact.
 *
 * Reads FC state via seqlock and prints one telemetry line per tick.
 * ---
 * @brief Seri telemetri kaydedici — Core 0, düşük-öncelikli, FC'ye etkisiz.
 *
 * FC durumunu seqlock ile okur ve tick başına bir telemetri satırı yazdırır.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Log interval in seconds | Kayıt aralığı (saniye) */
#define FC_LOGGER_INTERVAL_S 2

/**
 * @brief Start the FC logger task (Core 0, priority 1).
 * ---
 * @brief FC kaydedici görevini başlatır (Core 0, öncelik 1).
 */
esp_err_t fc_logger_start(void);

#ifdef __cplusplus
}
#endif
