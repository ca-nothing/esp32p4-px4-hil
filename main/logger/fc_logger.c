/**
 * @file fc_logger.c
 * @brief Serial telemetry logger — Core 0, no FC impact.
 *
 * A single task reads the Core-1 seqlock every 2 s and prints one telemetry
 * line (uptime, loop count, overrun rate, jitter, EKF attitude/vel/pos).
 * Serial-only — no file I/O, so it never blocks the FC loop.
 * ---
 * @brief Seri telemetri kaydedici — Core 0, FC'ye etkisiz.
 *
 * Tek bir görev Core-1 seqlock'unu her 2 s'de bir okur ve bir telemetri satırı
 * yazdırır (çalışma süresi, döngü sayısı, overrun oranı, jitter, EKF tutum/hız/konum).
 * Yalnızca-seri — dosya G/Ç yok, bu yüzden FC döngüsünü asla bloklamaz.
 */

#include "fc_logger.h"
#include "fc_core.h"
#include "fc_px4_modules.h"   /* fc_px4_armed() */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fc_logger";

/* Build tag: identifies which binary is running from boot logs.
   | Derleme etiketi: boot loglarından hangi binary'nin çalıştığını belirler. */
#define FC_DEBUG_BUILD_TAG "esp32p4-px4-hil"

/* ── Logger task: 2s tick, serial print ──
   | ── Kaydedici görevi: 2s tick, seri yazdırma ── */
static void fc_logger_task(void *arg) {
  /* build tag + compile date/time so boot logs pin down the exact binary.
     | derleme etiketi + derleme tarih/saati, boot logları tam binary'yi saptasın diye. */
  ESP_LOGI(TAG, "Logger task started build=%s (%s %s)",
           FC_DEBUG_BUILD_TAG, __DATE__, __TIME__);

  uint32_t prev_overrun  = 0;
  uint32_t prev_uptime_s = 0;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(FC_LOGGER_INTERVAL_S * 1000));

    fc_state_t st;
    fc_seqlock_read(&st);

    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    uint32_t dt = uptime_s - prev_uptime_s;
    if (dt == 0) dt = 1;

    uint32_t delta_overrun  = st.overrun_count - prev_overrun;
    float    overrun_per_s  = (float)delta_overrun / (float)dt;

    prev_overrun  = st.overrun_count;
    prev_uptime_s = uptime_s;

    ESP_LOGI(TAG,
             "uptime=%lus loop_count=%lu overrun/s=%.3f jitter_max=%luus ekf_max=%luus spi=%luus compute=%luus armed=%d "
             "ecl_healthy=%d quat=(%.3f,%.3f,%.3f,%.3f) vel_ned=(%.3f,%.3f,%.3f) pos_ned=(%.2f,%.2f,%.2f)",
             (unsigned long)uptime_s, (unsigned long)st.loop_count, overrun_per_s,
             (unsigned long)st.jitter_max_us,
             (unsigned long)st.dbg_ekf_max_us,
             (unsigned long)st.spi_time_us,
             (unsigned long)st.compute_time_us,
             fc_px4_armed() ? 1 : 0,
             st.ekf.healthy ? 1 : 0,
             st.ekf.attitude.w, st.ekf.attitude.x, st.ekf.attitude.y, st.ekf.attitude.z,
             st.ekf.state24[4], st.ekf.state24[5], st.ekf.state24[6],
             st.ekf.state24[7], st.ekf.state24[8], st.ekf.state24[9]);
  }
}

esp_err_t fc_logger_start(void) {
  BaseType_t r1 = xTaskCreatePinnedToCore(
      fc_logger_task, "fc_logger", 4096, NULL, 1, NULL, 0);
  if (r1 != pdPASS) {
    ESP_LOGE(TAG, "Failed to create logger task");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "FC logger task created (Core 0, interval=%ds)", FC_LOGGER_INTERVAL_S);
  return ESP_OK;
}
