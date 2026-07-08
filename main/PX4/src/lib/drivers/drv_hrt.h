/**
 * @file drv_hrt.h
 * @brief PX4 high-resolution timer arayuzunun MINIMAL shim'i (ESP32-P4 portu).
 *
 * Byte-kopyalanan PX4 siniflari (lib/hysteresis, mc_pos_control/Takeoff, ...)
 * <drivers/drv_hrt.h> bekler: hrt_abstime tipi + time_literals (1_s/1_ms/1_us).
 * P4'te zaman kaynagi esp_timer_get_time() (us). hrt_absolute_time() gerektiginde
 * eklenecek; mevcut byte-kopyalar now_us'u parametre olarak alir (kaynak degismez).
 */
#pragma once

#include <stdint.h>

typedef uint64_t hrt_abstime;

/* P4 hrt fonksiyonlari: uORB (SubscriptionInterval) + diger PX4 kodu bunlari bekler.
 * ESP32-P4 tek zaman kaynagi esp_timer_get_time() (mikrosaniye). static inline ->
 * her TU kendi kopyasi (cok-tanim link hatasi yok), C ve C++ uyumlu.
 * (Dosyanin onceki TODO'su "hrt_absolute_time() gerektiginde eklenecek" -> uORB icin simdi.) */
#include <esp_timer.h>
static inline hrt_abstime hrt_absolute_time(void) { return (hrt_abstime)esp_timer_get_time(); }
static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then) { return hrt_absolute_time() - *then; }

/* HRT latency histogram API (PX4 drv_hrt.h satir 90-284, !__PX4_NUTTX inline dali BIREBIR).
 * perf_counter.cpp::perf_print_latency() kullanir. P4'te hrt_call CALLBACK kuyrugu YOK
 * (kendi 8kHz dongusu var) -> latency_counters HEP 0 (gercek geciktme izlenmez; perf
 * 0 basar, zararsiz instrumentation). Diziler fc_px4_hrt_latency.c'de tanimli. */
#define LATENCY_BUCKET_COUNT 8
#ifdef __cplusplus
extern "C" {
#endif
extern const uint16_t latency_bucket_count;
extern const uint16_t latency_buckets[LATENCY_BUCKET_COUNT];
extern uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];
#ifdef __cplusplus
}
#endif

typedef struct latency_info {
	uint16_t bucket;
	uint32_t counter;
} latency_info_t;

static inline uint16_t get_latency_bucket_count(void) { return LATENCY_BUCKET_COUNT; }

static inline latency_info_t get_latency(uint16_t bucket_idx, uint16_t counter_idx)
{
	latency_info_t ret = {latency_buckets[bucket_idx], latency_counters[counter_idx]};
	return ret;
}

static inline void reset_latency_counters(void)
{
	for (int i = 0; i <= get_latency_bucket_count(); i++) {
		latency_counters[i] = 0;
	}
}

#ifdef __cplusplus
/* PX4 time_literals (platforms/common/include/px4_platform_common/time.h):
 * derleme-zamani literal -> mikrosaniye. Takeoff.hpp "using namespace time_literals". */
namespace time_literals
{
constexpr hrt_abstime operator"" _s(unsigned long long seconds) { return seconds * 1000000ULL; }
constexpr hrt_abstime operator"" _ms(unsigned long long milliseconds) { return milliseconds * 1000ULL; }
constexpr hrt_abstime operator"" _us(unsigned long long microseconds) { return microseconds; }
} // namespace time_literals
#endif
