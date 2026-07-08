#pragma once
/*
 * fc_wq_pthread_compat.h — pthread shim for the PX4 WorkQueue POSIX path.
 * Force-included only into px4_work_queue TUs (CMakeLists FC_PX4_WQ_SRCS).
 *
 * WorkQueueManager.cpp (POSIX path) needs two symbols ESP-IDF pthread lacks:
 *   - PTHREAD_STACK_MIN: in <esp_pthread.h>, not <limits.h> -> pulled in below.
 *   - pthread_attr_setschedpolicy: absent -> NO-OP (ESP-IDF FreeRTOS is always
 *     priority-preemptive, so scheduling policy is moot).
 *
 * POSIX path chosen over NuttX (__PX4_NUTTX) to avoid ODR/NuttX-header risk.
 * Core0 pinning + priority of WQ threads is set at runtime (esp_pthread); this
 * shim only closes compile-time gaps.
 * ---
 * fc_wq_pthread_compat.h — PX4 WorkQueue POSIX yolu icin pthread shim.
 * Yalnizca px4_work_queue TU'larina force-include edilir (CMakeLists FC_PX4_WQ_SRCS).
 *
 * WorkQueueManager.cpp (POSIX yolu) ESP-IDF pthread'in eksik oldugu iki sembol ister:
 *   - PTHREAD_STACK_MIN: <esp_pthread.h>'te, <limits.h>'te degil -> asagida cekilir.
 *   - pthread_attr_setschedpolicy: yok -> NO-OP (ESP-IDF FreeRTOS her zaman
 *     oncelik-kesmeli, bu yuzden zamanlama politikasi anlamsiz).
 *
 * ODR/NuttX-baslik riskinden kacinmak icin NuttX (__PX4_NUTTX) yerine POSIX yolu secildi.
 * WQ ipliklerinin Core0 sabitleme + onceligi calisma-zamaninda ayarlanir (esp_pthread); bu
 * shim yalnizca derleme-zamani bosluklarini kapatir.
 */
#include <esp_pthread.h>   /* PTHREAD_STACK_MIN = CONFIG_PTHREAD_STACK_MIN */
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* setschedpolicy absent in ESP-IDF pthread: NO-OP (always prio-preemptive -> policy moot).
   | setschedpolicy ESP-IDF pthread'te yok: NO-OP (her zaman oncelik-kesmeli -> politika anlamsiz). */
static inline int fc_pthread_attr_setschedpolicy_noop(pthread_attr_t *attr, int policy)
{
	(void)attr;
	(void)policy;
	return 0;
}
#define pthread_attr_setschedpolicy(attr, policy) fc_pthread_attr_setschedpolicy_noop((attr), (policy))

#ifdef __cplusplus
}
#endif
