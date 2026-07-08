/*
 * fc_hrt_esp_timer.c — P4-native hrt callout dispatcher (esp_timer backend).
 *
 * Replaces the PX4 legacy hrt dispatcher (platforms/common/work_queue/hrt_*.c: wkr_hrt worker-thread +
 * px4_usleep-polling + SIGCONT + _hrt_work_lock). The legacy path was broken on ESP-IDF: (1) if init is
 * not called, sem_wait(NULL) asserts = boot-loop; (2) wakeup uses px4_task_kill(SIGCONT), but here
 * px4_task_kill=task-delete -> every reschedule would destroy the task slot.
 *
 * Solution: a single esp_timer (one-shot, 1us, hardware). No thread/sem/signal/pid. The callback runs in the
 * esp_timer task-dispatch context -> FreeRTOS API (sem_post) is safe (as long as we do not block/yield) -> drv_hrt hrt_tim_isr.
 *
 * PRESERVED (verbatim PX4): drv_hrt.cpp callout_queue (sq_* and dq_*) + _hrt_lock + hrt_tim_isr. drv_hrt calls ONLY
 * hrt_work_queue/hrt_work_cancel(&_hrt_work) (ONE consumer, ONE work_s) -> a single global esp_timer suffices.
 *
 * SAFETY: all communication is on Core0; the 8kHz Core1 is a separate task -> UNAFFECTED by this file.
 *
 * Build flags: __PX4_POSIX;__PX4_ESPIDF;MODULE_NAME="hrt". Include: -I PX4/platforms/posix/include.
 * ---
 * fc_hrt_esp_timer.c — P4-yerel hrt callout dağıtıcısı (esp_timer arka ucu).
 *
 * PX4 legacy hrt dağıtıcısının yerini alır (platforms/common/work_queue/hrt_*.c: wkr_hrt işçi-iş-parçacığı +
 * px4_usleep-yoklama + SIGCONT + _hrt_work_lock). Legacy yol ESP-IDF'te bozuktu: (1) init çağrılmazsa
 * sem_wait(NULL) assert atar = boot-döngüsü; (2) uyandırma px4_task_kill(SIGCONT) kullanır, ama burada
 * px4_task_kill=görev-sil -> her yeniden-zamanlama görev yuvasını yok ederdi.
 *
 * Çözüm: tek bir esp_timer (tek-seferlik, 1us, donanım). İş-parçacığı/sem/sinyal/pid yok. Callback,
 * esp_timer görev-dağıtım bağlamında koşar -> FreeRTOS API'si (sem_post) güvenli (bloke/yield etmediğimiz sürece) -> drv_hrt hrt_tim_isr.
 *
 * KORUNDU (verbatim PX4): drv_hrt.cpp callout_queue (sq_* and dq_* | sq_* ve dq_*) + _hrt_lock + hrt_tim_isr. drv_hrt SADECE
 * hrt_work_queue/hrt_work_cancel(&_hrt_work) çağırır (TEK tüketici, TEK work_s) -> tek bir global esp_timer yeter.
 *
 * GÜVENLİK: tüm iletişim Core0'da; 8kHz Core1 ayrı bir görev -> bu dosyadan ETKİLENMEZ.
 *
 * Derleme bayrakları: __PX4_POSIX;__PX4_ESPIDF;MODULE_NAME="hrt". Include: -I PX4/platforms/posix/include.
 */

#include <esp_timer.h>
#include <px4_platform_common/log.h>
#include "hrt_work.h"   /* prototypes + struct work_s / worker_t (transitively workqueue.h) | prototipler + struct work_s / worker_t (dolaylı olarak workqueue.h) */

/* drv_hrt's single callout-timer (drv_hrt _hrt_work). | drv_hrt'nin tek callout-zamanlayıcısı (drv_hrt _hrt_work). */
static esp_timer_handle_t s_hrt_timer;
/* drv_hrt always passes the same worker (hrt_tim_isr) + arg=NULL; the callback reads these. | drv_hrt hep aynı worker'ı (hrt_tim_isr) + arg=NULL geçirir; callback bunları okur. */
static volatile worker_t  s_worker;
static void              *s_arg;

/* esp_timer callback (Task-Dispatch context). worker = drv_hrt hrt_tim_isr; non-blocking. | esp_timer callback'i (Görev-Dağıtım bağlamı). worker = drv_hrt hrt_tim_isr; bloke-etmez. */
static void fc_hrt_esp_timer_cb(void *unused)
{
	(void)unused;
	worker_t w = s_worker;
	if (w != NULL) {
		w(s_arg);
	}
}

/* Sibling of hrt_init() (fc_px4_modules_start calls it AFTER that). Idempotent. | hrt_init()'in kardeşi (fc_px4_modules_start onu ONDAN SONRA çağırır). İdempotent. */
void hrt_work_queue_init(void)
{
	if (s_hrt_timer != NULL) {
		return;   /* already set up (guard) | zaten kurulmuş (koruma) */
	}

	const esp_timer_create_args_t args = {
		.callback              = &fc_hrt_esp_timer_cb,
		.arg                   = NULL,
		.dispatch_method       = ESP_TIMER_TASK,   /* sem_post is safe; ISR-dispatch NOT required | sem_post güvenli; ISR-dağıtımı GEREKMEZ */
		.name                  = "fc_hrt",
		.skip_unhandled_events = false,
	};

	esp_err_t e = esp_timer_create(&args, &s_hrt_timer);
	if (e != ESP_OK) {
		PX4_ERR("fc_hrt esp_timer_create failed (%d)", (int)e);
		s_hrt_timer = NULL;
	}
}

/* drv_hrt hrt_call_reschedule: hrt_work_cancel(&_hrt_work) + hrt_work_queue(&_hrt_work, hrt_tim_isr, NULL, delay).
 * delay = us remaining until the next callout (drv_hrt in the HRT_INTERVAL_MIN=50 .. HRT_INTERVAL_MAX=50s range).
 * ---
 * drv_hrt hrt_call_reschedule: hrt_work_cancel(&_hrt_work) + hrt_work_queue(&_hrt_work, hrt_tim_isr, NULL, delay).
 * delay = bir sonraki callout'a kalan us (drv_hrt HRT_INTERVAL_MIN=50 .. HRT_INTERVAL_MAX=50s aralığında verir). */
int hrt_work_queue(struct work_s *work, worker_t worker, void *arg, uint32_t delay_us)
{
	if (s_hrt_timer == NULL) {
		hrt_work_queue_init();
	}
	if (s_hrt_timer == NULL) {
		return -1;
	}

	s_worker = worker;
	s_arg    = arg;
	if (work != NULL) {            /* keep work_s fields (legacy-faithful; drv_hrt does not read them but stays consistent) | work_s alanlarını tut (legacy-sadık; drv_hrt okumaz ama tutarlı kalır) */
		work->worker = worker;
		work->arg    = arg;
		work->delay  = delay_us;
	}

	(void)esp_timer_stop(s_hrt_timer);            /* cancel if pending (reschedule); otherwise INVALID_STATE = harmless | beklemedeyse iptal et (yeniden-zamanlama); aksi halde INVALID_STATE = zararsız */
	if (delay_us < 50u) {
		delay_us = 50u;                       /* esp_timer one-shot lower bound; drv_hrt already gives >=50us | esp_timer tek-seferlik alt sınırı; drv_hrt zaten >=50us verir */
	}
	(void)esp_timer_start_once(s_hrt_timer, (uint64_t)delay_us);
	return 0;
}

void hrt_work_cancel(struct work_s *work)
{
	if (s_hrt_timer != NULL) {
		(void)esp_timer_stop(s_hrt_timer);
	}
	if (work != NULL) {
		work->worker = NULL;   /* legacy semantics: "not queued" | legacy semantiği: "kuyruğa alınmadı" */
	}
}
