/*
 * fc_px4_tasks.cpp — P4/ESP-IDF platform TASK backend (FreeRTOS).
 *
 * Per-OS impl of px4_task_spawn_cmd/delete/exit/kill (the posix=pthread, nuttx=task_create
 * counterpart). px4_task_t = int = task-table index. Modules bind their run()-loop to a
 * FreeRTOS task THROUGH THIS. Low-rate modules are pinned to FC_MODULE_TASK_CORE; the 8kHz
 * control core runs on the OTHER core.
 * ---
 * fc_px4_tasks.cpp — P4/ESP-IDF platform GÖREV arka ucu (FreeRTOS).
 *
 * px4_task_spawn_cmd/delete/exit/kill'in OS'e-özel implementasyonu (posix=pthread, nuttx=task_create
 * karşılığı). px4_task_t = int = görev-tablosu indeksi. Modüller run()-döngülerini BUNUN ÜZERİNDEN bir
 * FreeRTOS görevine bağlar. Düşük-hızlı modüller FC_MODULE_TASK_CORE'a sabitlenir; 8kHz
 * kontrol çekirdeği DİĞER çekirdekte çalışır.
 */

#include <px4_platform_common/tasks.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pthread.h"   /* module tasks are PTHREADs (esp_pthread core-pin) | modül görevleri PTHREAD'dir (esp_pthread çekirdek-sabitleme) */
#include "esp_heap_caps.h" /* MALLOC_CAP_SPIRAM */
#include "esp_log.h"
#include <pthread.h>
#include <unistd.h>        /* sysconf / _SC_PAGESIZE (override at end of file) | (dosya sonunda ezilir) */

#include <cstdint>

#ifndef FC_MODULE_TASK_CORE
#define FC_MODULE_TASK_CORE 0   /* low-rate modules; 8kHz control core on the OTHER core | düşük-hızlı modüller; 8kHz kontrol çekirdeği DİĞER çekirdekte */
#endif

#ifndef FC_MODULE_MAX_PRIO
/* Priority ceiling for unnamed modules. Core0 priority stack:
 * EKF=5 > mavlink-RX/TX=3 (sensor I/O) > modules=2 > logger=1. Modules must stay BELOW
 * mavlink-RX so they do not starve the sensor stream when active (else EKF stalls).
 * ---
 * İsimsiz modüller için öncelik tavanı. Core0 öncelik yığını:
 * EKF=5 > mavlink-RX/TX=3 (sensör I/O) > modüller=2 > logger=1. Modüller mavlink-RX'in ALTINDA
 * kalmalı; aktifken sensör akışını aç bırakmasınlar (yoksa EKF durur). */
#define FC_MODULE_MAX_PRIO 2
#endif

#define FC_PX4_MAX_TASKS 16

struct fc_px4_task {
	pthread_t    handle;
	px4_main_t   entry;
	int          argc;
	char       **argv;
	char        *argv_buf[10];   /* persistent argv (caller's argv may be transient) | kalıcı argv (çağıranın argv'si geçici olabilir) */
	volatile bool used;
};

static fc_px4_task s_tasks[FC_PX4_MAX_TASKS];

/* Modules are created as PTHREADs (NOT xTaskCreate) because PX4 platform code (uORB
 * SubscriptionBlocking pthread_cond, LockGuard pthread_mutex, px4_sem) requires pthread_self;
 * it asserts from a native FreeRTOS task. esp_pthread preserves the core-pin.
 * ---
 * Modüller PTHREAD olarak oluşturulur (xTaskCreate DEĞİL) çünkü PX4 platform kodu (uORB
 * SubscriptionBlocking pthread_cond, LockGuard pthread_mutex, px4_sem) pthread_self gerektirir;
 * yerel bir FreeRTOS görevinden assert atar. esp_pthread çekirdek-sabitlemeyi korur. */
static void *fc_px4_task_trampoline(void *arg)
{
	fc_px4_task *t = static_cast<fc_px4_task *>(arg);
	t->entry(t->argc, t->argv);
	t->used = false;              /* run() returned via should_exit | run() should_exit ile döndü */
	return nullptr;
}

extern "C" px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority,
		int stack_size, px4_main_t entry, char *const argv[])
{
	(void)scheduler;

	int id = -1;

	for (int i = 0; i < FC_PX4_MAX_TASKS; i++) {
		if (!s_tasks[i].used) { id = i; break; }
	}

	if (id < 0) { return -1; }

	int argc = 0;

	if (argv) { while (argv[argc]) { argc++; } }

	/* run_trampoline_impl expects argv[0]=task-name and shifts (module_base.cpp:212). Prepend name
	 * to argv[0] + copy into a PERSISTENT buffer (caller's argv is transient). Without the prepend
	 * 'start' is swallowed -> commander cannot see argv[1]=="-h" -> enable_hil is not called.
	 * ---
	 * run_trampoline_impl argv[0]=görev-adı bekler ve kaydırır (module_base.cpp:212). İsmi argv[0]'a
	 * ÖNE-EKLE + KALICI bir tampona kopyala (çağıranın argv'si geçici). Öne-ekleme olmadan
	 * 'start' yutulur -> commander argv[1]=="-h"'yi göremez -> enable_hil çağrılmaz. */
	const int max_buf = (int)(sizeof(s_tasks[id].argv_buf) / sizeof(char *)) - 1;
	int n = 0;
	s_tasks[id].argv_buf[n++] = const_cast<char *>(name);
	for (int i = 0; i < argc && n < max_buf; i++) {
		s_tasks[id].argv_buf[n++] = const_cast<char *>(argv[i]);
	}
	s_tasks[id].argv_buf[n] = nullptr;

	s_tasks[id].entry = entry;
	s_tasks[id].argc  = n;
	s_tasks[id].argv  = s_tasks[id].argv_buf;
	s_tasks[id].used  = true;

	int prio = priority;

	if (prio >= (int)configMAX_PRIORITIES) { prio = configMAX_PRIORITIES - 1; }

	/* Fixed P4-prio for named PX4 tasks. PX4 ordering is preserved (commander>nav>dataman;
	 * sensor-source sim_mavlink ABOVE the estimator) but all stay BELOW 8kHz-Core1(24) and fc_ekf(21)
	 * -> they do NOT PREEMPT the estimator/feed. lwIP stays at 13 (absent in PX4 = P4-artifact).
	 * ---
	 * İsimli PX4 görevleri için sabit P4-önceliği. PX4 sıralaması korunur (commander>nav>dataman;
	 * sensör-kaynağı sim_mavlink kestirimcinin ÜSTÜNDE) ama hepsi 8kHz-Core1(24) ve fc_ekf(21)'in ALTINDA
	 * kalır -> kestirimciyi/beslemeyi ÖNCELEMEZLER. lwIP 13'te kalır (PX4'te yok = P4-artefaktı). */
	if (name && __builtin_strcmp(name, "simulator_mavlink") == 0) {
		prio = 19;   /* PX4 sim_mavlink=SCHED_PRIORITY_MAX (SimulatorMavlink.cpp:1922): sensor-SOURCE above all flight-WQ. | sensör-KAYNAĞI tüm uçuş-WQ'sunun üstünde.
		             * 19 = ABOVE rate_ctrl(18), BELOW event-task(20)/fc_ekf(21)/esp_timer(22). | 19 = rate_ctrl(18) ÜSTÜ, event-task(20)/fc_ekf(21)/esp_timer(22) ALTI. */
	} else if (name && __builtin_strcmp(name, "commander") == 0) {
		prio = 9;    /* PX4 commander=SCHED_PRIORITY_DEFAULT+40 (Commander.cpp:2849): TOP of the DEFAULT band. | DEFAULT bandının TEPESİ.
		             * ABOVE navigator(8)/sender(7)/dataman(6), BELOW receiver(10); does NOT PREEMPT the EKF chain (14-19). | navigator(8)/sender(7)/dataman(6) ÜSTÜ, receiver(10) ALTI; EKF zincirini (14-19) ÖNCELEMEZ. */
	} else if (name && __builtin_strcmp(name, "navigator") == 0) {
		prio = 8;    /* PX4 navigator=SCHED_PRIORITY_NAVIGATION=DEFAULT+5 (navigator_main.cpp:1096): BELOW commander(9), ABOVE sender(7). | commander(9) ALTI, sender(7) ÜSTÜ. */
	} else if (name && __builtin_strcmp(name, "dataman") == 0) {
		prio = 6;    /* PX4 dataman=SCHED_PRIORITY_DEFAULT-10: BELOW navigator(8)/sender(7). | navigator(8)/sender(7) ALTI. */
	} else if (name && __builtin_strcmp(name, "wq:manager") == 0) {
		/* WQ-CREATOR task. PX4 spawns it with SCHED_PRIORITY_MAX (WorkQueueManager.cpp:380) so it can
		 * create WQs IMMEDIATELY; if it falls to clamp-2 it STARVES under load -> hp_default WQ cannot be created -> pwm_out_sim
		 * init-fails -> motor path is dead. 23 = ABOVE all WQ/feed/ekf/esp_timer(<=22), BELOW Core1(24); mostly-blocked.
		 * ---
		 * WQ-ÜRETİCİ görev. PX4 onu SCHED_PRIORITY_MAX (WorkQueueManager.cpp:380) ile başlatır ki WQ'ları
		 * HEMEN oluşturabilsin; clamp-2'ye düşerse yük altında AÇ KALIR -> hp_default WQ oluşturulamaz -> pwm_out_sim
		 * init-başarısız -> motor yolu ölür. 23 = tüm WQ/besleme/ekf/esp_timer(<=22) ÜSTÜ, Core1(24) ALTI; çoğunlukla-bloke. */
		prio = 23;
	} else if (name && __builtin_strcmp(name, "mavlink_main") == 0) {
		/* PX4 mavlink SENDER=SCHED_PRIORITY_DEFAULT (mavlink_main.cpp:3078): BELOW navigator(8), ABOVE dataman(6);
		 * BELOW the realtime-WQ chain (receiver=10..rate=18) -> does NOT PREEMPT sensor/estimator.
		 * ---
		 * PX4 mavlink GÖNDERİCİ=SCHED_PRIORITY_DEFAULT (mavlink_main.cpp:3078): navigator(8) ALTI, dataman(6) ÜSTÜ;
		 * gerçek-zamanlı-WQ zincirinin (receiver=10..rate=18) ALTINDA -> sensörü/kestirimciyi ÖNCELEMEZ. */
		prio = 7;
	} else if (prio > FC_MODULE_MAX_PRIO) {
		prio = FC_MODULE_MAX_PRIO;
	}

	/* esp_pthread: configuration for the next pthread_create (core-pin + prio + stack). | bir sonraki pthread_create için yapılandırma (çekirdek-sabitleme + prio + stack). */
	esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
	/* MIN-STACK FLOOR 8192: PX4 stack values are NuttX-relative (e.g. sim_mavlink=1500); ESP-IDF FPU-context-save
	 * + deep libc frames need more (1500 -> "Stack protection fault"). Values requesting >8192 keep their own.
	 * ---
	 * MİN-STACK TABANI 8192: PX4 stack değerleri NuttX-göreceli (örn. sim_mavlink=1500); ESP-IDF FPU-bağlam-kaydı
	 * + derin libc çerçeveleri daha fazlasını gerektirir (1500 -> "Stack protection fault"). >8192 isteyen değerler kendilerini korur. */
	uint32_t req_stack = (stack_size > 0) ? (uint32_t)stack_size : 4096u;
	cfg.stack_size   = (req_stack < 8192u) ? 8192u : req_stack;
	cfg.prio         = prio;
	cfg.pin_to_core  = FC_MODULE_TASK_CORE;   /* 8kHz core on the OTHER core | 8kHz çekirdek DİĞER çekirdekte */
	cfg.thread_name  = name;
	/* inherit_cfg: runner threads that WQ-manager creates via raw pthread_create inherit this cfg (Core0-pin +
	 * prio<=mavlink-RX) -> they do not starve sensor-input. Stack in internal-RAM (a PSRAM-stack risks a
	 * flight-thread crash when the flash-cache is disabled = HAZARD). ENOMEM root-cause: sysconf() override at end of file.
	 * ---
	 * inherit_cfg: WQ-manager'ın ham pthread_create ile oluşturduğu runner iş parçacıkları bu cfg'yi devralır (Core0-sabit +
	 * prio<=mavlink-RX) -> sensör-girdisini aç bırakmazlar. Stack dahili-RAM'de (PSRAM-stack, flash-cache devre-dışıyken bir
	 * uçuş-iş-parçacığı çökmesi riski taşır = TEHLİKE). ENOMEM kök-nedeni: dosya sonundaki sysconf() ezmesi. */
	cfg.inherit_cfg = true;
	/* mavlink_main stack ~58KB does not fit the internal-RAM fragment -> pthread_create silently -1 -> mavlink does not open.
	 * PSRAM-stack (fragmentation-free) -> reliable spawn. No flash-write during operation -> stable.
	 * ---
	 * mavlink_main stack ~58KB dahili-RAM parçasına sığmaz -> pthread_create sessizce -1 -> mavlink açılmaz.
	 * PSRAM-stack (parçalanma-yok) -> güvenilir başlatma. Çalışma sırasında flash-yazma yok -> kararlı. */
	if (name && __builtin_strcmp(name, "mavlink_main") == 0) {
		cfg.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
	}
	esp_pthread_set_cfg(&cfg);

	/* WQ-runner ENOMEM diagnostic (internal-RAM free/largest-block + PSRAM free). | WQ-runner ENOMEM tanılaması (dahili-RAM boş/en-büyük-blok + PSRAM boş). */
	ESP_LOGW("FC_SPAWN", "%s caps=0x%lx int_free=%u int_blk=%u psram=%u", name,
	         (unsigned long)cfg.stack_alloc_caps,
	         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
	         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
	         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

	if (pthread_create(&s_tasks[id].handle, nullptr, fc_px4_task_trampoline, &s_tasks[id]) != 0) {
		s_tasks[id].used = false;
		return -1;
	}

	pthread_detach(s_tasks[id].handle);   /* no join; run() exits via should_exit | join yok; run() should_exit ile çıkar */

	return static_cast<px4_task_t>(id);
}

extern "C" int px4_task_delete(px4_task_t id)
{
	if (id < 0 || id >= FC_PX4_MAX_TASKS || !s_tasks[id].used) { return -1; }

	/* Modules exit run() via should_exit/request_stop -> trampoline returns -> pthread ends.
	 * Forced cancellation (pthread_cancel) is limited on ESP-IDF; just clear the flag.
	 * ---
	 * Modüller run()'dan should_exit/request_stop ile çıkar -> trampolin döner -> pthread biter.
	 * Zorla iptal (pthread_cancel) ESP-IDF'te kısıtlı; sadece bayrağı temizle. */
	s_tasks[id].used = false;
	return 0;
}

extern "C" void px4_task_exit(int ret) { (void)ret; pthread_exit(nullptr); }

extern "C" int px4_task_kill(px4_task_t id, int sig) { (void)sig; return px4_task_delete(id); }

extern "C" px4_task_t px4_getpid(void)
{
	return static_cast<px4_task_t>(reinterpret_cast<uintptr_t>(xTaskGetCurrentTaskHandle()));
}

/* ── PX4 platform extra primitives (posix tasks.cpp/drv_hrt.cpp counterpart) ── */
/* ── PX4 platform ek ilkelleri (posix tasks.cpp/drv_hrt.cpp karşılığı) ── */
#include <time.h>
#include <esp_timer.h>   /* HIL sim-time clock backend (esp_timer_get_time) | HIL sim-zamanı saat arka ucu */

/* px4_prctl (tasks.h:184): thread-name/listing on posix. No prctl-style query on P4 ->
 * no-op (0=success). PX4 module code does not use the return value critically (only PX4_INFO listing).
 * ---
 * px4_prctl (tasks.h:184): posix'te iş-parçacığı-adı/listeleme. P4'te prctl-tarzı sorgu yok ->
 * no-op (0=başarı). PX4 modül kodu dönüş değerini kritik kullanmaz (sadece PX4_INFO listeleme). */
extern "C" int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	(void)option; (void)arg2; (void)pid;
	return 0;
}

/* ENOMEM-fix: WorkQueueManager.cpp:277-279 rounds the stack to page-size (adj + sysconf(_SC_PAGESIZE)
 * - adj%pagesize). ESP-IDF/newlib sysconf(_SC_PAGESIZE)=-1 -> (unsigned)=0xFFFFFFFF -> stacksize ~4GB ->
 * pthread_create ENOMEM. FIX: return a sane value. The STRONG definition overrides the newlib weak-stub (libc.a).
 * ---
 * ENOMEM-düzeltmesi: WorkQueueManager.cpp:277-279 stack'i sayfa-boyutuna yuvarlar (adj + sysconf(_SC_PAGESIZE)
 * - adj%pagesize). ESP-IDF/newlib sysconf(_SC_PAGESIZE)=-1 -> (unsigned)=0xFFFFFFFF -> stacksize ~4GB ->
 * pthread_create ENOMEM. DÜZELTME: makul bir değer döndür. GÜÇLÜ tanım newlib zayıf-stub'ını (libc.a) ezer. */
extern "C" long sysconf(int name)
{
	switch (name) {
	case _SC_PAGESIZE:          return 4096;   /* no MMU; a reasonable page size for stacksize-rounding | MMU yok; stacksize-yuvarlaması için makul bir sayfa boyutu */
	case _SC_NPROCESSORS_ONLN:
	case _SC_NPROCESSORS_CONF:  return CONFIG_FREERTOS_NUMBER_OF_CORES;
	default:                    return -1;
	}
}

/* cdev (cdev_platform.cpp:348 px4_poll) calls `pthread_getname_np(pthread_self(),...)`; ABSENT in ESP-IDF
 * pthread -> newlib weak-stub ENOSYS -> cdev "failed getting thread name" WARN ~20-50 times/sec inside px4_poll
 * -> UART TX-buffer fills and blocks synchronously -> console is polluted + a Core1 jitter spike at arm-time. FIX: the STRONG
 * definition returns the running FreeRTOS task name (pcTaskGetName). weak-stub override (SAME pattern as sysconf).
 * ---
 * cdev (cdev_platform.cpp:348 px4_poll) `pthread_getname_np(pthread_self(),...)` çağırır; ESP-IDF pthread'te
 * YOK -> newlib zayıf-stub ENOSYS -> cdev "failed getting thread name" WARN px4_poll içinde ~20-50 kez/sn
 * -> UART TX-tamponu dolar ve senkron bloke eder -> konsol kirlenir + arm-anında bir Core1 seğirme sıçraması. DÜZELTME: GÜÇLÜ
 * tanım koşan FreeRTOS görev-adını (pcTaskGetName) döndürür. zayıf-stub ezmesi (sysconf ile AYNI desen). */
extern "C" int pthread_getname_np(pthread_t thread, char *name, size_t len)
{
	(void)thread;   /* cdev always pthread_self(); no pthread_t->TaskHandle map -> use the running task | cdev hep pthread_self(); pthread_t->TaskHandle eşlemesi yok -> koşan görevi kullan */
	if (len == 0) { return -1; }   /* 'name' has nonnull-attribute -> do not NULL-check it (nonnull-compare error) | 'name' nonnull-özniteliğine sahip -> NULL-kontrolü yapma (nonnull-compare hatası) */
	const char *tn = pcTaskGetName(nullptr);   /* NULL = running task name | NULL = koşan görev adı */
	size_t i = 0;
	if (tn != nullptr) {
		for (; i < len - 1 && tn[i] != '\0'; ++i) { name[i] = tn[i]; }
	}
	name[i] = '\0';
	return 0;
}

/* ── HIL SIM-TIME CLOCK BACKEND ──
 * ROOT: on HIL_SENSOR, SimulatorMavlink calls px4_clock_settime(CLOCK_MONOTONIC, imu.time_usec) (pulls hrt
 * to sim-time). CLOCK_MONOTONIC CANNOT be set on ESP-IDF = NO-OP -> IMU is stamped with real-arrival ->
 * dt JITTER (2-26ms) -> EKF accel-bias saturates -> "High Accel Bias" arming block.
 * FIX (--wrap; PX4/libc untouched): clock_settime(CLOCK_MONOTONIC) -> record sim-time; hrt_absolute_time
 * -> return sim-time (advances via esp_timer, re-synced on each HIL_SENSOR). Core1 uses esp_timer_get_time()
 * (NOT hrt) -> 8kHz UNAFFECTED; the seqlock stamp is also esp_timer -> no mismatch.
 * ---
 * ── HIL SİM-ZAMANI SAAT ARKA UCU ──
 * KÖK: HIL_SENSOR'da SimulatorMavlink px4_clock_settime(CLOCK_MONOTONIC, imu.time_usec) çağırır (hrt'yi
 * sim-zamanına çeker). CLOCK_MONOTONIC ESP-IDF'te AYARLANAMAZ = NO-OP -> IMU gerçek-varışla damgalanır ->
 * dt SEĞİRMESİ (2-26ms) -> EKF accel-bias doygunlaşır -> "High Accel Bias" arming engeli.
 * DÜZELTME (--wrap; PX4/libc dokunulmadan): clock_settime(CLOCK_MONOTONIC) -> sim-zamanını kaydet; hrt_absolute_time
 * -> sim-zamanını döndür (esp_timer ile ilerler, her HIL_SENSOR'da yeniden-senkronlanır). Core1 esp_timer_get_time()
 * kullanır (hrt DEĞİL) -> 8kHz ETKİLENMEZ; seqlock damgası da esp_timer -> uyumsuzluk yok. */
#ifdef FC_HIL_SIMTIME
extern "C" {

uint64_t __real_hrt_absolute_time(void);
int      __real_clock_settime(clockid_t clk_id, const struct timespec *tp);

static volatile bool     s_simclk_active  = false;
static volatile uint64_t s_simclk_base_us = 0;   /* last sim-time (imu.time_usec) | son sim-zamanı (imu.time_usec) */
static volatile int64_t  s_simclk_real_us = 0;   /* esp_timer (real) at settime moment | settime anındaki esp_timer (gerçek) */

/* SimulatorMavlink:544 px4_clock_settime(CLOCK_MONOTONIC, imu.time_usec) -> clock_settime -> here. | -> buraya. */
int __wrap_clock_settime(clockid_t clk_id, const struct timespec *tp)
{
	if (clk_id == CLOCK_MONOTONIC && tp != nullptr) {
		s_simclk_base_us = (uint64_t)tp->tv_sec * 1000000ull + (uint64_t)tp->tv_nsec / 1000ull;
		s_simclk_real_us = esp_timer_get_time();
		s_simclk_active  = true;
		return 0;
	}
	return __real_clock_settime(clk_id, tp);   /* CLOCK_REALTIME etc. -> real libc | CLOCK_REALTIME vb. -> gerçek libc */
}

/* hrt_absolute_time -> sim-time (if active), otherwise real (safe fallback at boot / before the first HIL_SENSOR). | -> sim-zamanı (aktifse), aksi halde gerçek (boot'ta / ilk HIL_SENSOR öncesi güvenli yedek). */
uint64_t __wrap_hrt_absolute_time(void)
{
	if (s_simclk_active) {
		int64_t drift = esp_timer_get_time() - s_simclk_real_us;
		return s_simclk_base_us + (uint64_t)drift;
	}
	return __real_hrt_absolute_time();
}

}
#endif /* FC_HIL_SIMTIME */

/* ── FC_POLL_CLOCK_FIX: cdev px4_poll MONOTONIC-deadline <-> ESP-IDF sem_timedwait REALTIME ──
 * ROOT: cdev_platform.cpp:397 px4_poll computes the deadline with px4_clock_gettime(CLOCK_MONOTONIC), but
 * ESP-IDF sem_timedwait (pthread_semaphore.c:88) compares against REALTIME + condattr_setclock(MONOTONIC) is a
 * NO-OP -> there is NO MONOTONIC-based timed-wait primitive. When QGC SYSTEM_TIME jumps REALTIME to the 2026-epoch,
 * the MONOTONIC-deadline stays in the past -> immediate ETIMEDOUT -> px4_poll spin -> navigator+dataman spin -> GCS-loss.
 * FIX (--wrap; PX4 untouched): convert the MONOTONIC-deadline to a REALTIME-deadline with the SAME relative-timeout.
 * Before SYSTEM_TIME (REALTIME~=MONOTONIC) behavior is preserved exactly.
 * ---
 * ── FC_POLL_CLOCK_FIX: cdev px4_poll MONOTONIC-deadline'ı <-> ESP-IDF sem_timedwait REALTIME'ı ──
 * KÖK: cdev_platform.cpp:397 px4_poll deadline'ı px4_clock_gettime(CLOCK_MONOTONIC) ile hesaplar, ama
 * ESP-IDF sem_timedwait (pthread_semaphore.c:88) REALTIME'a karşı karşılaştırır + condattr_setclock(MONOTONIC)
 * NO-OP'tur -> MONOTONIC-tabanlı zamanlı-bekleme ilkeli YOKTUR. QGC SYSTEM_TIME REALTIME'ı 2026-epoch'una atladığında,
 * MONOTONIC-deadline geçmişte kalır -> anında ETIMEDOUT -> px4_poll spin -> navigator+dataman spin -> GCS-kaybı.
 * DÜZELTME (--wrap; PX4 dokunulmadan): MONOTONIC-deadline'ı AYNI göreceli-zamanaşımıyla REALTIME-deadline'a çevir.
 * SYSTEM_TIME öncesi (REALTIME~=MONOTONIC) davranış birebir korunur. */
#ifdef FC_POLL_CLOCK_FIX
#include <semaphore.h>
#include <time.h>
extern "C" {
int __real_sem_timedwait(sem_t *sem, const struct timespec *abstime);

int __wrap_sem_timedwait(sem_t *sem, const struct timespec *abstime)
{
	if (abstime == nullptr) {
		return __real_sem_timedwait(sem, abstime);   /* NULL: do not break ESP-IDF EINVAL semantics | NULL: ESP-IDF EINVAL semantiğini bozma */
	}

	struct timespec mono_now, real_now;
	clock_gettime(CLOCK_MONOTONIC, &mono_now);
	clock_gettime(CLOCK_REALTIME, &real_now);

	/* rel = abstime - mono_now  (cdev computed the deadline MONOTONIC-based) | (cdev deadline'ı MONOTONIC-tabanlı hesapladı) */
	int64_t rel_ns = (int64_t)(abstime->tv_sec  - mono_now.tv_sec) * 1000000000ll
			 + (int64_t)(abstime->tv_nsec - mono_now.tv_nsec);
	if (rel_ns < 0) {
		rel_ns = 0;   /* past deadline -> immediate-timeout like ESP-IDF | geçmiş deadline -> ESP-IDF gibi anında-zamanaşımı */
	}

	/* real_abstime = real_now + rel  (ESP-IDF sem_timedwait compares this against REALTIME) | (ESP-IDF sem_timedwait bunu REALTIME'a karşı karşılaştırır) */
	int64_t acc_ns = (int64_t)real_now.tv_nsec + rel_ns;
	struct timespec real_abstime;
	real_abstime.tv_sec  = real_now.tv_sec + (time_t)(acc_ns / 1000000000ll);
	real_abstime.tv_nsec = (long)(acc_ns % 1000000000ll);

	return __real_sem_timedwait(sem, &real_abstime);
}
}
#endif /* FC_POLL_CLOCK_FIX */

/* ── WQ-RUNNER PRIORITY DIFFERENTIATION ──
 * ROOT: WorkQueueManager.cpp:316 pthread_attr_setschedparam(attr, {SCHED_FIFO_max + wq->relative_priority})
 * FAILS on esp_pthread -> ALL WQ-runners fall to prio-2 (cap) -> FreeRTOS equal-prio time-slice -> the EKF (INS0)
 * update spreads over 8.5ms instead of 174us -> ekf2 54Hz, IMU-missed, accel-bias breaks. Unnoticeable on a fast
 * host, fatal on P4. FIX: map PX4 relative_priority to a unique P4 FreeRTOS prio (below) +
 * apply it to the next pthread_create via esp_pthread cfg. On Core0 -> 8kHz Core1 (prio24, SEPARATE core) UNAFFECTED.
 * ---
 * ── WQ-RUNNER ÖNCELİK FARKLILAŞTIRMASI ──
 * KÖK: WorkQueueManager.cpp:316 pthread_attr_setschedparam(attr, {SCHED_FIFO_max + wq->relative_priority})
 * esp_pthread'te BAŞARISIZ olur -> TÜM WQ-runner'lar prio-2'ye (tavan) düşer -> FreeRTOS eşit-prio zaman-dilimi -> EKF (INS0)
 * güncellemesi 174us yerine 8.5ms'ye yayılır -> ekf2 54Hz, IMU-kaçırılır, accel-bias bozulur. Hızlı bir host'ta
 * fark edilmez, P4'te ölümcül. DÜZELTME: PX4 relative_priority'yi benzersiz bir P4 FreeRTOS prio'suna (aşağıda) eşle +
 * bir sonraki pthread_create'e esp_pthread cfg ile uygula. Core0'da -> 8kHz Core1 (prio24, AYRI çekirdek) ETKİLENMEZ. */
#ifdef FC_WQ_PRIO
#include <sched.h>
extern "C" {
int __real_pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param);

int __wrap_pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param)
{
	if (attr != nullptr && param != nullptr) {
		/* rel = wq->relative_priority (0..-50). PX4 sched_priority = sched_get_priority_max(SCHED_FIFO)
		 * (a FUNCTION, ESP-IDF=24) + rel; recovered here.
		 * ---
		 * rel = wq->relative_priority (0..-50). PX4 sched_priority = sched_get_priority_max(SCHED_FIFO)
		 * (bir FONKSİYON, ESP-IDF=24) + rel; burada geri kazanılır. */
		int rel = param->sched_priority - sched_get_priority_max(SCHED_FIFO);
		/* A UNIQUE FreeRTOS prio for each WQ (PX4 ordering preserved: rate_ctrl highest -> lp lowest).
		 * Being unique means the equal-prio time-slice branch NEVER fires = SCHED_FIFO-equivalent. Band 6-18:
		 * ABOVE TASK-module(2)+fc_ekf(5), BELOW feed-receiver(19)/event(20)/esp_timer(22)/Core1(24).
		 * ---
		 * Her WQ için BENZERSIZ bir FreeRTOS prio'su (PX4 sıralaması korunur: rate_ctrl en yüksek -> lp en düşük).
		 * Benzersiz olması, eşit-prio zaman-dilimi dalının ASLA çalışmaması demektir = SCHED_FIFO-eşdeğeri. Bant 6-18:
		 * TASK-modül(2)+fc_ekf(5) ÜSTÜ, besleme-receiver(19)/event(20)/esp_timer(22)/Core1(24) ALTI. */
		int p4_prio;
		bool psram_stack = false;
		/* ⚠️ LATENT-HAZARD: the ">100" branch places the receiver stack in PSRAM (>31KB, does not fit the internal-RAM ceiling).
		 * If a flash-cache-disable (nvs_commit) coincides while the receiver is active on its PSRAM-stack -> CRASH. CURRENTLY MASKED:
		 * param autosave OFF + SD-log OFF + dataman RAM(-r) -> no runtime flash-write. Remaining trigger:
		 * c6_controller.c WiFi/BT nvs_commit (rare). Holistic fix (separate task): shrink the receiver stack into internal-RAM.
		 * ---
		 * ⚠️ GİZLİ-TEHLİKE: ">100" dalı receiver stack'ini PSRAM'e koyar (>31KB, dahili-RAM tavanına sığmaz).
		 * Receiver PSRAM-stack'inde aktifken bir flash-cache-devre-dışı (nvs_commit) çakışırsa -> ÇÖKME. ŞU AN MASKELİ:
		 * param otomatik-kayıt KAPALI + SD-log KAPALI + dataman RAM(-r) -> çalışma-zamanı flash-yazma yok. Kalan tetikleyici:
		 * c6_controller.c WiFi/BT nvs_commit (nadir). Bütünsel düzeltme (ayrı görev): receiver stack'ini dahili-RAM'e küçült. */
		if (param->sched_priority > 200) {
			/* SimulatorMavlink SENDER (PX4 SCHED_PRIORITY_ACTUATOR_OUTPUTS+1). ~42% Core0; if ABOVE mavlink (sender=7/
			 * receiver=10) it STARVES the GCS at saturation -> heartbeat stops. Placed BELOW mavlink (6):
			 * mavlink finds CPU, sim_send takes the rest. Not a deviation (no lockstep -> sim_send need NOT be high).
			 * ---
			 * SimulatorMavlink GÖNDERİCİ (PX4 SCHED_PRIORITY_ACTUATOR_OUTPUTS+1). ~%42 Core0; mavlink'in (sender=7/
			 * receiver=10) ÜSTÜNDE olursa doygunlukta GCS'i AÇ BIRAKIR -> heartbeat durur. mavlink'in ALTINA (6) konur:
			 * mavlink CPU bulur, sim_send kalanı alır. Sapma değil (lockstep yok -> sim_send yüksek olmak ZORUNDA değil). */
			p4_prio = 6;
		}
		else if (param->sched_priority > 100) {
			/* mavlink RECEIVER (PX4 SCHED_PRIORITY_MAX-80=175; mavlink_receiver.cpp:3984): BELOW INS/controller
			 * -> 10 (BELOW hp_default=11, ABOVE commander=9); does NOT PREEMPT sensor/estimator. Stack >31KB -> PSRAM.
			 * ---
			 * mavlink RECEIVER (PX4 SCHED_PRIORITY_MAX-80=175; mavlink_receiver.cpp:3984): INS/kontrolcü ALTI
			 * -> 10 (hp_default=11 ALTI, commander=9 ÜSTÜ); sensörü/kestirimciyi ÖNCELEMEZ. Stack >31KB -> PSRAM. */
			p4_prio = 10;
			psram_stack = true;
		}
		else if (rel >=   0) { p4_prio = 18; }   /* rate_ctrl (PX4 MAX-0, HIGHEST WQ) -> mc_rate_control + control_allocator. BELOW sim_mavlink(19). | rate_ctrl (PX4 MAX-0, EN YÜKSEK WQ) -> ... . sim_mavlink(19) ALTI. */
		else if (rel >=  -7) { p4_prio = 17; }   /* SPI0-6 (PX4 MAX-1..-7; unused in HIL) -> BELOW rate, ABOVE nav_and_ctrl | SPI0-6 (PX4 MAX-1..-7; HIL'de kullanılmaz) -> rate ALTI, nav_and_ctrl ÜSTÜ */
		else if (rel >= -12) { p4_prio = 16; }   /* I2C0-4 (PX4 MAX-8..-12; unused) -> BELOW SPI | I2C0-4 (PX4 MAX-8..-12; kullanılmaz) -> SPI ALTI */
		else if (rel == -13) { p4_prio = 15; }   /* nav_and_controllers (PX4 MAX-13) -> mc_att/mc_pos/sensors/FMM. CONTROLLERS ABOVE ekf2 (invariant PX4 rule). | KONTROLCÜLER ekf2 ÜSTÜNDE (değişmez PX4 kuralı). */
		else if (rel == -14) { p4_prio = 14; }   /* INS0 = ekf2 + vehicle_imu (PX4 MAX-14) -> BELOW the controllers. | INS0 = ekf2 + vehicle_imu (PX4 MAX-14) -> kontrolcülerin ALTI. */
		else if (rel >= -17) { p4_prio = 12; }   /* INS1-3 (PX4 MAX-15..-17) -> BELOW INS0/ekf2 (AWAY from lwIP=13). | INS1-3 (PX4 MAX-15..-17) -> INS0/ekf2 ALTI (lwIP=13'ten UZAK). */
		else if (rel == -18) { p4_prio = 11; }   /* hp_default (PX4 MAX-18) -> BELOW INS. battery_simulator/pwm_out_sim here; ABOVE commander(9) -> battery_status flows at 100Hz. | hp_default (PX4 MAX-18) -> INS ALTI. battery_simulator/pwm_out_sim burada; commander(9) ÜSTÜ -> battery_status 100Hz akar. */
		else if (rel >= -32) { p4_prio =  4; }   /* uavcan/tty (PX4 MAX-19..-32; unused) | uavcan/tty (PX4 MAX-19..-32; kullanılmaz) */
		else                 { p4_prio =  3; }   /* lp_default (PX4 MAX-50, LOWEST WQ) -> land_detector | lp_default (PX4 MAX-50, EN DÜŞÜK WQ) -> land_detector */

		size_t ss = 0;
		pthread_attr_getstacksize(attr, &ss);

		esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
		cfg.stack_size  = (ss < 8192u) ? 8192u : (uint32_t)ss;
		cfg.prio        = p4_prio;
		cfg.pin_to_core = FC_MODULE_TASK_CORE;   /* Core0; 8kHz Core1 untouched | Core0; 8kHz Core1 dokunulmadan */
		cfg.inherit_cfg = true;
		/* ⚠️ psram_stack (receiver): PSRAM-stack -> flash-cache-disable crash-risk (latent-hazard note above). | PSRAM-stack -> flash-cache-devre-dışı çökme-riski (yukarıdaki gizli-tehlike notu). */
		if (psram_stack) {
			/* receiver stack in PSRAM (the internal-RAM 31KB-ceiling is not enough). | receiver stack'i PSRAM'de (dahili-RAM 31KB-tavanı yetmez). */
			cfg.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
		}
		esp_pthread_set_cfg(&cfg);
	}
	return __real_pthread_attr_setschedparam(attr, param);   /* also call the real one (no-op/fail harmless) | gerçek olanı da çağır (no-op/başarısız zararsız) */
}
}
#endif /* FC_WQ_PRIO */

/* ── legacy work_queue API stubs ──
 * The param backend px4_shutdown_lock (shutdown.cpp) uses legacy work_queue to arm a "force-release if shutdown
 * hangs for 60s" watchdog. Once hrt moved to native-esp_timer, legacy work_queue.c was removed ->
 * work_queue/work_cancel/PX4_TICKS_PER_SEC undefined. NO-OP suffices: px4_shutdown_lock/unlock is called
 * BALANCED -> the lock does not hang -> the watchdog is unnecessary. PX4_TICKS_PER_SEC: defines.h:97, the USEC2TICK divisor.
 * ---
 * ── legacy work_queue API stub'ları ──
 * param arka ucu px4_shutdown_lock (shutdown.cpp) "shutdown 60s asılırsa zorla-bırak" bekçisini kurmak için legacy
 * work_queue kullanır. hrt yerel-esp_timer'a taşınınca legacy work_queue.c kaldırıldı ->
 * work_queue/work_cancel/PX4_TICKS_PER_SEC tanımsız. NO-OP yeter: px4_shutdown_lock/unlock DENGELİ çağrılır ->
 * kilit asılmaz -> bekçi gereksiz. PX4_TICKS_PER_SEC: defines.h:97, USEC2TICK böleni. */
#include <px4_platform_common/workqueue.h>
extern "C" {
long PX4_TICKS_PER_SEC = CONFIG_FREERTOS_HZ;   /* FreeRTOS tick-rate; USEC_PER_TICK = 1e6/this | FreeRTOS tick-hızı; USEC_PER_TICK = 1e6/bu */
int work_queue(int qid, struct work_s *work, worker_t worker, void *arg, uint32_t delay)
{
	(void)qid; (void)work; (void)worker; (void)arg; (void)delay;
	/* No-op. Only shutdown.cpp reboot/shutdown-request (+ watchdog) calls in -> rare. Reboot/shutdown
	 * does NOT WORK on P4 via the legacy path (esp_restart risks an accidental reboot during HIL-test -> deliberately not wired).
	 * Visible warning: a QGC reboot command gets ACK-success but the board does NOT reboot; this log surfaces that.
	 * ---
	 * No-op. Sadece shutdown.cpp reboot/shutdown-isteği (+ bekçi) çağırır -> nadir. Reboot/shutdown
	 * P4'te legacy yol üzerinden ÇALIŞMAZ (esp_restart HIL-test sırasında kazara reboot riski taşır -> kasıtlı bağlanmadı).
	 * Görünür uyarı: bir QGC reboot komutu ACK-başarı alır ama kart reboot ETMEZ; bu log bunu ortaya çıkarır. */
	ESP_LOGW("FC_PLATFORM",
	         "work_queue no-op (legacy-WQ; reboot/shutdown NOT on the P4-native path -> board will NOT reboot) qid=%d delay=%lu",
	         qid, (unsigned long)delay);
	return 0;
}
int work_cancel(int qid, struct work_s *work) { (void)qid; (void)work; return 0; }
}
