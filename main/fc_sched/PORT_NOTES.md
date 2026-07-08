# fc_sched/ — P4-native scheduling backend (hrt dispatcher)

**🌐 Language / Dil:** [English](#english) · [Türkçe](#türkçe)

---

<a name="english"></a>
# English

What the native code in this folder is, which imports/includes it uses, and what is kept verbatim.

## Purpose
Make the POSIX-emulation layer of the **work-queue scheduling stack** that drives the PX4 modules
(sensors/ekf2/commander/nav/FMM/...) P4-native. uORB + the modern px4_work_queue (WorkQueue.cpp) +
the drv_hrt callout logic **stay VERBATIM**; only the "fake-ISR" hrt dispatcher is moved to the
native `esp_timer`.

## Files
| File | Role | Dependencies (import) |
|---|---|---|
| `fc_hrt_esp_timer.c` | Native hrt callout dispatcher: `hrt_work_queue_init/queue/cancel` (esp_timer one-shot) | `<esp_timer.h>` (REQUIRES esp_timer), `"hrt_work.h"` (posix; prototypes + transitively `<px4_platform_common/workqueue.h>` → `struct work_s`/`worker_t`), `<px4_platform_common/log.h>` (PX4_ERR) |

Compile flags (SAME as the CMakeLists `FC_PX4_HRT_SRCS` set): `__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME="hrt"`
+ `-I PX4/platforms/posix/include` (hrt_work.h) + force-include `errno.h/stdarg.h/px4_ioctl_compat.h/matrix time.h`.
`__PX4_POSIX` is MANDATORY: the workqueue.h matrix copy defines `struct work_s` only in that branch.

## Preserved API contract (drv_hrt.cpp CALLS these — signatures kept byte-for-byte)
- `void hrt_work_queue_init(void)` — fc_px4_modules_start calls it AFTER `hrt_init()`.
- `int  hrt_work_queue(struct work_s*, worker_t, void *arg, uint32_t delay_us)` — drv_hrt hrt_call_reschedule.
- `void hrt_work_cancel(struct work_s*)` — drv_hrt hrt_call_reschedule.
In the P4 build the ONLY consumer = drv_hrt.cpp, the ONLY `work_s` = `_hrt_work` → a single global esp_timer suffices.

## Native port: what changed
- **ADDED:** `fc_hrt_esp_timer.c` (esp_timer one-shot; NO thread/sem/signal/pid).
- **REMOVED FROM the CMakeLists `FC_PX4_HRT_SRCS`** (so there is no double-definition/duplicate-symbol): the PX4 legacy
  dispatcher `platforms/common/work_queue/{hrt_thread.c, hrt_queue.c, hrt_work_cancel.c}` — these also defined
  `hrt_work_queue_init/queue/cancel`, and the native file defines them too → all 3 must be dropped.
- **UNTOUCHED (verbatim):** `drv_hrt.cpp` (callout_queue sq_* + `_hrt_lock` + `hrt_tim_isr`), the modern
  px4_work_queue, all modules, the 8 kHz Core1.
- **Dangling-ref check:** `g_hrt_work`/`_hrt_work_lock` were used only in the 3 removed files; hrt_work.h's
  `hrt_work_lock/unlock` inlines are `static` → not emitted when unused → NO undefined-ref.

## Orphan cleanup (after the build is verified green)
The 3 removed dispatcher files used `dq_*`; drv_hrt uses only `sq_*` → these helpers are orphaned:
`PX4/platforms/common/work_queue/{dq_addlast,dq_rem,dq_remfirst}.c` (removed with linker verification).
`sq_addafter/sq_addlast/sq_remfirst.c` → KEPT for drv_hrt's sq_*.

## Verification
`idf.py build` (green) + no boot crash + periodic Run real-Hz + 8 kHz jitter no-regression + HIL takeoff/land.

---

<a name="türkçe"></a>
# Türkçe

Bu klasördeki native kodun ne olduğu, hangi import/include yapıldığı ve neyin verbatim korunduğu.

## Amaç
PX4 modüllerini (sensors/ekf2/commander/nav/FMM/...) süren **iş-kuyruğu zamanlama yığınının** POSIX-emülasyon
katmanını P4-native yapmak. uORB + modern px4_work_queue (WorkQueue.cpp) + drv_hrt callout-mantığı
**VERBATIM kalır**; sadece "sahte-ISR" hrt dispatcher'ı native `esp_timer`'a taşınır.

## Dosyalar
| Dosya | Rol | Bağımlılıklar (import) |
|---|---|---|
| `fc_hrt_esp_timer.c` | Native hrt callout dispatcher: `hrt_work_queue_init/queue/cancel` (esp_timer one-shot) | `<esp_timer.h>` (REQUIRES esp_timer), `"hrt_work.h"` (posix; prototipler + transitif `<px4_platform_common/workqueue.h>` → `struct work_s`/`worker_t`), `<px4_platform_common/log.h>` (PX4_ERR) |

Derleme bayrakları (CMakeLists `FC_PX4_HRT_SRCS` seti ile AYNI): `__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME="hrt"`
+ `-I PX4/platforms/posix/include` (hrt_work.h) + force-include `errno.h/stdarg.h/px4_ioctl_compat.h/matrix time.h`.
`__PX4_POSIX` ZORUNLU: workqueue.h matrix-kopyası `struct work_s`'i sadece o dalda tanımlar.

## Korunan API sözleşmesi (drv_hrt.cpp BUNLARI çağırır — imzalar birebir korundu)
- `void hrt_work_queue_init(void)` — fc_px4_modules_start `hrt_init()`'ten SONRA çağırır.
- `int  hrt_work_queue(struct work_s*, worker_t, void *arg, uint32_t delay_us)` — drv_hrt hrt_call_reschedule.
- `void hrt_work_cancel(struct work_s*)` — drv_hrt hrt_call_reschedule.
P4 build'de TEK tüketici = drv_hrt.cpp, TEK `work_s` = `_hrt_work` → tek global esp_timer yeterli.

## Native port: ne değişti
- **EKLENDİ:** `fc_hrt_esp_timer.c` (esp_timer one-shot; thread/sem/sinyal/pid YOK).
- **CMakeLists `FC_PX4_HRT_SRCS`'ten ÇIKARILDI** (çift-tanım/duplicate-symbol olmasın diye): PX4 legacy
  dispatcher `platforms/common/work_queue/{hrt_thread.c, hrt_queue.c, hrt_work_cancel.c}` — bunlar da
  `hrt_work_queue_init/queue/cancel`'i tanımlıyordu, native dosya da tanımlar → 3'ü de çıkmak zorunlu.
- **DOKUNULMADI (verbatim):** `drv_hrt.cpp` (callout_queue sq_* + `_hrt_lock` + `hrt_tim_isr`), modern
  px4_work_queue, tüm modüller, 8kHz Core1.
- **Dangling-ref kontrolü:** `g_hrt_work`/`_hrt_work_lock` yalnız silinen 3 dosyada kullanılırdı; hrt_work.h
  `hrt_work_lock/unlock` inline'ları `static`→kullanılmayınca emit edilmez → undefined-ref YOK.

## Orphan temizlik (build yeşil doğrulandıktan sonra)
Silinen 3 dispatcher dosyası `dq_*` kullanırdı; drv_hrt yalnız `sq_*` kullanır → şu helper'lar orphan:
`PX4/platforms/common/work_queue/{dq_addlast,dq_rem,dq_remfirst}.c` (linker-doğrulamayla çıkarılır).
`sq_addafter/sq_addlast/sq_remfirst.c` → drv_hrt sq_* için KALIR.

## Doğrulama
`idf.py build` (yeşil) + boot crash-yok + periyodik Run gerçek-Hz + 8kHz jitter regresyon-yok + HIL takeoff/land.
