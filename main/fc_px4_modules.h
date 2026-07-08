/**
 * @file fc_px4_modules.h
 * @brief PX4 module startup — spawns commander/navigator/FMM/mc_* as
 *        Core-0 WorkQueue tasks (the active flight path).
 *        ---
 *        PX4 modül başlatma — commander/navigator/FMM/mc_* modüllerini
 *        Core-0 WorkQueue görevleri olarak spawn eder (aktif uçuş yolu).
 *
 * FC_PX4_SHADOW=1: spawn enabled (default). 0: do not spawn.
 * ---
 * FC_PX4_SHADOW=1: spawn açık (varsayılan). 0: spawn etme.
 */
#pragma once

/* FC_PX4_COMMANDER_DRIVE: 1 = spawn real commander '-h' (HIL_STATE_ON -> calibration bypass,
 * Commander.cpp:2876/2883); commander publishes vehicle_status/vehicle_control_mode/
 * actuator_armed/home_position itself (no double-publisher). 0 = no commander spawn.
 * ---
 * FC_PX4_COMMANDER_DRIVE: 1 = gerçek commander'ı '-h' ile spawn et (HIL_STATE_ON -> kalibrasyon
 * atlanır, Commander.cpp:2876/2883); commander vehicle_status/vehicle_control_mode/
 * actuator_armed/home_position'ı kendisi yayınlar (çift-yayıncı yok). 0 = commander spawn edilmez. */
#ifndef FC_PX4_COMMANDER_DRIVE
#define FC_PX4_COMMANDER_DRIVE 1
#endif

/* FC_PX4_DIAG: 1 = verbose command/commander/trajectory diag logs. 0 = flight default (off).
 * Timing-critical: most of these logs sit on the 8kHz Core1 path; ESP_LOG blocks synchronously
 * when the UART TX buffer fills -> 10-25ms jitter. Off in flight -> jitter <1ms.
 * ---
 * FC_PX4_DIAG: 1 = ayrıntılı command/commander/trajectory tanı logları. 0 = uçuş varsayılanı (kapalı).
 * Zaman-kritik: bu logların çoğu 8kHz Core1 yolundadır; UART TX tamponu dolunca ESP_LOG senkron
 * bloklar -> 10-25ms jitter. Uçuşta kapalı -> jitter <1ms. */
#ifndef FC_PX4_DIAG
#define FC_PX4_DIAG 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** uORB Manager (singleton) init = PX4 'uorb start'. Required before any advertise/subscribe.
 * fc_failsafe (Core-1) events::send -> orb_advertise, so this must init before fc_task or it
 * null-deref panics. Called after fc_core_init, before fc_task (Core1) spawn. Idempotent.
 * ---
 * uORB Manager (singleton) init = PX4 'uorb start'. Herhangi bir advertise/subscribe öncesi gerekli.
 * fc_failsafe (Core-1) events::send -> orb_advertise, bu yüzden fc_task'tan önce init edilmeli yoksa
 * null-deref panik olur. fc_core_init sonrası, fc_task (Core1) spawn öncesi çağrılır. İdempotent. */
void fc_uorb_init(void);

/** Start PX4 modules (Core0, low prio). Called after fc_ekf_task_start + fc_logger_start.
 * ---
 * PX4 modüllerini başlat (Core0, düşük öncelik). fc_ekf_task_start + fc_logger_start sonrası çağrılır. */
void fc_px4_modules_start(void);

/** land_detector output (vehicle_land_detected) -> Core1 bridge poll (from adapter task;
 * land_detector runs in the WQ, here we only read -> s_landed).
 * ---
 * land_detector çıktısı (vehicle_land_detected) -> Core1 köprü yoklaması (adaptör görevinden;
 * land_detector WQ'da çalışır, burada sadece okuruz -> s_landed). */
void fc_px4_land_tick(void);
bool fc_px4_land_landed(void);
bool fc_px4_land_maybe_landed(void);
bool fc_px4_land_ground_contact(void);

/** Real arming state (vehicle_status.arming_state==ARMED) for the logger. Core0.
 * ---
 * Logger için gerçek arm-durumu (vehicle_status.arming_state==ARMED). Core0. */
bool fc_px4_armed(void);

/** Bridge real ekf2 output (vehicle_attitude/local_position) to the Core1 seqlock.
 * fc_ekf_task (Core0, IMU-rate) calls this. Returns: was there new ekf2 output.
 * ---
 * Gerçek ekf2 çıktısını (vehicle_attitude/local_position) Core1 seqlock'una köprüle.
 * fc_ekf_task (Core0, IMU-hızında) bunu çağırır. Döndürür: yeni ekf2 çıktısı var mıydı. */
bool fc_px4_ekf2_to_seqlock(void);

/** Bridge real PX4 actuator_motors to the Core1 motor-cmd seqlock. fc_ekf_task (Core0) calls
 * this; Core1 (8kHz) reads it and drives 4x DShot. Returns: was a new value written.
 * ---
 * Gerçek PX4 actuator_motors'u Core1 motor-komut seqlock'una köprüle. fc_ekf_task (Core0) bunu
 * çağırır; Core1 (8kHz) okur ve 4x DShot sürer. Döndürür: yeni bir değer yazıldı mı. */
bool fc_px4_actuator_to_seqlock(void);

#ifdef __cplusplus
}
#endif
