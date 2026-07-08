/*
 * fc_px4_modules.cpp — spawns the PX4 modules (commander/navigator/FMM/mc_pos/att/rate) as
 * Core-0 WorkQueue tasks and bridges their uORB output to the Core-1 8 kHz loop.
 *
 * commander_main = Commander.cpp:610 (ModuleBase<Commander>::main -> task_spawn ->
 * px4_task_spawn_cmd [fc_px4_tasks.cpp, Core0 + prio-cap] -> instantiate + run()).
 *
 * uORB Manager must start before any advertise/subscribe, else get_device_master()
 * null-derefs. TU scoped __PX4_POSIX (CMakeLists) for the uORBManager.hpp platform header.
 * ---
 * fc_px4_modules.cpp — PX4 modüllerini (commander/navigator/FMM/mc_pos/att/rate) Core-0 WorkQueue
 * görevleri olarak spawn eder ve uORB çıktılarını Core-1 8 kHz döngüsüne köprüler.
 *
 * commander_main = Commander.cpp:610 (ModuleBase<Commander>::main -> task_spawn ->
 * px4_task_spawn_cmd [fc_px4_tasks.cpp, Core0 + öncelik-tavanı] -> örnekle + run()).
 *
 * uORB Manager herhangi bir advertise/subscribe öncesi başlamalı, aksi halde get_device_master()
 * null-deref olur. uORBManager.hpp platform başlığı için TU'ya özel __PX4_POSIX (CMakeLists).
 */
#include "fc_px4_modules.h"

#include <uORB/uORBManager.hpp>
#include "esp_log.h"

/* Real PX4 work-queue: modules run in the WQ.
 * ScheduleNow/ScheduleOnInterval -> WorkQueueManager -> WorkQueueRunner (Core0).
 * ---
 * Gerçek PX4 iş-kuyruğu: modüller WQ'da çalışır.
 * ScheduleNow/ScheduleOnInterval -> WorkQueueManager -> WorkQueueRunner (Core0). */
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
/* land_detector output (vehicle_land_detected) -> Core1 bridge (poll; WQ drives it). */
/* land_detector çıktısı (vehicle_land_detected) -> Core1 köprüsü (yoklama; WQ sürer). */
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_land_detected.h>
/* ekf2 output (vehicle_attitude/local_position) -> Core1 seqlock bridge. */
/* ekf2 çıktısı (vehicle_attitude/local_position) -> Core1 seqlock köprüsü. */
#include "fc_core.h"                              /* fc_ekf_output_t + fc_ekf_output_seqlock_write */
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_motors.h>         /* Core1 DShot bridge (control_allocator output) | Core1 DShot köprüsü (control_allocator çıktısı) */
#include <uORB/topics/actuator_armed.h>          /* SAFETY: MixingOutput-faithful armed gate (disarm/kill/termination -> motor 0) | GÜVENLİK: MixingOutput-sadık arm-kapısı (disarm/kill/termination -> motor 0) */
#include <uORB/topics/vehicle_command.h>         /* early-advertise at boot (verbatim mavlink idiom) | boot'ta erken-advertise (verbatim mavlink deyimi) */
#include <uORB/topics/vehicle_command_ack.h>     /* ARM-DIAG: did commander produce an ACK | ARM-TANI: commander bir ACK üretti mi */
#include <uORB/topics/vehicle_status.h>          /* ARM-DIAG: arming_state/nav_state | ARM-TANI: arming_state/nav_state */
#include <uORB/topics/estimator_status.h>        /* PREARM: hgt/vel/pos_test_ratio + pre_flt_fail_innov_* + timestamp_sample */
#include <uORB/topics/battery_status.h>          /* PREARM: voltage_v/remaining/warning/connected/faults + staleness (batteryCheck 5s threshold) | + bayatlık (batteryCheck 5s eşiği) */
#include <uORB/topics/sensor_gps.h>               /* PREARM: gps fix_type/satellites_used/eph */
#include <drivers/drv_hrt.h>                      /* hrt_absolute_time */
#include <esp_timer.h>                            /* esp_timer_get_time — seqlock time-stamp (Core1 staleness clock) | seqlock zaman-damgası (Core1 bayatlık saati) */
#include "freertos/FreeRTOS.h"                    /* boot-task prio-raise during spawn | spawn sırasında boot-görevi öncelik-yükseltme */
#include "freertos/task.h"                        /* vTaskPrioritySet / uxTaskPriorityGet / xTaskGetCurrentTaskHandle */
#include <matrix/math.hpp>                        /* Eulerf(Quatf) -> euler (deg) */
#include <lib/mathlib/mathlib.h>                  /* math::degrees */

/* Init the P4-native hrt callout dispatcher (fc_sched/fc_hrt_esp_timer.c): one esp_timer
 * (one-shot, 1us). Without it ekf2::Run() hrt-reschedule -> hrt_work_lock -> sem_wait(NULL) ->
 * assert. hrt_work.h posix-include is absent in this TU -> forward-decl.
 * ---
 * P4-yerel hrt callout dağıtıcısını init et (fc_sched/fc_hrt_esp_timer.c): tek esp_timer
 * (tek-atış, 1us). Onsuz ekf2::Run() hrt-yeniden-zamanlama -> hrt_work_lock -> sem_wait(NULL) ->
 * assert. hrt_work.h posix-include'u bu TU'da yok -> ileri-bildirim. */
extern "C" void hrt_work_queue_init(void);

#ifndef FC_PX4_SHADOW
#define FC_PX4_SHADOW 1
#endif


static const char *TAG = "FC_PX4_MOD";

#include <parameters/param.h>   /* param_init/find/get/set/type/control_autosave (C-API, extern "C") */
#include <math.h>               /* lroundf (FLOAT->INT32 param_set) */

/* airframe + HIL param override = PX4 rcS + ROMFS/init.d-posix/airframes/4001_gz_x500 'param set'.
 * Uses real param_find + param_set. param_type -> INT32/FLOAT (param_set wants void*).
 * ---
 * airframe + HIL param geçersiz-kılma = PX4 rcS + ROMFS/init.d-posix/airframes/4001_gz_x500 'param set'.
 * Gerçek param_find + param_set kullanır. param_type -> INT32/FLOAT (param_set void* ister). */
static void fc_param_set_auto(const char *name, float v)
{
	param_t p = param_find(name);
	/* param not in this build's set -> skip (module compiled out) | param bu build kümesinde yok -> atla (modül derleme-dışı) */
	if (p == PARAM_INVALID) { ESP_LOGW(TAG, "airframe-config: '%s' param NOT FOUND (skipped)", name); return; }
	if (param_type(p) == PARAM_TYPE_INT32) { int32_t i = (int32_t)lroundf(v); param_set(p, &i); }
	else                                   { float   f = v;                    param_set(p, &f); }
}

static void fc_apply_airframe_config(void)
{
	/* Docker-verbatim params: gz_x500 / SYS_AUTOSTART 4001. Mirrors every 'param set(-default)'
	 * line of the Docker SITL rcS -> airframe(4001_gz_x500) -> rc.mc_defaults -> px4-rc.simulator
	 * chain. Only deviation = HIL-vs-SITL: P4 sensor input is simulator_mavlink (HIL_SENSOR), so
	 * SIM_GZ_* (Docker gz_bridge, SIM_GZ_EN=1) is skipped; HIL_ACT_FUNC + CAL_*_ID (simulator_mavlink
	 * device_id) kept instead.
	 * ---
	 * Docker-verbatim paramlar: gz_x500 / SYS_AUTOSTART 4001. Docker SITL rcS -> airframe(4001_gz_x500)
	 * -> rc.mc_defaults -> px4-rc.simulator zincirinin her 'param set(-default)' satırını yansıtır.
	 * Tek sapma = HIL-vs-SITL: P4 sensör girişi simulator_mavlink (HIL_SENSOR), bu yüzden
	 * SIM_GZ_* (Docker gz_bridge, SIM_GZ_EN=1) atlanır; onun yerine HIL_ACT_FUNC + CAL_*_ID
	 * (simulator_mavlink device_id) tutulur. */

	/* --- rcS (global, all airframes) --- */
	/* --- rcS (global, tüm airframe'ler) --- */
	fc_param_set_auto("SYS_AUTOSTART", 4001.0f);
	fc_param_set_auto("SYS_AUTOCONFIG", 0.0f);        /* P4: RAM-param, no autoconf-reset (explicit CAL set) | P4: RAM-param, autoconf-reset yok (açık CAL set) */
	fc_param_set_auto("BAT1_N_CELLS", 4.0f);
	fc_param_set_auto("COM_CPU_MAX", -1.0f);          /* CPU-load arming-check off (rcS posix) -> Core0 load won't block arming | CPU-yük arm-kontrolü kapalı (rcS posix) -> Core0 yükü arm'ı bloklamaz */
	fc_param_set_auto("COM_RAM_MAX", -1.0f);          /* RAM-check off (rcS posix) | RAM-kontrolü kapalı (rcS posix) */
	fc_param_set_auto("COM_RC_IN_MODE", 1.0f);        /* RC not required (rcS) -> sole arming gate is GCS | RC gerekmez (rcS) -> tek arm-kapısı GCS */
	fc_param_set_auto("COM_LOW_BAT_ACT", 3.0f);       /* (rcS) */
	fc_param_set_auto("EKF2_REQ_GPS_H", 0.5f);        /* SITL GPS-health speedup (rcS); real-HW 10.0 | SITL GPS-sağlık hızlandırma (rcS); gerçek-HW 10.0 */
	fc_param_set_auto("IMU_GYRO_FFT_EN", 1.0f);       /* (rcS) gyro_fft module absent on P4 -> inert | (rcS) gyro_fft modülü P4'te yok -> etkisiz */
	fc_param_set_auto("MC_AT_EN", 1.0f);              /* (rcS) mc_autotune absent -> inert | (rcS) mc_autotune yok -> etkisiz */
	fc_param_set_auto("SDLOG_MODE", 1.0f);            /* (rcS) */
	fc_param_set_auto("SDLOG_PROFILE", 131.0f);       /* (rcS) */
	fc_param_set_auto("SDLOG_DIRS_MAX", 7.0f);        /* (rcS) */
	fc_param_set_auto("TRIG_INTERFACE", 3.0f);        /* (rcS) */
	fc_param_set_auto("SYS_FAILURE_EN", 1.0f);        /* (rcS) failure-inject module absent -> inert | (rcS) failure-inject modülü yok -> etkisiz */
	fc_param_set_auto("SENS_BOARD_X_OFF", 0.000001f); /* (rcS AUTOCNF) */
	fc_param_set_auto("SENS_DPRES_OFF", 0.001f);      /* (rcS AUTOCNF) */
	fc_param_set_auto("MAV_SYS_ID", 1.0f);            /* (rcS) px4_instance 0 -> 1 */

	/* --- airframe 4001_gz_x500 --- */
	fc_param_set_auto("CA_AIRFRAME", 0.0f);
	fc_param_set_auto("CA_ROTOR_COUNT", 4.0f);
	fc_param_set_auto("CA_ROTOR0_PX", 0.13f);  fc_param_set_auto("CA_ROTOR0_PY", 0.22f);  fc_param_set_auto("CA_ROTOR0_KM", 0.05f);
	fc_param_set_auto("CA_ROTOR1_PX", -0.13f); fc_param_set_auto("CA_ROTOR1_PY", -0.20f); fc_param_set_auto("CA_ROTOR1_KM", 0.05f);
	fc_param_set_auto("CA_ROTOR2_PX", 0.13f);  fc_param_set_auto("CA_ROTOR2_PY", -0.22f); fc_param_set_auto("CA_ROTOR2_KM", -0.05f);
	fc_param_set_auto("CA_ROTOR3_PX", -0.13f); fc_param_set_auto("CA_ROTOR3_PY", 0.20f);  fc_param_set_auto("CA_ROTOR3_KM", -0.05f);
	fc_param_set_auto("MPC_THR_HOVER", 0.60f);        /* Docker airframe value | Docker airframe değeri */
	fc_param_set_auto("NAV_DLL_ACT", 2.0f);

	/* --- rc.mc_defaults --- */
	fc_param_set_auto("MAV_TYPE", 2.0f);
	fc_param_set_auto("IMU_GYRO_RATEMAX", 800.0f);
	fc_param_set_auto("NAV_ACC_RAD", 2.0f);
	fc_param_set_auto("RTL_RETURN_ALT", 30.0f);
	fc_param_set_auto("RTL_DESCEND_ALT", 10.0f);
	fc_param_set_auto("EKF2_RNG_FOG", 1.0f);

	/* --- px4-rc.simulator --- */
	fc_param_set_auto("IMU_INTEG_RATE", 250.0f);      /* Docker = 250 (gz IMU 250Hz) */
	fc_param_set_auto("COM_MODE_ARM_CHK", 1.0f);

	/* --- HIL-specific (replaces SIM_GZ_*; simulator_mavlink HIL device_ids) --- */
	/* --- HIL'e-özel (SIM_GZ_* yerine; simulator_mavlink HIL device_id'leri) --- */
	fc_param_set_auto("HIL_ACT_FUNC1", 101.0f); fc_param_set_auto("HIL_ACT_FUNC2", 102.0f);
	fc_param_set_auto("HIL_ACT_FUNC3", 103.0f); fc_param_set_auto("HIL_ACT_FUNC4", 104.0f);
	fc_param_set_auto("CAL_GYRO0_ID", 4353.0f); fc_param_set_auto("CAL_GYRO0_PRIO", 50.0f);
	fc_param_set_auto("CAL_ACC0_ID", 4609.0f);  fc_param_set_auto("CAL_ACC0_PRIO", 50.0f);
	fc_param_set_auto("CAL_MAG0_ID", 4865.0f);  fc_param_set_auto("CAL_MAG0_PRIO", 50.0f); fc_param_set_auto("CAL_MAG0_ROT", 0.0f);
	fc_param_set_auto("CAL_BARO0_ID", 5121.0f); fc_param_set_auto("CAL_BARO0_PRIO", 50.0f);
}


#if FC_PX4_SHADOW
extern "C" int commander_main(int argc, char *argv[]);
extern "C" int navigator_main(int argc, char *argv[]);
extern "C" int dataman_main(int argc, char *argv[]);   /* mission/geofence/safepoint storage server (RAM -r); navigator DatamanClient DM_GET_ID handshake connects here | mission/geofence/safepoint depolama sunucusu (RAM -r); navigator DatamanClient DM_GET_ID el-sıkışması buraya bağlanır */
extern "C" int flight_mode_manager_main(int argc, char *argv[]);
extern "C" int battery_simulator_main(int argc, char *argv[]);
extern "C" int land_detector_main(int argc, char *argv[]);
extern "C" int simulator_mavlink_main(int argc, char *argv[]); /* HIL: Gazebo HIL_SENSOR/HIL_GPS -> sensor_* */
extern "C" int mavlink_main(int argc, char *argv[]);   /* real PX4 mavlink — QGC/GCS telemetry+command (UDP 14550); receiver publishes vehicle_command | gerçek PX4 mavlink — QGC/GCS telemetri+komut (UDP 14550); receiver vehicle_command yayınlar */
extern "C" int sensors_main(int argc, char *argv[]);   /* sensor_* -> vehicle_imu/mag/air_data */
extern "C" int ekf2_main(int argc, char *argv[]);      /* vehicle_imu -> vehicle_local_position */
extern "C" int mc_pos_control_main(int argc, char *argv[]);  /* trajectory_setpoint -> vehicle_attitude_setpoint */
extern "C" int mc_att_control_main(int argc, char *argv[]);  /* vehicle_attitude_setpoint -> vehicle_rates_setpoint */
extern "C" int mc_rate_control_main(int argc, char *argv[]); /* vehicle_rates_setpoint -> vehicle_torque/thrust_setpoint */
extern "C" int control_allocator_main(int argc, char *argv[]); /* real PX4: torque/thrust_setpoint -> actuator_motors | gerçek PX4: torque/thrust_setpoint -> actuator_motors */
extern "C" int pwm_out_sim_main(int argc, char *argv[]);     /* real PX4 PWMSim: actuator_motors -> actuator_outputs_sim | gerçek PX4 PWMSim: actuator_motors -> actuator_outputs_sim */
/* 4 modules spawned at their Docker SITL rcS positions (declared for link): */
/* Docker SITL rcS konumlarında spawn edilen 4 modül (link için bildirildi): */
extern "C" int rc_update_main(int argc, char *argv[]);
extern "C" int manual_control_main(int argc, char *argv[]);
extern "C" int mc_hover_thrust_estimator_main(int argc, char *argv[]);
extern "C" int mc_autotune_attitude_control_main(int argc, char *argv[]);

static void fc_spawn_module(const char *name, int (*entry)(int, char *[]))
{
	char *argv[] = { (char *)name, (char *)"start", nullptr };
	int r = entry(2, argv);
	ESP_LOGI(TAG, "spawn '%s' -> %d (0=ok)", name, r);
}
#endif

/* uORB Manager (singleton) init — PX4 'uorb start'. Must be called before fc_task (Core1):
 * Core-1 fc_failsafe (PX4 FailsafeBase byte-copy) events::send -> orb_advertise.
 * ---
 * uORB Manager (singleton) init — PX4 'uorb start'. fc_task (Core1) öncesi çağrılmalı:
 * Core-1 fc_failsafe (PX4 FailsafeBase byte-kopya) events::send -> orb_advertise. */
extern "C" void fc_uorb_init(void)
{
	if (uORB::Manager::get_instance() == nullptr) {
		if (!uORB::Manager::initialize()) {
			ESP_LOGE(TAG, "uORB::Manager::initialize() FAILED");
			return;
		}
		ESP_LOGI(TAG, "uORB Manager initialized");
	}
}

/* land_detector output (vehicle_land_detected) -> Core1-readable global.
 * land_detector runs in the WQ; here we only poll (adapter task ~50Hz Core0).
 * ---
 * land_detector çıktısı (vehicle_land_detected) -> Core1-okunabilir global.
 * land_detector WQ'da çalışır; burada sadece yokluyoruz (adaptör görevi ~50Hz Core0). */
static uORB::Subscription s_land_sub{ORB_ID(vehicle_land_detected)};
static volatile bool s_landed = false, s_maybe_landed = false, s_ground_contact = false;

extern "C" void fc_px4_land_tick(void)
{
	vehicle_land_detected_s d;
	if (s_land_sub.update(&d)) {
		s_landed = d.landed;
		s_maybe_landed = d.maybe_landed;
		s_ground_contact = d.ground_contact;
	}
}
extern "C" bool fc_px4_land_landed(void)        { return s_landed; }
extern "C" bool fc_px4_land_maybe_landed(void)  { return s_maybe_landed; }
extern "C" bool fc_px4_land_ground_contact(void){ return s_ground_contact; }

/* Real arming state (PX4 vehicle_status.arming_state) for the logger, replacing its always-0
 * Core1 'armed' flag. Core0 getter; fc_logger (Core0, every 2s) calls it -> sparse, cheap.
 * ---
 * Logger için gerçek arm-durumu (PX4 vehicle_status.arming_state), onun daima-0 olan Core1 'armed'
 * bayrağının yerine. Core0 getter; fc_logger (Core0, her 2s) çağırır -> seyrek, ucuz. */
static uORB::Subscription s_arm_status_sub{ORB_ID(vehicle_status)};
extern "C" bool fc_px4_armed(void)
{
	vehicle_status_s vs{};
	s_arm_status_sub.copy(&vs);
	return vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
}

/* ekf2 output (vehicle_attitude q + vehicle_local_position NED) -> Core1-readable
 * g_fc_ekf_output_seqlock. fc_ekf_task (Core0, IMU-rate) calls this. Data flow:
 * sensor_* -> sensors -> vehicle_imu -> ekf2 (WQ) -> vehicle_local_position/attitude
 * -> read here into the seqlock -> Core1 (8kHz) reads it. euler (deg) from q (matrix Eulerf).
 * ---
 * ekf2 çıktısı (vehicle_attitude q + vehicle_local_position NED) -> Core1-okunabilir
 * g_fc_ekf_output_seqlock. fc_ekf_task (Core0, IMU-hızında) bunu çağırır. Veri akışı:
 * sensor_* -> sensors -> vehicle_imu -> ekf2 (WQ) -> vehicle_local_position/attitude
 * -> burada seqlock'a okunur -> Core1 (8kHz) okur. euler (derece) q'dan (matrix Eulerf). */
static uORB::Subscription s_ekf_att_sub{ORB_ID(vehicle_attitude)};
static uORB::Subscription s_ekf_lpos_sub{ORB_ID(vehicle_local_position)};
static vehicle_attitude_s       s_ekf_att{};
static vehicle_local_position_s s_ekf_lpos{};


extern "C" bool fc_px4_ekf2_to_seqlock(void)
{

	bool got_att = s_ekf_att_sub.update(&s_ekf_att);
	bool got_lp  = s_ekf_lpos_sub.update(&s_ekf_lpos);
	if (!got_att && !got_lp) {
		return false;  /* no new ekf2 output -> don't write seqlock (keep last value) | yeni ekf2 çıktısı yok -> seqlock yazma (son değeri koru) */
	}
	fc_ekf_output_t eo{};
	/* Stamp the seqlock on the SAME clock base as the Core1 consumer = esp_timer. fc_core.c
	 * staleness: age = esp_timer_get_time() - eo.time_us (<500ms => fresh). A different base (e.g.
	 * hrt_absolute_time CLOCK_MONOTONIC) makes age>500ms -> fresh=0 -> ekf.healthy stuck 0 even with
	 * valid lpos -> Core1 flight-control never opens.
	 * ---
	 * seqlock'u Core1 tüketicisiyle AYNI saat tabanında damgala = esp_timer. fc_core.c
	 * bayatlık: age = esp_timer_get_time() - eo.time_us (<500ms => taze). Farklı bir taban (örn.
	 * hrt_absolute_time CLOCK_MONOTONIC) age>500ms yapar -> fresh=0 -> lpos geçerli olsa bile
	 * ekf.healthy 0'da takılır -> Core1 uçuş-kontrolü hiç açılmaz. */
	eo.time_us = (uint64_t)esp_timer_get_time();
	eo.q0 = s_ekf_att.q[0]; eo.q1 = s_ekf_att.q[1]; eo.q2 = s_ekf_att.q[2]; eo.q3 = s_ekf_att.q[3];
	/* euler in degrees (fc_core euler.* expects degrees). | euler derece cinsinden (fc_core euler.* derece bekler). */
	matrix::Eulerf e{matrix::Quatf(s_ekf_att.q[0], s_ekf_att.q[1], s_ekf_att.q[2], s_ekf_att.q[3])};
	eo.euler.roll  = math::degrees(e.phi());
	eo.euler.pitch = math::degrees(e.theta());
	eo.euler.yaw   = math::degrees(e.psi());
	eo.pos_n = s_ekf_lpos.x;  eo.pos_e = s_ekf_lpos.y;  eo.pos_d = s_ekf_lpos.z;
	eo.vel_n = s_ekf_lpos.vx; eo.vel_e = s_ekf_lpos.vy; eo.vel_d = s_ekf_lpos.vz;
	eo.healthy = s_ekf_lpos.xy_valid && s_ekf_lpos.z_valid &&
	             s_ekf_lpos.v_xy_valid && s_ekf_lpos.v_z_valid;
	eo.compute_us = 0;
	fc_ekf_output_seqlock_write(&eo);
	return true;
}

/* Real PX4 actuator_motors (control_allocator output) -> Core1-readable motor-cmd seqlock.
 * fc_ekf_task (Core0) calls this; Core1 (8kHz) reads it and drives 4x DShot. control[0..3] =
 * [0,1] thrust (NAN=disarmed -> 0 idle). No ESC -> DShot to a bare pin; what's measured is the
 * P4 CPU/RMT motor load (jitter/overrun watch).
 * ---
 * Gerçek PX4 actuator_motors (control_allocator çıktısı) -> Core1-okunabilir motor-komut seqlock.
 * fc_ekf_task (Core0) bunu çağırır; Core1 (8kHz) okur ve 4x DShot sürer. control[0..3] =
 * [0,1] itki (NAN=disarm -> 0 rölanti). ESC yok -> DShot çıplak bir pine; ölçülen şey
 * P4 CPU/RMT motor yükü (jitter/overrun gözlemi). */
static uORB::Subscription s_actuator_motors_sub{ORB_ID(actuator_motors)};
static uORB::Subscription s_actuator_armed_sub{ORB_ID(actuator_armed)};   /* SAFETY armed gate | GÜVENLİK arm-kapısı */
static actuator_motors_s   s_actuator_motors{};

extern "C" bool fc_px4_actuator_to_seqlock(void)
{
#if FC_CORE1_MOTOR_SAFEGATE
	/* SAFETY: PX4 routes motor output through MixingOutput, which forces _disarmed_value(motor=0)
	 * on disarm/lockdown/kill/termination (mixer_module.cpp:433,491-505). control_allocator does
	 * NOT zero actuator_motors on disarm, it just stops publishing (ControlAllocator.cpp:709) — so a
	 * raw read that skips this gate would let Core1 hold the last armed throttle forever = post-kill
	 * flyaway. Gate (MixingOutput-faithful): 0 unless armed && !lockdown && !kill && !termination.
	 * copy() keeps armed state current every call; write 0 immediately on disarm/kill without waiting
	 * for new data (staleness + safety).
	 * ---
	 * GÜVENLİK: PX4 motor çıktısını MixingOutput üzerinden yönlendirir; bu, disarm/lockdown/kill/
	 * termination'da _disarmed_value(motor=0)'ı zorlar (mixer_module.cpp:433,491-505). control_allocator
	 * disarm'da actuator_motors'u SIFIRLAMAZ, sadece yayını durdurur (ControlAllocator.cpp:709) — dolayısıyla
	 * bu kapıyı atlayan ham bir okuma, Core1'in son armed gazı sonsuza dek tutmasına izin verir = kill-sonrası
	 * kaçış (flyaway). Kapı (MixingOutput-sadık): armed && !lockdown && !kill && !termination olmadıkça 0.
	 * copy() her çağrıda arm-durumunu güncel tutar; disarm/kill'de yeni veri beklemeden hemen 0 yaz
	 * (bayatlık + güvenlik). */
	{
		actuator_armed_s armed{};
		s_actuator_armed_sub.copy(&armed);
		const bool motors_allowed = armed.armed && !armed.lockdown && !armed.kill && !armed.termination;

		if (!motors_allowed) {
			fc_motor_cmd_t mc{};                       /* all 0 = MixingOutput disarmed_value(motor)=stop | tümü 0 = MixingOutput disarmed_value(motor)=dur */
			mc.time_us = (uint64_t)esp_timer_get_time();
			mc.valid = true;                           /* valid+0 = definite idle (fresh STOP command) | valid+0 = kesin rölanti (taze DUR komutu) */
			fc_motor_cmd_seqlock_write(&mc);
			return true;
		}
	}
#endif
	if (!s_actuator_motors_sub.update(&s_actuator_motors)) {
		return false;   /* armed + no new actuator_motors -> Core1 keeps last (fresh) value | armed + yeni actuator_motors yok -> Core1 son (taze) değeri korur */
	}
	fc_motor_cmd_t mc{};
	mc.time_us = (uint64_t)esp_timer_get_time();
	for (int i = 0; i < FC_MAX_MOTORS; i++) {
		float v = s_actuator_motors.control[i];
		mc.motor[i] = (v == v) ? v : 0.0f;   /* NAN (stopped motor) -> 0 idle | NAN (durmuş motor) -> 0 rölanti */
	}
	mc.valid = true;
	fc_motor_cmd_seqlock_write(&mc);
	return true;
}


extern "C" void fc_px4_modules_start(void)
{
#if FC_PX4_SHADOW
	/* 1) uORB Manager — already called in app_main (idempotent guard). */
	/* 1) uORB Manager — app_main'de zaten çağrıldı (idempotent koruma). */
	fc_uorb_init();
	if (uORB::Manager::get_instance() == nullptr) {
		ESP_LOGE(TAG, "uORB missing -> module spawn ABORTED");
		return;
	}


	/* Real PX4 param backend (parameters.cpp). param_init() builds the store (ConstLayer
	 * firmware-default <- DynamicSparseLayer runtime <- user_config), defaults read lazily from
	 * px4_parameters.hpp. param_control_autosave(false): RAM-only (no flash file) -> silence autosave
	 * retry-log noise. fc_apply_airframe_config(): airframe+HIL 'param set' sequence, before spawn.
	 * ---
	 * Gerçek PX4 param arka-ucu (parameters.cpp). param_init() depoyu kurar (ConstLayer
	 * firmware-varsayılan <- DynamicSparseLayer çalışma-zamanı <- user_config), varsayılanlar
	 * px4_parameters.hpp'den tembel okunur. param_control_autosave(false): sadece-RAM (flash dosyası yok)
	 * -> autosave yeniden-deneme log gürültüsünü sustur. fc_apply_airframe_config(): airframe+HIL
	 * 'param set' dizisi, spawn öncesi. */
	param_init();
	param_control_autosave(false);
	fc_apply_airframe_config();
	ESP_LOGI(TAG, "param backend (real parameters.cpp) init + airframe-config (SYS_AUTOSTART=4001, EKF2_* real defaults)");

	/* Start real PX4 hrt (_hrt_lock semaphore + callout_queue + _hrt_work). drv_hrt.cpp is linked
	 * but hrt_init() wasn't wired -> _hrt_lock NULL -> first ScheduleOnInterval -> hrt_lock ->
	 * px4_sem_wait(NULL) -> assert = boot-loop. Must be called before module spawn + WQ
	 * (ScheduledWorkItem + uORB SubscriptionInterval use hrt_call_*).
	 * ---
	 * Gerçek PX4 hrt'yi başlat (_hrt_lock semaforu + callout_queue + _hrt_work). drv_hrt.cpp link'li
	 * ama hrt_init() bağlanmamıştı -> _hrt_lock NULL -> ilk ScheduleOnInterval -> hrt_lock ->
	 * px4_sem_wait(NULL) -> assert = boot-döngüsü. Modül spawn + WQ öncesi çağrılmalı
	 * (ScheduledWorkItem + uORB SubscriptionInterval hrt_call_* kullanır). */
	hrt_init();

	/* Sibling of hrt_init() — start the hrt callout dispatcher (now native esp_timer,
	 * fc_sched/fc_hrt_esp_timer.c). Without it ScheduleOnInterval/Delayed callouts never fire (no
	 * periodic Run). Must be called before module spawn + WQ.
	 * ---
	 * hrt_init()'in kardeşi — hrt callout dağıtıcısını başlat (artık yerel esp_timer,
	 * fc_sched/fc_hrt_esp_timer.c). Onsuz ScheduleOnInterval/Delayed callout'lar hiç tetiklenmez
	 * (periyodik Run yok). Modül spawn + WQ öncesi çağrılmalı. */
	hrt_work_queue_init();
	ESP_LOGI(TAG, "hrt_work_queue_init() done (native esp_timer hrt dispatcher)");

	/* Start the PX4 work-queue manager (Core0). Modules register via ScheduleOnInterval/ScheduleNow
	 * -> WQ-runner threads drive their Run(). Before module spawn so _wq is ready early. WQ-runner
	 * Core0-pin + prio = px4_task_spawn_cmd sim (fc_px4_tasks.cpp) + sdkconfig pthread-default-core.
	 * ---
	 * PX4 iş-kuyruğu yöneticisini başlat (Core0). Modüller ScheduleOnInterval/ScheduleNow ile kayıt olur
	 * -> WQ-runner iş-parçacıkları Run()'larını sürer. Modül spawn öncesi ki _wq erken hazır olsun. WQ-runner
	 * Core0-sabitleme + öncelik = px4_task_spawn_cmd sim (fc_px4_tasks.cpp) + sdkconfig pthread-varsayılan-çekirdek. */
	{
		int wqr = px4::WorkQueueManagerStart();
		ESP_LOGI(TAG, "WorkQueueManagerStart -> %d (0=ok)", wqr);
	}

	/* This boot-task (main_task, prio~1) spawns modules in sequence. Once mc_pos/att_control start
	 * running they saturate Core0 -> the prio-1 boot-task starves -> remaining spawns
	 * (mc_rate/control_allocator/pwm_out_sim/battery/land_detector) are delayed/never happen ->
	 * control_allocator+pwm_out_sim "not running" -> dead motor path. Fix: raise prio above the
	 * control band (<=19) to 20 for the duration of spawn so all spawns finish, then restore.
	 * esp_timer(22)/fc_ekf_task(21)/Core1(24) unaffected. Boot orchestration only; PX4 untouched.
	 * ---
	 * Bu boot-görevi (main_task, öncelik~1) modülleri sırayla spawn eder. mc_pos/att_control çalışmaya
	 * başladığında Core0'ı doyururlar -> öncelik-1 boot-görevi aç kalır -> kalan spawn'lar
	 * (mc_rate/control_allocator/pwm_out_sim/battery/land_detector) gecikir/hiç olmaz ->
	 * control_allocator+pwm_out_sim "çalışmıyor" -> ölü motor yolu. Düzeltme: spawn süresince önceliği
	 * kontrol bandının (<=19) üstüne, 20'ye çıkar ki tüm spawn'lar bitsin, sonra geri yükle.
	 * esp_timer(22)/fc_ekf_task(21)/Core1(24) etkilenmez. Sadece boot orkestrasyonu; PX4'e dokunulmaz. */
	TaskHandle_t boot_task = xTaskGetCurrentTaskHandle();
	UBaseType_t boot_old_prio = uxTaskPriorityGet(boot_task);
	vTaskPrioritySet(boot_task, 20);
	ESP_LOGI(TAG, "boot-task prio %d->20 (during spawn; prevents module starvation)", (int)boot_old_prio);


	/* Spawn order = PX4 Docker SITL rcS boot order (init.d-posix/rcS + rc.mc_apps + px4-rc.simulator),
	 * with the 4 newly-compiled modules at their Docker positions. Order:
	 * dataman -> simulator_mavlink -> battery_simulator -> rc_update -> manual_control -> sensors ->
	 * commander -> ekf2 -> pwm_out_sim -> control_allocator -> mc_rate_control -> mc_att_control ->
	 * mc_autotune_attitude_control -> mc_hover_thrust_estimator -> flight_mode_manager -> mc_pos_control ->
	 * land_detector -> navigator -> mavlink. Each spawn keeps its HIL args/idiom (-h/-r/-m sim/-u/-o/-t);
	 * the 4 new modules take "start" (fc_spawn_module).
	 * ---
	 * Spawn sırası = PX4 Docker SITL rcS boot sırası (init.d-posix/rcS + rc.mc_apps + px4-rc.simulator),
	 * 4 yeni-derlenen modül Docker konumlarında. Sıra:
	 * dataman -> simulator_mavlink -> battery_simulator -> rc_update -> manual_control -> sensors ->
	 * commander -> ekf2 -> pwm_out_sim -> control_allocator -> mc_rate_control -> mc_att_control ->
	 * mc_autotune_attitude_control -> mc_hover_thrust_estimator -> flight_mode_manager -> mc_pos_control ->
	 * land_detector -> navigator -> mavlink. Her spawn kendi HIL argümanlarını/deyimini korur (-h/-r/-m sim/-u/-o/-t);
	 * 4 yeni modül "start" alır (fc_spawn_module). */

	/* (a) dataman: mission/geofence/safepoint storage SERVER (RAM backend, -r). navigator's
	 * DatamanClient does a DM_GET_ID handshake in its constructor; with no server it hits 4x1000ms
	 * timeout -> "Failed to get client ID" -> navigator 0-cycle. RAM (-r): CONFIG_DATAMAN_PERSISTENT_STORAGE
	 * undefined -> file/FS code compiled out, pure malloc (dataman.cpp:616); no writable FS on P4. PX4
	 * boot order (init.d-posix/rcS:238): dataman before commander(259)+navigator(288). dataman start()
	 * blocks on px4_sem until task_main is up -> when this call returns the server is ready.
	 * ---
	 * (a) dataman: mission/geofence/safepoint depolama SUNUCUSU (RAM arka-uç, -r). navigator'ın
	 * DatamanClient'ı yapıcısında bir DM_GET_ID el-sıkışması yapar; sunucu yoksa 4x1000ms zaman-aşımına
	 * çarpar -> "Failed to get client ID" -> navigator 0-döngü. RAM (-r): CONFIG_DATAMAN_PERSISTENT_STORAGE
	 * tanımsız -> dosya/FS kodu derleme-dışı, saf malloc (dataman.cpp:616); P4'te yazılabilir FS yok. PX4
	 * boot sırası (init.d-posix/rcS:238): dataman, commander(259)+navigator(288) öncesi. dataman start()
	 * task_main ayağa kalkana dek px4_sem'de bloklar -> bu çağrı döndüğünde sunucu hazırdır. */
	{
		char *argv[] = { (char *)"dataman", (char *)"start", (char *)"-r", nullptr };
		int r = dataman_main(3, argv);
		ESP_LOGI(TAG, "spawn 'dataman start -r' (RAM backend) -> %d (0=ok)", r);
	}

	/* (b) HIL sensor source: real PX4 simulator_mavlink. Waits for Gazebo on UDP:14560 -> HIL_SENSOR
	 * -> PX4Accelerometer/Gyroscope/Magnetometer + sensor_baro; HIL_GPS -> sensor_gps. Spawn before
	 * sensors/ekf2 so sensor_* topics are advertised and the sensors module can build its VehicleIMUs
	 * (sensors.cpp:441 advertised-gate). Lockstep off -> real-time HIL (hrt=esp_timer).
	 * ---
	 * (b) HIL sensör kaynağı: gerçek PX4 simulator_mavlink. UDP:14560'ta Gazebo'yu bekler -> HIL_SENSOR
	 * -> PX4Accelerometer/Gyroscope/Magnetometer + sensor_baro; HIL_GPS -> sensor_gps. sensors/ekf2 öncesi
	 * spawn ki sensor_* topic'leri advertise edilsin ve sensors modülü VehicleIMU'larını kurabilsin
	 * (sensors.cpp:441 advertised-kapısı). Lockstep kapalı -> gerçek-zamanlı HIL (hrt=esp_timer). */
	{
		char *argv[] = { (char *)"simulator_mavlink", (char *)"start", (char *)"-u", (char *)"14560", nullptr };
		int r = simulator_mavlink_main(4, argv);
		ESP_LOGI(TAG, "spawn 'simulator_mavlink start -u 14560' -> %d (0=ok)", r);
	}

	/* (c) battery_simulator (ScheduledWorkItem; publishes battery_status). */
	/* (c) battery_simulator (ScheduledWorkItem; battery_status yayınlar). */
	fc_spawn_module("battery_simulator", battery_simulator_main);

	/* (d) rc_update: manual_control_setpoint/RC -> rc_channels/manual_control_switches.
	 * rc.mc_apps: rc_update before manual_control.
	 * ---
	 * (d) rc_update: manual_control_setpoint/RC -> rc_channels/manual_control_switches.
	 * rc.mc_apps: rc_update, manual_control öncesi. */
	fc_spawn_module("rc_update", rc_update_main);

	/* (e) manual_control: input_rc/manual_control_input -> manual_control_setpoint +
	 * mode-switch -> vehicle_command. rc.mc_apps: after rc_update, before sensors.
	 * ---
	 * (e) manual_control: input_rc/manual_control_input -> manual_control_setpoint +
	 * mod-anahtarı -> vehicle_command. rc.mc_apps: rc_update sonrası, sensors öncesi. */
	fc_spawn_module("manual_control", manual_control_main);

	/* (f) estimation pipeline (PX4 boot order: sensors -> ekf2). sensor_gyro/accel/mag/baro/gps (from
	 * simulator_mavlink HIL) -> sensors -> vehicle_imu/magnetometer/air_data -> ekf2 ->
	 * vehicle_local_position/attitude. WQ drives it; fc_ekf_task bridges output to the seqlock.
	 * sensors '-h' = HIL mode: sensors.cpp:621 hil_enabled=true -> the 500ms-tolerant voter timeout
	 * (voted_sensors_update.cpp:60-63); without it the strict default timeout fires "Accel #0 fail:
	 * TIMEOUT" (:308) on a jittery HIL feed.
	 * ---
	 * (f) kestirim boru-hattı (PX4 boot sırası: sensors -> ekf2). sensor_gyro/accel/mag/baro/gps
	 * (simulator_mavlink HIL'den) -> sensors -> vehicle_imu/magnetometer/air_data -> ekf2 ->
	 * vehicle_local_position/attitude. WQ sürer; fc_ekf_task çıktıyı seqlock'a köprüler.
	 * sensors '-h' = HIL modu: sensors.cpp:621 hil_enabled=true -> 500ms-toleranslı oylayıcı zaman-aşımı
	 * (voted_sensors_update.cpp:60-63); onsuz katı varsayılan zaman-aşımı jitterli HIL beslemesinde
	 * "Accel #0 fail: TIMEOUT" (:308) tetikler. */
	{
		char *argv[] = { (char *)"sensors", (char *)"start", (char *)"-h", nullptr };
		int r = sensors_main(3, argv);
		ESP_LOGI(TAG, "spawn 'sensors start -h' -> %d (0=ok)", r);
	}

	/* 3) Spawn the PX4 flight-decision chain (Core0, low prio; PX4 boot order).
	 * FC_PX4_COMMANDER_DRIVE=1 (default): the real PX4 commander publishes
	 * vehicle_status / vehicle_control_mode / actuator_armed / home_position itself
	 * (authoritative publisher); the Core1 8 kHz arming/mode gate reads those.
	 * ---
	 * 3) PX4 uçuş-karar zincirini spawn et (Core0, düşük öncelik; PX4 boot sırası).
	 * FC_PX4_COMMANDER_DRIVE=1 (varsayılan): gerçek PX4 commander
	 * vehicle_status / vehicle_control_mode / actuator_armed / home_position'ı kendisi yayınlar
	 * (yetkili yayıncı); Core1 8 kHz arm/mod kapısı bunları okur. */
#if FC_PX4_COMMANDER_DRIVE
	/* (g) commander '-h' (HIL_STATE_ON -> gyro/accel/mag calibration bypass, Commander.cpp:2876/2883).
	 * Publishes vehicle_status/control_mode/actuator_armed/home itself. navigator+FMM spawn below.
	 * ---
	 * (g) commander '-h' (HIL_STATE_ON -> gyro/accel/mag kalibrasyonu atlanır, Commander.cpp:2876/2883).
	 * vehicle_status/control_mode/actuator_armed/home'u kendisi yayınlar. navigator+FMM aşağıda spawn edilir. */
	ESP_LOGI(TAG, "PX4 zinciri spawn: commander(-h)");
	{
		char *argv[] = { (char *)"commander", (char *)"start", (char *)"-h", nullptr };
		int r = commander_main(3, argv);
		ESP_LOGI(TAG, "spawn 'commander -h' -> %d (0=ok)", r);
	}
#else
	ESP_LOGI(TAG, "commander=adapter-feed (FC_PX4_COMMANDER_DRIVE=0; no spawn)");
	(void)commander_main;
#endif

	/* (h) ekf2 (vehicle_imu -> vehicle_local_position/attitude). WQ drives it; fc_ekf_task bridges to seqlock. */
	/* (h) ekf2 (vehicle_imu -> vehicle_local_position/attitude). WQ sürer; fc_ekf_task seqlock'a köprüler. */
	fc_spawn_module("ekf2", ekf2_main);

	/* (i) pwm_out_sim (real PX4 PWMSim) — last link of the verbatim output chain: converts
	 * control_allocator's actuator_motors to actuator_outputs_sim via MixingOutput (FunctionMotors ->
	 * ORB_ID(actuator_motors)). simulator_mavlink::send() (SimulatorMavlink.cpp:1280
	 * orb_subscribe_multi(actuator_outputs_sim,0)) reads that and sends HIL_ACTUATOR_CONTROLS (#93) to
	 * Gazebo = full-verbatim motor path. SITL rcS pattern "pwm_out_sim start -m sim" (respects lockdown;
	 * -m hil = ignoreLockdown if needed). PWM_MAIN_FUNC1..4=Motor1..4 (101..104) maps actuator_outputs
	 * to motors (without it no function assignment -> no output, no crash).
	 * ---
	 * (i) pwm_out_sim (gerçek PX4 PWMSim) — verbatim çıktı zincirinin son halkası: control_allocator'ın
	 * actuator_motors'unu MixingOutput ile actuator_outputs_sim'e çevirir (FunctionMotors ->
	 * ORB_ID(actuator_motors)). simulator_mavlink::send() (SimulatorMavlink.cpp:1280
	 * orb_subscribe_multi(actuator_outputs_sim,0)) bunu okur ve Gazebo'ya HIL_ACTUATOR_CONTROLS (#93)
	 * gönderir = tam-verbatim motor yolu. SITL rcS kalıbı "pwm_out_sim start -m sim" (lockdown'a uyar;
	 * gerekirse -m hil = lockdown-yoksay). PWM_MAIN_FUNC1..4=Motor1..4 (101..104) actuator_outputs'u
	 * motorlara eşler (onsuz fonksiyon-ataması yok -> çıktı yok, çökme yok). */
	{
		char *argv[] = { (char *)"pwm_out_sim", (char *)"start", (char *)"-m", (char *)"sim", nullptr };
		int r = pwm_out_sim_main(4, argv);
		ESP_LOGI(TAG, "spawn 'pwm_out_sim start -m sim' -> %d (0=ok)", r);
	}

	/* Real PX4 control chain. rc.mc_apps order: control_allocator -> mc_rate_control ->
	 * mc_att_control -> mc_autotune_attitude_control -> mc_hover_thrust_estimator.
	 * ---
	 * Gerçek PX4 kontrol zinciri. rc.mc_apps sırası: control_allocator -> mc_rate_control ->
	 * mc_att_control -> mc_autotune_attitude_control -> mc_hover_thrust_estimator. */
	/* (j) control_allocator: torque/thrust_setpoint -> actuator_motors. */
	fc_spawn_module("control_allocator", control_allocator_main);
	/* (k) mc_rate_control: vehicle_rates_setpoint -> vehicle_torque/thrust_setpoint. */
	fc_spawn_module("mc_rate_control", mc_rate_control_main);
	/* (l) mc_att_control: vehicle_attitude_setpoint -> vehicle_rates_setpoint. */
	fc_spawn_module("mc_att_control", mc_att_control_main);
	/* (m) mc_autotune_attitude_control: PID-autotune (MC_AT_EN). rc.mc_apps: after mc_att_control. | (m) mc_autotune_attitude_control: PID-otoayar (MC_AT_EN). rc.mc_apps: mc_att_control sonrası. */
	fc_spawn_module("mc_autotune_attitude_control", mc_autotune_attitude_control_main);
	/* (n) mc_hover_thrust_estimator: hover-thrust (MPC_THR_HOVER) estimation. rc.mc_apps. | (n) mc_hover_thrust_estimator: hover-itki (MPC_THR_HOVER) kestirimi. rc.mc_apps. */
	fc_spawn_module("mc_hover_thrust_estimator", mc_hover_thrust_estimator_main);

	/* (o) flight_mode_manager: vehicle_command/mode -> trajectory_setpoint (FlightTask). rc.mc_apps:
	 * after mc_hover_thrust_estimator, before mc_pos_control.
	 * ---
	 * (o) flight_mode_manager: vehicle_command/mode -> trajectory_setpoint (FlightTask). rc.mc_apps:
	 * mc_hover_thrust_estimator sonrası, mc_pos_control öncesi. */
	fc_spawn_module("flight_mode_manager", flight_mode_manager_main);

	/* (p) mc_pos_control (PX4 MulticopterPositionControl): FMM trajectory_setpoint + ekf2
	 * vehicle_local_position -> vehicle_attitude_setpoint (q_d + thrust).
	 * ---
	 * (p) mc_pos_control (PX4 MulticopterPositionControl): FMM trajectory_setpoint + ekf2
	 * vehicle_local_position -> vehicle_attitude_setpoint (q_d + itki). */
	fc_spawn_module("mc_pos_control", mc_pos_control_main);

	/* (q) land_detector (publishes vehicle_land_detected; Core1 reads it). task_spawn
	 * (land_detector_main.cpp:60) expects argv[1] = vehicle type (multicopter/fixedwing/...); after
	 * "start" is stripped the mode arg must remain -> pass "start multicopter". Otherwise the module
	 * won't start -> no vehicle_land_detected -> commander never sees "Takeoff detected" ->
	 * COM_DISARM_PRFLT auto-disarm -> motor cut at takeoff.
	 * ---
	 * (q) land_detector (vehicle_land_detected yayınlar; Core1 okur). task_spawn
	 * (land_detector_main.cpp:60) argv[1] = araç tipi bekler (multicopter/fixedwing/...); "start"
	 * ayıklandıktan sonra mod argümanı kalmalı -> "start multicopter" geç. Aksi halde modül
	 * başlamaz -> vehicle_land_detected yok -> commander "Takeoff detected"ı hiç görmez ->
	 * COM_DISARM_PRFLT oto-disarm -> kalkışta motor kesilir. */
	{
		char *argv[] = { (char *)"land_detector", (char *)"start", (char *)"multicopter", nullptr };
		int r = land_detector_main(3, argv);
		ESP_LOGI(TAG, "spawn 'land_detector start multicopter' -> %d (0=ok)", r);
	}

	/* (r) navigator: mission/RTL/loiter -> position_setpoint_triplet (DatamanClient -> dataman). */
	fc_spawn_module("navigator", navigator_main);

	/* (s) real PX4 mavlink — QGC/GCS telemetry+command. mavlink_receiver advertises vehicle_command
	 * and publishes QGC commands (ARM/TAKEOFF/mode) to uORB. -x = FTP (param/mission/log file transfer).
	 * No broadcast (-p not given) -> find_broadcast_address (no lwIP SIOCGIFCONF/SIOCGIFNETMASK) not
	 * called. No port clash with simulator_mavlink (14560 HIL). Spawned via px4_task_spawn_cmd (receiver
	 * pthread SCHED_PRIORITY_MAX-80).
	 * ---
	 * (s) gerçek PX4 mavlink — QGC/GCS telemetri+komut. mavlink_receiver vehicle_command'ı advertise eder
	 * ve QGC komutlarını (ARM/TAKEOFF/mod) uORB'a yayınlar. -x = FTP (param/mission/log dosya transferi).
	 * Broadcast yok (-p verilmedi) -> find_broadcast_address (lwIP SIOCGIFCONF/SIOCGIFNETMASK yok)
	 * çağrılmaz. simulator_mavlink (14560 HIL) ile port çakışması yok. px4_task_spawn_cmd ile spawn edilir
	 * (receiver pthread SCHED_PRIORITY_MAX-80). */
	{
		/* -t <GCS-IP> -o 14550: locks the SENDER to the GCS target -> _src_addr_initialized=true
		 * (mavlink_main.cpp:2135), sends on the first packet (doesn't wait for receiver source-learning;
		 * mavlink_receiver.cpp:3702 bypass). No broadcast (-p absent). -x = FTP.
		 * ---
		 * -t <GCS-IP> -o 14550: GÖNDERİCİYİ GCS hedefine kilitler -> _src_addr_initialized=true
		 * (mavlink_main.cpp:2135), ilk pakette gönderir (receiver kaynak-öğrenmesini beklemez;
		 * mavlink_receiver.cpp:3702 baypas). Broadcast yok (-p yok). -x = FTP. */
		char *argv[] = { (char *)"mavlink", (char *)"start", (char *)"-u", (char *)"14550",
		                 (char *)"-o", (char *)"14550", (char *)"-t", (char *)"172.16.16.105", (char *)"-x", nullptr };
		int r = mavlink_main(9, argv);
		ESP_LOGI(TAG, "spawn 'mavlink start -u 14550 -o 14550 -t 172.16.16.105 -x' -> %d (0=ok)", r);
	}


	/* all module-spawn done -> restore boot-task to its old (low) prio, yield Core0 to the modules. */
	/* tüm modül-spawn bitti -> boot-görevini eski (düşük) önceliğine geri yükle, Core0'ı modüllere bırak. */
	vTaskPrioritySet(boot_task, boot_old_prio);
	ESP_LOGI(TAG, "all module spawns DONE; boot-task prio 20->%d restored", (int)boot_old_prio);

	/* mavlink boot_complete last (mavlink_main.cpp:3489 set_boot_complete -> param-transmission
	 * unblock; without it the QGC param list may not fully download). Only param waits on this.
	 * ---
	 * mavlink boot_complete en son (mavlink_main.cpp:3489 set_boot_complete -> param-iletimi
	 * kilidini açar; onsuz QGC param listesi tam inmeyebilir). Bunu sadece param bekler. */
	{
		char *argv[] = { (char *)"mavlink", (char *)"boot_complete", nullptr };
		mavlink_main(2, argv);
		ESP_LOGI(TAG, "mavlink boot_complete (param-transmission unblock)");
	}
#else
	ESP_LOGI(TAG, "FC_PX4_SHADOW=0 -> PX4 modules NOT STARTED (no spawn)");
#endif
}
