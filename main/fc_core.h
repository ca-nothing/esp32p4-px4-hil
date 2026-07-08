/**
 * @file fc_core.h
 * @brief SMP Pinned Flight Controller — data structures | SMP-sabitlenmiş Uçuş Kontrolcüsü — veri yapıları
 *
 * Core 1: pinned max-priority, cycle-accurate polling loop.
 * Core 0: FreeRTOS (HTTP, Ethernet), reads the shared structs.
 * ---
 * Core 1: sabitlenmiş azami-öncelik, çevrim-hassas yoklama döngüsü.
 * Core 0: FreeRTOS (HTTP, Ethernet), paylaşılan struct'ları okur.
 */

#pragma once

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "hal/rmt_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants | Sabitler
 * ========================================================================== */

#define FC_MAX_MOTORS 4
#define DSHOT600_BITRATE 600000
#define DSHOT_FRAME_BITS 16
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

/* 4 motor DShot GPIOs. No ESC -> outputs to free pins; used to bench Core1 motor-output load.
 * Pins datasheet-safe: clear of firmware use, flash/PSRAM/DSI/CSI/USB pads,
 * eth-RMII (28-35,49-52), hosted-SDIO (14-19,54), and strapping pins.
 * ---
 * 4 motorluk DShot GPIO'ları. ESC yok -> boş pinlere çıkış; Core1 motor-çıkış yükünü ölçmek için.
 * Pinler datasheet-güvenli: firmware kullanımı, flash/PSRAM/DSI/CSI/USB pad'leri,
 * eth-RMII (28-35,49-52), hosted-SDIO (14-19,54) ve strapping pinlerinden uzak. */
#define FC_DSHOT_GPIO_M0 21
#define FC_DSHOT_GPIO_M1 20
#define FC_DSHOT_GPIO_M2 26
#define FC_DSHOT_GPIO_M3 27

/** 8kHz = 125µs = 45,000 cycles @ 360MHz (P4 v1.0 chip) | 8kHz = 125µs = 45.000 çevrim @ 360MHz (P4 v1.0 çip) */
#define FC_PID_LOOP_FREQ_HZ 8000
#define FC_LOOP_CYCLES 45000 /* 125µs at 360MHz (v1.x max) | 360MHz'de 125µs (v1.x azami) */
#define FC_TARGET_PERIOD_US 125

#define FC_TASK_STACK_SIZE (8 * 1024)
#define FC_TASK_PRIORITY 24

/* ==========================================================================
 * DShot
 * ========================================================================== */

typedef struct {
  rmt_channel_handle_t tx_channel;
  rmt_encoder_handle_t copy_encoder;
  rmt_encoder_handle_t dshot_encoder;
  int gpio_num;
  uint16_t throttle_value;
  bool telemetry_request;
  rmt_symbol_word_t dshot_syms[16];   /* persistent per-channel RMT payload (IDF reads payload async after return; FC_DSHOT_STATIC_PAYLOAD) | kalıcı kanal-başına RMT yükü (IDF yükü dönüş sonrası async okur; FC_DSHOT_STATIC_PAYLOAD) */
} fc_dshot_motor_t;

typedef struct {
  fc_dshot_motor_t motors[FC_MAX_MOTORS];
  uint32_t bitrate;
  bool loop_enabled;
  bool dma_enabled;
  bool bitscrambler_enabled;
} fc_dshot_config_t;

/* ==========================================================================
 * Sensor / EKF / PID | Sensör / EKF / PID
 * ========================================================================== */

typedef struct {
  float x, y, z;
} fc_vec3_t;
typedef struct {
  float w, x, y, z;
} fc_quat_t;
typedef struct {
  float roll, pitch, yaw;
} fc_euler_t;

typedef struct {
  fc_quat_t attitude;
  fc_euler_t euler;
  fc_vec3_t gyro_bias, accel_bias;
  float baro_bias;
  bool initialized, healthy;
  uint32_t last_update_us;
  /* Simplified Kalman (FC_EKF_MODE==1) — diagonal covariance, state = [q0..q3, bias_x..z] | Basitleştirilmiş Kalman (FC_EKF_MODE==1) — köşegen kovaryans, durum = [q0..q3, bias_x..z] */
  float P[7];

  /* PX4-style full EKF (FC_EKF_MODE==2) — 24-state, full symmetric covariance.
   * State layout matches PX4 EKF2:
   *   [0..3]   quaternion (q0,q1,q2,q3)
   *   [4..6]   velocity NED (vx,vy,vz)
   *   [7..9]   position NED (px,py,pz)
   *   [10..12] gyro bias (bx,by,bz)
   *   [13..15] accel bias (bx,by,bz)
   *   [16..18] earth magnetic field (mn,me,md)
   *   [19..21] body magnetic field (mx,my,mz)
   *   [22..23] wind velocity NE (wn,we)
   * Same math class as PX4 (full-covariance, Jacobian-based) but not a verbatim port.
   * ---
   * PX4-tarzı tam EKF (FC_EKF_MODE==2) — 24-durum, tam simetrik kovaryans.
   * Durum yerleşimi PX4 EKF2 ile eşleşir:
   *   [0..3]   dördey/quaternion (q0,q1,q2,q3)
   *   [4..6]   hız NED (vx,vy,vz)
   *   [7..9]   konum NED (px,py,pz)
   *   [10..12] jiroskop bias (bx,by,bz)
   *   [13..15] ivmeölçer bias (bx,by,bz)
   *   [16..18] yer manyetik alanı (mn,me,md)
   *   [19..21] gövde manyetik alanı (mx,my,mz)
   *   [22..23] rüzgar hızı NE (wn,we)
   * PX4 ile aynı matematik sınıfı (tam-kovaryans, Jacobian-tabanlı) ama verbatim port değil. */
  float state24[24];
  float P24[24][24];
  /* Innovation-gating reject count (PX4 "innovation_rejected"): ++ when test_ratio > 1.0, state/P not updated that step. | Yenilik-kapılama red sayısı (PX4 "innovation_rejected"): test_ratio > 1.0 olunca ++, o adımda state/P güncellenmez. */
  uint32_t fusion_reject_count;

} fc_ekf_state_t;

/* Estimator select (A/B/C):
 *   0 = Mahony complementary filter (constant-time, no matrix)
 *   1 = 7-state diagonal Kalman (light)
 *   2 = PX4-EKF2-style 24-state full-covariance EKF (real O(n^3) load)
 * ---
 * Kestirici seçimi (A/B/C):
 *   0 = Mahony tümleyen filtre (sabit-zaman, matris yok)
 *   1 = 7-durumlu köşegen Kalman (hafif)
 *   2 = PX4-EKF2-tarzı 24-durumlu tam-kovaryans EKF (gerçek O(n^3) yük) */
#define FC_EKF_MODE_MAHONY        0
#define FC_EKF_MODE_SIMPLE_KALMAN 1
#define FC_EKF_MODE_PX4_FULL      2
#define FC_EKF_MODE FC_EKF_MODE_PX4_FULL

/* FC-internal load generator: 0 = HTTP-driven (default), 1 = autonomous bench-pilot.
 * Kept at 0 for HIL: bench-pilot runs a single ~171s ARM/TAKEOFF/HOVER/LAND window
 * then latches disarmed, so it can't drive repeated arm/disarm cycles.
 * ---
 * FC-içi yük üreteci: 0 = HTTP-güdümlü (varsayılan), 1 = otonom tezgah-pilotu.
 * HIL için 0'da tutulur: tezgah-pilotu tek bir ~171s ARM/TAKEOFF/HOVER/LAND penceresi
 * çalıştırıp sonra disarm'da mandallanır, bu yüzden tekrarlı arm/disarm çevrimi süremez. */
#define FC_BENCH_AUTOPILOT 0

typedef struct {
  float kp, ki, kd;
  float integral, prev_error, integral_max;
} fc_pid_axis_t;

typedef struct {
  fc_pid_axis_t rate_roll, rate_pitch, rate_yaw;
  fc_pid_axis_t angle_roll, angle_pitch;
  uint32_t last_update_us;
} fc_pid_config_t;

typedef struct {
  float motor[FC_MAX_MOTORS];
} fc_mixer_output_t;

typedef struct {
  uint16_t channels[16];
  bool failsafe, signal_valid;
  uint32_t last_packet_us;
} fc_rx_data_t;

/* ==========================================================================
 * FC State | FC Durumu
 * ========================================================================== */

typedef enum {
  FC_STATE_INIT = 0,
  FC_STATE_DISARMED,
  FC_STATE_ARMING,
  FC_STATE_ARMED,
  FC_STATE_FLYING,
  FC_STATE_FAILSAFE,
  FC_STATE_EMERGENCY,
} fc_state_enum_t;

typedef struct {
  fc_state_enum_t state;
  fc_dshot_config_t dshot;
  fc_ekf_state_t ekf;
  fc_pid_config_t pid;
  fc_mixer_output_t mixer_output;
  fc_rx_data_t rx;
  float motor_rpm[FC_MAX_MOTORS];

  fc_euler_t setpoint_attitude;
  float setpoint_throttle;

  /* quaternion attitude setpoint (mc_pos_control output). | dördey/quaternion tutum ayar-noktası (mc_pos_control çıktısı). */
  float setpoint_q_d[4];     /* [w,x,y,z] */
  float setpoint_yawspeed;   /* yaw FF [rad/s] | sapma ileri-besleme [rad/s] */

  bool armed;
  uint32_t loop_count;
  uint32_t cpu_time_us;
  uint32_t jitter_min_us;
  uint32_t jitter_max_us;
  uint32_t jitter_avg_us;

  /* True jitter = (loop_start - ideal_wake) - TARGET_PERIOD_US | Gerçek seğirme = (loop_start - ideal_wake) - TARGET_PERIOD_US */
  uint32_t true_jitter_min_us;
  uint32_t true_jitter_max_us;
  uint32_t true_jitter_avg_us;

  /* Per-section profiling (which section spiked on overrun) | Bölüm-başına profilleme (aşımda hangi bölüm sıçradı) */
  uint32_t spi_time_us;
  uint32_t compute_time_us;
  uint32_t rmt_time_us;
  uint32_t overrun_count;

  /* EKF segment max time (jitter diagnosis) | EKF bölütü azami süre (seğirme teşhisi) */
  uint32_t dbg_ekf_max_us;

} fc_state_t;

/* ==========================================================================
 * Seqlock — race-free data sharing between Core 0/1 | Seqlock — Core 0/1 arası yarış-koşulsuz veri paylaşımı
 * ========================================================================== */

/** Seqlock — Core 1 (writer) → Core 0 (reader) telemetry | Seqlock — Core 1 (yazıcı) → Core 0 (okuyucu) telemetri */
typedef struct {
  uint32_t sequence;
  fc_state_t data;
} fc_seqlock_t;
extern fc_seqlock_t g_fc_seqlock;

/** Seqlock writer (call from Core 1, fc_task_main) | Seqlock yazıcı (Core 1'den çağır, fc_task_main) */
static inline void fc_seqlock_write(const fc_state_t *src) {
  g_fc_seqlock.sequence++;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_seqlock.data = *src;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_seqlock.sequence++;
}

/** Seqlock reader (call from Core 0, HTTP handler) | Seqlock okuyucu (Core 0'dan çağır, HTTP işleyici) */
static inline void fc_seqlock_read(fc_state_t *dst) {
  uint32_t s1, s2;
  do {
    s1 = g_fc_seqlock.sequence;
    __asm__ volatile("fence r, r" ::: "memory");
    *dst = g_fc_seqlock.data;
    __asm__ volatile("fence r, r" ::: "memory");
    s2 = g_fc_seqlock.sequence;
  } while ((s1 & 1) || s1 != s2);
}

/* ==========================================================================
 * EKF output bridge (Core 0 EKF task -> Core 1 8kHz PID) | EKF çıktı köprüsü (Core 0 EKF task -> Core 1 8kHz PID)
 * ========================================================================== */

/** EKF runs on Core 0 at IMU rate, not in the 8kHz Core-1 loop (973us compute
 * doesn't fit the 125us slot). Core 0 fills this; Core 1 reads it each cycle
 * into s_fc_state.ekf (PID uses euler.roll/pitch; vel/pos are telemetry).
 * ---
 * EKF, 8kHz Core-1 döngüsünde değil, Core 0'da IMU hızında çalışır (973us hesap
 * 125us yuvaya sığmaz). Core 0 bunu doldurur; Core 1 her çevrimde bunu
 * s_fc_state.ekf'e okur (PID euler.roll/pitch kullanır; vel/pos telemetridir). */
typedef struct {
  uint64_t time_us;
  float q0, q1, q2, q3;       /* quaternion (w,x,y,z) | dördey/quaternion (w,x,y,z) */
  fc_euler_t euler;           /* deg - PID critical path | derece - PID kritik yol */
  float vel_n, vel_e, vel_d;  /* m/s, NED - telemetry | m/s, NED - telemetri */
  float pos_n, pos_e, pos_d;  /* m, NED - telemetry | m, NED - telemetri */
  uint32_t compute_us;        /* EKF step time (measured on Core 0) | EKF adım süresi (Core 0'da ölçülür) */
  bool healthy;
} fc_ekf_output_t;

/** Seqlock — Core 0 (fc_ekf_task, writer) -> Core 1 (fc_task_main, reader) | Seqlock — Core 0 (fc_ekf_task, yazıcı) -> Core 1 (fc_task_main, okuyucu) */
typedef struct {
  uint32_t sequence;
  fc_ekf_output_t data;
} fc_ekf_output_seqlock_t;
extern fc_ekf_output_seqlock_t g_fc_ekf_output_seqlock;

static inline void fc_ekf_output_seqlock_write(const fc_ekf_output_t *src) {
  g_fc_ekf_output_seqlock.sequence++;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_ekf_output_seqlock.data = *src;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_ekf_output_seqlock.sequence++;
}

/* SAFETY (FC_CORE1_STALE_GUARD): Core1 failsafe if Core0 (fc_ekf_task) dies/stalls.
 * (1) seqlock-read retry cap: if the writer is stuck at an odd sequence (died mid-write),
 *     don't spin the 8kHz loop forever -> return false so the caller uses a safe default.
 * (2) motor-cmd freshness (fc_core.c): time_us > FC_MOTOR_CMD_MAX_AGE_US -> stale -> cut motors (anti-flyaway).
 * Zero effect in normal operation (reads settle in 1-3 iters; mc is fresh every ~4ms).
 * ---
 * GÜVENLİK (FC_CORE1_STALE_GUARD): Core0 (fc_ekf_task) ölür/takılırsa Core1 failsafe.
 * (1) seqlock-okuma yeniden-deneme sınırı: yazıcı tek bir sequence'te takılıysa (yazma-ortasında öldü),
 *     8kHz döngüyü sonsuza dek döndürme -> false döndür ki çağıran güvenli varsayılan kullansın.
 * (2) motor-komut tazeliği (fc_core.c): time_us > FC_MOTOR_CMD_MAX_AGE_US -> bayat -> motorları kes (uçup-gitme-önleme).
 * Normal çalışmada sıfır etki (okumalar 1-3 yinelemede oturur; mc her ~4ms'de taze). */
#define FC_SEQLOCK_MAX_TRIES     16       /* normal contention settles in 1-3 iters; 16 = safe cap for Core0-death detection | normal çekişme 1-3 yinelemede oturur; 16 = Core0-ölümü tespiti için güvenli sınır */
#define FC_MOTOR_CMD_MAX_AGE_US  100000   /* 100ms: fc_ekf_task writes ~250Hz; >100ms = Core0 dead/stall -> cut motors | 100ms: fc_ekf_task ~250Hz yazar; >100ms = Core0 ölü/takılı -> motorları kes */

/* return: true = consistent read; false = writer stuck (Core0 died mid-write) -> caller uses safe default. | dönüş: true = tutarlı okuma; false = yazıcı takılı (Core0 yazma-ortasında öldü) -> çağıran güvenli varsayılan kullanır. */
static inline bool fc_ekf_output_seqlock_read(fc_ekf_output_t *dst) {
  uint32_t s1, s2;
#if FC_CORE1_STALE_GUARD
  int tries = 0;
#endif
  do {
#if FC_CORE1_STALE_GUARD
    if (++tries > FC_SEQLOCK_MAX_TRIES) return false;   /* spin-cap | dönme-sınırı */
#endif
    s1 = g_fc_ekf_output_seqlock.sequence;
    __asm__ volatile("fence r, r" ::: "memory");
    *dst = g_fc_ekf_output_seqlock.data;
    __asm__ volatile("fence r, r" ::: "memory");
    s2 = g_fc_ekf_output_seqlock.sequence;
  } while ((s1 & 1) || s1 != s2);
  return true;
}

/* ==========================================================================
 * motor-command bridge (Core 0 -> Core 1)
 * Core 0 (fc_ekf_task) reads PX4 actuator_motors and writes here; Core 1 (8kHz)
 * reads and drives 4x DShot RMT. Same pattern as the EKF-output seqlock.
 * ---
 * motor-komut köprüsü (Core 0 -> Core 1)
 * Core 0 (fc_ekf_task) PX4 actuator_motors'u okur ve buraya yazar; Core 1 (8kHz)
 * okur ve 4x DShot RMT sürer. EKF-çıktı seqlock'u ile aynı desen.
 * ========================================================================== */
typedef struct {
  uint64_t time_us;
  float motor[FC_MAX_MOTORS];   /* [0,1] normalized (actuator_motors.control; NAN->0 in bridge) | [0,1] normalize (actuator_motors.control; köprüde NAN->0) */
  bool valid;                   /* false = no fresh actuator_motors -> idle | false = taze actuator_motors yok -> boşta */
} fc_motor_cmd_t;

typedef struct {
  uint32_t sequence;
  fc_motor_cmd_t data;
} fc_motor_cmd_seqlock_t;
extern fc_motor_cmd_seqlock_t g_fc_motor_cmd_seqlock;

static inline void fc_motor_cmd_seqlock_write(const fc_motor_cmd_t *src) {
  g_fc_motor_cmd_seqlock.sequence++;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_motor_cmd_seqlock.data = *src;
  __asm__ volatile("fence rw, rw" ::: "memory");
  g_fc_motor_cmd_seqlock.sequence++;
}

/* return: true = consistent read; false = writer stuck (Core0 died mid-write) -> caller must cut motors. | dönüş: true = tutarlı okuma; false = yazıcı takılı (Core0 yazma-ortasında öldü) -> çağıran motorları kesmeli. */
static inline bool fc_motor_cmd_seqlock_read(fc_motor_cmd_t *dst) {
  uint32_t s1, s2;
#if FC_CORE1_STALE_GUARD
  int tries = 0;
#endif
  do {
#if FC_CORE1_STALE_GUARD
    if (++tries > FC_SEQLOCK_MAX_TRIES) return false;   /* spin-cap | dönme-sınırı */
#endif
    s1 = g_fc_motor_cmd_seqlock.sequence;
    __asm__ volatile("fence r, r" ::: "memory");
    *dst = g_fc_motor_cmd_seqlock.data;
    __asm__ volatile("fence r, r" ::: "memory");
    s2 = g_fc_motor_cmd_seqlock.sequence;
  } while ((s1 & 1) || s1 != s2);
  return true;
}

/** SPI mutex — prevents HTTP test endpoint from clashing with the FC loop | SPI mutex — HTTP test uç-noktasının FC döngüsüyle çakışmasını önler */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/** Core 0/1 CPU load % for Web UI | Web arayüzü için Core 0/1 CPU yükü % */
extern volatile uint32_t g_fc_load_pct;

/** SPI handle | SPI tanıtıcısı */
#include "driver/spi_master.h"
extern spi_device_handle_t s_spi_handle;
extern spi_device_handle_t g_http_spi;

/* PX4 custom_mode encoding (commander_state.h / px4_custom_mode.h):
 * data = main_mode<<16 | sub_mode<<24.
 * ---
 * PX4 custom_mode kodlaması (commander_state.h / px4_custom_mode.h):
 * data = main_mode<<16 | sub_mode<<24. */
#define FC_PX4_MAIN_MODE_MANUAL 1
#define FC_PX4_MAIN_MODE_ALTCTL 2
#define FC_PX4_MAIN_MODE_POSCTL 3
#define FC_PX4_MAIN_MODE_AUTO 4
#define FC_PX4_MAIN_MODE_OFFBOARD 6
#define FC_PX4_MAIN_MODE_STABILIZED 7

#define FC_PX4_SUB_MODE_AUTO_READY 1
#define FC_PX4_SUB_MODE_AUTO_TAKEOFF 2
#define FC_PX4_SUB_MODE_AUTO_LOITER 3
#define FC_PX4_SUB_MODE_AUTO_MISSION 4
#define FC_PX4_SUB_MODE_AUTO_RTL 5
#define FC_PX4_SUB_MODE_AUTO_LAND 6

#define FC_PX4_CUSTOM_MODE(main, sub)                                          \
  (((uint32_t)(main) << 16) | ((uint32_t)(sub) << 24))

#define FC_PX4_CUSTOM_MODE_MANUAL FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_MANUAL, 0)
#define FC_PX4_CUSTOM_MODE_STABILIZED                                          \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_STABILIZED, 0)
#define FC_PX4_CUSTOM_MODE_POSCTL FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_POSCTL, 0)
#define FC_PX4_CUSTOM_MODE_AUTO_READY                                          \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_READY)
#define FC_PX4_CUSTOM_MODE_AUTO_TAKEOFF                                        \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_TAKEOFF)
#define FC_PX4_CUSTOM_MODE_AUTO_LOITER                                         \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_LOITER)
#define FC_PX4_CUSTOM_MODE_AUTO_MISSION                                        \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_MISSION)
#define FC_PX4_CUSTOM_MODE_AUTO_RTL                                            \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_RTL)
#define FC_PX4_CUSTOM_MODE_AUTO_LAND                                           \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_AUTO, FC_PX4_SUB_MODE_AUTO_LAND)
#define FC_PX4_CUSTOM_MODE_OFFBOARD                                            \
  FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_OFFBOARD, 0)
#define FC_PX4_CUSTOM_MODE_ALTCTL FC_PX4_CUSTOM_MODE(FC_PX4_MAIN_MODE_ALTCTL, 0)

/* ==========================================================================
 * Functions | Fonksiyonlar
 * ========================================================================== */

uint16_t fc_dshot_encode_frame(uint16_t throttle, bool telemetry);
void fc_dshot_set_throttle(uint8_t motor, uint16_t throttle);
void fc_dshot_stop_all(void);

/* Core 0 EKF task + IMU trigger.
 * fc_ekf_task_start(): creates the EKF task on Core 0 (called by main.c).
 * fc_ekf_notify_new_imu(): RX task (Core 0) calls on new HIGHRES_IMU to wake the EKF task.
 * ---
 * Core 0 EKF task + IMU tetiği.
 * fc_ekf_task_start(): EKF task'ını Core 0'da oluşturur (main.c çağırır).
 * fc_ekf_notify_new_imu(): RX task (Core 0) yeni HIGHRES_IMU'da EKF task'ını uyandırmak için çağırır. */
void fc_ekf_task_start(void);
void fc_ekf_notify_new_imu(void);
void fc_mixer_apply(const fc_mixer_output_t *output);
esp_err_t fc_core_init(void);

/** FC ana task (Core 1, pinned, max priority, polling SPI) | FC ana task (Core 1, sabitlenmiş, azami öncelik, yoklama SPI) */
#ifdef __cplusplus
[[noreturn]] void fc_task_main(void *arg); /* C++ path uses [[noreturn]] (C11 _Noreturn below) | C++ yolu [[noreturn]] kullanır (aşağıda C11 _Noreturn) */
#else
_Noreturn void fc_task_main(void *arg);
#endif

#ifdef __cplusplus
}
#endif
