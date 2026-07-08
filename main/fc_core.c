/**
 * @file fc_core.c
 * @brief SMP Pinned Flight Controller — Core 1, cycle-accurate loop | SMP-sabitlenmiş Uçuş Kontrolcüsü — Core 1, çevrim-hassas döngü
 *
 * Core 1 polling loop, no interrupt disable (would SMP-deadlock).
 * Timing: esp_cpu_get_cycle_count(), 45,000 cycles (125µs @360MHz).
 * Core 0: FreeRTOS + HTTP/Ethernet.
 * ---
 * Core 1 yoklama döngüsü, kesme-kapatma yok (SMP-kilitlenme olurdu).
 * Zamanlama: esp_cpu_get_cycle_count(), 45.000 çevrim (125µs @360MHz).
 * Core 0: FreeRTOS + HTTP/Ethernet.
 */

#include "fc_core.h"
#include "fc_px4_modules.h"  /* ekf2 output -> Core1 seqlock (fc_px4_ekf2_to_seqlock) | ekf2 çıktısı -> Core1 seqlock (fc_px4_ekf2_to_seqlock) */
/* fc_core = only 8kHz Core1 orchestration; control/EKF/commander/navigator/mission +
 * sensor (simulator_mavlink)/motor (control_allocator) come from real PX4 modules (Core0 WQ).
 * ---
 * fc_core = yalnızca 8kHz Core1 orkestrasyonu; kontrol/EKF/commander/navigator/mission +
 * sensör (simulator_mavlink)/motor (control_allocator) gerçek PX4 modüllerinden gelir (Core0 WQ). */
#include "driver/rmt_tx.h"
#include "driver/spi_master.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/cpu_hal.h"

static const char *TAG = "FC_CORE";

/* ==========================================================================
 * Global State — Seqlock + Atomic + Mutex | Global Durum — Seqlock + Atomik + Mutex
 * ========================================================================== */

volatile uint32_t g_fc_load_pct = 0;

/* Seqlock: Core 0 (HTTP) reads, Core 1 (FC) writes | Seqlock: Core 0 (HTTP) okur, Core 1 (FC) yazar */
fc_seqlock_t g_fc_seqlock = {0};


/* Seqlock: Core 0 (fc_ekf_task) writes, Core 1 (FC) reads | Seqlock: Core 0 (fc_ekf_task) yazar, Core 1 (FC) okur */
fc_ekf_output_seqlock_t g_fc_ekf_output_seqlock = {0};

/* motor-cmd seqlock (Core 0 fc_ekf_task writes, Core 1 8kHz reads). | motor-komut seqlock (Core 0 fc_ekf_task yazar, Core 1 8kHz okur). */
fc_motor_cmd_seqlock_t g_fc_motor_cmd_seqlock = {0};


/* HTTP SPI handle — separate device (arbitrated via bus lock) | HTTP SPI tanıtıcısı — ayrı cihaz (bus kilidiyle hakemlenir) */
spi_device_handle_t g_http_spi = NULL;


/* FC task's own local state (written to the seqlock) | FC task'ının kendi yerel durumu (seqlock'a yazılır) */
static fc_state_t s_fc_state = {
    .state = FC_STATE_INIT,
    .armed = false,
    .loop_count = 0,
    .cpu_time_us = 0,
    .pid =
        {
            .rate_roll =
                {.kp = 0.1f, .ki = 0.01f, .kd = 0.001f, .integral_max = 0.3f},
            .rate_pitch =
                {.kp = 0.1f, .ki = 0.01f, .kd = 0.001f, .integral_max = 0.3f},
            .rate_yaw =
                {.kp = 0.15f, .ki = 0.01f, .kd = 0.0f, .integral_max = 0.05f},
            .angle_roll =
                {.kp = 5.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.3f},
            .angle_pitch =
                {.kp = 5.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.3f},
        },
    .setpoint_attitude = {.roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f},
    .setpoint_throttle = 0.0f,
};

/* ==========================================================================
 * DShot — RMT TX (called directly from Core 1) | DShot — RMT TX (doğrudan Core 1'den çağrılır)
 * ========================================================================== */

#define DSHOT_T1H 50 // 1250ns @40MHz | 1250ns @40MHz
#define DSHOT_T1L 17 //  417ns | 417ns
#define DSHOT_T0H 25 //  625ns | 625ns
#define DSHOT_T0L 42 // 1042ns | 1042ns

uint16_t fc_dshot_encode_frame(uint16_t throttle, bool telemetry) {
  if (throttle > DSHOT_THROTTLE_MAX)
    throttle = DSHOT_THROTTLE_MAX;
  if (throttle < DSHOT_THROTTLE_MIN)
    throttle = DSHOT_THROTTLE_MIN;
  uint16_t frame = (throttle << 5) | ((telemetry ? 1 : 0) << 4);
  uint16_t crc = 0;
  uint16_t data = frame >> 4;
  for (int i = 0; i < 12; i++) {
    crc <<= 1;
    if ((crc & 0x10) || (data & 0x800))
      crc ^= 0x0D;
    data <<= 1;
  }
  frame |= (crc & 0x0F);
  return frame;
}

void IRAM_ATTR fc_dshot_set_throttle(uint8_t motor, uint16_t throttle) {
  if (motor >= FC_MAX_MOTORS)
    return;
  fc_dshot_motor_t *m = &s_fc_state.dshot.motors[motor];
  if (m->tx_channel == NULL || m->copy_encoder == NULL)
    return;
  if (throttle > DSHOT_THROTTLE_MAX)
    throttle = DSHOT_THROTTLE_MAX;
  if (throttle < DSHOT_THROTTLE_MIN)
    throttle = DSHOT_THROTTLE_MIN;
  m->throttle_value = throttle;

  uint16_t frame = fc_dshot_encode_frame(throttle, m->telemetry_request);
#if FC_DSHOT_STATIC_PAYLOAD
  /* Persistent per-channel buffer, not stack: IDF rmt_transmit reads the payload async
   * (in ISR after this returns) and requires internal RAM. A stack array only worked by
   * timing luck (DShot600@8kHz idle channel); persistent .bss buffer satisfies the contract.
   * ---
   * Kalıcı kanal-başına tampon, yığın değil: IDF rmt_transmit yükü async okur
   * (bu dönüş sonrası ISR'da) ve dahili RAM ister. Yığın dizisi yalnızca
   * zamanlama şansıyla çalışıyordu (DShot600@8kHz boşta kanal); kalıcı .bss tamponu kontratı sağlar. */
  rmt_symbol_word_t *syms = m->dshot_syms;
#else
  rmt_symbol_word_t syms[16];   /* old: stack (IDF async-payload contract risk) | eski: yığın (IDF async-yük kontrat riski) */
#endif
  for (int i = 0; i < 16; i++) {
    bool bit = (frame >> (15 - i)) & 1;
    if (bit) {
      syms[i].duration0 = DSHOT_T1H;
      syms[i].level0 = 1;
      syms[i].duration1 = DSHOT_T1L;
      syms[i].level1 = 0;
    } else {
      syms[i].duration0 = DSHOT_T0H;
      syms[i].level0 = 1;
      syms[i].duration1 = DSHOT_T0L;
      syms[i].level1 = 0;
    }
  }
  rmt_transmit_config_t tx_cfg = {.loop_count = 0};
  /* syms may now be a pointer -> sizeof(syms) would be wrong; pass explicit 16-sym size. | syms artık işaretçi olabilir -> sizeof(syms) yanlış olurdu; açık 16-sembol boyutu geç. */
  rmt_transmit(m->tx_channel, m->copy_encoder, syms, 16 * sizeof(rmt_symbol_word_t), &tx_cfg);
}

void IRAM_ATTR fc_dshot_stop_all(void) {
  for (int i = 0; i < FC_MAX_MOTORS; i++)
    fc_dshot_set_throttle(i, 0);
}

/* Set up one DShot RMT channel (idx = motor, gpio = output pin). P4 RMT = 4 TX channels;
 * mem_block_symbols=48 (1 block/channel) fits 4 motors in 4 blocks (64 = 2 blocks/channel
 * would need 8 -> ch2/3 alloc-fail). DShot600 = 16 sym << 48.
 * ---
 * Tek bir DShot RMT kanalı kur (idx = motor, gpio = çıkış pini). P4 RMT = 4 TX kanalı;
 * mem_block_symbols=48 (1 blok/kanal) 4 motoru 4 bloğa sığdırır (64 = 2 blok/kanal
 * 8 gerektirir -> ch2/3 tahsis-hatası). DShot600 = 16 sembol << 48. */
static esp_err_t fc_dshot_init_channel(int idx, int gpio) {
  fc_dshot_motor_t *m = &s_fc_state.dshot.motors[idx];
  m->gpio_num = gpio;
  m->throttle_value = DSHOT_THROTTLE_MIN;
  m->telemetry_request = false;

  rmt_tx_channel_config_t tx_cfg = {
      .gpio_num = gpio,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 40 * 1000 * 1000,
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,
  };
  esp_err_t ret = rmt_new_tx_channel(&tx_cfg, &m->tx_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX ch%d (GPIO%d) fail: %s", idx, gpio, esp_err_to_name(ret));
    return ret;
  }
  rmt_copy_encoder_config_t copy_cfg = {};
  ret = rmt_new_copy_encoder(&copy_cfg, &m->copy_encoder);
  if (ret != ESP_OK)
    return ret;
  ret = rmt_enable(m->tx_channel);
  if (ret != ESP_OK)
    return ret;

  fc_dshot_set_throttle(idx, DSHOT_THROTTLE_MIN);
  return ESP_OK;
}

static esp_err_t fc_dshot_init(void) {
  ESP_LOGI(TAG, "DShot init (RMT TX)...");
  s_fc_state.dshot.bitrate = DSHOT600_BITRATE;

  /* Mark all absent first (uninit channels are skipped in fc_dshot_set_throttle via tx_channel==NULL). | Önce tümünü yok işaretle (init-edilmemiş kanallar fc_dshot_set_throttle'da tx_channel==NULL ile atlanır). */
  for (int i = 0; i < FC_MAX_MOTORS; i++) {
    s_fc_state.dshot.motors[i].gpio_num = -1;
    s_fc_state.dshot.motors[i].throttle_value = DSHOT_THROTTLE_MIN;
    s_fc_state.dshot.motors[i].tx_channel = NULL;
    s_fc_state.dshot.motors[i].copy_encoder = NULL;
  }

#if FC_CORE1_MOTOR_OUT
  /* 4 DShot channels (Core1 motor-output load). Don't abort on channel fail (log + continue)
   * so one bad pin doesn't kill boot; that channel stays tx_channel=NULL and is skipped.
   * ---
   * 4 DShot kanalı (Core1 motor-çıkış yükü). Kanal hatasında iptal etme (logla + devam et)
   * ki tek bozuk pin boot'u öldürmesin; o kanal tx_channel=NULL kalır ve atlanır. */
  const int gpios[FC_MAX_MOTORS] = {FC_DSHOT_GPIO_M0, FC_DSHOT_GPIO_M1, FC_DSHOT_GPIO_M2, FC_DSHOT_GPIO_M3};
  for (int i = 0; i < FC_MAX_MOTORS; i++) {
    if (fc_dshot_init_channel(i, gpios[i]) != ESP_OK) {
      ESP_LOGE(TAG, "DShot ch%d init FAIL -> stress missing on this channel (boot continues)", i);
    }
  }
  ESP_LOGI(TAG, "DShot ready 4ch GPIO %d,%d,%d,%d [Core1-motor-out]",
           FC_DSHOT_GPIO_M0, FC_DSHOT_GPIO_M1, FC_DSHOT_GPIO_M2, FC_DSHOT_GPIO_M3);
#else
  esp_err_t ret = fc_dshot_init_channel(0, FC_DSHOT_GPIO_M0);
  if (ret != ESP_OK)
    return ret;
  ESP_LOGI(TAG, "DShot ready (GPIO%d, 1ch)", FC_DSHOT_GPIO_M0);
#endif
  return ESP_OK;
}

/* ==========================================================================
 * SPI bus | SPI veri yolu
 * ========================================================================== */

#define SPI_HOST SPI3_HOST
#define SPI_PIN_MISO 5
#define SPI_PIN_MOSI 22
#define SPI_PIN_SCLK 23
#define SPI_PIN_CS 4
#define SPI_CLK_SPEED (20 * 1000 * 1000)

spi_device_handle_t s_spi_handle = NULL;
static bool s_spi_ok = false;

static esp_err_t spi_sensors_init(void) {
  spi_bus_config_t bus = {
      .mosi_io_num = SPI_PIN_MOSI,
      .miso_io_num = SPI_PIN_MISO,
      .sclk_io_num = SPI_PIN_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 16,
  };
  spi_device_interface_config_t dev = {
      .clock_speed_hz = SPI_CLK_SPEED,
      .mode = 0,
      .spics_io_num = SPI_PIN_CS,
      .queue_size = 1,
  };
  esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK)
    return ret;
  ret = spi_bus_add_device(SPI_HOST, &dev, &s_spi_handle);
  if (ret != ESP_OK)
    return ret;

  /* Separate device handle for the HTTP test endpoint (arbitrated via bus lock) | HTTP test uç-noktası için ayrı cihaz tanıtıcısı (bus kilidiyle hakemlenir) */
  spi_device_interface_config_t http_dev = {
      .clock_speed_hz = 1 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = -1,
      .queue_size = 1,
  };
  ret = spi_bus_add_device(SPI_HOST, &http_dev, &g_http_spi);
  if (ret != ESP_OK)
    return ret;

  s_spi_ok = true;
  ESP_LOGI(TAG, "SPI OK (8MHz)");
  return ESP_OK;
}


/* ==========================================================================
 * EKF — Quaternion-based Madgwick/Mahony | EKF — Dördey/Quaternion-tabanlı Madgwick/Mahony
 * ==========================================================================
 */

#define SQ(x) ((x) * (x))
#define FC_DEG_TO_RAD 0.01745329252f
#define FC_RAD_TO_DEG 57.2957795131f
#define EKF_GAIN 0.5f



/* ==========================================================================
 * EKF Core 0 task: split out of the 8kHz hot-loop.
 * EKF step ~973us doesn't fit the 125us slot (measured: ekf_max≈jitter_max), so it
 * runs on Core 0 at IMU rate (~125Hz); Core 1 does PID and reads the latest estimate
 * from the seqlock -> Core 1 stays deterministic.
 * ---
 * EKF Core 0 task'ı: 8kHz sıcak-döngüden ayrıldı.
 * EKF adımı ~973us, 125us yuvaya sığmaz (ölçülen: ekf_max≈jitter_max), bu yüzden
 * Core 0'da IMU hızında çalışır (~125Hz); Core 1 PID yapar ve en güncel kestirimi
 * seqlock'tan okur -> Core 1 belirlenimci kalır.
 * ========================================================================== */
static TaskHandle_t s_ekf_task_handle = NULL;

/* RX task (Core 0) calls on new HIGHRES_IMU -> wakes the EKF task. | RX task (Core 0) yeni HIGHRES_IMU'da çağırır -> EKF task'ını uyandırır. */
void fc_ekf_notify_new_imu(void) {
  if (s_ekf_task_handle)
    xTaskNotifyGive(s_ekf_task_handle);
}

static void fc_ekf_task(void *pv) {
  (void)pv;
  ESP_LOGI(TAG, "EKF task started on Core %d (real ekf2 module output -> Core1 seqlock)",
           xPortGetCoreID());

  for (;;) {
    /* Periodic poll instead of notify: no notify-caller, portMAX_DELAY would block forever.
     * ekf2 (Core0 WQ) publishes ~100-250Hz; 4ms (~250Hz) matches it.
     * ---
     * Bildirim yerine periyodik yoklama: bildirim-çağırıcı yok, portMAX_DELAY sonsuza dek bloklardı.
     * ekf2 (Core0 WQ) ~100-250Hz yayınlar; 4ms (~250Hz) buna uyar. */
    vTaskDelay(pdMS_TO_TICKS(4));

    /* Bridge ekf2 output (vehicle_local_position/attitude) to the Core1 seqlock;
     * nothing written if no new output -> Core1 reads the last valid estimate.
     * ---
     * ekf2 çıktısını (vehicle_local_position/attitude) Core1 seqlock'una köprüle;
     * yeni çıktı yoksa hiçbir şey yazılmaz -> Core1 son geçerli kestirimi okur. */
    fc_px4_ekf2_to_seqlock();
#if FC_CORE1_MOTOR_OUT
    fc_px4_actuator_to_seqlock();   /* actuator_motors -> motor-cmd seqlock (Core1 8kHz DShot reads) | actuator_motors -> motor-komut seqlock (Core1 8kHz DShot okur) */
#endif
  }
}

void fc_ekf_task_start(void) {
  /* prio 21: writes the seqlock Core1 reads. Must sit above WQ-band (6-18)/receiver (19)/event (20)
   * and below esp_timer (22)/8kHz-Core1 (24) so the (us-scale) seqlock write isn't preempted by
   * WQs and Core1's read doesn't spin. Runs on Core 0 (Core1 untouched).
   * ---
   * prio 21: Core1'in okuduğu seqlock'u yazar. WQ-bandı (6-18)/receiver (19)/event (20) üstünde
   * ve esp_timer (22)/8kHz-Core1 (24) altında olmalı ki (us-ölçekli) seqlock yazımı WQ'larca
   * kesilmesin ve Core1'in okuması dönmesin. Core 0'da çalışır (Core1'e dokunulmaz). */
  BaseType_t r = xTaskCreatePinnedToCore(fc_ekf_task, "fc_ekf", 16384, NULL, 21,
                                         &s_ekf_task_handle, 0);
  if (r != pdPASS)
    ESP_LOGE(TAG, "Failed to create EKF task");
}

/* ==========================================================================
 * Bench-pilot — FC-internal load generator (FC_BENCH_AUTOPILOT)
 * Continuous ramp + sine + pseudo-random "turbulence" signal at full 8kHz rate,
 * replacing HTTP-driven ARM/throttle/setpoint to remove HTTP as a measurement variable.
 * ---
 * Tezgah-pilotu — FC-içi yük üreteci (FC_BENCH_AUTOPILOT)
 * Tam 8kHz hızında sürekli rampa + sinüs + sözde-rastgele "türbülans" sinyali,
 * ölçüm değişkeni olarak HTTP'yi kaldırmak için HTTP-güdümlü ARM/gaz/ayar-noktasının yerine geçer.
 * ==========================================================================
 */


/* ==========================================================================
 * PID + Mixer | PID + Karıştırıcı
 * ==========================================================================
 */

#define PID_OUTPUT_LIMIT 1.0f
#define PID_INTEGRAL_LIMIT 0.3f


void IRAM_ATTR fc_mixer_apply(const fc_mixer_output_t *output) {
  if (!output)
    return;
  for (int i = 0; i < FC_MAX_MOTORS; i++) {
    float n = output->motor[i];
    if (n < 0)
      n = 0;
    if (n > 1)
      n = 1;
    uint16_t thr = (uint16_t)(DSHOT_THROTTLE_MIN +
                              n * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN));
    fc_dshot_set_throttle(i, thr);
  }
}


/* ==========================================================================
 * FC Init | FC Başlatma
 * ==========================================================================
 */

esp_err_t fc_core_init(void) {
  ESP_LOGI(TAG, "SMP FC init...");
  esp_err_t ret = fc_dshot_init();
  if (ret != ESP_OK)
    return ret;
  ret = spi_sensors_init();
  if (ret != ESP_OK)
    return ret;
  ret = spi_device_acquire_bus(s_spi_handle, portMAX_DELAY);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI acquire failed: %s", esp_err_to_name(ret));
    return ret;
  }
  s_fc_state.state = FC_STATE_DISARMED;
  ESP_LOGI(TAG, "FC ready (DISARMED), SPI bus acquired");
  return ESP_OK;
}

/* ==========================================================================
 * FC TASK — Core 1, SMP pinned task, max priority | FC TASK — Core 1, SMP sabitlenmiş task, azami öncelik
 * ==========================================================================
 */

/* Memory barrier for shared volatile struct (RISC-V weak ordering) | Paylaşılan volatile struct için bellek bariyeri (RISC-V zayıf sıralama) */
static inline void mem_barrier(void) {
  __asm__ volatile("fence rw, rw" : : : "memory");
}

/* ── Core 1 FC task (pinned, max priority, never yields) ── | ── Core 1 FC task (sabitlenmiş, azami öncelik, asla teslim etmez) ── */
void IRAM_ATTR fc_task_main(void *arg) {
  (void)arg;

  /* RISC-V global pointer init (required for linker relaxation) | RISC-V global işaretçi başlatma (linker gevşetmesi için gerekli) */
  __asm__ __volatile__(".option push\n"
                       ".option norelax\n"
                       "la gp, __global_pointer$\n"
                       ".option pop");

  const uint32_t TARGET_CYCLES = FC_LOOP_CYCLES; // 45000

  /* Compute jitter (cpu_time_us variation) | Hesap seğirmesi (cpu_time_us değişimi) */
  uint32_t jmin = UINT32_MAX, jmax = 0;
  uint64_t jsum = 0;
  uint32_t jsamples = 0;


  /* True jitter (loop timing deviation) | Gerçek seğirme (döngü zamanlama sapması) */
  uint32_t tjmin = UINT32_MAX, tjmax = 0;
  uint64_t tjsum = 0;
  uint32_t tjsamples = 0;

  uint32_t slow = 0;
  uint32_t next_cycle = esp_cpu_get_cycle_count() + TARGET_CYCLES;

  /* Max EKF step time from Core 0 (via seqlock, logged for validation). | Core 0'dan azami EKF adım süresi (seqlock ile, doğrulama için loglanır). */
  uint32_t ekf_max_us = 0;


  while (1) {
    /* ── Spin until target cycle ── | ── Hedef çevrime kadar dön ── */
    while ((int32_t)(esp_cpu_get_cycle_count() - next_cycle) < 0) {
      __asm__ volatile("nop");
    }
    uint32_t loop_wake = esp_cpu_get_cycle_count();
    next_cycle += TARGET_CYCLES;

    /* HIL: sensor source is Gazebo (simulator_mavlink -> ekf2); no local physical-IMU read. | HIL: sensör kaynağı Gazebo (simulator_mavlink -> ekf2); yerel fiziksel-IMU okuması yok. */
    uint32_t t0 = esp_cpu_get_cycle_count();
    uint32_t t1 = esp_cpu_get_cycle_count();


    /* 3. EKF */
#if FC_EKF_MODE == FC_EKF_MODE_PX4_FULL
    /* EKF is on Core 0 (fc_ekf_task); here just read the latest estimate from the
     * seqlock (fast copy) -> Core 1 stays deterministic.
     * ---
     * EKF Core 0'da (fc_ekf_task); burada yalnızca en güncel kestirimi
     * seqlock'tan oku (hızlı kopya) -> Core 1 belirlenimci kalır. */
    {
      fc_ekf_output_t eo;
      if (fc_ekf_output_seqlock_read(&eo)) {   /* true=consistent; false=Core0 died mid-write -> torn eo, don't use | true=tutarlı; false=Core0 yazma-ortasında öldü -> yırtık eo, kullanma */
      s_fc_state.ekf.attitude.w = eo.q0;
      s_fc_state.ekf.attitude.x = eo.q1;
      s_fc_state.ekf.attitude.y = eo.q2;
      s_fc_state.ekf.attitude.z = eo.q3;
      s_fc_state.ekf.euler = eo.euler;
      s_fc_state.ekf.state24[4] = eo.vel_n;
      s_fc_state.ekf.state24[5] = eo.vel_e;
      s_fc_state.ekf.state24[6] = eo.vel_d;
      s_fc_state.ekf.state24[7] = eo.pos_n;
      s_fc_state.ekf.state24[8] = eo.pos_e;
      s_fc_state.ekf.state24[9] = eo.pos_d;
      /* Staleness check (PX4 estimatorCheck): if the IMU/sensor stream stops, fc_ekf_task
       * waits and the last state freezes with healthy=1. Not updated >500ms -> mark unhealthy
       * so failsafe can trigger.
       * ---
       * Bayatlık kontrolü (PX4 estimatorCheck): IMU/sensör akışı durursa, fc_ekf_task
       * bekler ve son durum healthy=1 ile donar. >500ms güncellenmezse -> sağlıksız işaretle
       * ki failsafe tetiklenebilsin. */
      {
        int64_t age_us = esp_timer_get_time() - (int64_t)eo.time_us;
        bool fresh = (age_us >= 0) && (age_us < 500000); /* 500ms */
        s_fc_state.ekf.healthy = eo.healthy && fresh;
      }
      if (eo.compute_us > ekf_max_us) /* Core 0 EKF time (validation) | Core 0 EKF süresi (doğrulama) */
        ekf_max_us = eo.compute_us;
      } else {
        /* SAFETY: seqlock-read stuck (Core0 fc_ekf_task died mid-write) -> don't use torn eo;
         * last estimate kept but EKF marked unhealthy so failsafe triggers (no flying on stale state).
         * ---
         * GÜVENLİK: seqlock-okuma takıldı (Core0 fc_ekf_task yazma-ortasında öldü) -> yırtık eo kullanma;
         * son kestirim tutulur ama EKF sağlıksız işaretlenir ki failsafe tetiklensin (bayat durumda uçuş yok). */
        s_fc_state.ekf.healthy = false;
      }
    }
#endif /* dead SIMPLE_KALMAN/MAHONY branches removed (ECL active) | ölü SIMPLE_KALMAN/MAHONY dalları kaldırıldı (ECL etkin) */




#if FC_CORE1_MOTOR_OUT
    /* actuator_motors (Core0 fc_ekf_task bridge) -> 4x DShot RMT. Adds motor-output load to
     * Core1 (no ESC -> free pins); rmt_time_us tracks the in-8kHz cost (overrun/jitter).
     * Gazebo flight is unaffected (that goes Core0 pwm_out_sim->simulator_mavlink); parallel load.
     * ---
     * actuator_motors (Core0 fc_ekf_task köprüsü) -> 4x DShot RMT. Core1'e motor-çıkış yükü
     * ekler (ESC yok -> boş pinler); rmt_time_us 8kHz-içi maliyeti izler (aşım/seğirme).
     * Gazebo uçuşu etkilenmez (o Core0 pwm_out_sim->simulator_mavlink'ten gider); paralel yük. */
    {
      uint32_t rmt_t0 = esp_cpu_get_cycle_count();
      fc_motor_cmd_t mc;
      bool mc_ok = fc_motor_cmd_seqlock_read(&mc);
#if FC_CORE1_STALE_GUARD
      /* SAFETY: if Core0 dies/stalls, mc goes stale -> last throttle held forever = flyaway.
       * mc_ok=false (writer stuck) OR time_us > FC_MOTOR_CMD_MAX_AGE_US (Core0 not writing) -> cut motors.
       * ---
       * GÜVENLİK: Core0 ölür/takılırsa, mc bayatlar -> son gaz sonsuza dek tutulur = uçup-gitme.
       * mc_ok=false (yazıcı takılı) VEYA time_us > FC_MOTOR_CMD_MAX_AGE_US (Core0 yazmıyor) -> motorları kes. */
      int64_t mc_age = (int64_t)esp_timer_get_time() - (int64_t)mc.time_us;
      bool mc_fresh = mc_ok && mc.valid && (mc_age >= 0) && (mc_age < FC_MOTOR_CMD_MAX_AGE_US);
#else
      bool mc_fresh = mc.valid;   /* guard OFF: old behavior (valid only) | koruma KAPALI: eski davranış (yalnızca valid) */
      (void)mc_ok;
#endif
      fc_mixer_output_t mo;
      for (int i = 0; i < FC_MAX_MOTORS; i++) {
        float v = mc_fresh ? mc.motor[i] : 0.0f;   /* stale/stuck -> 0 (Core1 failsafe) | bayat/takılı -> 0 (Core1 failsafe) */
        if (!(v == v)) { v = 0.0f; }   /* NAN-guard (disarmed motor -> idle) | NAN-koruması (disarm motor -> boşta) */
        mo.motor[i] = v;
      }
      fc_mixer_apply(&mo);   /* 4x fc_dshot_set_throttle -> rmt_transmit (real RMT TX) | 4x fc_dshot_set_throttle -> rmt_transmit (gerçek RMT TX) */
      s_fc_state.rmt_time_us = (esp_cpu_get_cycle_count() - rmt_t0) / 360;
    }
#endif
    uint32_t t2 = esp_cpu_get_cycle_count();


    /* ── Timing ── | ── Zamanlama ── */
    uint32_t total_cycles = t2 - t0;
    uint32_t spi_cycles = t1 - t0;
    uint32_t compute_cycles = t2 - t1;
    uint32_t total_us = total_cycles / 360;

    s_fc_state.cpu_time_us = total_us;
    s_fc_state.spi_time_us = spi_cycles / 360;
    s_fc_state.compute_time_us = compute_cycles / 360;
    s_fc_state.loop_count++;
    slow++;

    /* True jitter: deviation of loop_wake from ideal wake (in cycles) = actual - expected. | Gerçek seğirme: loop_wake'in ideal uyanmadan sapması (çevrim cinsinden) = gerçek - beklenen. */
    int32_t true_jitter_cycles =
        (int32_t)(loop_wake - (next_cycle - TARGET_CYCLES));
    uint32_t true_jitter_us = (true_jitter_cycles < 0)
                                  ? (uint32_t)(-true_jitter_cycles / 360)
                                  : (uint32_t)(true_jitter_cycles / 360);

    if (true_jitter_us < tjmin)
      tjmin = true_jitter_us;
    if (true_jitter_us > tjmax)
      tjmax = true_jitter_us;
    tjsum += true_jitter_us;
    tjsamples++;

    /* Compute jitter | Hesap seğirmesi */
    if (total_us < jmin)
      jmin = total_us;
    if (total_us > jmax)
      jmax = total_us;
    jsum += total_us;
    jsamples++;

    /* Overrun detection | Aşım tespiti */
    if (total_us > FC_TARGET_PERIOD_US) {
      s_fc_state.overrun_count++;
    }

    /* Periodic reporting (every 5s) | Periyodik raporlama (her 5s) */
    if (slow >= FC_PID_LOOP_FREQ_HZ * 5) {
      s_fc_state.jitter_min_us = jmin;
      s_fc_state.jitter_max_us = jmax;
      s_fc_state.jitter_avg_us = (uint32_t)(jsum / jsamples);
      s_fc_state.true_jitter_min_us = tjmin;
      s_fc_state.true_jitter_max_us = tjmax;
      s_fc_state.true_jitter_avg_us = (uint32_t)(tjsum / tjsamples);
      s_fc_state.dbg_ekf_max_us = ekf_max_us; /* Core 0 EKF step time (us) | Core 0 EKF adım süresi (us) */
      g_fc_load_pct = (total_us * FC_PID_LOOP_FREQ_HZ) / 10000;
      if (g_fc_load_pct > 100)
        g_fc_load_pct = 100;

      /* Publish to Core 0 via seqlock | Seqlock ile Core 0'a yayınla */
      fc_seqlock_write(&s_fc_state);


      jmin = UINT32_MAX;
      jmax = 0;
      jsum = 0;
      jsamples = 0;
      tjmin = UINT32_MAX;
      tjmax = 0;
      tjsum = 0;
      tjsamples = 0;
      ekf_max_us = 0; /* reset measurement | ölçümü sıfırla */
      slow = 0;
    }
  }
}
