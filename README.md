# ESP32-P4 · Verbatim-PX4 Flight Stack (HIL)

> ⚠️ **EXPERIMENTAL / DENEYSEL.** This is a research project, **not** a flight-ready product.
> It runs the PX4 autopilot on an ESP32-P4 and is validated **only in Hardware-in-the-Loop (HIL)**
> against Gazebo. **No real sensors or real motors/ESCs are integrated.** Do not fly a real aircraft with this.

**🌐 Language / Dil:** [English](#english) · [Türkçe](#türkçe)

---

<a name="english"></a>
# English

## What this project is

An attempt to run the **unmodified PX4 multicopter flight stack** (estimator + controllers + navigator +
commander + MAVLink) on a **dual-core ESP32-P4** microcontroller, and to prove — *before* buying flight
hardware — that the P4 has enough compute/timing headroom to carry a real-hardware-equivalent load.

Validation is done in **Hardware-in-the-Loop (HIL)**: the real ESP32-P4 board runs the firmware; a Gazebo
simulation (x500 quadcopter) provides the sensor data and consumes the motor output; QGroundControl is the
ground station. **The P4 is real; the world around it is simulated.**

## What this project is NOT (honesty first)

- ❌ **Not a real flying drone.** Real IMU/baro/mag/GPS drivers and real ESC/motor output are **not implemented**. Sensors come from Gazebo; the Core-1 DShot output goes to **empty pins** (a real-hardware *rehearsal*, not real motors).
- ❌ **Not production/flight-safe.** It is an experimental bring-up.
- ❌ **Not a from-scratch reimplementation of PX4.** It runs the actual PX4 source (see [PX4 base & modifications](#px4-base--modifications)).

## What has been demonstrated (in HIL)

- The full verbatim-PX4 flight chain runs on the P4 and closes a HIL loop through the real hardware: arm → takeoff → go-to → RTL → land, controlled from QGroundControl.
- Under a **real-hardware-equivalent motor-output load** (Core-1 @ 8 kHz driving 4× DShot600 via RMT), the 8 kHz loop sustained **0 overrun** for a full flight, with ~48–72 µs jitter inside the 125 µs budget.
- **Conclusion:** the P4's *compute/timing* is sufficient for this workload. Real-hardware I/O integration is a separate, unfinished step.

## Hardware

| Item | Detail |
|------|--------|
| **Board** | Waveshare **ESP32-P4-NANO** |
| **MCU** | ESP32-P4 — dual RISC-V @ 360 MHz, 32 MB PSRAM, 16 MB Flash |
| **Wireless co-processor** | ESP32-C6-MINI over SDIO (Wi-Fi STA/AP, BLE, ESP-NOW — the P4 has no native radio) |
| **Toolchain** | ESP-IDF v6.0.1, `riscv32-esp-elf` |
| **Motor output (Core-1)** | 4× DShot600 via RMT on GPIO **21 / 20 / 26 / 27** *(empty pins — no ESC connected)* |
| **P4 ↔ C6 link** | SDIO on GPIO 14–19, 54 |

> **No physical sensors are connected.** All sensor data (IMU, baro, mag, GPS) comes from Gazebo over the network (HIL). No real IMU/ESC is used or required for this HIL work; RC is not implemented.

## Architecture — dual core

| | **Core 0** — "PX4 brain" | **Core 1** — "lifeguard" (deterministic 8 kHz) |
|---|---|---|
| Runs | Full verbatim-PX4 stack as FreeRTOS/WorkQueue tasks: `simulator_mavlink → sensors → ekf2 → commander → navigator → flight_mode_manager → mc_pos/att/rate_control → control_allocator → mavlink`, plus `pwm_out_sim` (→ Gazebo). | A single fixed-rate 8 kHz loop (`fc_task_main`): reads the EKF & motor-command seqlocks from Core 0, drives 4× DShot, measures jitter/overrun, and acts as a safety net. |
| In HIL | Does the actual flying (motors → Gazebo via `pwm_out_sim`). | Drives empty-pin DShot to measure the real motor-output load on the 8 kHz loop. |
| Safety | — | If Core 0 (the PX4 brain) dies/stalls, Core 1 cuts the motors (freshness + spin guards). |

The two cores communicate through lock-free **seqlocks** (Core 0 writes, Core 1 reads). The heavy, non-deterministic PX4 stack lives on Core 0 so it cannot break Core 1's hard-real-time 8 kHz loop.

## PX4 base & modifications

- **Base:** PX4-Autopilot, upstream commit `e24cda2fd96be7078a7a1ab763585b81c6142e85` (with some sub-trees imported from adjacent versions — *mixed vintage*).
- **Fidelity:** the flight-critical chain (ekf2 / mc_* / control_allocator / uORB / work_queue / cdev / hrt) is used **as upstream** — the only hand-edits are the 3 marked port files below (`grep __PX4_ESPIDF` finds them all). Some sub-trees are imported from adjacent PX4 versions (*mixed vintage*, e.g. the EKF before latitude-dependent gravity), so it is not a single-commit checkout. No hidden hand-edits — every deviation is marked in-place.
- **Actual edits inside PX4 files:** 3 files, all at the ESP-IDF port boundary and marked in-place — `src/lib/matrix/px4_platform_common/tasks.h` (add an `__PX4_ESPIDF` branch for FreeRTOS fixed priorities) plus `sem.h` and `px4_sem.cpp` (exclude `__PX4_ESPIDF` from the POSIX-semaphore path). Nothing else in PX4 source is hand-edited.
- **Project platform layer (in `main/fc_*`, *linked against* PX4 — does not modify PX4 files):** `hrt` clock wraps (soft-lockstep sim time), a `sem_timedwait` clock fix, WorkQueue priority mapping, uORB init, the Core0↔Core1 seqlock bridges, the DShot driver, and the Core-1 motor-load bridge with its safety flags (`FC_CORE1_MOTOR_SAFEGATE`, `FC_CORE1_STALE_GUARD`, `FC_DSHOT_STATIC_PAYLOAD`).
- **Shims:** ~8 stub/generated headers that stand in for what PX4's own build normally produces, or that provide platform glue — e.g. `build_git_version.h`, `component_information/checksums.h`, `mixer_module/output_functions.hpp`, `px4_platform_common/micro_hal.h`, and the `fc_px4_board_compat.h` / `fc_px4_netif_compat.h` / `fc_wq_pthread_compat.h` shims — plus a generated parameter table (`tools/gen_px4_params.py`). All needed so verbatim PX4 compiles under ESP-IDF.
- **HIL fidelity fix (host side, not on the P4):** the sensor bridge saturates accelerometer values to ±16 g (`_sat_accel`), reproducing what a real FIFO-IMU chip does (a real chip *saturates*; PX4's `SimulatorMavlink` FIFO emulation was *wrapping* on >16 g). See `hil/HIL_SETUP.md`.

## Known limits & caveats (honest)

- **HIL is not reality.** It proves compute/timing, not hardware integration.
- **18 g touchdown spike is partly a Gazebo artifact:** rigid contact + a 4 ms physics step render a gentle ~0.7 m/s landing as a sharp ~18 g impulse (a real airframe/landing gear would absorb it to a few g).
- **Slightly optimistic sensor rates:** the bridge sends all sensors at 250 Hz, over-sampling baro/mag versus their native rates — so the HIL estimate may look a bit smoother than real hardware would.
- **Two real-hardware-before fixes are coded but not hardware-validated** (`FC_DSHOT_STATIC_PAYLOAD`, `FC_CORE1_STALE_GUARD`).
- Beyond the flight stack, the board also runs C6 Wi-Fi/BLE and Ethernet networking (Ethernet carries the HIL/QGC link) plus a minimal static HTTP page.

## HIL setup

See **[`hil/HIL_SETUP.md`](hil/HIL_SETUP.md)** — architecture, data flow, dependencies (Docker `px4io/px4-sitl-gazebo`, gz-python, pymavlink), and exact run commands (Gazebo, the two bridges, P4 flash, QGC).

## Build & flash

Requires **ESP-IDF v6.0.1**. Activate the IDF environment (`export.sh` / `export.bat`), then from the repo root:

```bash
idf.py build                 # build
idf.py -p <PORT> flash       # flash    (<PORT> = COM3, /dev/ttyUSB0, …)
idf.py -p <PORT> monitor     # serial monitor
```
C6 companion firmware: prebuilt binary at `c6_firmwares/network_adapter.bin` (see `c6_firmwares/README.md`).

## Repository layout

| Path | Contents |
|------|----------|
| `main/` | Firmware source: `fc_*` platform layer + `main/PX4/` (the PX4 tree) |
| `hil/` | Gazebo HIL bridges (`_sat_accel` landing fix) + `HIL_SETUP.md` + world file |
| `c6_firmwares/` | Prebuilt ESP32-C6 companion firmware (`network_adapter.bin`) |

## License & credits

Built on [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (BSD-3-Clause). PX4 source retains its original license and copyright. The ESP32-P4 port layer and HIL bridges are this project's contribution.

---

<a name="türkçe"></a>
# Türkçe

## Bu proje nedir

**Değiştirilmemiş PX4 multicopter uçuş yığınını** (kestirim + kontrolcüler + navigator + commander + MAVLink)
**çift-çekirdekli ESP32-P4** mikrodenetleyicide çalıştırma denemesi — ve uçuş donanımı satın almadan *önce*
P4'ün gerçek-donanım-eşdeğeri yükü taşıyacak hesaplama/zamanlama payına sahip olduğunu kanıtlamak.

Doğrulama **Hardware-in-the-Loop (HIL)** ile yapılır: gerçek ESP32-P4 kartı firmware'i koşar; Gazebo
simülasyonu (x500 quadcopter) sensör verisini sağlar ve motor-çıkışını tüketir; QGroundControl yer
istasyonudur. **P4 gerçek; etrafındaki dünya simülasyon.**

## Bu proje NE DEĞİL (önce dürüstlük)

- ❌ **Uçan gerçek bir drone değil.** Gerçek IMU/baro/mag/GPS sürücüleri ve gerçek ESC/motor çıkışı **kurulmadı**. Sensörler Gazebo'dan gelir; Core-1 DShot çıkışı **boş pinlere** gider (gerçek-donanım *provası*, gerçek motor değil).
- ❌ **Üretim/uçuş-güvenli değil.** Deneysel bir ayağa-kaldırma çalışması.
- ❌ **PX4'ün sıfırdan yeniden yazımı değil.** Gerçek PX4 kaynağını çalıştırır (bkz. [PX4 tabanı ve değişiklikler](#px4-tabanı--değişiklikler)).

## Kanıtlanan (HIL'de)

- Tam verbatim-PX4 uçuş-zinciri P4'te koşuyor ve kapalı HIL döngüsünü gerçek donanımdan geçiriyor: arm → kalkış → git-oraya → RTL → iniş, QGroundControl'den kontrollü.
- **Gerçek-donanım-eşdeğeri motor-çıkış yükü** altında (Core-1 @ 8 kHz, RMT ile 4× DShot600), 8 kHz döngü tüm bir uçuş boyunca **0 overrun** sürdürdü; jitter ~48–72 µs (125 µs bütçesi içinde).
- **Sonuç:** P4'ün *hesaplama/zamanlama'sı* bu yük için yeterli. Gerçek-donanım G/Ç entegrasyonu ayrı, tamamlanmamış bir adım.

## Donanım

| Öğe | Ayrıntı |
|-----|---------|
| **Kart** | Waveshare **ESP32-P4-NANO** |
| **MCU** | ESP32-P4 — çift RISC-V @ 360 MHz, 32 MB PSRAM, 16 MB Flash |
| **Kablosuz yardımcı-işlemci** | SDIO üzerinden ESP32-C6-MINI (Wi-Fi STA/AP, BLE, ESP-NOW — P4'ün kendi radyosu yok) |
| **Araç zinciri** | ESP-IDF v6.0.1, `riscv32-esp-elf` |
| **Motor çıkışı (Core-1)** | GPIO **21 / 20 / 26 / 27** üzerinde RMT ile 4× DShot600 *(boş pinler — ESC bağlı değil)* |
| **P4 ↔ C6 bağlantısı** | GPIO 14–19, 54 üzerinde SDIO |

> **Hiçbir fiziksel sensör bağlı değil.** Tüm sensör verisi (IMU, baro, mag, GPS) Gazebo'dan network üzerinden gelir (HIL). Bu HIL çalışması için gerçek IMU/ESC kullanılmaz veya gerekmez; RC uygulanmadı.

## Mimari — çift çekirdek

| | **Core 0** — "PX4 beyni" | **Core 1** — "cankurtaran" (deterministik 8 kHz) |
|---|---|---|
| Koşan | Tam verbatim-PX4 yığını, FreeRTOS/WorkQueue task'ları olarak: `simulator_mavlink → sensors → ekf2 → commander → navigator → flight_mode_manager → mc_pos/att/rate_control → control_allocator → mavlink`, artı `pwm_out_sim` (→ Gazebo). | Tek, sabit-hızlı 8 kHz döngü (`fc_task_main`): Core 0'ın EKF & motor-komut seqlock'larını okur, 4× DShot sürer, jitter/overrun ölçer ve güvenlik-ağı görevi görür. |
| HIL'de | Fiilen uçurur (motorlar → Gazebo, `pwm_out_sim` üzerinden). | 8 kHz döngüde gerçek motor-çıkış yükünü ölçmek için boş-pin DShot sürer. |
| Güvenlik | — | Core 0 (PX4 beyni) ölür/stall olursa, Core 1 motorları keser (tazelik + spin koruması). |

İki çekirdek kilitsiz **seqlock**'larla haberleşir (Core 0 yazar, Core 1 okur). Ağır, deterministik-olmayan PX4 yığını Core 0'da yaşar ki Core 1'in hard-real-time 8 kHz döngüsünü bozmasın.

## PX4 tabanı & değişiklikler

- **Taban:** PX4-Autopilot, upstream commit `e24cda2fd96be7078a7a1ab763585b81c6142e85` (bazı alt-ağaçlar komşu sürümlerden alınmış — *karma-vintage*).
- **Sadakat:** uçuş-kritik zincir (ekf2 / mc_* / control_allocator / uORB / work_queue / cdev / hrt) **upstream'in aynısı** kullanılır — tek elle-edit aşağıdaki 3 işaretli port-dosyasıdır (`grep __PX4_ESPIDF` hepsini bulur). Birkaç alt-ağaç komşu-PX4-sürümlerinden alınmış (*karma-vintage*, ör. enlem-bağımlı yerçekimi öncesi EKF), bu yüzden tek-commit checkout değil. Gizli elle-edit yok — her sapma yerinde işaretli.
- **PX4 dosyalarındaki gerçek edit'ler:** 3 dosya, hepsi ESP-IDF port-sınırında ve yerinde-işaretli — `src/lib/matrix/px4_platform_common/tasks.h` (FreeRTOS sabit-öncelik için `__PX4_ESPIDF` dalı eklendi) + `sem.h` ve `px4_sem.cpp` (POSIX-semafor yolu `__PX4_ESPIDF`'te dışlandı). PX4 kaynağında başka elle-edit yok.
- **Proje platform-katmanı (`main/fc_*` içinde, PX4'e *link'lenir* — PX4 dosyalarını DEĞİŞTİRMEZ):** `hrt` saat-wrap'ları (soft-lockstep sim-zamanı), `sem_timedwait` saat-düzeltmesi, WorkQueue öncelik-eşlemesi, uORB init, Core0↔Core1 seqlock köprüleri, DShot sürücüsü ve güvenlik-flag'li Core-1 motor-yük köprüsü (`FC_CORE1_MOTOR_SAFEGATE`, `FC_CORE1_STALE_GUARD`, `FC_DSHOT_STATIC_PAYLOAD`).
- **Shim'ler:** PX4'ün kendi build'inin normalde ürettiği veya platform-glue olan ~8 stub/üretilmiş header — ör. `build_git_version.h`, `component_information/checksums.h`, `mixer_module/output_functions.hpp`, `px4_platform_common/micro_hal.h` ve `fc_px4_board_compat.h` / `fc_px4_netif_compat.h` / `fc_wq_pthread_compat.h` shim'leri — artı üretilmiş parametre tablosu (`tools/gen_px4_params.py`). Hepsi verbatim-PX4'ün ESP-IDF'te derlenmesi için.
- **HIL-sadakat düzeltmesi (host tarafı, P4'te değil):** sensör-köprüsü ivmeyi ±16 g'ye doyurur (`_sat_accel`) — gerçek FIFO-IMU çipinin yaptığını üretir (gerçek çip *doyar*; PX4 `SimulatorMavlink` FIFO-emülasyonu >16 g'de *sarıyordu*). Bkz. `hil/HIL_SETUP.md`.

## Bilinen sınırlar & çekinceler (dürüst)

- **HIL gerçeklik değildir.** Hesaplama/zamanlamayı kanıtlar, donanım-entegrasyonunu değil.
- **18 g temas-darbesi kısmen Gazebo-artefaktı:** rijit-temas + 4 ms fizik-adımı, nazik ~0.7 m/s inişi keskin ~18 g darbe gösterir (gerçek gövde/iniş-takımı bunu birkaç-g'ye yumuşatır).
- **Biraz iyimser sensör-hızları:** köprü tüm sensörleri 250 Hz'de gönderir, baro/mag'ı native-hızlarından fazla örnekler — HIL kestirimi gerçek-donanımdan biraz daha pürüzsüz görünebilir.
- **İki gerçek-donanım-öncesi düzeltme kodlandı ama donanımda doğrulanmadı** (`FC_DSHOT_STATIC_PAYLOAD`, `FC_CORE1_STALE_GUARD`).
- Uçuş-yığınının dışında kart ayrıca C6 Wi-Fi/BLE ve Ethernet ağını (Ethernet = HIL/QGC linki) + minimal statik HTTP sayfasını da koşar.

## HIL kurulumu

Bkz. **[`hil/HIL_SETUP.md`](hil/HIL_SETUP.md)** — mimari, veri-akışı, bağımlılıklar (Docker `px4io/px4-sitl-gazebo`, gz-python, pymavlink) ve tam çalıştırma-komutları (Gazebo, iki köprü, P4 flash, QGC).

## Derleme & flash

**ESP-IDF v6.0.1** gerekir. IDF ortamını aktive et (`export.sh` / `export.bat`), sonra depo kökünden:

```bash
idf.py build                 # derle
idf.py -p <PORT> flash       # flash    (<PORT> = COM3, /dev/ttyUSB0, …)
idf.py -p <PORT> monitor     # seri monitor
```
C6 companion firmware: hazır binary `c6_firmwares/network_adapter.bin` (bkz. `c6_firmwares/README.md`).

## Depo yerleşimi

| Yol | İçerik |
|-----|--------|
| `main/` | Firmware kaynağı: `fc_*` platform-katmanı + `main/PX4/` (PX4 ağacı) |
| `hil/` | Gazebo HIL köprüleri (`_sat_accel` iniş-fix'i) + `HIL_SETUP.md` + world dosyası |
| `c6_firmwares/` | Hazır ESP32-C6 companion firmware'i (`network_adapter.bin`) |

## Lisans & atıf

[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (BSD-3-Clause) üzerine kuruludur. PX4 kaynağı kendi orijinal lisansını ve telif hakkını korur. ESP32-P4 port-katmanı ve HIL köprüleri bu projenin katkısıdır.
