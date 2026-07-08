# HIL Setup — ESP32-P4 ↔ Gazebo (Hardware-in-the-Loop)

**🌐 Language / Dil:** [English](#english) · [Türkçe](#türkçe)

---

<a name="english"></a>
# English

This folder holds the **HIL bridges** that connect a real ESP32-P4 board to a Gazebo physics
simulation. Verbatim-PX4 flight firmware runs on the P4; sensors come from Gazebo and the motor
output goes back to Gazebo. QGroundControl is the ground station.

> **HONESTY NOTE:** This is a **HIL** setup — it proves the P4's compute/timing sufficiency.
> **No real sensor drivers and no real ESC/motor are used** (Gazebo emulates them). Details + caveats below.

## Architecture / data flow

```
   ┌─────────── Docker: drone_sim_gz (px4io/px4-sitl-gazebo) — host .91 ───────────┐
   │                                                                                │
   │   gz sim (default_gz_only.sdf, x500 quad, physics 250 Hz)                      │
   │        │  gz-transport (IMU/mag/baro/navsat topics)                            │
   │        ▼                                                                        │
   │   hil_sensor_bridge.py ──HIL_SENSOR/HIL_GPS(MAVLink,UDP)──►  P4 :14560         │
   │        ▲                                                       (172.16.16.57)   │
   │        │  ACT-FWD (127.0.0.1:14600)          P4 HIL_ACTUATOR ─┘                 │
   │        │                                                                        │
   │   hil_motor_bridge_robust.py ──gz.msgs.Actuators──► /x500_0/command/motor_speed│
   └────────────────────────────────────────────────────────────────────────────────┘
                                    ▲
                                    │ MAVLink (QGC ↔ P4 mavlink module)
                              QGroundControl
```

- **Sensor direction:** Gazebo → `hil_sensor_bridge.py` → P4 (the `simulator_mavlink` module publishes the sensor topics → ekf2).
- **Motor direction:** P4 `control_allocator → actuator_motors → pwm_out_sim → HIL_ACTUATOR` → bridge → Gazebo motors.
- On the P4: sensor → ekf2 → mc_pos/att/rate → control_allocator (the full PX4 chain, on Core 0).

## Requirements
- **Docker image:** `px4io/px4-sitl-gazebo` (Gazebo Harmonic + x500 model + gz-transport Python bindings).
- **Python packages** (present in the image): `gz.transport13`, `gz.msgs10`, `pymavlink`.
- **P4 firmware:** this repo (`idf.py flash`); at boot it expects `simulator_mavlink start -u 14560`.
- **QGroundControl** (ground station).
- **Network:** the P4 and the Docker host are on the same LAN. The P4 IP is set in `hil_sensor_bridge.py` as `P4_IP` (default `172.16.16.57:14560`) — edit it to your own P4 IP.

## Running

**1) Start the container** (gz sim + world; the entrypoint runs `gz sim ... default_gz_only.sdf`):
```bash
docker start drone_sim_gz          # or: docker run ... px4io/px4-sitl-gazebo
# the gz world is ready in ~25-30 s
```

**2) Start the two bridges** (CRITICAL pattern — NOT `sh -c "nohup ... &"`; a trailing `&` gets reaped):
```bash
docker exec -d drone_sim_gz sh -c "export PYTHONPATH=/projects/scripts/pylibs \
  GZ_SIM_RESOURCE_PATH=/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds \
  GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/px4-gazebo/lib/gz/plugins \
  GZ_SIM_SERVER_CONFIG_PATH=/opt/px4-gazebo/share/gz/server.config PX4_SIM_MODEL=gz_x500; \
  nohup python3 /projects/scripts/hil_sensor_bridge.py > /tmp/sensor_bridge.log 2>&1"

docker exec -d drone_sim_gz sh -c "export PYTHONPATH=/projects/scripts/pylibs; \
  nohup python3 /projects/scripts/hil_motor_bridge_robust.py > /tmp/motor_bridge.log 2>&1"
```
Verify: `docker exec drone_sim_gz tail -4 /tmp/sensor_bridge.log` → `[SENSOR BRIDGE] imu_sent=N (~250Hz)`.

**3) Flash + boot the P4** (this repo). At boot the EKF aligns (~30–60 s), then you can arm from QGC.

**4) Connect QGC** → arm → takeoff → fly.

> **Note:** After a sim reset (drone tipped/flipped), **restart the motor bridge** (the gz publisher handle goes stale). `hil_motor_bridge_robust.py` has a watchdog (it relaunches itself) but on a container restart, start both bridges by hand.

## `_sat_accel` — HIL fidelity fix (accelerometer saturation)

In `hil_sensor_bridge.py`, accel is **saturated** to ±16 g (`_sat_accel`). **Why:** a real FIFO-IMU chip **saturates** at the ADC rail on an impulse beyond its range (it does not wrap). The P4's `SimulatorMavlink` FIFO emulation was **wrapping** >16 g in the float→int16 conversion (a sign flip) — on a hard touchdown impulse (>16 g) this poisoned the EKF vertical estimate. `_sat_accel` mimics the real-chip saturation in the bridge → wrapping becomes impossible. This is a **faithful fix** (real-hardware behavior); the P4/PX4 code is untouched; the root cause was verified by measurement (2026-07-04). Note also that upstream PX4 `SimulatorMavlink` lacks a `constrain` here = a latent bug.

## Honest caveats (fidelity limits)
- **The 18 g touchdown spike is partly a Gazebo artifact:** rigid contact + a 4 ms physics step, with no landing-gear compliance, render a gentle 0.7 m/s landing as a sharp ~18 g reading. On a real drone the gear/airframe would soften this to ~a few g. `_sat_accel` fixes the catastrophe (the wrap); the impulse itself is a separate Gazebo-fidelity matter.
- **250 Hz bundle optimism:** the bridge sends all sensors at 250 Hz → baro/mag are over-sampled versus their native rates (~50/100 Hz). Since the EKF genuinely gets more measurements, the HIL can be **slightly optimistic** versus real hardware. (PX4's own `gz_bridge` uses per-sensor native-rate callbacks.)

---

<a name="türkçe"></a>
# Türkçe

Bu klasör, gerçek ESP32-P4 kartını Gazebo fizik-simülasyonuna bağlayan **HIL köprülerini** içerir.
P4 üzerinde verbatim-PX4 uçuş yazılımı koşar; sensörler Gazebo'dan gelir, motor-çıkışı Gazebo'ya gider.
QGroundControl yer-istasyonu olarak kullanılır.

> **DÜRÜSTLÜK NOTU:** Bu bir **HIL** kurulumudur — P4'ün hesaplama/zamanlama yeterliliğini kanıtlar.
> **Gerçek sensör sürücüleri ve gerçek ESC/motor KULLANILMAZ** (Gazebo emüle eder). Ayrıntı + çekinceler aşağıda.

## Mimari / Veri Akışı

```
   ┌─────────── Docker: drone_sim_gz (px4io/px4-sitl-gazebo) — host .91 ───────────┐
   │                                                                                │
   │   gz sim (default_gz_only.sdf, x500 quad, fizik 250Hz)                         │
   │        │  gz-transport (IMU/mag/baro/navsat topic'leri)                        │
   │        ▼                                                                        │
   │   hil_sensor_bridge.py ──HIL_SENSOR/HIL_GPS(MAVLink,UDP)──►  P4 :14560         │
   │        ▲                                                       (172.16.16.57)   │
   │        │  ACT-FWD (127.0.0.1:14600)          P4 HIL_ACTUATOR ─┘                 │
   │        │                                                                        │
   │   hil_motor_bridge_robust.py ──gz.msgs.Actuators──► /x500_0/command/motor_speed│
   └────────────────────────────────────────────────────────────────────────────────┘
                                    ▲
                                    │ MAVLink (QGC ↔ P4 mavlink modülü)
                              QGroundControl
```

- **Sensör yönü:** Gazebo → `hil_sensor_bridge.py` → P4 (`simulator_mavlink` modülü sensör-topic'lerini yayınlar → ekf2).
- **Motor yönü:** P4 `control_allocator → actuator_motors → pwm_out_sim → HIL_ACTUATOR` → köprü → Gazebo motorları.
- P4-içi: sensör → ekf2 → mc_pos/att/rate → control_allocator (tam PX4 zinciri, Core0'da).

## Gereksinimler
- **Docker image:** `px4io/px4-sitl-gazebo` (Gazebo Harmonic + x500 modeli + gz-transport python bindings).
- **Python paketleri** (image'da mevcut): `gz.transport13`, `gz.msgs10`, `pymavlink`.
- **P4 firmware:** bu repo (`idf.py flash`), boot'ta `simulator_mavlink start -u 14560` bekler.
- **QGroundControl** (yer istasyonu).
- **Ağ:** P4 ve Docker-host aynı LAN'da. P4 IP'si `hil_sensor_bridge.py`'de `P4_IP` (varsayılan `172.16.16.57:14560`) — kendi P4-IP'nize göre düzenleyin.

## Çalıştırma

**1) Container'ı başlat** (gz sim + world; entrypoint `gz sim ... default_gz_only.sdf` koşar):
```bash
docker start drone_sim_gz          # ya da: docker run ... px4io/px4-sitl-gazebo
# gz world ~25-30s'de hazır olur
```

**2) İki köprüyü başlat** (KRİTİK kalıp — `sh -c "nohup ... &"` DEĞİL; trailing `&` reap eder):
```bash
docker exec -d drone_sim_gz sh -c "export PYTHONPATH=/projects/scripts/pylibs \
  GZ_SIM_RESOURCE_PATH=/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds \
  GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/px4-gazebo/lib/gz/plugins \
  GZ_SIM_SERVER_CONFIG_PATH=/opt/px4-gazebo/share/gz/server.config PX4_SIM_MODEL=gz_x500; \
  nohup python3 /projects/scripts/hil_sensor_bridge.py > /tmp/sensor_bridge.log 2>&1"

docker exec -d drone_sim_gz sh -c "export PYTHONPATH=/projects/scripts/pylibs; \
  nohup python3 /projects/scripts/hil_motor_bridge_robust.py > /tmp/motor_bridge.log 2>&1"
```
Doğrula: `docker exec drone_sim_gz tail -4 /tmp/sensor_bridge.log` → `[SENSOR BRIDGE] imu_sent=N (~250Hz)`.

**3) P4'ü flash'la + boot et** (bu repo). Boot'ta EKF align olur (~30-60s), sonra QGC'de arm'lanabilir.

**4) QGC'yi bağla** → arm → takeoff → uç.

> **Not:** Sim-reset (drone tipe/devirme) sonrası **motor-köprüsünü yeniden başlat** (gz-publisher handle'ı bayatlar). `hil_motor_bridge_robust.py` watchdog'lu (kendini relaunch eder) ama container-restart'ta iki köprüyü de elle başlat.

## `_sat_accel` — HIL-sadakat düzeltmesi (ivme doygunluğu)

`hil_sensor_bridge.py`'de accel ±16g'ye **doyurulur** (`_sat_accel`). **Neden:** gerçek FIFO-IMU çipi menzilini aşan darbede ADC-railine **doyar** (sarmaz). P4'ün `SimulatorMavlink` FIFO-emülasyonu ise float→int16'da >16g'yi **sarıyordu** (işaret-dönmesi) — inişte sert temas darbesinde (>16g) EKF-dikey tahminini zehirliyordu. `_sat_accel` gerçek-çip doygunluğunu köprüde taklit eder → sarma imkânsız. Bu **sadık bir fix'tir** (gerçek-donanım davranışı), P4/PX4 koduna dokunulmaz; kök-neden ölçümle doğrulandı (2026-07-04). Ayrıca upstream PX4 `SimulatorMavlink`'te `constrain` eksikliği = latent bug.

## Dürüst çekinceler (fidelity sınırları)
- **18g temas darbesi kısmen Gazebo-artefaktı:** rijit-temas + 4ms fizik-adımı, iniş-takımı esnekliği olmadığından yumuşak bir 0.7 m/s inişi keskin ~18g gösterir. Gerçek dronda takım/gövde bunu ~birkaç-g'ye yumuşatır. `_sat_accel` katastrofu (sarmayı) çözer; darbenin kendisi ayrı bir Gazebo-fidelity konusu.
- **250Hz-bundle iyimserliği:** köprü tüm sensörleri 250Hz'de gönderir → baro/mag native-hızlarından (~50/100Hz) fazla örneklenir. EKF'ye gerçekten fazla ölçüm verdiğinden HIL, gerçek-donanımdan **birazcık iyimser** olabilir. (PX4'ün kendi `gz_bridge`'i sensör-başına native-rate callback kullanır.)
