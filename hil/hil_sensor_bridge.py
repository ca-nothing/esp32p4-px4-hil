#!/usr/bin/env python3
"""
Yol B — PX4'suz Gazebo Sensor -> MAVLink -> ESP32-P4 Koprusu.

Gazebo Harmonic sensor topic'lerini (gz-transport) okuyup
MAVLink v2 HIL_SENSOR ve HIL_GPS mesajlarina cevirip
UDP ile P4'e (172.16.16.57:14560) gonderir.

Calistirma (container icinde):
  docker exec drone_sim_gz python3 /projects/scripts/hil_sensor_bridge.py

=== EVENT-DRIVEN FIX (2026-06-27) ===
ESKI TASARIM: gz callback'leri SADECE _latest_* state'ini doldururdu; ayri bir
sIKI main-loop (while True + time.sleep(0.0001)) state'i POLL'leyip gonderirdi.
SORUN: sIKI main-loop GIL'i tutar -> gz-transport callback-thread'i GIL'e ulasamaz
-> DUSUK-hizli NavSat callback'i AC KALIR (yuksek-hizli IMU hayatta kalir).
KANIT: standalone Python subscriber (meWgul-loop YOK) navsat'i 30Hz/imu'yu 250Hz
steady alir; bridge ise navsat'i 5-29s araliklarla alirdi -> ayni bayat _latest_gps
tekrar gonderilir -> P4 EKF GPS-timeout -> fake-pos -> xy_valid=0 -> takeoff RED.
FIX: send'leri DOGRUDAN gz callback'lerinden yap (gz-rate'inde, callback-thread'inde);
main-thread'i BLOCKING recv'e cevir (ACT-FWD). Blocking recv GIL'i BIRAKIR -> callback'ler
tam-hizda kosar. Boylece IMU 250Hz + GPS ~10Hz P4'e SUREKLI akar.
"""

import socket
import math
import time
import threading

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.navsat_pb2 import NavSat

# pymavlink (container icinde venv'e kurulacak)
from pymavlink.dialects.v20 import common as mavlink2

# ============================================================
# AYARLAR
# ============================================================
P4_IP = "172.16.16.57"
P4_PORT = 14560

# Gazebo topic'leri (canli dogrulandi 2026-06-14)
IMU_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu"
MAG_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer"
BARO_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure"
GPS_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat"

# GPS gonderim hizi: gz navsat ~30Hz -> her N navsat callback'inde bir HIL_GPS (>=1).
# 3 -> ~10Hz GPS (PX4 tipik GPS hizi; EKF buffer'lar).
GPS_DECIMATE = 3

# MAVLink HIGHRES_IMU fields_updated bitmask
HIGHRES_IMU_FIELDS = (
    (1 << 0)   # xacc
    | (1 << 1)   # yacc
    | (1 << 2)   # zacc
    | (1 << 3)   # xgyro
    | (1 << 4)   # ygyro
    | (1 << 5)   # zgyro
    | (1 << 6)   # xmag
    | (1 << 7)   # ymag
    | (1 << 8)   # zmag
    | (1 << 9)   # abs_pressure
    | (1 << 11)  # pressure_alt
    | (1 << 12)  # temperature
)

# ============================================================
# PAYLASILAN DURUM
# ============================================================
_state_lock = threading.Lock()
_latest_mag = None       # (time_us, mx, my, mz) — gauss
_latest_baro = None      # (time_us, pressure_pa)

_sock = None             # main()'de kurulur; callback'ler dogrudan gonderir

# sayaclar / hz-log
_imu_sent = 0
_gps_sent = 0
_navsat_cb = 0
_hz_count = 0
_hz_t0 = time.time()


# ============================================================
# BIRIM DONUSUMLERI  (verbatim)
# ============================================================

def pressure_to_altitude(pressure_pa):
    """Basinc (Pascal) -> barometrik irtifa (metre)."""
    if pressure_pa <= 0:
        return 0.0
    return 44330.0 * (1.0 - math.pow(pressure_pa / 101325.0, 0.1903))


def deg_to_degE7(deg):
    return int(round(deg * 1e7))


def m_to_mm(m):
    return int(round(m * 1000.0))


def mps_to_cms(mps):
    return int(round(mps * 100.0))


def ned_ground_cog(vn, ve):
    gs = math.sqrt(vn * vn + ve * ve)
    cog = math.degrees(math.atan2(ve, vn))
    if cog < 0:
        cog += 360.0
    return mps_to_cms(gs), int(round(cog * 100.0))


def flu_to_frd(x, y, z):
    return (x, -y, -z)


# ============================================================
# MAVLink MESAJ KURMA VE GONDERME  (verbatim govde)
# ============================================================

# ACCEL_SATURATE (2026-07-04 KALICI HIL-sadakat fix): gercek FIFO-IMU cipi gibi ivmeyi menzilde
# (+/-16g) DOYUR (int16 raili) -- SARMA yerine. Upstream SimulatorMavlink FIFO float->int16'da
# constrain YOK (e24cda2 + main latent-bug): >16g temas darbesi sarip isaret-donduruyordu
# (-18.7g up -> +13.3g down sahte-dalis) -> EKF-zehir -> kontrol-guDUMLU pogo (inis-yukseklik bug'i).
# Gercek cip ADC-railine doyar + clip-flag set eder; bu onu koprude TAKLIT eder. P4/PX4 verbatim
# DOKUNULMADI. Dogrulama: over16g 9->1, EKF-z +3->0. GERI-AL: cp .pre_accelprobe hil_sensor_bridge.py
_ACC_G = 9.80665
_ACC_SCALE = _ACC_G / 2048.0          # SimulatorMavlink ACCEL_FIFO_SCALE ile AYNI
_ACC_HI =  32766.0 * _ACC_SCALE       # +count: clip-flag ateşler (>=32766), +32768 sarma YOK
_ACC_LO = -32768.0 * _ACC_SCALE       # -count: INT16_MIN raili -> clip-flag ateşler
def _sat_accel(v):
    if v > _ACC_HI: return _ACC_HI
    if v < _ACC_LO: return _ACC_LO
    return v


def send_highres_imu(sock, imu, mag, baro):
    """En guncel IMU + mag + baro -> HIL_SENSOR gonder."""
    if imu is None:
        return

    t_us, ax, ay, az, gx, gy, gz = imu

    ax, ay, az = flu_to_frd(ax, ay, az)
    ax = _sat_accel(ax); ay = _sat_accel(ay); az = _sat_accel(az)   # HIL-sadakat: gercek FIFO-IMU cip doygunlugu (int16 sarma yerine)
    gx, gy, gz = flu_to_frd(gx, gy, gz)

    mx, my, mz = 0.0, 0.0, 0.0
    if mag is not None:
        _, mx_raw, my_raw, mz_raw = mag
        mx, my, mz = (-my_raw, -mx_raw, mz_raw)  # PX4 gz_bridge verbatim x=-y y=-x z=+z

    pressure_alt = 0.0
    temperature = 0.0
    abs_pressure_hpa = 0.0
    if baro is not None:
        _, pressure_pa = baro
        pressure_alt = pressure_to_altitude(pressure_pa)
        abs_pressure_hpa = pressure_pa / 100.0

    msg = mavlink2.MAVLink_hil_sensor_message(
        time_usec=t_us,
        xacc=ax, yacc=ay, zacc=az,
        xgyro=gx, ygyro=gy, zgyro=gz,
        xmag=mx, ymag=my, zmag=mz,
        abs_pressure=abs_pressure_hpa,
        diff_pressure=0.0,
        pressure_alt=pressure_alt,
        temperature=temperature,
        fields_updated=0x1BFF,
        id=0,
    )
    buf = msg.pack(mavlink2.MAVLink(0, 0))
    sock.sendto(buf, (P4_IP, P4_PORT))


def send_hil_gps(sock, gps):
    """GPS verisi -> HIL_GPS (#113) gonder. NED vn/ve/vd tasir."""
    if gps is None:
        return

    t_us, lat, lon, alt, vn, ve, vup = gps

    vd = -vup  # NED down = -(up)
    vel_cms, cog_cdeg = ned_ground_cog(vn, ve)

    msg = mavlink2.MAVLink_hil_gps_message(
        time_usec=t_us,
        fix_type=3,  # 3D fix
        lat=deg_to_degE7(lat),
        lon=deg_to_degE7(lon),
        alt=m_to_mm(alt),
        eph=30,
        epv=40,
        vel=vel_cms,
        vn=mps_to_cms(vn),
        ve=mps_to_cms(ve),
        vd=mps_to_cms(vd),
        cog=cog_cdeg,
        satellites_visible=12,
        id=0,
        yaw=0,
    )
    buf = msg.pack(mavlink2.MAVLink(0, 0))
    sock.sendto(buf, (P4_IP, P4_PORT))


# ============================================================
# GAZEBO CALLBACK'LER — ARTIK DOGRUDAN GONDERIR (event-driven)
# ============================================================

def imu_callback(msg):
    global _imu_sent, _hz_count, _hz_t0
    t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
    imu = (
        t_us,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    )
    with _state_lock:
        mag = _latest_mag
        baro = _latest_baro
    send_highres_imu(_sock, imu, mag, baro)   # HER IMU (gz ~250Hz) -> HIL_SENSOR
    _imu_sent += 1

    # Hz log (her ~10s)
    _hz_count += 1
    if time.time() - _hz_t0 >= 10.0:
        dt = time.time() - _hz_t0
        print("[SENSOR BRIDGE] imu_sent=%d (~%.0fHz) gps_sent=%d (~%.0fHz) navsat_cb=%d/10s"
              % (_imu_sent, _hz_count / dt, _gps_sent, _gps_sent / dt, _navsat_cb), flush=True)
        _hz_count = 0
        _hz_t0 = time.time()


def mag_callback(msg):
    global _latest_mag
    t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
    with _state_lock:
        _latest_mag = (t_us, msg.field_tesla.x, msg.field_tesla.y, msg.field_tesla.z)


def baro_callback(msg):
    global _latest_baro
    t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
    with _state_lock:
        _latest_baro = (t_us, msg.pressure)


def navsat_callback(msg):
    global _navsat_cb, _gps_sent
    _navsat_cb += 1
    if _navsat_cb % GPS_DECIMATE != 0:
        return
    t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
    gps = (
        t_us,
        msg.latitude_deg,
        msg.longitude_deg,
        msg.altitude,
        msg.velocity_north,
        msg.velocity_east,
        msg.velocity_up,
    )
    send_hil_gps(_sock, gps)   # gz navsat-rate / DECIMATE -> ~10Hz FRESH HIL_GPS
    _gps_sent += 1


# ============================================================
# ANA DONGU — sadece ACT-FWD (BLOCKING recv -> GIL'i birakir)
# ============================================================

def main():
    global _sock
    print("=== Yol B — Sensor Koprusu (EVENT-DRIVEN, GIL-safe) ===", flush=True)
    print("Gazebo sensorleri -> MAVLink -> P4 (%s:%d)" % (P4_IP, P4_PORT), flush=True)
    print("Send'ler gz-callback'lerinden (IMU her cagri, GPS her %d navsat)." % GPS_DECIMATE, flush=True)

    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
    # ACT-FWD: P4'un HIL_ACTUATOR reply'lerini al -> motor_bridge:14600'e forward.
    # settimeout -> blocking recv (GIL'i birakir; bos iken 1s'de bir uyanir).
    _sock.settimeout(1.0)

    node = Node()
    node.subscribe(IMU, IMU_TOPIC, imu_callback)
    node.subscribe(Magnetometer, MAG_TOPIC, mag_callback)
    node.subscribe(FluidPressure, BARO_TOPIC, baro_callback)
    node.subscribe(NavSat, GPS_TOPIC, navsat_callback)

    print("Abone olundu (IMU/Mag/Baro/NavSat). Kopru basladi (event-driven).", flush=True)

    _act_fwd = 0
    while True:
        try:
            d, _ = _sock.recvfrom(512)
            _sock.sendto(d, ("127.0.0.1", 14600))   # ACT-FWD -> motor bridge
            _act_fwd += 1
            if _act_fwd % 500 == 1:
                print("[ACT-FWD] HIL_ACTUATOR reply -> 14600, total %d" % _act_fwd, flush=True)
        except socket.timeout:
            pass
        except OSError:
            pass


if __name__ == "__main__":
    main()
