#!/usr/bin/env python3
"""
HIL_ACTUATOR_CONTROLS (msg 93, P4 :14600) -> Gazebo /x500_0/command/motor_speed.
ROBUST surum (2026-06-24) — PX4 GZMixingInterfaceESC-sadik, sim-reset'e dayanikli.

NEDEN: eski tek-thread surum sim/model-reset sonrasi BAYAT gz-publisher'da
kilitleniyordu (pub.publish blok -> UDP-recv de donar -> motor relay OLU -> drone kalkmiyor).

ROBUSTLUK:
  - recv (UDP) ve publish (gz) AYRI thread -> gz-publish UDP-recv'i BLOKLAYAMAZ (latest hep taze).
  - watchdog: publish-loop ilerlemezse (publish kilidi) os._exit -> supervisor taze process'i
    yeniden baslatir -> taze gz Node yeni topic'e baglanir (kanitlanmis restart-fix, otomatik).
OLCEK (PX4-sadik, 4001_gz_x500): SIM_GZ_EC_MIN=150 MAX=1000
  -> armed: velocity = 150 + ctrl*850 (ctrl=0'da MIN=150 idle); disarmed: 0 (DIS).
"""
import socket, struct, threading, time, os
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators

P4_LISTEN_PORT = 14600
GZ_TOPIC = "/x500_0/command/motor_speed"
NUM_MOTORS = 4
EC_MIN, EC_MAX = 0.0, 1000.0   # P4 control-alloc MIN=0 varsayar (PX4 MixingOutput MIN P4-te YOK -> 150 takla attirir)
PUB_HZ = 250.0
WATCHDOG_S = 2.0

HIL_AC_MSGID = 93
HIL_AC_PAYLOAD_LEN = 81
HIL_AC_STRUCT = struct.Struct("<QQ16fB")

_lock = threading.Lock()
_latest = {"controls": [0.0]*16, "armed": False, "rx": 0.0}
_pub_hb = [time.time()]

def parse_mavlink_v2(buf):
    if len(buf) < 12 or buf[0] != 0xFD:
        return None
    payload_len = buf[1]
    msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
    if msgid != HIL_AC_MSGID:
        return None
    payload = buf[10:10 + payload_len]
    if len(payload) < HIL_AC_PAYLOAD_LEN:
        return None
    _t, _f, *rest = HIL_AC_STRUCT.unpack(payload[:HIL_AC_STRUCT.size])
    return rest[:16], rest[16]

def recv_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", P4_LISTEN_PORT))
    print(f"[recv] UDP {P4_LISTEN_PORT} dinleniyor", flush=True)
    while True:
        data, _ = s.recvfrom(512)
        r = parse_mavlink_v2(data)
        if r is None:
            continue
        controls, mode = r
        with _lock:
            _latest["controls"] = controls
            _latest["armed"] = bool(mode & 0x80)
            _latest["rx"] = time.time()

def ec_scale(ctrl, armed):
    if not armed:
        return 0.0
    ctrl = 0.0 if ctrl < 0.0 else (1.0 if ctrl > 1.0 else ctrl)
    return EC_MIN + ctrl * (EC_MAX - EC_MIN)

def publish_thread():
    node = Node()
    pub = node.advertise(GZ_TOPIC, Actuators)
    period = 1.0 / PUB_HZ
    count = 0
    while True:
        with _lock:
            controls, armed = _latest["controls"], _latest["armed"]
        msg = Actuators()
        for i in range(NUM_MOTORS):
            msg.velocity.append(ec_scale(controls[i], armed))
        try:
            pub.publish(msg)
        except Exception as e:
            print(f"[pub] publish hata: {e}", flush=True)
        _pub_hb[0] = time.time()
        count += 1
        if count <= 5 or count % 1000 == 0:
            print(f"[pub] #{count} armed={armed} vel={[round(v,1) for v in msg.velocity]}", flush=True)
        time.sleep(period)

def watchdog_thread():
    while True:
        time.sleep(WATCHDOG_S)
        if time.time() - _pub_hb[0] > WATCHDOG_S * 2:
            print("[wd] publish-loop kilitlendi -> process exit (supervisor relaunch)", flush=True)
            os._exit(2)

def main():
    threading.Thread(target=recv_thread, daemon=True).start()
    threading.Thread(target=watchdog_thread, daemon=True).start()
    publish_thread()

if __name__ == "__main__":
    main()
