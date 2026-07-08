#!/usr/bin/env python3
"""Minimal watchdog'lu seri yakalama: COM portu N saniye oku, dosyaya yaz, temiz cik.
Kullanim: python capture_boot.py COM3 115200 45 serialcap_takeoffbc.txt
Zombie-onler: kesin sure sonunda port kapanir + process exit (COM3 kilidi birakir)."""
import sys, time
import serial  # IDF python env (esptool ile gelir)

port = sys.argv[1]
baud = int(sys.argv[2])
secs = float(sys.argv[3])
out = sys.argv[4]

end = time.time() + secs
n = 0
try:
    with serial.Serial(port, baud, timeout=1) as s, open(out, "wb") as f:
        while time.time() < end:
            data = s.read(4096)
            if data:
                f.write(data)
                f.flush()
                n += len(data)
    print(f"CAPTURE_DONE bytes={n} file={out}")
except Exception as e:
    print(f"CAPTURE_ERROR {e}")
    sys.exit(1)
