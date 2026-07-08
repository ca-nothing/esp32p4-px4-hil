#!/usr/bin/env python3
"""capture_boot + DONANIM RESET: portu ac, ESP32 auto-reset (EN puls) ile TAZE boot
zorla, N saniye oku, dosyaya yaz. Kullanim: python capture_boot_reset.py COM3 115200 80 out.txt
RTS->EN(reset), DTR->IO0(boot-sec). run-mode reset = DTR False (IO0 high) + RTS puls."""
import sys, time
import serial

port = sys.argv[1]; baud = int(sys.argv[2]); secs = float(sys.argv[3]); out = sys.argv[4]

s = serial.Serial(port, baud, timeout=1)
# ESP32 hard-reset to RUN mode: IO0 high (normal boot), pulse EN low->high.
s.dtr = False     # IO0 = HIGH (normal boot, not download)
s.rts = True      # EN = LOW (chip held in reset)
time.sleep(0.15)
s.rts = False     # EN = HIGH (release -> app boots fresh)

end = time.time() + secs
n = 0
try:
    with open(out, "wb") as f:
        while time.time() < end:
            data = s.read(4096)
            if data:
                f.write(data); f.flush(); n += len(data)
    print(f"CAPTURE_DONE bytes={n} file={out}")
finally:
    s.close()
