# fake_motor_card.py
# Simulateur de carte moteurs pour tester maman sans le vrai hardware.
# Tourne sur la Pi (ou ton PC) et se branche sur un PTY virtuel créé avec socat.
#
# Usage :
#   1) Crée un PTY paire :
#        socat -d -d pty,raw,echo=0,link=/tmp/tty_motor_master \
#                    pty,raw,echo=0,link=/tmp/tty_motor_slave
#   2) Lance maman avec /tmp/tty_motor_master comme port moteurs
#   3) Lance ce script avec /tmp/tty_motor_slave :
#        python3 fake_motor_card.py /tmp/tty_motor_slave

import sys
import time
import math
import serial

if len(sys.argv) < 2:
    print("Usage: fake_motor_card.py <tty>")
    sys.exit(1)

ser = serial.Serial(sys.argv[1], 115200, timeout=0.05)
print(f"[motor-fake] listening on {sys.argv[1]} @115200")

# État simulé
x, y, theta = 1150.0, 800.0, 0.0
target_x, target_y = None, None

V_MAX  = 500.0   # mm/s
EPS    = 30.0    # tolérance arrivée
DT_REF = 0.05    # période physique 50 ms

last_push = 0.0
last_t    = time.time()

while True:
    now = time.time()
    dt  = now - last_t
    last_t = now

    # 1) Lire les commandes maman
    try:
        line = ser.readline().decode(errors='ignore').strip()
    except Exception:
        line = ""
    if line:
        if line.startswith("GOTO"):
            try:
                _, sx, sy = line.split()
                target_x, target_y = float(sx), float(sy)
                print(f"[motor-fake] GOTO -> ({target_x:.0f}, {target_y:.0f})")
            except ValueError:
                ser.write(b"ERR bad_goto\n")
        elif line == "STOP":
            target_x, target_y = None, None
            print("[motor-fake] STOP")
        elif line == "STATUS":
            ser.write(f"POS {x:.1f} {y:.1f} {theta:.4f}\n".encode())

    # 2) Avancer la physique
    if target_x is not None:
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        if dist < EPS:
            x, y = target_x, target_y
            target_x, target_y = None, None
            ser.write(b"DONE\n")
            print(f"[motor-fake] arrivé ({x:.0f}, {y:.0f}) -> DONE")
        else:
            move = min(V_MAX * dt, dist)
            x += move * dx / dist
            y += move * dy / dist
            theta = math.atan2(dy, dx)

    # 3) Push POS toutes les 100 ms
    if now - last_push > 0.1:
        ser.write(f"POS {x:.1f} {y:.1f} {theta:.4f}\n".encode())
        last_push = now

    time.sleep(DT_REF)