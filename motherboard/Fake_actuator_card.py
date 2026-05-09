# fake_actuator_card.py
# Simulateur de carte actionneurs (servos / bras) pour tester maman.
#
# Usage :
#   socat -d -d pty,raw,echo=0,link=/tmp/tty_actuator_master \
#               pty,raw,echo=0,link=/tmp/tty_actuator_slave
#   python3 fake_actuator_card.py /tmp/tty_actuator_slave

import sys
import time
import serial

if len(sys.argv) < 2:
    print("Usage: fake_actuator_card.py <tty>")
    sys.exit(1)

ser = serial.Serial(sys.argv[1], 115200, timeout=0.05)
print(f"[act-fake] listening on {sys.argv[1]} @115200")

PICK_DUR = 1.2
DROP_DUR = 0.9

state    = "idle"   # idle | picking | dropping
timer    = 0.0
last_t   = time.time()

while True:
    now = time.time()
    dt  = now - last_t
    last_t = now

    # Commandes maman
    try:
        line = ser.readline().decode(errors='ignore').strip()
    except Exception:
        line = ""
    if line:
        if line == "PICK" and state == "idle":
            state = "picking"
            timer = PICK_DUR
            ser.write(b"STATUS picking\n")
            print("[act-fake] PICK -> picking")
        elif line == "DROP" and state == "idle":
            state = "dropping"
            timer = DROP_DUR
            ser.write(b"STATUS dropping\n")
            print("[act-fake] DROP -> dropping")
        elif line == "STATUS":
            ser.write(f"STATUS {state}\n".encode())

    # Timer
    if state in ("picking", "dropping"):
        timer -= dt
        if timer <= 0:
            print(f"[act-fake] {state} terminé -> DONE")
            state = "idle"
            ser.write(b"DONE\n")

    time.sleep(0.05)