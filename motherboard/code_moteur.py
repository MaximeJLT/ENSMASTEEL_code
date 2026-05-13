import rp2
import time
import qwiic_otos
from machine import UART, Pin, StateMachine
import uasyncio as asyncio
import math

#================================SETUP CONSTANTE================================#
Kp = 1
Ki = 1
Kd = 1
Rr = 29
dr = 1
f_max = 5000
F_MIN = 2
rad_per_step = 2 * math.pi / 200

#================================SETUP VARIABLE================================#
consigne_xyO = [0, 0, 0]
v_robot = [0, 0, 0]
v_pas = [0, 0, 0]
dt = 0.1
freq_1 = 1000
freq_2 = 1000
freq_3 = 1000
erreur_prec_x = 0
erreur_prec_y = 0
erreur_prec_angle = 0
integrale_x = 0
integrale_y = 0
integrale_angle = 0
Position = None
mission_active = False

#================================SETUP CONNEXION UART================================#
uart0 = UART(0, baudrate=115200, tx=Pin(6), rx=Pin(7))

#================================SETUP STEPPER FONCTION================================#
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
    wrap_target()
    set(pins, 1)
    nop()[31]
    set(pins, 0)
    nop()[31]
    wrap()

#================================SETUP STEPPER================================#
STEP_PIN_1 = 20
DIR_PIN_1 = 19
STEP_PIN_2 = 18
DIR_PIN_2 = 17
STEP_PIN_3 = 16
DIR_PIN_3 = 13

dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)
en_pin.value(0)

sm_1 = StateMachine(1, stepper, freq_1, set_base=Pin(STEP_PIN_1))
sm_2 = StateMachine(2, stepper, freq_2, set_base=Pin(STEP_PIN_2))
sm_3 = StateMachine(3, stepper, freq_3, set_base=Pin(STEP_PIN_3))

#================================SETUP ODOMETRE================================#
myOtos = qwiic_otos.QwiicOTOS()
myOtos.begin()
time.sleep(1)

myOtos.calibrateImu()
myOtos.setLinearScalar(1.0)
myOtos.setAngularScalar(1.0)
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)
myOtos.resetTracking()

Position = myOtos.getPosition()

#================================SETUP FONCTIONS================================#
def Odometre():
    global Position
    Position = myOtos.getPosition()
    return Position

def read_line():
    if uart0.any():
        line = uart0.readline()
        if line:
            return line.decode().strip()
    return None

def consigne():
    global consigne_xyO
    global mission_active

    cmd = read_line()
    if not cmd:
        return

    parts = cmd.split()

    if parts[0] == "GOTO" and len(parts) >= 3:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3]) if len(parts) > 3 else 0.0
            consigne_xyO = [x, y, h]
            mission_active = True
        except ValueError:
            pass

    elif parts[0] == "STOP":
        consigne_xyO = [0.0, 0.0, 0.0]
        mission_active = False
        sm_1.active(0)
        sm_2.active(0)
        sm_3.active(0)

def objectif_atteint(pos, cons):
    tol_xy = 5
    tol_angle = 2
    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)
    return dx < tol_xy and dy < tol_xy and dh < tol_angle

def send_pos(Pos):
    msg = f"POS {Pos.x:.2f} {Pos.y:.2f} {Pos.h:.3f}\n"
    uart0.write(msg)

def send_done():
    uart0.write("DONE\n")

def PID(pos, cons):
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    output = [0, 0, 0]
    erreur_x = cons[0] - pos.x
    erreur_y = cons[1] - pos.y
    erreur_angle = cons[2] - pos.h

    P_x = Kp * erreur_x
    P_y = Kp * erreur_y
    P_angle = Kp * erreur_angle

    integrale_x += erreur_x * dt
    integrale_y += erreur_y * dt
    integrale_angle += erreur_angle * dt

    I_x = Ki * integrale_x
    I_y = Ki * integrale_y
    I_angle = Ki * integrale_angle

    D_x = Kd * (erreur_x - erreur_prec_x) / dt
    D_y = Kd * (erreur_y - erreur_prec_y) / dt
    D_angle = Kd * (erreur_angle - erreur_prec_angle) / dt

    erreur_prec_x = erreur_x
    erreur_prec_y = erreur_y
    erreur_prec_angle = erreur_angle

    output[0] = P_x + I_x + D_x
    output[1] = P_y + I_y + D_y
    output[2] = P_angle + I_angle + D_angle

    return output

def calcul_vitesse_en_pas(v):
    vx = v[0]
    vy = v[1]
    w = v[2]

    Va = 0.5 * vx - (math.sqrt(3)/2) * vy - dr * w
    Vb = 0.5 * vx + (math.sqrt(3)/2) * vy - dr * w
    Vc = -vx - dr * w

    Wa = Va / Rr
    Wb = Vb / Rr
    Wc = Vc / Rr

    pas_a = Wa / rad_per_step
    pas_b = Wb / rad_per_step
    pas_c = Wc / rad_per_step

    return [pas_a, pas_b, pas_c]

def frequence_def(v_pas):
    Va = abs(v_pas[0])
    Vb = abs(v_pas[1])
    Vc = abs(v_pas[2])

    max_val = max(Va, Vb, Vc)

    if max_val > f_max:
        scale = f_max / max_val
        Va *= scale
        Vb *= scale
        Vc *= scale

    return Va, Vb, Vc

def update_motor_directions_and_freq(v_pas):
    Va, Vb, Vc = v_pas

    if Va >= 0:
        dir_pin_1.value(1)
    else:
        dir_pin_1.value(0)
    f1 = abs(Va)

    if Vb >= 0:
        dir_pin_2.value(1)
    else:
        dir_pin_2.value(0)
    f2 = abs(Vb)

    if Vc >= 0:
        dir_pin_3.value(1)
    else:
        dir_pin_3.value(0)
    f3 = abs(Vc)

    return f1, f2, f3

#================================LOOP PRINCIPALE================================#
async def Task_Odo():
    global Position
    while True:
        Position = myOtos.getPosition()
        await asyncio.sleep(0.05)

async def Task_UART():
    while True:
        consigne()
        await asyncio.sleep(0.05)

async def Task_Control():
    global v_robot, v_pas
    while True:
        if Position is not None:
            v_robot = PID(Position, consigne_xyO)
            v_pas = calcul_vitesse_en_pas(v_robot)
            f1, f2, f3 = update_motor_directions_and_freq(v_pas)
            f1, f2, f3 = frequence_def([f1, f2, f3])
            sm_1.freq(int(f1) if f1 > 0 else 1)
            sm_2.freq(int(f2) if f2 > 0 else 1)
            sm_3.freq(int(f3) if f3 > 0 else 1)

            for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
                if f < F_MIN:
                    sm.active(0)
                else:
                    sm.freq(int(f))
                    sm.active(1)

        await asyncio.sleep(0.01)

async def Task_Telemetry():
    global mission_active
    while True:
        # Toujours envoyer POS, maman a besoin de la position en continu
        if Position is not None:
            send_pos(Position)

        # Envoyer DONE une seule fois à l'arrivée
        if mission_active and Position is not None:
            if objectif_atteint(Position, consigne_xyO):
                send_done()
                mission_active = False

        await asyncio.sleep(0.1)

async def main():
    asyncio.create_task(Task_Odo())
    asyncio.create_task(Task_UART())
    asyncio.create_task(Task_Control())
    asyncio.create_task(Task_Telemetry())

    while True:
        await asyncio.sleep(1)

asyncio.run(main())
