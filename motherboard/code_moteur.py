import rp2
import time
import qwiic_otos
from machine import UART, Pin, StateMachine
import uasyncio as asyncio
import math

#================================SETUP CONSTANTE================================#
# Gains PID
Kp = 1
Ki = 1
Kd = 1

# Géométrie robot
Rr = 0.029    # rayon roue en mètres
dr = 0.0155   # distance centre robot / roue en mètres
f_max = 5000  # Hz max steppers
F_MIN = 2     # Hz min steppers
rad_per_step = 2 * math.pi / 200

# Gains capteur OTOS (calibrés à 1 par défaut)
gainL = 1.0
gainA = 1.0

# Distance de référence pour la calibration linéaire (en mètres)
ref_distance_m = 0.2   # 200 mm théoriques par cycle de calibration

#================================SETUP VARIABLE================================#
consigne_xyO = [0.0, 0.0, 0.0]   # cible (x, y, theta) en mètres / radians
v_robot = [0.0, 0.0, 0.0]
v_pas = [0.0, 0.0, 0.0]
dt = 0.01  # 10 ms

freq_1 = 1000
freq_2 = 1000
freq_3 = 1000

erreur_prec_x = 0.0
erreur_prec_y = 0.0
erreur_prec_angle = 0.0
integrale_x = 0.0
integrale_y = 0.0
integrale_angle = 0.0

Position = None
mission_active = False

#================================SETUP CONNEXION UART================================#
uart0 = UART(0, baudrate=115200, tx=Pin(6), rx=Pin(7))

#================================SETUP STEPPER PIO================================#
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
    wrap_target()
    set(pins, 1)
    nop()[31]
    set(pins, 0)
    nop()[31]
    wrap()

#================================SETUP STEPPER================================#
# Moteur 1 (label 1) → broches reel (18, 17)
STEP_PIN_1 = 18
DIR_PIN_1 = 17

# Moteur 2 (label 2) → broches reel (16, 13)
STEP_PIN_2 = 16
DIR_PIN_2 = 13

# Moteur 3 (label 3) → broches reel (20, 19)
STEP_PIN_3 = 20
DIR_PIN_3 = 19

dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)
en_pin.value(0)   # active les drivers

sm_1 = StateMachine(1, stepper, freq_1, set_base=Pin(STEP_PIN_1))
sm_2 = StateMachine(2, stepper, freq_2, set_base=Pin(STEP_PIN_2))
sm_3 = StateMachine(3, stepper, freq_3, set_base=Pin(STEP_PIN_3))

#================================SETUP ODOMETRE================================#
myOtos = qwiic_otos.QwiicOTOS()
myOtos.begin()
time.sleep(1)

myOtos.calibrateImu()
myOtos.setLinearScalar(gainL)
myOtos.setAngularScalar(gainA)
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)
myOtos.resetTracking()

Position = myOtos.getPosition()

#================================FONCTIONS================================#
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
    """
    Lit une commande sur l'UART et met à jour la consigne.
    
    Format attendu : 'GOTO x_mm y_mm theta_rad' (maman envoie en mm + radians)
    On convertit en mètres pour cohérence avec OTOS.
    """
    global consigne_xyO
    global mission_active

    cmd = read_line()
    if not cmd:
        return

    parts = cmd.split()

    if parts[0] == "GOTO" and len(parts) >= 3:
        try:
            # Maman envoie x et y en MILLIMÈTRES, on convertit en mètres
            x_m = float(parts[1]) / 1000.0
            y_m = float(parts[2]) / 1000.0
            # theta déjà en radians
            h_rad = float(parts[3]) if len(parts) > 3 else 0.0
            consigne_xyO = [x_m, y_m, h_rad]
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
    """
    Tolérance : 10 mm = 0.01 m, 3° = ~0.052 rad
    """
    tol_xy = 0.01                  # 10 mm en mètres
    tol_angle = math.radians(3)    # 3° en radians

    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)

    return dx < tol_xy and dy < tol_xy and dh < tol_angle


def send_pos(Pos):
    """
    Envoie la position à maman en MILLIMÈTRES (pour cohérence avec son format).
    """
    if Pos is None:
        return
    # OTOS donne en mètres → on convertit en mm pour maman
    x_mm = Pos.x * 1000.0
    y_mm = Pos.y * 1000.0
    msg = "POS {:.2f} {:.2f} {:.3f}\n".format(x_mm, y_mm, Pos.h)
    uart0.write(msg)


def send_done():
    uart0.write("DONE\n")


def PID(pos, cons):
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    output = [0.0, 0.0, 0.0]

    erreur_x = cons[0] - pos.x
    erreur_y = cons[1] - pos.y
    erreur_angle = cons[2] - pos.h

    # --- Proportionnel ---
    P_x = Kp * erreur_x
    P_y = Kp * erreur_y
    P_angle = Kp * erreur_angle

    # --- Intégral ---
    integrale_x += erreur_x * dt
    integrale_y += erreur_y * dt
    integrale_angle += erreur_angle * dt

    I_x = Ki * integrale_x
    I_y = Ki * integrale_y
    I_angle = Ki * integrale_angle

    # --- Dérivé ---
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
    """
    v = [vx, vy, w] en m/s et rad/s
    Retourne [Va, Vb, Vc] en pas/s pour 3 roues holonomes à 120°
    """
    vx = v[0]
    vy = v[1]
    w = v[2]

    # Cinématique inverse holonome 3 roues
    Va = 0.5 * vx - (math.sqrt(3) / 2) * vy - dr * w
    Vb = 0.5 * vx + (math.sqrt(3) / 2) * vy - dr * w
    Vc = -vx - dr * w

    # m/s → rad/s roue
    Wa = Va / Rr
    Wb = Vb / Rr
    Wc = Vc / Rr

    # rad/s → pas/s
    pas_a = Wa / rad_per_step
    pas_b = Wb / rad_per_step
    pas_c = Wc / rad_per_step

    return [pas_a, pas_b, pas_c]


def frequence_def(v_pas):
    """
    v_pas : [Va, Vb, Vc] en pas/s
    Retourne (freq1, freq2, freq3) en Hz absolus, saturés à f_max.
    """
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
    """
    v_pas : [Va, Vb, Vc] en pas/s signés
    Met à jour les pins de direction, retourne (f1, f2, f3) absolus.
    """
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


def calibration_odo():
    """
    Calibration des gains du capteur OTOS.
    
    Phase angulaire : on fait tourner le robot de 10 tours et on mesure
    l'écart entre 3600° théorique et ce qu'OTOS rapporte.
    
    Phase linéaire : on fait 10 aller-retour de 200 mm chacun, on calcule
    le gain pour que la mesure corresponde à la consigne.
    """
    global gainL, gainA

    # ===== Phase angulaire =====
    conO = [0.0, 0.0, math.radians(3600)]   # 10 tours en radians
    pos_calib = myOtos.getPosition()

    while not objectif_atteint(pos_calib, conO):
        v_robot_local = PID(pos_calib, conO)
        v_pas_local = calcul_vitesse_en_pas(v_robot_local)
        f1, f2, f3 = update_motor_directions_and_freq(v_pas_local)
        f1, f2, f3 = frequence_def([f1, f2, f3])

        for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
            if f < F_MIN:
                sm.active(0)
            else:
                sm.freq(int(f))
                sm.active(1)

        pos_calib = myOtos.getPosition()
        time.sleep(0.05)

    sm_1.active(0)
    sm_2.active(0)
    sm_3.active(0)

    theta_mesure = myOtos.getPosition().h
    if theta_mesure != 0:
        gainA = math.radians(3600) / theta_mesure

    # ===== Phase linéaire =====
    Liste_calL = []
    vpp = [0.2, 0.2, 0.0]    # vitesse aller (m/s)
    vpm = [-0.2, -0.2, 0.0]  # vitesse retour (m/s)
    y_prev = myOtos.getPosition().y

    for i in range(10):
        v_local = vpp if (i % 2 == 0) else vpm
        v_pas_local = calcul_vitesse_en_pas(v_local)
        f1, f2, f3 = update_motor_directions_and_freq(v_pas_local)
        f1, f2, f3 = frequence_def([f1, f2, f3])

        for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
            if f < F_MIN:
                sm.active(0)
            else:
                sm.freq(int(f))
                sm.active(1)

        # On laisse tourner pendant 1 seconde (à ajuster selon ton hardware)
        time.sleep(1.0)

        sm_1.active(0)
        sm_2.active(0)
        sm_3.active(0)

        Liste_calL.append(abs(y_prev - myOtos.getPosition().y))
        y_prev = myOtos.getPosition().y

    moyenne = sum(Liste_calL) / len(Liste_calL)
    if moyenne > 0:
        gainL = ref_distance_m / moyenne

    myOtos.setLinearScalar(gainL)
    myOtos.setAngularScalar(gainA)


def calibration_msg():
    """Lance la calibration si on reçoit la commande 'CALIBRATION' sur l'UART."""
    cmd = read_line()
    if not cmd:
        return

    parts = cmd.split()
    if parts[0] == "CALIBRATION":
        calibration_odo()
        sm_1.active(0)
        sm_2.active(0)
        sm_3.active(0)


#================================LOOP PRINCIPALE================================#
async def Task_Odo():
    global Position
    while True:
        Position = myOtos.getPosition()
        # NOTE : on laisse OTOS gérer la convention d'angle nativement
        # (radians, [-π, π]). Pas de conversion artificielle.
        await asyncio.sleep(0.05)


async def Task_CALIB():
    while True:
        calibration_msg()
        await asyncio.sleep(1)


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

            for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
                if f < F_MIN:
                    sm.active(0)
                else:
                    sm.freq(int(f))   # int obligatoire pour MicroPython
                    sm.active(1)

        await asyncio.sleep(0.01)


async def Task_Telemetry():
    """
    Envoie TOUJOURS la position à maman (pour que maman puisse alimenter
    le robot_state envoyé à la Jetson).
    En plus, envoie DONE une seule fois à l'arrivée.
    """
    global mission_active
    while True:
        # Toujours envoyer POS
        if Position is not None:
            send_pos(Position)

        # En plus, signaler l'arrivée
        if mission_active and Position is not None:
            if objectif_atteint(Position, consigne_xyO):
                send_done()
                mission_active = False
                sm_1.active(0)
                sm_2.active(0)
                sm_3.active(0)

        await asyncio.sleep(0.1)


async def main():
    asyncio.create_task(Task_CALIB())
    asyncio.create_task(Task_Odo())
    asyncio.create_task(Task_UART())
    asyncio.create_task(Task_Control())
    asyncio.create_task(Task_Telemetry())

    while True:
        await asyncio.sleep(1)


asyncio.run(main())
