import rp2
import time as t
import qwiic_otos
from machine import UART, Pin
from rp2 import StateMachine
import uasyncio as asyncio
import math

from machine import I2C

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

print(i2c.scan())

#================================ SETUP CONSTANTES ================================#
# Gains PID
Kp = 1.0
Ki = 0.1
Kd = 0.05

# Paramètres mécaniques du robot
Rr = 0.029   # Rayon des roues (m)
dr = 0.0155  # Distance entre centre du robot et roue (m)

# Limites des moteurs
f_max = 5000  # Fréquence max du signal STEP (Hz)
F_MIN = 2     # Fréquence minimale pour bouger (Hz)

# Fréquence PIO de base
# Le programme PIO fait 4 cycles par période (set+nop+set+nop)
# donc freq_pio = f_step * 4
# On choisit une base haute et on scale via sm.freq()
PIO_CYCLES_PER_STEP = 4
PIO_BASE_FREQ = f_max * PIO_CYCLES_PER_STEP  # 20 000 Hz

# Conversion radian → pas moteur
rad_per_step = 2 * math.pi / 200  # 200 pas par tour moteur

# Gains d'odométrie (calibrés via CALIBRATION)
gainL = 1.0
gainA = 1.0

#================================ VARIABLES GLOBALES ================================#
# Consigne [x, y, angle] en MÈTRES et RADIANS
consigne_xyO = [0.0, 0.0, 0.0]

# Vitesse robot (vx, vy, w)
v_robot = [0.0, 0.0, 0.0]

# Vitesse en pas moteur
v_pas = [0.0, 0.0, 0.0]

# Période PID (s)
dt = 0.01

# États PID
erreur_prec_x     = 0.0
erreur_prec_y     = 0.0
erreur_prec_angle = 0.0
integrale_x       = 0.0
integrale_y       = 0.0
integrale_angle   = 0.0

# Position odométrique courante
Position = None

# Indique si une mission GOTO est en cours
mission_active = False

# Flag : demande de calibration (posé par Task_UART, exécuté par Task_Calib)
calibration_requested = False

#================================ ANGLE CUMULATIF (UNWRAP) ================================#
angle_cumul = 0.0
angle_prec  = None

# Verrou système : empêche le PID de tourner pendant la calibration
system_locked = False

#================================ UART ================================================#
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

#================================ PIO STEPPER =========================================#
# Programme PIO simplifié : 4 cycles par période → pas de nop()[31]
# La fréquence du signal STEP = sm.freq() / PIO_CYCLES_PER_STEP
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
    wrap_target()
    set(pins, 1)   # cycle 1
    nop()          # cycle 2
    set(pins, 0)   # cycle 3
    nop()          # cycle 4
    wrap()

#================================ CONFIGURATION MOTEURS ================================#
STEP_PIN_1 = 18; DIR_PIN_1 = 17
STEP_PIN_2 = 16; DIR_PIN_2 = 13
STEP_PIN_3 = 20; DIR_PIN_3 = 19

dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)
en_pin.value(0)  # Active les drivers (LOW = actif)

# Initialisation des StateMachines avec PIO_BASE_FREQ (valeur sûre)
sm_1 = StateMachine(1, stepper, freq=PIO_BASE_FREQ, set_base=Pin(STEP_PIN_1))
sm_2 = StateMachine(2, stepper, freq=PIO_BASE_FREQ, set_base=Pin(STEP_PIN_2))
sm_3 = StateMachine(3, stepper, freq=PIO_BASE_FREQ, set_base=Pin(STEP_PIN_3))

#================================ ODOMETRE OTOS =========================================#
myOtos = qwiic_otos.QwiicOTOS()
myOtos.begin()
t.sleep(1)

myOtos.calibrateImu()
myOtos.setLinearScalar(gainL)
myOtos.setAngularScalar(gainA)
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)
myOtos.resetTracking()

Position = myOtos.getPosition()

#================================ UNWRAP ANGLE =========================================#
def unwrap_angle(angle):
    """Transforme un angle borné [-π, +π] en angle cumulatif illimité."""
    global angle_cumul, angle_prec

    if angle_prec is None:
        angle_prec = angle
        return angle

    delta = angle - angle_prec

    if delta < -math.pi:
        angle_cumul += 2 * math.pi
    elif delta > math.pi:
        angle_cumul -= 2 * math.pi

    angle_prec = angle
    return angle + angle_cumul

#================================ CONVERSION CONSIGNE ANGLE ============================#
def convertir_consigne_angle(angle_absolu_rad, angle_cumulatif_rad):
    """
    Convertit une consigne d'angle absolue (radians)
    en angle cumulatif cohérent avec Position.h.
    """
    angle_actuel_mod = angle_cumulatif_rad % (2 * math.pi)
    diff = angle_absolu_rad - angle_actuel_mod

    # Normalisation dans [-π, +π]
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi

    return angle_cumulatif_rad + diff

#================================ LECTURE ODOMETRE =====================================#
def Odometre():
    global Position
    raw = myOtos.getPosition()
    raw.h = unwrap_angle(raw.h)
    Position = raw
    return Position

#================================ UART : LECTURE LIGNE =================================#
def read_line():
    if uart0.any():
        line = uart0.readline()
        if line:
            try:
                return line.decode().strip()
            except Exception:
                return None
    return None

#================================ ENVOI TELEMETRIE =====================================#
def send_pos(pos):
    """Envoie la position à maman en mètres + radians."""
    if pos is None:
        return
    msg = "POS {:.4f} {:.4f} {:.4f}\n".format(pos.x, pos.y, pos.h)
    uart0.write(msg)

def send_done():
    uart0.write("DONE\n")

#================================ OBJECTIF ATTEINT =====================================#
def objectif_atteint(pos, cons):
    """
    Tolérance : 10 mm (0.01 m) en XY, 3° en angle.
    """
    tol_xy    = 0.01
    tol_angle = math.radians(3)

    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)

    return dx < tol_xy and dy < tol_xy and dh < tol_angle

#================================ RESET PID ============================================#
def reset_pid():
    """Remet à zéro les états PID (à appeler avant chaque nouvelle mission)."""
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    erreur_prec_x     = 0.0
    erreur_prec_y     = 0.0
    erreur_prec_angle = 0.0
    integrale_x       = 0.0
    integrale_y       = 0.0
    integrale_angle   = 0.0

#================================ PID ROBOT ============================================#
def PID(pos, cons):
    """
    PID 3 axes : X, Y, Angle (mètres / radians).
    Retourne (vx, vy, w) en m/s et rad/s.
    """
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    erreur_x     = cons[0] - pos.x
    erreur_y     = cons[1] - pos.y
    erreur_angle = cons[2] - pos.h

    # Proportionnel
    P_x     = Kp * erreur_x
    P_y     = Kp * erreur_y
    P_angle = Kp * erreur_angle

    # Intégral
    integrale_x     += erreur_x     * dt
    integrale_y     += erreur_y     * dt
    integrale_angle += erreur_angle * dt

    I_x     = Ki * integrale_x
    I_y     = Ki * integrale_y
    I_angle = Ki * integrale_angle

    # Dérivé
    D_x     = Kd * (erreur_x     - erreur_prec_x)     / dt
    D_y     = Kd * (erreur_y     - erreur_prec_y)     / dt
    D_angle = Kd * (erreur_angle - erreur_prec_angle) / dt

    erreur_prec_x     = erreur_x
    erreur_prec_y     = erreur_y
    erreur_prec_angle = erreur_angle

    vx = P_x     + I_x     + D_x
    vy = P_y     + I_y     + D_y
    w  = P_angle + I_angle + D_angle

    return [vx, vy, w]

#================================ CINEMATIQUE INVERSE ==================================#
def calcul_vitesse_en_pas(v):
    """
    Convertit (vx, vy, w) → vitesses roues (pas/s) pour tri-roues 120°.
    """
    vx, vy, w = v

    Va = ( 0.5 * vx - (math.sqrt(3) / 2) * vy - dr * w) / Rr
    Vb = ( 0.5 * vx + (math.sqrt(3) / 2) * vy - dr * w) / Rr
    Vc = (-vx                                  - dr * w) / Rr

    return [Va / rad_per_step, Vb / rad_per_step, Vc / rad_per_step]

#================================ LIMITATION FREQUENCE =================================#
def frequence_def(v_pas_list):
    """
    Reçoit une liste [Va, Vb, Vc] en valeur absolue.
    Scale si le max dépasse f_max.
    Retourne (f1, f2, f3) scalées.
    """
    Va, Vb, Vc = abs(v_pas_list[0]), abs(v_pas_list[1]), abs(v_pas_list[2])
    max_val = max(Va, Vb, Vc)

    if max_val > f_max:
        scale = f_max / max_val
        Va   *= scale
        Vb   *= scale
        Vc   *= scale

    return Va, Vb, Vc

#================================ DIRECTION + FREQUENCE =================================#
def update_motor_directions(v_pas_list):
    """
    Applique les directions sur les DIR pins.
    Retourne les valeurs absolues pour apply_motors.
    """
    Va, Vb, Vc = v_pas_list

    dir_pin_1.value(1 if Va >= 0 else 0)
    dir_pin_2.value(1 if Vb >= 0 else 0)
    dir_pin_3.value(1 if Vc >= 0 else 0)

    return abs(Va), abs(Vb), abs(Vc)

#================================ APPLICATION MOTEURS ==================================#
def apply_motors(f1, f2, f3):
    """
    Applique les fréquences aux 3 moteurs.
    sm.freq() reçoit la fréquence PIO = f_step * PIO_CYCLES_PER_STEP.
    Nécessite un int (MicroPython).
    """
    for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
        if f < F_MIN:
            sm.active(0)
        else:
            pio_freq = int(f * PIO_CYCLES_PER_STEP)
            sm.freq(pio_freq)
            sm.active(1)

def stop_motors():
    sm_1.active(0)
    sm_2.active(0)
    sm_3.active(0)

#================================ CALIBRATION ODOMETRIE =================================#
async def calibration_odo():
    """
    Calibration (coroutine asyncio — n'est plus bloquante).
    1) 10 tours → gain angulaire
    2) 10 aller-retours → gain linéaire
    """
    global gainL, gainA, mission_active

    mission_active = False
    stop_motors()
    await asyncio.sleep(0.2)

    # Reset unwrap local pour la calibration
    global angle_cumul, angle_prec
    angle_cumul = 0.0
    angle_prec  = None

    #------------------ 1) CALIBRATION ANGULAIRE ------------------#
    myOtos.resetTracking()
    await asyncio.sleep(0.1)

    objectif_angle = 10 * 2 * math.pi  # 10 tours

    while True:
        pos = myOtos.getPosition()
        pos.h = unwrap_angle(pos.h)

        erreur = objectif_angle - pos.h
        if abs(erreur) < math.radians(2):
            break

        v_loc  = [0.0, 0.0, 1.5 * erreur]
        vp     = calcul_vitesse_en_pas(v_loc)
        fa, fb, fc = update_motor_directions(vp)
        fa, fb, fc = frequence_def([fa, fb, fc])
        apply_motors(fa, fb, fc)
        await asyncio.sleep(0.02)

    stop_motors()
    await asyncio.sleep(0.1)

    theta_mesure = unwrap_angle(myOtos.getPosition().h)
    if theta_mesure != 0:
        gainA = objectif_angle / theta_mesure
    myOtos.setAngularScalar(gainA)

    # Reset unwrap après calib angulaire
    angle_cumul = 0.0
    angle_prec  = None

    #------------------ 2) CALIBRATION LINEAIRE ------------------#
    distances_mesurees = []
    distance_commande  = 0.20  # 20 cm

    for i in range(10):
        myOtos.resetTracking()
        await asyncio.sleep(0.05)
        pos0 = myOtos.getPosition()

        direction = 1 if i % 2 == 0 else -1
        objectif  = direction * distance_commande

        while True:
            pos    = myOtos.getPosition()
            erreur = objectif - pos.x

            if abs(erreur) < 0.005:
                break

            v_loc  = [1.5 * erreur, 0.0, 0.0]
            vp     = calcul_vitesse_en_pas(v_loc)
            fa, fb, fc = update_motor_directions(vp)
            fa, fb, fc = frequence_def([fa, fb, fc])
            apply_motors(fa, fb, fc)
            await asyncio.sleep(0.02)

        stop_motors()
        await asyncio.sleep(0.05)

        pos1 = myOtos.getPosition()
        dx   = pos1.x - pos0.x
        dy   = pos1.y - pos0.y
        dist = math.sqrt(dx * dx + dy * dy)
        distances_mesurees.append(dist)

    if distances_mesurees:
        moyenne = sum(distances_mesurees) / len(distances_mesurees)
        if moyenne > 0:
            gainL = distance_commande / moyenne
            myOtos.setLinearScalar(gainL)

    stop_motors()

#================================ DISPATCHER UART CENTRALISÉ ===========================#
def handle_uart_command(cmd):
    """
    Dispatcher unique pour toutes les commandes UART.

    Commandes (mètres + radians) :
        GOTO  x y theta   → aller vers (x, y, theta)
        STOP              → arrêt immédiat (consigne = position courante)
        CALIBRATION       → demande de calibration
        COULEUR x y theta → repositionne l'odométrie
    """
    global consigne_xyO, mission_active, calibration_requested
    global angle_cumul, angle_prec

    if not cmd:
        return

    parts = cmd.split()
    if not parts:
        return

    kw = parts[0]

    # ----- GOTO -----
    if kw == "GOTO" and len(parts) >= 3:
        try:
            x   = float(parts[1])
            y   = float(parts[2])
            h_in = float(parts[3]) if len(parts) > 3 else 0.0

            if Position is not None:
                h_cumul = convertir_consigne_angle(h_in, Position.h)
            else:
                h_cumul = h_in

            reset_pid()
            consigne_xyO   = [x, y, h_cumul]
            mission_active = True
        except ValueError:
            pass

    # ----- STOP -----
    elif kw == "STOP":
        # Figer la consigne sur la position actuelle pour éviter de foncer vers (0,0,0)
        if Position is not None:
            consigne_xyO = [Position.x, Position.y, Position.h]
        mission_active = False
        stop_motors()

    # ----- CALIBRATION -----
    elif kw == "CALIBRATION":
        calibration_requested = True   # La tâche Task_Calib prend le relais

    # ----- COULEUR -----
    elif kw == "COULEUR" and len(parts) >= 4:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3])
            new_pos = qwiic_otos.Pose2D(x, y, h)
            myOtos.setPosition(new_pos)
            # Reset unwrap pour coller à la nouvelle position
            angle_cumul = 0.0
            angle_prec  = h
            # Met à jour la consigne pour éviter un saut
            if not mission_active:
                consigne_xyO = [x, y, h]
        except ValueError:
            pass

#================================ TASKS ASYNCIO ========================================#

async def Task_Odo():
    """Met à jour Position en continu (20 Hz)."""
    global Position
    while True:
        raw   = myOtos.getPosition()
        raw.h = unwrap_angle(raw.h)
        Position = raw
        await asyncio.sleep(0.05)

async def Task_UART():
    """Lit l'UART et dispatche les commandes (50 Hz)."""
    while True:
        cmd = read_line()
        if cmd:
            handle_uart_command(cmd)
        await asyncio.sleep(0.02)

async def Task_Calib():
    """
    Tâche dédiée à la calibration.
    Surveille calibration_requested, exécute calibration_odo() de manière
    non-bloquante (coroutine), puis signal DONE à maman.
    """
    global calibration_requested, system_locked

    while True:
        if calibration_requested:
            calibration_requested = False
            system_locked = True
            stop_motors()
            await asyncio.sleep(0.2)

            await calibration_odo()

            system_locked = False
            send_done()

        await asyncio.sleep(0.05)

async def Task_Control():
    """
    Boucle de contrôle principale (100 Hz) :
    - PID → cinématique inverse → moteurs
    - Respecte system_locked et mission_active
    """
    global v_robot, v_pas

    while True:
        if system_locked or not mission_active:
            stop_motors()
            await asyncio.sleep(dt)
            continue

        if Position is not None:
            v_robot        = PID(Position, consigne_xyO)
            v_pas          = calcul_vitesse_en_pas(v_robot)
            fa, fb, fc     = update_motor_directions(v_pas)
            fa, fb, fc     = frequence_def([fa, fb, fc])
            apply_motors(fa, fb, fc)

        await asyncio.sleep(dt)

async def Task_Telemetry():
    """
    Envoie POS en continu (10 Hz).
    Signale DONE une seule fois à l'arrivée.
    """
    global mission_active

    while True:
        if Position is not None:
            send_pos(Position)

            if mission_active and objectif_atteint(Position, consigne_xyO):
                stop_motors()
                send_done()
                mission_active = False

        await asyncio.sleep(0.1)

#================================ POINT D'ENTREE =======================================#
async def main():
    asyncio.create_task(Task_Odo())        # odométrie (20 Hz)
    asyncio.create_task(Task_UART())       # dispatcher UART (50 Hz)
    asyncio.create_task(Task_Calib())      # calibration non-bloquante
    asyncio.create_task(Task_Control())    # PID + moteurs (100 Hz)
    asyncio.create_task(Task_Telemetry())  # POS / DONE (10 Hz)

    while True:
        await asyncio.sleep(1)

asyncio.run(main())
