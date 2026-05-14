import rp2
import time as t
import qwiic_otos
from machine import UART, Pin, StateMachine
import uasyncio as asyncio
import math

#================================ SETUP CONSTANTES ================================#
# Gains PID
Kp = 1
Ki = 1
Kd = 1

# Paramètres mécaniques du robot
Rr = 0.029   # Rayon des roues (m)
dr = 0.0155  # Distance entre centre du robot et roue (m)

# Limites des moteurs
f_max = 5000 # Fréquence max des moteurs (Hz)
F_MIN = 2    # Fréquence minimale pour bouger (Hz)

# Conversion radian → pas moteur
rad_per_step = 2 * math.pi / 200  # 200 pas par tour moteur

# Gains d'odométrie (calibrés plus tard)
gainL = 1.0
gainA = 1.0

#================================ VARIABLES GLOBALES ================================#
# Consigne : [x, y, angle] en MÈTRES et RADIANS (cohérent avec OTOS)
consigne_xyO = [0.0, 0.0, 0.0]

# Vitesse robot (vx, vy, w)
v_robot = [0.0, 0.0, 0.0]

# Vitesse en pas moteur
v_pas = [0.0, 0.0, 0.0]

# Période PID
dt = 0.01

# Fréquences initiales moteurs
freq_1 = 1000
freq_2 = 1000
freq_3 = 1000

# États PID
erreur_prec_x = 0.0
erreur_prec_y = 0.0
erreur_prec_angle = 0.0
integrale_x = 0.0
integrale_y = 0.0
integrale_angle = 0.0

# Position odométrique
Position = None

# Indique si une mission GOTO est en cours
mission_active = False

#================================ ANGLE CUMULATIF (UNWRAP) ================================#
angle_cumul = 0.0
angle_prec = None

# Verrou système : empêche le PID de tourner pendant la calibration
system_locked = False

#================================ UART ================================================#
uart0 = UART(0, baudrate=115200, tx=Pin(6), rx=Pin(7))

#================================ PIO STEPPER =========================================#
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
    """
    Programme PIO minimaliste :
    - génère un signal carré sur la pin STEP
    - la fréquence est contrôlée par sm.freq()
    """
    wrap_target()
    set(pins, 1)      # STEP HIGH
    nop()[31]         # durée du HIGH
    set(pins, 0)      # STEP LOW
    nop()[31]         # durée du LOW
    wrap()

#================================ CONFIGURATION MOTEURS ================================#
STEP_PIN_1 = 18 ; DIR_PIN_1 = 17
STEP_PIN_2 = 16 ; DIR_PIN_2 = 13
STEP_PIN_3 = 20 ; DIR_PIN_3 = 19

dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)
en_pin.value(0)  # Active les drivers

sm_1 = StateMachine(1, stepper, freq_1, set_base=Pin(STEP_PIN_1))
sm_2 = StateMachine(2, stepper, freq_2, set_base=Pin(STEP_PIN_2))
sm_3 = StateMachine(3, stepper, freq_3, set_base=Pin(STEP_PIN_3))

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
    """
    Transforme un angle borné [-π, +π] en angle cumulatif illimité.
    """
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
    Convertit une consigne d'angle absolue (en radians, [0, 2π])
    en angle cumulatif cohérent avec Position.h.
    
    NOTE : maman envoie déjà en radians, donc on prend directement.
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
    Position = myOtos.getPosition()
    Position.h = unwrap_angle(Position.h)
    return Position

#================================ UART : LECTURE LIGNE =================================#
def read_line():
    if uart0.any():
        line = uart0.readline()
        if line:
            return line.decode().strip()
    return None

#================================ ENVOI TELEMETRIE =====================================#
def send_pos(Pos):
    """
    Envoie la position à maman en MÈTRES (maman convertit en mm chez elle).
    """
    if Pos is None:
        return
    msg = "POS {:.4f} {:.4f} {:.4f}\n".format(Pos.x, Pos.y, Pos.h)
    uart0.write(msg)

def send_done():
    uart0.write("DONE\n")

#================================ OBJECTIF ATTEINT =====================================#
def objectif_atteint(pos, cons):
    """
    Tolérance cohérente : 10 mm = 0.01 m et 3° en radians.
    Position et consigne sont toutes deux en mètres + radians.
    """
    tol_xy = 0.01                  # 10 mm en mètres
    tol_angle = math.radians(3)    # 3° en radians

    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)

    return dx < tol_xy and dy < tol_xy and dh < tol_angle

#================================ PID ROBOT ============================================#
def PID(pos, cons):
    """
    PID 3 axes : X, Y, Angle. Tout en mètres / radians.
    Retourne (vx, vy, w) en m/s et rad/s.
    """
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    output = [0.0, 0.0, 0.0]

    erreur_x = cons[0] - pos.x
    erreur_y = cons[1] - pos.y
    erreur_angle = cons[2] - pos.h

    # Proportionnel
    P_x = Kp * erreur_x
    P_y = Kp * erreur_y
    P_angle = Kp * erreur_angle

    # Intégral
    integrale_x += erreur_x * dt
    integrale_y += erreur_y * dt
    integrale_angle += erreur_angle * dt

    I_x = Ki * integrale_x
    I_y = Ki * integrale_y
    I_angle = Ki * integrale_angle

    # Dérivé
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

#================================ CINEMATIQUE INVERSE ==================================#
def calcul_vitesse_en_pas(v):
    """
    Convertit (vx, vy, w) en vitesses roues (pas/s)
    pour un robot tri-roues 120°.
    """
    vx, vy, w = v

    Va = 0.5 * vx - (math.sqrt(3)/2) * vy - dr * w
    Vb = 0.5 * vx + (math.sqrt(3)/2) * vy - dr * w
    Vc = -vx - dr * w

    Wa = Va / Rr
    Wb = Vb / Rr
    Wc = Vc / Rr

    return [Wa / rad_per_step, Wb / rad_per_step, Wc / rad_per_step]

#================================ LIMITATION FREQUENCE =================================#
def frequence_def(v_pas):
    Va, Vb, Vc = map(abs, v_pas)
    max_val = max(Va, Vb, Vc)

    if max_val > f_max:
        scale = f_max / max_val
        Va *= scale
        Vb *= scale
        Vc *= scale

    return Va, Vb, Vc

#================================ DIRECTION + FREQUENCE =================================#
def update_motor_directions_and_freq(v_pas):
    Va, Vb, Vc = v_pas

    dir_pin_1.value(1 if Va >= 0 else 0)
    dir_pin_2.value(1 if Vb >= 0 else 0)
    dir_pin_3.value(1 if Vc >= 0 else 0)

    return abs(Va), abs(Vb), abs(Vc)

#================================ APPLICATION MOTEURS ==================================#
def apply_motors(f1, f2, f3):
    """
    Helper : applique freq + active aux 3 moteurs avec gestion F_MIN.
    IMPORTANT : sm.freq() exige un int en MicroPython, pas un float.
    """
    for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
        if f < F_MIN:
            sm.active(0)
        else:
            sm.freq(int(f))     # ← int() obligatoire
            sm.active(1)

def stop_motors():
    sm_1.active(0)
    sm_2.active(0)
    sm_3.active(0)

#================================ CALIBRATION ODOMETRIE =================================#
def calibration_odo():
    """
    Calibration complète :
    1) 10 tours → gain angulaire
    2) 10 translations → gain linéaire
    """
    global gainL, gainA, mission_active

    mission_active = False
    stop_motors()
    t.sleep(0.2)

    #------------------ 1) CALIBRATION ANGULAIRE ------------------#
    myOtos.resetTracking()
    t.sleep(0.1)

    objectif_angle = 10 * 2 * math.pi  # 10 tours en radians

    while True:
        pos = myOtos.getPosition()
        pos.h = unwrap_angle(pos.h)

        erreur = objectif_angle - pos.h
        if abs(erreur) < math.radians(2):
            break

        v_robot_local = [0.0, 0.0, 1.5 * erreur]
        v_pas_local = calcul_vitesse_en_pas(v_robot_local)
        f1, f2, f3 = update_motor_directions_and_freq(v_pas_local)
        f1, f2, f3 = frequence_def([f1, f2, f3])

        apply_motors(f1, f2, f3)
        t.sleep(0.02)

    stop_motors()

    theta_mesure = unwrap_angle(myOtos.getPosition().h)
    if theta_mesure != 0:
        gainA = objectif_angle / theta_mesure
    myOtos.setAngularScalar(gainA)

    #------------------ 2) CALIBRATION LINEAIRE ------------------#
    distances_mesurees = []
    distance_commande = 0.20  # 20 cm

    for i in range(10):
        myOtos.resetTracking()
        t.sleep(0.05)
        pos0 = myOtos.getPosition()

        direction = 1 if i % 2 == 0 else -1
        objectif = direction * distance_commande

        while True:
            pos = myOtos.getPosition()
            erreur = objectif - pos.x

            if abs(erreur) < 0.005:
                break

            v_robot_local = [1.5 * erreur, 0.0, 0.0]
            v_pas_local = calcul_vitesse_en_pas(v_robot_local)
            f1, f2, f3 = update_motor_directions_and_freq(v_pas_local)
            f1, f2, f3 = frequence_def([f1, f2, f3])

            apply_motors(f1, f2, f3)
            t.sleep(0.02)

        stop_motors()

        pos1 = myOtos.getPosition()
        dx = pos1.x - pos0.x
        dy = pos1.y - pos0.y
        dist = math.sqrt(dx*dx + dy*dy)
        distances_mesurees.append(dist)

    if len(distances_mesurees) > 0:
        moyenne = sum(distances_mesurees) / len(distances_mesurees)
        if moyenne > 0:
            gainL = distance_commande / moyenne
            myOtos.setLinearScalar(gainL)

    stop_motors()

#================================ DISPATCHER UART CENTRALISÉ ===========================#
def handle_uart_command(cmd):
    """
    DISPATCHER UNIQUE pour toutes les commandes UART.
    
    Une seule fonction lit les commandes et dispatche selon le mot-clé.
    Évite le conflit où plusieurs tâches lisaient l'UART en parallèle
    et se volaient les commandes.
    
    Commandes acceptées (toutes en MÈTRES + RADIANS) :
        GOTO         x y theta      → translation vers (x, y, theta)
        STOP                        → arrêt immédiat
        CALIBRATION                 → lance calibration_odo()
        COULEUR      x y theta      → repositionne l'odométrie
    """
    global consigne_xyO, mission_active, system_locked

    if not cmd:
        return

    parts = cmd.split()
    if len(parts) == 0:
        return

    kw = parts[0]

    # ----- GOTO -----
    if kw == "GOTO" and len(parts) >= 3:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h_in = float(parts[3]) if len(parts) > 3 else 0.0

            # Convertit l'angle absolu en angle cumulatif
            h_cumul = convertir_consigne_angle(h_in, Position.h)

            consigne_xyO = [x, y, h_cumul]
            mission_active = True
        except ValueError:
            pass

    # ----- STOP -----
    elif kw == "STOP":
        consigne_xyO = [0.0, 0.0, 0.0]
        mission_active = False
        stop_motors()

    # ----- CALIBRATION -----
    elif kw == "CALIBRATION":
        system_locked = True
        stop_motors()
        t.sleep(0.2)
        calibration_odo()
        system_locked = False
        send_done()    # signal à maman : calibration terminée

    # ----- COULEUR -----
    elif kw == "COULEUR" and len(parts) >= 4:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3])
            currentPosition = qwiic_otos.Pose2D(x, y, h)
            myOtos.setPosition(currentPosition)
            # Reset aussi l'unwrap
            global angle_cumul, angle_prec
            angle_cumul = 0.0
            angle_prec = h
        except ValueError:
            pass

#================================ TASKS ASYNCIO ========================================#
async def Task_Odo():
    """Lit l'odométrie en continu (20 Hz)"""
    global Position
    while True:
        raw = myOtos.getPosition()
        raw.h = unwrap_angle(raw.h)
        Position = raw
        await asyncio.sleep(0.05)

async def Task_UART():
    """
    TÂCHE UNIQUE qui lit l'UART et dispatche.
    Remplace Task_CALIB, Task_CALIB_OR et Task_UART originales.
    """
    while True:
        cmd = read_line()
        if cmd:
            handle_uart_command(cmd)
        await asyncio.sleep(0.02)

async def Task_Control():
    """
    Boucle de contrôle principale :
    - applique le PID
    - convertit en vitesses roues
    - applique aux moteurs
    - respecte le verrou 'system_locked' (pendant calibration)
    """
    global v_robot, v_pas

    while True:
        if system_locked:
            stop_motors()
            await asyncio.sleep(0.01)
            continue

        if Position is not None:
            v_robot = PID(Position, consigne_xyO)
            v_pas = calcul_vitesse_en_pas(v_robot)
            f1, f2, f3 = update_motor_directions_and_freq(v_pas)
            f1, f2, f3 = frequence_def([f1, f2, f3])
            apply_motors(f1, f2, f3)

        await asyncio.sleep(0.01)

async def Task_Telemetry():
    """
    Envoie TOUJOURS la position à maman (besoin en continu pour la Jetson).
    En plus, signale l'arrivée avec DONE une seule fois.
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
                stop_motors()

        await asyncio.sleep(0.1)

async def main():
    """
    Point d'entrée :
    - lance toutes les tâches asynchrones
    """
    asyncio.create_task(Task_Odo())        # met à jour Position
    asyncio.create_task(Task_UART())       # dispatcher UART unique (GOTO/STOP/CALIB/COULEUR)
    asyncio.create_task(Task_Control())    # PID + moteurs
    asyncio.create_task(Task_Telemetry())  # POS / DONE

    while True:
        await asyncio.sleep(1)

asyncio.run(main())
