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

# Gains d’odométrie (calibrés plus tard)
gainL = 1.0
gainA = 1.0

#================================ VARIABLES GLOBALES ================================#
# Consigne : [x, y, angle]
consigne_xyO = [0, 0, 0]

# Vitesse robot (vx, vy, w)
v_robot = [0, 0, 0]

# Vitesse en pas moteur
v_pas = [0, 0, 0]

# Période PID
dt = 0.01

# Fréquences initiales moteurs
freq_1 = 1000
freq_2 = 1000
freq_3 = 1000

# États PID
erreur_prec_x = 0
erreur_prec_y = 0
erreur_prec_angle = 0
integrale_x = 0
integrale_y = 0
integrale_angle = 0

# Position odométrique
Position = None

# Indique si une mission GOTO est en cours
mission_active = False

#================================ ANGLE CUMULATIF (UNWRAP) ================================#
# L’OTOS renvoie un angle entre -π et +π → on reconstruit un angle cumulatif
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
# Mapping logique → physique
STEP_PIN_1 = 18 ; DIR_PIN_1 = 17
STEP_PIN_2 = 16 ; DIR_PIN_2 = 13
STEP_PIN_3 = 20 ; DIR_PIN_3 = 19

# Pins direction
dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

# Enable drivers
EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)
en_pin.value(0)  # Active les drivers

# State machines PIO
sm_1 = StateMachine(1, stepper, freq_1, set_base=Pin(STEP_PIN_1))
sm_2 = StateMachine(2, stepper, freq_2, set_base=Pin(STEP_PIN_2))
sm_3 = StateMachine(3, stepper, freq_3, set_base=Pin(STEP_PIN_3))

#================================ ODOMETRE OTOS =========================================#
myOtos = qwiic_otos.QwiicOTOS()
myOtos.begin()
t.sleep(1)

myOtos.calibrateImu()

# Applique les gains (seront recalibrés)
myOtos.setLinearScalar(gainL)
myOtos.setAngularScalar(gainA)

# Unités : mètres et radians
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)

myOtos.resetTracking()

Position = myOtos.getPosition()

#================================ UNWRAP ANGLE =========================================#
def unwrap_angle(angle):
    """
    Transforme un angle borné [-π, +π] en angle cumulatif illimité.
    Permet de mesurer plusieurs tours complets.
    """
    global angle_cumul, angle_prec

    if angle_prec is None:
        angle_prec = angle
        return angle

    delta = angle - angle_prec

    # Passage +179° → -179° → +1 tour
    if delta < -math.pi:
        angle_cumul += 2 * math.pi

    # Passage -179° → +179° → -1 tour
    elif delta > math.pi:
        angle_cumul -= 2 * math.pi

    angle_prec = angle
    return angle + angle_cumul

#================================ CONVERSION CONSIGNE ANGLE ============================#
def convertir_consigne_angle(angle_absolu_deg, angle_cumulatif_rad):
    """
    Convertit une consigne d’angle absolue (0–360°)
    en angle cumulatif cohérent avec Position.h.
    """
    angle_absolu = math.radians(angle_absolu_deg)
    angle_actuel_mod = angle_cumulatif_rad % (2 * math.pi)

    diff = angle_absolu - angle_actuel_mod

    # Normalisation dans [-π, +π]
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi

    return angle_cumulatif_rad + diff

#================================ LECTURE ODOMETRE =====================================#
def Odometre():
    """
    Lit la position OTOS et applique l’unwrapping de l’angle.
    """
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

#================================ TRAITEMENT CONSIGNE ==================================#
def consigne():
    """
    Interprète les commandes UART :
    - GOTO x y h
    - STOP
    """
    global consigne_xyO, mission_active

    cmd = read_line()
    if not cmd:
        return

    parts = cmd.split()

    # Commande GOTO
    if parts[0] == "GOTO" and len(parts) >= 3:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3]) if len(parts) > 3 else 0.0

            # Convertit l’angle absolu en angle cumulatif
            h_cumul = convertir_consigne_angle(h, Position.h)

            consigne_xyO = [x, y, h_cumul]
            mission_active = True

        except ValueError:
            pass

    # Commande STOP
    elif parts[0] == "STOP":
        consigne_xyO = [0.0, 0.0, 0.0]
        mission_active = False
        sm_1.active(0)
        sm_2.active(0)
        sm_3.active(0)

#================================ OBJECTIF ATTEINT =====================================#
def objectif_atteint(pos, cons):
    """
    Vérifie si le robot est proche de la consigne.
    """
    tol_xy = 10      # mm
    tol_angle = 3    # degrés

    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)

    return dx < tol_xy and dy < tol_xy and dh < tol_angle

#================================ ENVOI TELEMETRIE =====================================#
def send_pos(Pos):
    uart0.write(f"POS {Pos.x:.2f} {Pos.y:.2f} {Pos.h:.3f}\n")

def send_done():
    uart0.write("DONE\n")

#================================ PID ROBOT ============================================#
def PID(pos, cons):
    """
    PID 3 axes :
    - X
    - Y
    - Angle
    Retourne (vx, vy, w)
    """
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    output = [0, 0, 0]

    # Erreurs
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

    # Mise à jour erreurs précédentes
    erreur_prec_x = erreur_x
    erreur_prec_y = erreur_y
    erreur_prec_angle = erreur_angle

    # Sortie PID
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

    # Vitesses roues (modèle cinématique)
    Va = 0.5 * vx - (math.sqrt(3)/2) * vy - dr * w
    Vb = 0.5 * vx + (math.sqrt(3)/2) * vy - dr * w
    Vc = -vx - dr * w

    # Conversion en rad/s
    Wa = Va / Rr
    Wb = Vb / Rr
    Wc = Vc / Rr

    # Conversion rad/s → pas/s
    return [Wa / rad_per_step, Wb / rad_per_step, Wc / rad_per_step]

#================================ LIMITATION FREQUENCE =================================#
def frequence_def(v_pas):
    """
    Limite les fréquences moteurs à f_max.
    """
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
    """
    Applique les directions moteurs et retourne les fréquences absolues.
    """
    Va, Vb, Vc = v_pas

    dir_pin_1.value(1 if Va >= 0 else 0)
    dir_pin_2.value(1 if Vb >= 0 else 0)
    dir_pin_3.value(1 if Vc >= 0 else 0)

    return abs(Va), abs(Vb), abs(Vc)

#================================ CALIBRATION ODOMETRIE =================================#
def calibration_odo():
    """
    Calibration complète :
    1) 10 tours → gain angulaire
    2) 10 translations → gain linéaire
    """
    global gainL, gainA, mission_active

    # Stop PID
    mission_active = False
    sm_1.active(0); sm_2.active(0); sm_3.active(0)
    t.sleep(0.2)

    #------------------ 1) CALIBRATION ANGULAIRE ------------------#
    myOtos.resetTracking()
    t.sleep(0.1)

    objectif_angle = 10 * 2 * math.pi  # 10 tours

    while True:
        pos = myOtos.getPosition()
        pos.h = unwrap_angle(pos.h)

        erreur = objectif_angle - pos.h
        if abs(erreur) < math.radians(2):
            break

        v_robot = [0, 0, 1.5 * erreur]
        v_pas = calcul_vitesse_en_pas(v_robot)
        f1, f2, f3 = update_motor_directions_and_freq(v_pas)
        f1, f2, f3 = frequence_def([f1, f2, f3])

        sm_1.freq(f1); sm_2.freq(f2); sm_3.freq(f3)
        sm_1.active(1); sm_2.active(1); sm_3.active(1)

        t.sleep(0.02)

    sm_1.active(0); sm_2.active(0); sm_3.active(0)

    theta_mesure = unwrap_angle(myOtos.getPosition().h)
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

            v_robot = [1.5 * erreur, 0, 0]
            v_pas = calcul_vitesse_en_pas(v_robot)
            f1, f2, f3 = update_motor_directions_and_freq(v_pas)
            f1, f2, f3 = frequence_def([f1, f2, f3])

            sm_1.freq(f1); sm_2.freq(f2); sm_3.freq(f3)
            sm_1.active(1); sm_2.active(1); sm_3.active(1)

            t.sleep(0.02)

        sm_1.active(0); sm_2.active(0); sm_3.active(0)

        pos1 = myOtos.getPosition()
        dx = pos1.x - pos0.x
        dy = pos1.y - pos0.y
        dist = math.sqrt(dx*dx + dy*dy)

        distances_mesurees.append(dist)

    moyenne = sum(distances_mesurees) / len(distances_mesurees)
    gainL = distance_commande / moyenne
    myOtos.setLinearScalar(gainL)

    sm_1.active(0); sm_2.active(0); sm_3.active(0)

#================================ CALIBRATION ORIGINE ==================================#
def calibration_origine():
    """
    Commande UART :
    COULEUR x y h
    → repositionne l’odométrie
    """
    cmd = read_line()
    if not cmd:
        return

    parts = cmd.split()
    if parts[0] == "COULEUR":
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3]) if len(parts) > 3 else 0.0
            currentPosition = qwiic_otos.Pose2D(x, y, h)
            myOtos.setPosition(currentPosition)
        except ValueError:
            pass

#================================ TASKS ASYNCIO ========================================#
async def Task_Odo():
    """
    Lit l’odométrie en continu (20 Hz)
    """
    global Position
    while True:
        raw = myOtos.getPosition()
        raw.h = unwrap_angle(raw.h)
        Position = raw
        await asyncio.sleep(0.05)

async def Task_CALIB():
    """
    Attend la commande CALIBRATION
    et lance la calibration complète.
    """
    global system_locked

    while True:
        cmd = read_line()

        if cmd is None:
            await asyncio.sleep(0.05)
            continue

        parts = cmd.split()

        if parts[0] == "CALIBRATION":
            system_locked = True

            sm_1.active(0); sm_2.active(0); sm_3.active(0)
            await asyncio.sleep(0.2)

            calibration_odo()

            system_locked = False
            send_done()

        await asyncio.sleep(0.05)

async def Task_CALIB_OR():
    """
    Attend la commande COULEUR x y h
    pour repositionner l’odométrie.
    """
    while True:
        calibration_origine()
        await asyncio.sleep(0.05)

async def Task_UART():
    """
    Lit les commandes GOTO / STOP
    """
    while True:
        consigne()
        await asyncio.sleep(0.05)

async def Task_Control():
    """
    Boucle de contrôle principale :
    - applique le PID sur la position actuelle
    - convertit en vitesses roues
    - applique directions + fréquences aux moteurs
    - respecte le verrou 'system_locked' (ex: pendant calibration)
    """
    global v_robot, v_pas

    while True:
        # Si le système est verrouillé (calibration en cours) → moteurs à l'arrêt
        if system_locked:
            sm_1.active(0)
            sm_2.active(0)
            sm_3.active(0)
            await asyncio.sleep(0.01)
            continue

        # Si on a une position valide
        if Position is not None:
            # Calcul des vitesses robot via PID
            v_robot = PID(Position, consigne_xyO)

            # Conversion en pas/s pour chaque moteur
            v_pas = calcul_vitesse_en_pas(v_robot)

            # Mise à jour des directions + fréquences brutes
            f1, f2, f3 = update_motor_directions_and_freq(v_pas)

            # Limitation des fréquences à f_max
            f1, f2, f3 = frequence_def([f1, f2, f3])

            # Application aux 3 moteurs
            for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
                if f < F_MIN:
                    # En dessous d’un certain seuil → on coupe le moteur
                    sm.active(0)
                else:
                    sm.freq(f)
                    sm.active(1)

        # Période de la boucle de contrôle
        await asyncio.sleep(0.01)


async def Task_Telemetry():
    """
    Gère :
    - l’envoi de POS en continu quand aucune mission n’est active
    - l’envoi de DONE quand la consigne est atteinte
    """
    global mission_active

    while True:
        if mission_active and Position is not None:
            # Si une mission est en cours, on vérifie si l’objectif est atteint
            if objectif_atteint(Position, consigne_xyO):
                send_done()
                mission_active = False

                # On arrête les moteurs
                sm_1.active(0)
                sm_2.active(0)
                sm_3.active(0)
        else:
            # Sinon, on envoie simplement la position actuelle
            if Position is not None:
                send_pos(Position)

        await asyncio.sleep(0.05)


async def main():
    """
    Point d’entrée :
    - lance toutes les tâches asynchrones
    - boucle infinie pour garder le programme vivant
    """
    asyncio.create_task(Task_CALIB())      # écoute commande CALIBRATION
    asyncio.create_task(Task_CALIB_OR())   # écoute commande COULEUR x y h
    asyncio.create_task(Task_Odo())        # met à jour Position
    asyncio.create_task(Task_UART())       # écoute GOTO / STOP
    asyncio.create_task(Task_Control())    # PID + moteurs
    asyncio.create_task(Task_Telemetry())  # POS / DONE

    while True:
        await asyncio.sleep(1)


# Lancement du scheduler asyncio

