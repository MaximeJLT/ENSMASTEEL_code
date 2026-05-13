import rp2
import time as t
import qwiic_otos
from machine import UART, Pin, StateMachine
import uasyncio as asyncio
import math
#================================SETUP CONSTANTE================================#
# Gains propotionnel, intégral et dérivé
Kp = 1
Ki = 1
Kd = 1
Rr = 29 #mm
dr = 1 #mm
f_max = 5000 #Hz
F_MIN = 2  # Hz
rad_per_step = 2 * math.pi / 200 
#================================SETUP VARIABLE================================#
consigne_xyO = [0, 0, 0]
v_robot = [0, 0, 0]   
v_pas = [0, 0, 0]      
dt = 0.1 # 100 milliseconde
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
#uart1 = UART(1, baudrate=115200, tx=Pin(14), rx=Pin(15))
#================================SETUP STEPPER FONCTION================================#
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
    wrap_target()
    set(pins, 1)      # STEP HIGH
    nop()[31]         # durée du HIGH
    set(pins, 0)      # STEP LOW
    nop()[31]         # durée du LOW
    wrap()
#================================SETUP STEPPER================================#
# Broches moteur 1
STEP_PIN_1 = 20
DIR_PIN_1 = 19

# Broches moteur 2
STEP_PIN_2 = 18
DIR_PIN_2 = 17

# Broches moteur 3
STEP_PIN_3 = 16
DIR_PIN_3 = 13

# Définitions des pins
dir_pin_1 = Pin(DIR_PIN_1, Pin.OUT)
dir_pin_2 = Pin(DIR_PIN_2, Pin.OUT)
dir_pin_3 = Pin(DIR_PIN_3, Pin.OUT)

EN_PIN = 21
en_pin = Pin(EN_PIN, Pin.OUT)

# Mise en route des drivers
en_pin.value(0)

#Définition des PIO des steppers
sm_1 = StateMachine(
    1, stepper,
    freq_1,
    set_base=Pin(STEP_PIN_1)
)
sm_2 = StateMachine(
    2, stepper,
    freq_2,
    set_base=Pin(STEP_PIN_2)
)
sm_3 = StateMachine(
    3, stepper,
    freq_3,
    set_base=Pin(STEP_PIN_3)
)
#================================SETUP ODOMETRE================================#
# Définition de l'objet capteur odométrique
myOtos = qwiic_otos.QwiicOTOS()

# Initialisation du capteur
myOtos.begin()
time.sleep(1)

# Calibration du capteur (reset les offsets du capteur à 0)
myOtos.calibrateImu()

# Met les gains du capteur à 1
myOtos.setLinearScalar(1.0)
myOtos.setAngularScalar(1.0)

# Met les unités du capteur en unités S.I.
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)

# Reset la position du capteur à l'origine définie (sur le robot ou avec un offset)
myOtos.resetTracking()

Position = myOtos.getPosition()
#================================SETUP FONCTIONS================================#
def Odometre():
    global Position
    Position = myOtos.getPosition()
    return Position
------------------------------------------------
def read_line():
    if uart0.any():
        line = uart0.readline()
        if line:
            return line.decode().strip()
    return None
------------------------------------------------
def consigne():
    global consigne_xyO
    global mission_active

    cmd = read_line()
    if not cmd:
        return  # rien à faire

    parts = cmd.split()

    # --- Commande GOTO x y h ---
    if parts[0] == "GOTO" and len(parts) >= 3:
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3]) if len(parts) > 3 else 0.0

            consigne_xyO = [x, y, h]
            mission_active = True

        except ValueError:
            # Commande mal formée → on ignore
            pass

    # --- Commande STOP ---
    elif parts[0] == "STOP":
        consigne_xyO = [0.0, 0.0, 0.0]
        mission_active = False
        # Arrêt immédiat des moteurs
        sm_1.active(0)
        sm_2.active(0)
        sm_3.active(0)
------------------------------------------------    
def objectif_atteint(pos, cons):
    tol_xy = 5      # mm
    tol_angle = 2   # degrés

    dx = abs(cons[0] - pos.x)
    dy = abs(cons[1] - pos.y)
    dh = abs(cons[2] - pos.h)

    return dx < tol_xy and dy < tol_xy and dh < tol_angle
------------------------------------------------
def send_pos(Pos):
    msg = f"POS {Pos.x:.2f} {Pos.y:.2f} {Pos.h:.3f}\n"
    uart0.write(msg)
------------------------------------------------
def send_done():
    uart0.write("DONE\n")
------------------------------------------------
def PID(pos, cons):
    global erreur_prec_x, erreur_prec_y, erreur_prec_angle
    global integrale_x, integrale_y, integrale_angle

    # Sortie du PID : vitesse cible (vx, vy, w)
    output = [0, 0, 0]

    # Erreurs
    erreur_x = cons[0] - pos.x
    erreur_y = cons[1] - pos.y
    erreur_angle = cons[2] - pos.h   # correction de l’index

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

    # Mise à jour des erreurs précédentes
    erreur_prec_x = erreur_x
    erreur_prec_y = erreur_y
    erreur_prec_angle = erreur_angle

    # Sortie finale
    output[0] = P_x + I_x + D_x
    output[1] = P_y + I_y + D_y
    output[2] = P_angle + I_angle + D_angle

    return output
------------------------------------------------
def calcul_vitesse_en_pas(v):
    """
    v = [vx, vy, w] en mm/s et rad/s
    Retourne [Va, Vb, Vc] en pas/s
    """

    vx = v[0]
    vy = v[1]
    w  = v[2]

    # --- Cinématique directe ---
    Va = 0.5 * vx - (math.sqrt(3)/2) * vy - dr * w
    Vb = 0.5 * vx + (math.sqrt(3)/2) * vy - dr * w
    Vc = -vx - dr * w

    # --- Conversion mm/s → rad/s roue ---
    Wa = Va / Rr
    Wb = Vb / Rr
    Wc = Vc / Rr

    # --- Conversion rad/s → pas/s ---
    pas_a = Wa / rad_per_step
    pas_b = Wb / rad_per_step
    pas_c = Wc / rad_per_step

    return [pas_a, pas_b, pas_c]
------------------------------------------------
def frequence_def(v_pas):
    """
    v_pas : liste [Va, Vb, Vc] en pas/s
    Retourne (freq1, freq2, freq3) en Hz
    """

    # Valeurs absolues pour la fréquence (la direction est gérée ailleurs)
    Va = abs(v_pas[0])
    Vb = abs(v_pas[1])
    Vc = abs(v_pas[2])

    # Saturation si un moteur dépasse f_max
    max_val = max(Va, Vb, Vc)

    if max_val > f_max:
        # On réduit proportionnellement toutes les vitesses
        scale = f_max / max_val
        Va *= scale
        Vb *= scale
        Vc *= scale

    return Va, Vb, Vc
------------------------------------------------
def update_motor_directions_and_freq(v_pas):
    """
    v_pas : [Va, Vb, Vc] en pas/s (positif = avant, négatif = arrière)
    Retourne (f1, f2, f3) en Hz
    """

    Va, Vb, Vc = v_pas

    # --- MOTEUR A ---
    if Va >= 0:
        dir_pin_1.value(1)   # sens avant
    else:
        dir_pin_1.value(0)   # sens arrière
    f1 = abs(Va)

    # --- MOTEUR B ---
    if Vb >= 0:
        dir_pin_2.value(1)
    else:
        dir_pin_2.value(0)
    f2 = abs(Vb)

    # --- MOTEUR C ---
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
        await asyncio.sleep(0.05)   # 20 Hz
------------------------------------------------
async def Task_UART():
    while True:
        consigne()
        await asyncio.sleep(0.05)
------------------------------------------------
async def Task_Control():
    global v_robot, v_pas

    while True:
        if Position is not None:

            # PID → vitesse robot
            v_robot = PID(Position, consigne_xyO)

            # Cinématique → pas/s
            v_pas = calcul_vitesse_en_pas(v_robot)

            # Direction + fréquence
            f1, f2, f3 = update_motor_directions_and_freq(v_pas)

            # Saturation
            f1, f2, f3 = frequence_def([f1, f2, f3])

            # Application aux moteurs
            sm_1.freq(f1)
            sm_2.freq(f2)
            sm_3.freq(f3)

            # Application aux moteurs
            for sm, f in [(sm_1, f1), (sm_2, f2), (sm_3, f3)]:
                if f < F_MIN:
                    sm.active(0)      # moteur OFF
                else:
                    sm.freq(f)        # mise à jour fréquence
                    sm.active(1)      # moteur ON

        await asyncio.sleep(0.01)   # 20 Hz
------------------------------------------------
async def Task_Telemetry():
    global mission_active
    while True:
        if mission_active:
            if objectif_atteint(Position, consigne_xyO):
                send_done()
                mission_active = False
        else:
            send_pos(Position)

        await asyncio.sleep(0.1)
------------------------------------------------
async def main():
    asyncio.create_task(Task_Odo())
    asyncio.create_task(Task_UART())
    asyncio.create_task(Task_Control())
    asyncio.create_task(Task_Telemetry())

    while True:
        await asyncio.sleep(1)

asyncio.run(main())
