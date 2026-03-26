# maman_fictive_v2.py
# Simulateur de maman \u2014 v2 : ex�cute vraiment les commandes re�ues.
#
# V1 se contentait d'afficher le world et d'ACK.
# V2 :
#   - Maintient un �tat robot interne (position, bras)
#   - Ex�cute les Command re�ues (GOTO, PICKUP, DROP_ALL, MOVE_CURSOR)
#   - Renvoie sa position dans l'ACK \u2192 la Jetson peut fusionner
#     vision + odom�trie maman pour une meilleure localisation
#
# Utilit� : tester la boucle compl�te Jetson \u2194 maman sans le vrai robot.
#
# Physique simplifi�e :
#   - Holonome, vitesse max V_MAX mm/s
#   - PICKUP : timer de PICKUP_DURATION_S secondes
#   - DROP   : timer de DROP_DURATION_S secondes
#   - MOVE_CURSOR : d�placement � V_CURSOR mm/s sur l'axe X

import json
import socket
import time
import math
from dataclasses import dataclass, field
from typing import List, Optional

# ---- Config r�seau ----
LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5005
ACK_PORT    = 5006   # port Jetson pour les ACK

# ---- Physique ----
V_MAX            = 500.0   # mm/s
V_CURSOR         = 300.0   # mm/s
ARRIVAL_EPS      = 30.0    # mm
PICKUP_DURATION  = 1.2     # s
DROP_DURATION    = 0.9     # s
MAX_CARRIED      = 8


# -----------------------------------------------------------------------
# �tat interne du robot simul�
# -----------------------------------------------------------------------

@dataclass
class SimRobot:
    x_mm:       float = 1150.0
    y_mm:       float = 800.0
    theta_rad:  float = 0.0

    carried_ids: List[int] = field(default_factory=list)
    action_state: str = "idle"    # idle | going | picking | dropping | dragging
    action_timer: float = 0.0     # secondes restantes pour actions temporis�es

    # Cible GOTO
    goto_x: Optional[float] = None
    goto_y: Optional[float] = None

    # Cible curseur
    cursor_target_x: Optional[float] = None


def step_robot(robot: SimRobot, dt: float) -> None:
    """Avance la physique du robot simul� de dt secondes."""

    if robot.action_state == "going":
        if robot.goto_x is None:
            robot.action_state = "idle"
            return
        dx = robot.goto_x - robot.x_mm
        dy = robot.goto_y - robot.y_mm
        dist = math.hypot(dx, dy)
        if dist < ARRIVAL_EPS:
            robot.x_mm = robot.goto_x
            robot.y_mm = robot.goto_y
            robot.action_state = "idle"
            robot.goto_x = None
            robot.goto_y = None
            print(f"[maman] GOTO atteint ({robot.x_mm:.0f}, {robot.y_mm:.0f})")
        else:
            move = min(V_MAX * dt, dist)
            robot.x_mm += move * dx / dist
            robot.y_mm += move * dy / dist
            robot.theta_rad = math.atan2(dy, dx)

    elif robot.action_state in ("picking", "dropping"):
        robot.action_timer -= dt
        if robot.action_timer <= 0.0:
            robot.action_timer = 0.0
            robot.action_state = "idle"
            print(f"[maman] action termin�e")

    elif robot.action_state == "dragging":
        if robot.cursor_target_x is None:
            robot.action_state = "idle"
            return
        dx = robot.cursor_target_x - robot.x_mm
        dist = abs(dx)
        if dist < ARRIVAL_EPS:
            robot.x_mm = robot.cursor_target_x
            robot.action_state = "idle"
            robot.cursor_target_x = None
            print(f"[maman] curseur l�ch� en x={robot.x_mm:.0f}")
        else:
            move = min(V_CURSOR * dt, dist)
            robot.x_mm += move * (1 if dx > 0 else -1)


def apply_command(robot: SimRobot, cmd: dict) -> None:
    """Applique une Command re�ue de la Jetson sur le robot simul�."""
    if cmd is None:
        return

    kind = cmd.get("kind", "")

    if kind == "GOTO":
        # On accepte le GOTO m�me si d�j� en mouvement (la Jetson peut
        # envoyer le m�me GOTO plusieurs fois jusqu'� confirmation)
        tx = cmd.get("x_mm")
        ty = cmd.get("y_mm")
        if tx is not None and ty is not None:
            if robot.goto_x != tx or robot.goto_y != ty:
                print(f"[maman] GOTO \u2192 ({tx:.0f}, {ty:.0f})")
            robot.goto_x = tx
            robot.goto_y = ty
            robot.action_state = "going"

    elif kind == "PICKUP":
        if robot.action_state == "idle":
            cid = cmd.get("crate_id", "?")
            robot.carried_ids.append(cid)
            robot.action_state = "picking"
            robot.action_timer = PICKUP_DURATION
            print(f"[maman] PICKUP caisse {cid} \u2192 porte {len(robot.carried_ids)}/{MAX_CARRIED}")

    elif kind == "DROP_ALL":
        if robot.action_state == "idle":
            zone = cmd.get("zone_name", "?")
            dropped = list(robot.carried_ids)
            robot.carried_ids.clear()
            robot.action_state = "dropping"
            robot.action_timer = DROP_DURATION
            print(f"[maman] DROP_ALL dans '{zone}' \u2192 d�pos� {dropped}")

    elif kind == "MOVE_CURSOR":
        tx = cmd.get("x_mm")
        if tx is not None and robot.action_state == "idle":
            robot.cursor_target_x = tx
            robot.action_state = "dragging"
            print(f"[maman] MOVE_CURSOR \u2192 x={tx:.0f}")

    elif kind == "STOP":
        robot.action_state = "idle"
        robot.goto_x = None
        robot.goto_y = None
        print("[maman] STOP")


# -----------------------------------------------------------------------
# Boucle principale
# -----------------------------------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
sock.settimeout(0.05)   # 50ms timeout \u2192 boucle physique � ~20Hz m�me sans paquet

print(f"[maman-v2] listening on UDP {LISTEN_IP}:{LISTEN_PORT}")

robot = SimRobot()
last_rx = time.time()
last_t  = time.time()

action_labels = {
    "idle":     "En route",
    "going":    "GOTO...",
    "picking":  "Ramassage...",
    "dropping": "D�p�t...",
    "dragging": "Curseur...",
}

while True:
    now = time.time()
    dt  = now - last_t
    last_t = now

    # 1) Physique robot (m�me sans paquet entrant)
    step_robot(robot, dt)

    # 2) R�ception paquet Jetson
    try:
        data, addr = sock.recvfrom(65535)
        last_rx = now

        msg = json.loads(data.decode("utf-8"))
        frame_id = msg.get("frame_id", -1)
        cmd      = msg.get("command", None)

        # Appliquer la commande
        apply_command(robot, cmd)

        # Log compact
        world   = msg.get("world", {})
        caisses = world.get("caisses", [])
        action_str = action_labels.get(robot.action_state, robot.action_state)
        print(f"[maman-v2] frame={frame_id:4d} | "
              f"robot=({robot.x_mm:6.0f},{robot.y_mm:6.0f}) | "
              f"{action_str:<18} | "
              f"porte={len(robot.carried_ids)}/{MAX_CARRIED} | "
              f"cmd={cmd['kind'] if cmd else 'None':<12} | "
              f"caisses_vues={len(caisses)}")

        # ACK avec �tat robot (la Jetson peut l'utiliser pour odom�trie)
        ack = {
            "type":       "ack",
            "frame_id":   frame_id,
            "t_ms":       int(time.time() * 1000),
            "robot_state": {           # \u2190 �tat maman renvoy� � la Jetson
                "x_mm":       robot.x_mm,
                "y_mm":       robot.y_mm,
                "theta_rad":  robot.theta_rad,
                "action":     robot.action_state,
                "carried":    list(robot.carried_ids),
            }
        }
        sock.sendto(json.dumps(ack).encode("utf-8"), (addr[0], ACK_PORT))

    except socket.timeout:
        # Pas de paquet \u2014 on a quand m�me avanc� la physique
        if now - last_rx > 3.0:
            print(f"[maman-v2] (aucun paquet depuis 3s) | "
                  f"robot=({robot.x_mm:.0f},{robot.y_mm:.0f}) action={robot.action_state}")
            last_rx = now