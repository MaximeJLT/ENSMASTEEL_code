import json
import socket
import time
import math
from dataclasses import dataclass, field
from typing import List, Optional

LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5005
ACK_PORT    = 5006   

V_MAX            = 500.0  
V_CURSOR         = 300.0   
ARRIVAL_EPS      = 30.0    
PICKUP_DURATION  = 1.2     
DROP_DURATION    = 0.9   
MAX_CARRIED      = 8


@dataclass
class SimRobot:
    x_mm:       float = 1150.0
    y_mm:       float = 800.0
    theta_rad:  float = 0.0

    carried_ids: List[int] = field(default_factory=list)
    action_state: str = "idle"    
    action_timer: float = 0.0    

    goto_x: Optional[float] = None
    goto_y: Optional[float] = None

    cursor_target_x: Optional[float] = None


def step_robot(robot: SimRobot, dt: float) -> None:
    """Avance la physique du robot simulé de dt secondes."""

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
            print(f"[maman] action terminée")

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
            print(f"[maman] curseur laché en x={robot.x_mm:.0f}")
        else:
            move = min(V_CURSOR * dt, dist)
            robot.x_mm += move * (1 if dx > 0 else -1)


def apply_command(robot: SimRobot, cmd: dict) -> None:
    """Applique une Command reçue de la Jetson sur le robot simulé."""
    if cmd is None:
        return

    kind = cmd.get("kind", "")

    if kind == "GOTO":
        tx = cmd.get("x_mm")
        ty = cmd.get("y_mm")
        if tx is not None and ty is not None:
            if robot.goto_x != tx or robot.goto_y != ty:
                print(f"[maman] GOTO ({tx:.0f}, {ty:.0f})")
            robot.goto_x = tx
            robot.goto_y = ty
            robot.action_state = "going"

    elif kind == "PICKUP":
        if robot.action_state == "idle":
            cid = cmd.get("crate_id", "?")
            robot.carried_ids.append(cid)
            robot.action_state = "picking"
            robot.action_timer = PICKUP_DURATION
            print(f"[maman] PICKUP caisse {cid} porte {len(robot.carried_ids)}/{MAX_CARRIED}")

    elif kind == "DROP_ALL":
        if robot.action_state == "idle":
            zone = cmd.get("zone_name", "?")
            dropped = list(robot.carried_ids)
            robot.carried_ids.clear()
            robot.action_state = "dropping"
            robot.action_timer = DROP_DURATION
            print(f"[maman] DROP_ALL dans '{zone}' déposé {dropped}")

    elif kind == "MOVE_CURSOR":
        tx = cmd.get("x_mm")
        if tx is not None and robot.action_state == "idle":
            robot.cursor_target_x = tx
            robot.action_state = "dragging"
            print(f"[maman] MOVE_CURSOR x={tx:.0f}")

    elif kind == "STOP":
        robot.action_state = "idle"
        robot.goto_x = None
        robot.goto_y = None
        print("[maman] STOP")


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
sock.settimeout(0.05)  

print(f"[maman-v2] listening on UDP {LISTEN_IP}:{LISTEN_PORT}")

robot = SimRobot()
last_rx = time.time()
last_t  = time.time()

action_labels = {
    "idle":     "En route",
    "going":    "GOTO...",
    "picking":  "Ramassage...",
    "dropping": "Dépot...",
    "dragging": "Curseur...",
}

while True:
    now = time.time()
    dt  = now - last_t
    last_t = now

    step_robot(robot, dt)

    try:
        data, addr = sock.recvfrom(65535)
        last_rx = now

        msg = json.loads(data.decode("utf-8"))
        frame_id = msg.get("frame_id", -1)
        cmd      = msg.get("command", None)

        apply_command(robot, cmd)

        world   = msg.get("world", {})
        caisses = world.get("caisses", [])
        action_str = action_labels.get(robot.action_state, robot.action_state)
        print(f"[maman-v2] frame={frame_id:4d} | "
              f"robot=({robot.x_mm:6.0f},{robot.y_mm:6.0f}) | "
              f"{action_str:<18} | "
              f"porte={len(robot.carried_ids)}/{MAX_CARRIED} | "
              f"cmd={cmd['kind'] if cmd else 'None':<12} | "
              f"caisses_vues={len(caisses)}")

        ack = {
            "type":       "ack",
            "frame_id":   frame_id,
            "t_ms":       int(time.time() * 1000),
            "robot_state": {          
                "x_mm":       robot.x_mm,
                "y_mm":       robot.y_mm,
                "theta_rad":  robot.theta_rad,
                "action":     robot.action_state,
                "carried":    list(robot.carried_ids),
            }
        }
        sock.sendto(json.dumps(ack).encode("utf-8"), (addr[0], ACK_PORT))

    except socket.timeout:
        if now - last_rx > 3.0:
            print(f"[maman-v2] (aucun paquet depuis 3s) | "
                  f"robot=({robot.x_mm:.0f},{robot.y_mm:.0f}) action={robot.action_state}")
            last_rx = now
