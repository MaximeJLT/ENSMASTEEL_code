# json_main_v3.py
# Boucle principale Jetson — v3
# Intègre JetsonStrategyRunner v2 (plan fixe + action_done maman)

import socket
import json
import time

from vision_aruco import get_objects
from mapping import TableMapper
from world_init import init_world
from world_updater import update_world_state
from json_strategy import JetsonStrategyRunner

# ---- Réseau ----
JETSON_IP   = "127.0.0.1"
JETSON_PORT = 5006
MAMAN_IP    = "127.0.0.1"
MAMAN_PORT  = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((JETSON_IP, JETSON_PORT))
sock.settimeout(0.08)   # 80ms — laisse la place au 10Hz sans bloquer trop longtemps

# ---- Init ----
frame_id    = 0
period_s    = 0.1
mapper      = TableMapper()
world       = init_world("zones.json")
runner      = JetsonStrategyRunner("scenario.json")
maman_state = {}   # dernier état connu de maman (rempli par l'ACK)


def world_to_dict(w):
    rob = w.robot.get("us", None)
    opp = w.opponent.get("enemy", None)
    return {
        "robot": {
            "x_mm": rob.x_mm, "y_mm": rob.y_mm, "theta_rad": rob.theta_rad
        } if rob else None,
        "opponent": {
            "x_mm": opp.x_mm, "y_mm": opp.y_mm, "theta_rad": opp.theta_rad
        } if opp else None,
        "caisses": [
            {"id": int(cid), "x_mm": c.x_mm, "y_mm": c.y_mm, "status": c.status}
            for cid, c in w.caisses.items()
        ],
        "zones":     list(w.zones.keys()),
        "matchtime": w.matchtime,
    }


print(f"[jetson] démarrage → maman {MAMAN_IP}:{MAMAN_PORT}")

while True:
    t0 = time.time()
    frame_id += 1

    # 1) Vision + mapping + world
    objects, table_markers = get_objects()
    try:
        mapping_ok = mapper.update(table_markers)
    except Exception as e:
        print(f"[jetson] mapping error: {e}")
        mapping_ok = False

    update_world_state(world, objects, mapper)

    # 2) Stratégie — on passe l'état maman du cycle précédent
    cmd = runner.step(world, maman_state)
    cmd_dict = cmd.to_dict() if cmd is not None else None

    # 3) Envoi UDP
    msg = {
        "type":       "world_state",
        "t_ms":       int(time.time() * 1000),
        "frame_id":   frame_id,
        "mapping_ok": bool(mapping_ok),
        "world":      world_to_dict(world),
        "command":    cmd_dict,
    }
    payload = json.dumps(msg).encode("utf-8")
    sock.sendto(payload, (MAMAN_IP, MAMAN_PORT))
    t_send = time.time()

    # 4) ACK maman
    try:
        data, addr = sock.recvfrom(65535)
        ack = json.loads(data.decode("utf-8"))

        if ack.get("type") == "ack" and ack.get("frame_id") == frame_id:
            rtt_ms      = (time.time() - t_send) * 1000.0
            maman_state = ack.get("robot_state", {})   # ← état maman mis à jour

            if frame_id % 10 == 0:
                status = runner.status()
                print(
                    f"[jetson] frame={frame_id:4d} "
                    f"rtt={rtt_ms:4.1f}ms "
                    f"étape={status['plan_idx']}/{status['plan_total']} "
                    f"state={status['state']:<18} "
                    f"cmd={cmd_dict['kind'] if cmd_dict else 'None':<14} "
                    f"maman_action={maman_state.get('action','?')}"
                )
        else:
            maman_state = {}

    except socket.timeout:
        maman_state = {}   # pas d'ACK → on repart sans état maman
        if frame_id % 20 == 0:
            print(f"[jetson] frame={frame_id} ACK TIMEOUT")

    # 5) Cadencer
    elapsed = time.time() - t0
    time.sleep(max(0.0, period_s - elapsed))