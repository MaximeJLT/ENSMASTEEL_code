# world_state_updater.py
# But: updater le WorldState a partir des detections ArUco (u_px, v_px)
# v0: on met u_px/v_px dans x_mm/y_mm (TEMPORAIRE) en attendant le mapping.

from world_state import Robot, Caisse, Opponent  # adapte si ton fichier s'appelle autrement
from mapping import TableMapper

# TODO: 
ROBOT_IDS = set([       # IDs ArUco de notre robot
   13
])

OPPONENT_IDS = set([     # IDs ArUco de l'adversaire
   None # ex: 20
])

TABLE_CORNER_IDS = {0, 1, 2, 3}  # ArUco fixes de mapping, à ignorer pour les caisses

def _px_to_mm(mapper, u_px, v_px):
    """
    Retourne les coordonnées mm correspondantes aux coordonnées px, 
    en utilisant le mapper.
    """
    if mapper is None:
        return None
    if getattr(mapper, "last_ok", False) is not True:
        return None
    return mapper.pixel_to_world(u_px, v_px)

def update_world_state(world, detections, mapper=None):
    """
    world: instance de WorldState
    detections: liste de dicts avec au moins {"id": int, "u_px": float, "v_px": float}
    Met a jour le WorldState avec les detections ArUco, donc pour le robot, 
    le/les adversaires et les caisses.
    """

    table_corner_ids = mapper.table_ids if mapper is not None else set()

    world.caisses.clear()  # on clear les caisses a chaque update, on les recrée a partir des detections (v0)
    world.robot.clear()   # idem pour le robot (v0: on gère pas encore la persistance des objets, juste update "us")
    world.opponent.clear() # idem pour l'adversaire

    for det in detections:
        det_id = det.get("id", None)
        u_px = det.get("u_px", None)
        v_px = det.get("v_px", None)

        if det_id is None or u_px is None or v_px is None:
            continue

        mm = _px_to_mm(mapper, u_px, v_px)
        if mm is None:
            continue #mapping pas dispo, on skip la detection pour l'instant

        x_mm, y_mm = mm

        # 1) Classer la detection: robot / adversaire / caisse
        if det_id in ROBOT_IDS:
            # 2) Creer l'objet si besoin
            if "us" not in world.robot:
                world.robot["us"] = Robot()
            # 3) Mettre a jour (v0: px -> "mm" temporaire)
            world.robot["us"].x_mm = float(x_mm)
            world.robot["us"].y_mm = float(y_mm)

        elif det_id in OPPONENT_IDS:
            if "enemy" not in world.opponent:
                world.opponent["enemy"] = Opponent()

            world.opponent["enemy"].x_mm = float(x_mm)
            world.opponent["enemy"].y_mm = float(y_mm)

        elif det_id in table_corner_ids:
            continue  # coin de table, on skip

        else:
            # CAISSE
            if det_id not in world.caisses:
                world.caisses[det_id] = Caisse()
                world.caisses[det_id].id = int(det_id)
            world.caisses[det_id].x_mm = float(x_mm)
            world.caisses[det_id].y_mm = float(y_mm)



