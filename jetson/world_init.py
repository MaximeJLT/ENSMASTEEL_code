# world_init.py
import json
from world_state import WorldState, Zone

def init_world(zones_path: str = "zones.json") -> WorldState:
    """
    Initialise le WorldState avec les zones lues depuis un fichier JSON.
    zones_path: chemin vers le fichier JSON des zones.
    Retourne une instance de WorldState avec les zones initialisées.
    """
    world = WorldState()

    # 1) lire le JSON
    with open(zones_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 2) créer les Zone et les mettre dans world.zones
    for z in data.get("zones", []):
        zone = Zone()
        zone.name = z["name"]
        zone.x_mm = z["x_mm"]
        zone.y_mm = z["y_mm"]
        zone.width_mm = z["width_mm"]
        zone.height_mm = z["height_mm"]

        world.zones[zone.name] = zone

    return world
