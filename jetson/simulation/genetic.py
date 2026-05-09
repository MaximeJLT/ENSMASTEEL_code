# genetic.py
# Génération d'individus (plans de jeu) pour l'algo génétique
import json
import random
from typing import List, Dict, Tuple

# Capacité max des bras du robot (cohérent avec sim_core.py)
MAX_CARRIED = 8

def load_world_data(
    zones_path: str = "zones.json",
    scenario_path: str = "scenario.json",
) -> Dict:
    """
    Charge les zones et caisses depuis les JSON et retourne un dict
    contenant les infos utiles à la génération d'individus.
    """
    with open(zones_path, "r", encoding="utf-8") as f:
        zones_data = json.load(f)

    with open(scenario_path, "r", encoding="utf-8") as f:
        scenario_data = json.load(f)

    # On regroupe les caisses bleues par paire (caisses très proches = même tas).
    # On ne garde que les paires côté droit (chez nous, x > 0).
    blue_crates = [c for c in scenario_data.get("crates", [])
                   if c.get("color") == "blue" and c.get("x_mm", 0) > 0]

    reservoirs = []
    used = set()
    for c in blue_crates:
        if c["id"] in used:
            continue
        partners = [c]
        used.add(c["id"])
        for c2 in blue_crates:
            if c2["id"] in used:
                continue
            dist = ((c["x_mm"] - c2["x_mm"])**2 + (c["y_mm"] - c2["y_mm"])**2) ** 0.5
            if dist < 100:
                partners.append(c2)
                used.add(c2["id"])
        cx = sum(p["x_mm"] for p in partners) / len(partners)
        cy = sum(p["y_mm"] for p in partners) / len(partners)
        reservoirs.append({
            "x_mm": cx,
            "y_mm": cy,
            "crate_ids": [p["id"] for p in partners],
        })

    # Les zones sont dans une liste sous la clé "zones"
    zones_list = zones_data.get("zones", [])
    zones_by_name = {z["name"]: z for z in zones_list}

    # --- Pantries valides pour le drop ---
    valid_pantries = ["pantry_1", "pantry_2", "pantry_5",
                      "pantry_6", "pantry_7", "pantry_8"]
    pantries = {}
    for name in valid_pantries:
        if name in zones_by_name:
            z = zones_by_name[name]
            cx = z["x_mm"] + z["width_mm"] / 2
            cy = z["y_mm"] + z["height_mm"] / 2
            pantries[name] = {"x_mm": cx, "y_mm": cy}

    # --- Nid ---
    nest = zones_by_name.get("our_nest", {"x_mm": 975, "y_mm": 625,
                                           "width_mm": 350, "height_mm": 350})
    nest_pos = {
        "x_mm": nest["x_mm"] + nest["width_mm"] / 2,
        "y_mm": nest["y_mm"] + nest["height_mm"] / 2,
    }

    cursor = {
        "from_x_mm": 1250,
        "from_y_mm": -950,
        "x_mm": 800,
        "y_mm": -900,
    }

    return {
        "reservoirs": reservoirs,
        "pantries": pantries,
        "nest": nest_pos,
        "cursor": cursor,
        "robot_start": scenario_data.get("robot_start", {"x_mm": 1150, "y_mm": 800}),
    }








def generate_individual(world_data: Dict) -> List[Dict]:
    """
    Génère un plan de jeu aléatoire mais valide.
    
    Retourne une liste d'actions au format compatible scenario.json :
        [{"type": "GOTO", "x_mm": ..., "y_mm": ...}, 
         {"type": "PICKUP_NEARBY"},
         {"type": "DROP_ALL", "zone_name": ...}, ...]
    """
    reservoirs = world_data["reservoirs"]
    pantries = world_data["pantries"]
    nest = world_data["nest"]

    # 1) Mélanger les réservoirs dans un ordre aléatoire
    shuffled_reservoirs = reservoirs.copy()
    random.shuffle(shuffled_reservoirs)

    # 2) Liste des noms de pantries pour tirage aléatoire
    pantry_names = list(pantries.keys())

    # 3) Construire le plan cycle par cycle
    plan = []
    reservoir_idx = 0
    total_reservoirs = len(shuffled_reservoirs)

    while reservoir_idx < total_reservoirs:
        # a) Choisir combien de réservoirs visiter avant le prochain drop (1 ou 2)
        nb_pickups = random.choice([1, 2])
        nb_pickups = min(nb_pickups, total_reservoirs - reservoir_idx)   #???

        # b) Ajouter les GOTO + PICKUP pour ces réservoirs
        for _ in range(nb_pickups):
            r = shuffled_reservoirs[reservoir_idx]
            plan.append({"type": "GOTO", "x_mm": r["x_mm"], "y_mm": r["y_mm"]})
            plan.append({"type": "PICKUP_NEARBY"})
            reservoir_idx += 1

        # c) Choisir une pantry aléatoire et ajouter GOTO + DROP
        chosen_pantry = random.choice(pantry_names)
        p = pantries[chosen_pantry]
        plan.append({"type": "GOTO", "x_mm": p["x_mm"], "y_mm": p["y_mm"]})
        plan.append({"type": "DROP_ALL", "zone_name": chosen_pantry})

    # 4) Optionnel : tirer le curseur (50% de chance)
    if random.random() < 0.5:
        cursor = world_data["cursor"]
        # Pour tirer le curseur, il faut d'abord aller à sa position de départ
        cursor_block = [
            {"type": "GOTO",
            "x_mm": cursor["from_x_mm"],
            "y_mm": cursor["from_y_mm"]},
            {"type": "MOVE_CURSOR",
            "from_x_mm": cursor["from_x_mm"],
            "from_y_mm": cursor["from_y_mm"],
            "x_mm":      cursor["x_mm"],
            "y_mm":      cursor["y_mm"]},
        ]
        # Insérer ce bloc à une position aléatoire dans le plan
        insert_pos = random.randint(0, len(plan))
        plan[insert_pos:insert_pos] = cursor_block

    # 5) Optionnel : retour au nid (70% de chance)
    if random.random() < 0.7:
        plan.append({"type": "GOTO", "x_mm": nest["x_mm"], "y_mm": nest["y_mm"]})

    return plan