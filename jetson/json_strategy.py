# jetson_strategy.py
# Layer stratégie Jetson : plan fixe scenario.json
#
#
# Utilisation dans json_main_v2.py :
#
#   runner = JetsonStrategyRunner("scenario.json")
#
#   # À chaque cycle :
#   maman_state = ack.get("robot_state", {})    # renvoyé par maman dans l'ACK
#   cmd = runner.step(world, maman_state)
#   # inclure cmd.to_dict() dans le payload UDP

import math
import json
import time
from dataclasses import dataclass, field
from typing import Optional, List

# -----------------------------------------------------------------------
# Command
# -----------------------------------------------------------------------

@dataclass
class Command:
    kind: str
    x_mm:      Optional[float] = None
    y_mm:      Optional[float] = None
    zone_name: Optional[str]   = None
    crate_id:  Optional[int]   = None
    from_x_mm: Optional[float] = None
    from_y_mm: Optional[float] = None

    def to_dict(self) -> dict:
        d = {"kind": self.kind}
        if self.x_mm      is not None: d["x_mm"]      = round(self.x_mm, 1)
        if self.y_mm      is not None: d["y_mm"]      = round(self.y_mm, 1)
        if self.zone_name is not None: d["zone_name"] = self.zone_name
        if self.crate_id  is not None: d["crate_id"]  = self.crate_id
        if self.from_x_mm is not None: d["from_x_mm"] = round(self.from_x_mm, 1)
        if self.from_y_mm is not None: d["from_y_mm"] = round(self.from_y_mm, 1)
        return d

    def __repr__(self):
        parts = [self.kind]
        if self.x_mm      is not None: parts.append(f"x={self.x_mm:.0f}")
        if self.y_mm      is not None: parts.append(f"y={self.y_mm:.0f}")
        if self.zone_name is not None: parts.append(f"zone={self.zone_name}")
        if self.crate_id  is not None: parts.append(f"crate={self.crate_id}")
        return f"Command({', '.join(parts)})"


# -----------------------------------------------------------------------
# Paramètres
# -----------------------------------------------------------------------

ARRIVAL_TOL        = 60.0    # mm — tolérance arrivée GOTO (vision bruitée)
PICKUP_RANGE       = 150.0   # mm — portée bras (doit matcher maman)
PICKUP_TIMEOUT_S   = 3.0     # s  — timeout si maman ne confirme pas PICKUP
DROP_TIMEOUT_S     = 2.5     # s  — timeout DROP_ALL
GOTO_TIMEOUT_S     = 30.0    # s  — timeout GOTO (évite blocage infini)
MAX_CARRIED        = 8


# -----------------------------------------------------------------------
# JetsonStrategyRunner v2
# -----------------------------------------------------------------------

class JetsonStrategyRunner:
    """
    Exécute le strategy_plan de scenario.json pas à pas.

    Le runner maintient une machine à états simple :
        WAITING_GOTO    → on envoie GOTO, on attend action_done (ou arrivée vision)
        WAITING_PICKUP  → on envoie PICKUP, on attend action_done (ou timeout)
        WAITING_DROP    → on envoie DROP_ALL, on attend action_done (ou timeout)
        WAITING_CURSOR  → on envoie MOVE_CURSOR, on attend action_done
        IDLE            → prêt pour l'étape suivante

    maman_state : dict renvoyé par maman dans son ACK
        {
          "action": "going" | "picking" | "dropping" | "idle" | ...,
          "action_done": true/false,
          "x_mm": ...,          # optionnel (si odométrie)
          "y_mm": ...,
          "carried_count": int
        }
    """

    def __init__(self, scenario_path: str = "scenario.json"):
        self.plan: List[dict] = []
        self.plan_idx: int    = 0

        self._state: str              = "IDLE"
        self._current_cmd: Optional[Command] = None
        self._step_start_time: float  = 0.0

        # Suivi local des caisses portées (cohérence avec maman)
        self._carried_ids: List[int]  = []

        self._load_plan(scenario_path)

    # ------------------------------------------------------------------
    def _load_plan(self, path: str) -> None:
        try:
            with open(path, "r", encoding="utf-8") as f:
                sc = json.load(f)
            self.plan = sc.get("strategy_plan", [])
            print(f"[strategy] plan chargé : {len(self.plan)} étapes")
        except Exception as e:
            print(f"[strategy] ERREUR chargement plan : {e}")
            self.plan = []

    # ------------------------------------------------------------------
    def step(self, world, maman_state: dict = None) -> Optional[Command]:
        """
        Appeler à chaque cycle (10 Hz).
        world       : WorldState réel (world_state.py)
        maman_state : dict extrait de l'ACK maman (peut être None)
        Retourne la Command à envoyer à maman, ou None.
        """
        if maman_state is None:
            maman_state = {}

        now = time.time()
        action_done = maman_state.get("action_done", False)

        # ---- Machine à états ----------------------------------------

        if self._state == "IDLE":
            return self._next_step(world)

        elif self._state == "WAITING_GOTO":
            arrived = self._check_goto_arrived(world, maman_state)
            timeout = (now - self._step_start_time) > GOTO_TIMEOUT_S

            if arrived or timeout:
                if timeout and not arrived:
                    print(f"[strategy] GOTO timeout après {GOTO_TIMEOUT_S}s — on avance quand même")
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            else:
                return self._current_cmd   # continuer d'envoyer le GOTO

        elif self._state == "WAITING_PICKUP":
            timeout = (now - self._step_start_time) > PICKUP_TIMEOUT_S
            if action_done or timeout:
                if timeout and not action_done:
                    print(f"[strategy] PICKUP timeout — on avance")
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            else:
                return self._current_cmd

        elif self._state == "WAITING_DROP":
            timeout = (now - self._step_start_time) > DROP_TIMEOUT_S
            if action_done or timeout:
                if timeout and not action_done:
                    print(f"[strategy] DROP timeout — on avance")
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            else:
                return self._current_cmd

        elif self._state == "WAITING_CURSOR":
            timeout = (now - self._step_start_time) > GOTO_TIMEOUT_S
            if action_done or timeout:
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            else:
                return self._current_cmd

        return None

    # ------------------------------------------------------------------
    def _next_step(self, world) -> Optional[Command]:
        """Lit et lance l'étape courante du plan."""
        if self.plan_idx >= len(self.plan):
            print("[strategy] plan terminé !")
            return None

        step = self.plan[self.plan_idx]
        kind = step.get("type", "WAIT")
        print(f"[strategy] étape {self.plan_idx}/{len(self.plan)-1} : {kind}")

        if kind == "GOTO":
            return self._start_goto(step, world)

        elif kind == "PICKUP_NEARBY":
            return self._start_pickup(world)

        elif kind == "DROP_ALL":
            return self._start_drop(step)

        elif kind == "MOVE_CURSOR":
            return self._start_cursor(step)

        else:
            # WAIT ou inconnu → passer directement
            self.plan_idx += 1
            return self._next_step(world)

    # ------------------------------------------------------------------
    # Démarrage de chaque type d'étape
    # ------------------------------------------------------------------

    def _start_goto(self, step: dict, world) -> Optional[Command]:
        tx = float(step["x_mm"])
        ty = float(step["y_mm"])

        # Vérifier si déjà sur place (ex: étape de départ)
        robot = world.robot.get("us", None)
        if robot is not None:
            d = math.hypot(tx - robot.x_mm, ty - robot.y_mm)
            if d < ARRIVAL_TOL:
                print(f"[strategy] GOTO ({tx:.0f},{ty:.0f}) déjà atteint (d={d:.0f}mm), étape suivante")
                self.plan_idx += 1
                return self._next_step(world)

        cmd = Command(kind="GOTO", x_mm=tx, y_mm=ty)
        self._current_cmd   = cmd
        self._state         = "WAITING_GOTO"
        self._step_start_time = time.time()
        print(f"[strategy] → GOTO ({tx:.0f}, {ty:.0f})")
        return cmd

    def _start_pickup(self, world) -> Optional[Command]:
        """
        Plan fixe : on ramasse la caisse la plus proche détectée.
        (Le plan dit "PICKUP_NEARBY" sans spécifier laquelle — c'est la vision
        qui détermine la caisse réelle à portée.)
        """
        robot = world.robot.get("us", None)
        if robot is None:
            print("[strategy] PICKUP : robot non localisé, on skip")
            self.plan_idx += 1
            return self._next_step(world)

        if len(self._carried_ids) >= MAX_CARRIED:
            print("[strategy] PICKUP : bras pleins, on skip")
            self.plan_idx += 1
            return self._next_step(world)

        # Chercher la caisse au sol la plus proche dans le WorldState
        best = None
        best_dist = float("inf")
        for cid, caisse in world.caisses.items():
            if caisse.status in ("carried", "delivered"):
                continue
            d = math.hypot(caisse.x_mm - robot.x_mm, caisse.y_mm - robot.y_mm)
            if d < best_dist:
                best_dist = d
                best = (cid, caisse.x_mm, caisse.y_mm)

        if best is None:
            print("[strategy] PICKUP : aucune caisse visible, on skip")
            self.plan_idx += 1
            return self._next_step(world)

        cid, cx, cy = best
        print(f"[strategy] → PICKUP caisse {cid} à distance {best_dist:.0f}mm")

        cmd = Command(kind="PICKUP", crate_id=cid)
        self._current_cmd   = cmd
        self._state         = "WAITING_PICKUP"
        self._step_start_time = time.time()
        self._carried_ids.append(cid)
        return cmd

    def _start_drop(self, step: dict) -> Command:
        zone = step.get("zone_name", "our_nest")
        dropped = list(self._carried_ids)
        self._carried_ids.clear()

        cmd = Command(kind="DROP_ALL", zone_name=zone)
        self._current_cmd   = cmd
        self._state         = "WAITING_DROP"
        self._step_start_time = time.time()
        print(f"[strategy] → DROP_ALL zone='{zone}' (caisses: {dropped})")
        return cmd

    def _start_cursor(self, step: dict) -> Command:
        tx = float(step["x_mm"])
        ty = float(step.get("y_mm", 0.0))
        fx = float(step.get("from_x_mm", tx))
        fy = float(step.get("from_y_mm", ty))

        cmd = Command(kind="MOVE_CURSOR", x_mm=tx, y_mm=ty, from_x_mm=fx, from_y_mm=fy)
        self._current_cmd   = cmd
        self._state         = "WAITING_CURSOR"
        self._step_start_time = time.time()
        print(f"[strategy] → MOVE_CURSOR ({fx:.0f},{fy:.0f}) → ({tx:.0f},{ty:.0f})")
        return cmd

    # ------------------------------------------------------------------
    # Vérification arrivée GOTO
    # ------------------------------------------------------------------

    def _check_goto_arrived(self, world, maman_state: dict) -> bool:
        """
        Considère le GOTO accompli si :
        - maman renvoie action_done=True  (odométrie maman)
        - OU la vision confirme que le robot est à < ARRIVAL_TOL de la cible
        Les deux sources sont valides, la première à confirmer l'emporte.
        """
        # Signal maman
        if maman_state.get("action_done", False):
            return True

        # Vision ArUco
        robot = world.robot.get("us", None)
        if robot is not None and self._current_cmd is not None:
            tx = self._current_cmd.x_mm
            ty = self._current_cmd.y_mm
            if tx is not None and ty is not None:
                d = math.hypot(tx - robot.x_mm, ty - robot.y_mm)
                if d < ARRIVAL_TOL:
                    return True

        return False

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "plan_idx":    self.plan_idx,
            "plan_total":  len(self.plan),
            "state":       self._state,
            "current_cmd": repr(self._current_cmd),
            "carried":     list(self._carried_ids),
        }