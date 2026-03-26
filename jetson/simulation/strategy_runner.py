import math
from typing import Optional
from sim_core import WorldState, Command

class StrategyRunner:
    """
    Exécute séquentiellement les étapes de world.strategy_plan.

    Chaque étape est un dict avec un champ "type" parmi :
        GOTO         : {"type": "GOTO", "x_mm": ..., "y_mm": ...}
        PICKUP_NEARBY: {"type": "PICKUP_NEARBY"}
        DROP_ALL     : {"type": "DROP_ALL", "zone_name": ...}
        MOVE_CURSOR  : {"type": "MOVE_CURSOR", "x_mm": ..., "y_mm": ...}
        WAIT         : {"type": "WAIT"}

    Le runner bloque tant que le robot n'est pas arrivé à destination (GOTO)
    ou tant que l'action physique n'est pas terminée (action_state != "idle").
    """

    def __init__(self):
        self.plan_idx: int = 0
        self.arrival_tol: float = 40.0   # mm — distance pour valider un GOTO

    # ------------------------------------------------------------------
    def step(self, world: WorldState) -> Optional[Command]:

        if world.match_finished:
            return None

        # Attendre la fin d'une action physique en cours (pickup / drop / cursor)
        if world.robot.action_state != "idle":
            return None

        plan = getattr(world, "strategy_plan", None)

        # Fallback : si pas de strategy_plan, utiliser strategy_path (ancienne API)
        if not plan:
            return self._step_legacy(world)

        if self.plan_idx >= len(plan):
            return None

        step = plan[self.plan_idx]
        kind = step.get("type", "WAIT")

        # ---- GOTO ----
        if kind == "GOTO":
            tx = float(step["x_mm"])
            ty = float(step["y_mm"])
            rx, ry = world.robot.x_mm, world.robot.y_mm
            d = math.hypot(tx - rx, ty - ry)

            if d < self.arrival_tol:
                # Arrivé — passer à l'étape suivante
                self.plan_idx += 1
                return None
            else:
                return Command(kind="GOTO", x_mm=tx, y_mm=ty)

        # ---- PICKUP_NEARBY ----
        elif kind == "PICKUP_NEARBY":
            self.plan_idx += 1
            return Command(kind="PICKUP_NEARBY")

        # ---- DROP_ALL ----
        elif kind == "DROP_ALL":
            zone = step.get("zone_name", "our_nest")
            self.plan_idx += 1
            return Command(kind="DROP_ALL", zone_name=zone)

        # ---- MOVE_CURSOR ----
        elif kind == "MOVE_CURSOR":
            from_x = float(step.get("from_x_mm", step["x_mm"]))
            from_y = float(step.get("from_y_mm", step["y_mm"]))
            tx = float(step["x_mm"])
            ty = float(step["y_mm"])
            self.plan_idx += 1
            return Command(kind="MOVE_CURSOR",
                           from_x_mm=from_x, from_y_mm=from_y,
                           x_mm=tx, y_mm=ty)

        # ---- WAIT / inconnu ----
        else:
            self.plan_idx += 1
            return None

    # ------------------------------------------------------------------
    # Fallback legacy : strategy_path simple (liste de GOTO)
    # ------------------------------------------------------------------
    def _step_legacy(self, world: WorldState) -> Optional[Command]:
        if self.plan_idx >= len(world.strategy_path):
            return None

        tx, ty = world.strategy_path[self.plan_idx]
        rx, ry = world.robot.x_mm, world.robot.y_mm
        d = math.hypot(tx - rx, ty - ry)

        if d < self.arrival_tol:
            self.plan_idx += 1
            if self.plan_idx >= len(world.strategy_path):
                return None
            tx, ty = world.strategy_path[self.plan_idx]

        return Command(kind="GOTO", x_mm=tx, y_mm=ty)