# json_strategy.py
# Layer stratégie Jetson : plan fixe scenario.json + évitement adversaire
#
# Utilisation dans json_main.py :
#
#   runner = JetsonStrategyRunner("scenario.json")
#
#   # À chaque cycle :
#   maman_state = ack.get("robot_state", {})
#   cmd = runner.step(world, maman_state)
#   # inclure cmd.to_dict() dans le payload UDP

import math
import json
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

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
# Paramètres stratégie
# -----------------------------------------------------------------------

ARRIVAL_TOL        = 60.0    # mm — tolérance arrivée GOTO (vision bruitée)
PICKUP_RANGE       = 150.0   # mm — portée bras
PICKUP_TIMEOUT_S   = 3.0     # s  — timeout PICKUP
DROP_TIMEOUT_S     = 2.5     # s  — timeout DROP_ALL
GOTO_TIMEOUT_S     = 30.0    # s  — timeout GOTO global
MAX_CARRIED        = 8

# -----------------------------------------------------------------------
# Paramètres évitement adversaire
# -----------------------------------------------------------------------

DANGER_RADIUS_MM   = 450.0   # mm — distance adversaire/chemin déclenchant le stop
STOP_TIMEOUT_S     = 0.1     # s  — attente avant de tenter le contournement
DETOUR_MARGIN_MM   = 100.0   # mm — marge au-delà du rayon de danger pour le waypoint
DETOUR_ARRIVAL_TOL = 80.0    # mm — tolérance arrivée au waypoint de détour
AVOID_COOLDOWN_S   = 4.0     # s  — après un détour, délai avant de re-checker
OPP_STATIC_SPEED_MM_S = 80.0   # mm/s — sous ce seuil, adversaire considéré immobile
FRONTAL_COS_THRESHOLD = 0.3    # > ce seuil = adversaire fonce sur nous

# Limites de la map
MAP_X_MIN, MAP_X_MAX = -1450.0, 1450.0
MAP_Y_MIN, MAP_Y_MAX =  -950.0,  950.0

# -----------------------------------------------------------------------
# Helpers géométriques (portés depuis strategy_runner.py)
# -----------------------------------------------------------------------

def _dist_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float, float]:
    """Distance du point P au segment AB. Retourne (dist, cx, cy)."""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-9:
        return math.hypot(px - ax, py - ay), ax, ay
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy), cx, cy

def _segments_min_distance(
    a1x: float, a1y: float, a2x: float, a2y: float,
    b1x: float, b1y: float, b2x: float, b2y: float,
) -> float:
    """Distance minimale entre deux segments [A1A2] et [B1B2]."""
    d1, _, _ = _dist_point_to_segment(b1x, b1y, a1x, a1y, a2x, a2y)
    d2, _, _ = _dist_point_to_segment(b2x, b2y, a1x, a1y, a2x, a2y)
    d3, _, _ = _dist_point_to_segment(a1x, a1y, b1x, b1y, b2x, b2y)
    d4, _, _ = _dist_point_to_segment(a2x, a2y, b1x, b1y, b2x, b2y)
    return min(d1, d2, d3, d4)

def _compute_detour_waypoint(
    rx: float, ry: float,
    tx: float, ty: float,
    ox: float, oy: float,
    danger_r: float,
    margin: float,
) -> Tuple[float, float]:
    """Waypoint perpendiculaire au segment RT, côté opposé à l'adversaire."""
    dx, dy = tx - rx, ty - ry
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return tx, ty
    ux, uy = dx / seg_len, dy / seg_len
    # cross > 0 → adversaire à gauche → on passe à droite
    cross = ux * (oy - ry) - uy * (ox - rx)
    perp = (uy, -ux) if cross > 0 else (-uy, ux)
    _, cpx, cpy = _dist_point_to_segment(ox, oy, rx, ry, tx, ty)
    offset = danger_r + margin
    return cpx + perp[0] * offset, cpy + perp[1] * offset


# -----------------------------------------------------------------------
# JetsonStrategyRunner v3 — avec évitement adversaire
# -----------------------------------------------------------------------

class JetsonStrategyRunner:
    """
    Exécute le strategy_plan de scenario.json pas à pas,
    avec détection et évitement de l'adversaire pendant les GOTO.

    États principaux :
        IDLE            → prêt pour l'étape suivante du plan
        WAITING_GOTO    → en route vers une cible (plan normal)
        WAITING_PICKUP  → attend confirmation ramassage de maman
        WAITING_DROP    → attend confirmation dépôt de maman
        WAITING_CURSOR  → attend fin déplacement curseur

    Sous-états d'évitement (actifs pendant WAITING_GOTO) :
        AVOID_STOPPED   → adversaire détecté, on attend STOP_TIMEOUT_S
        AVOID_DETOURING → on se dirige vers le waypoint de contournement

    maman_state (ACK maman) :
        {
          "action": "going" | "picking" | "dropping" | "idle",
          "action_done": true/false,
          "x_mm": ...,
          "y_mm": ...,
          "carried_count": int
        }
    """

    def __init__(self, scenario_path: str = "scenario.json"):
        self.plan: List[dict] = []
        self.plan_idx: int    = 0

        # État principal
        self._state: str              = "IDLE"
        self._current_cmd: Optional[Command] = None
        self._step_start_time: float  = 0.0

        # Caisses portées (suivi local)
        self._carried_ids: List[int]  = []

        # --- Évitement ---
        self._avoid_state: str                          = "NORMAL"
        self._avoid_stop_start: float                   = 0.0
        self._avoid_detour_target: Optional[Tuple[float, float]] = None
        self._avoid_resume_target: Optional[Tuple[float, float]] = None
        self._avoid_cooldown_until: float               = 0.0
        self._opp_prev_x: Optional[float] = None
        self._opp_prev_y: Optional[float] = None
        self._opp_prev_t: float = 0.0

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
        """
        if maman_state is None:
            maman_state = {}

        now         = time.time()
        action_done = maman_state.get("action_done", False)

        # ---- IDLE : lancer l'étape suivante ----------------------------
        if self._state == "IDLE":
            return self._next_step(world)

        # ---- WAITING_GOTO : déplacement en cours ----------------------
        elif self._state == "WAITING_GOTO":

            # 1) Vérifier évitement adversaire (prioritaire)
            avoid_cmd = self._handle_avoidance(world, now)
            if avoid_cmd is not None:
                return avoid_cmd   # STOP ou DÉTOUR → on renvoie ça à maman

            # 2) Évitement terminé ou inactif → vérifier arrivée normale
            arrived = self._check_goto_arrived(world, maman_state)
            timeout = (now - self._step_start_time) > GOTO_TIMEOUT_S

            if arrived or timeout:
                if timeout and not arrived:
                    print(f"[strategy] GOTO timeout {GOTO_TIMEOUT_S}s — on avance")
                self._state = "IDLE"
                self.plan_idx += 1
                # Réinitialiser l'évitement pour la prochaine étape
                self._avoid_state = "NORMAL"
                self._avoid_detour_target = None
                self._avoid_resume_target = None
                return self._next_step(world)
            else:
                return self._current_cmd   # continuer d'envoyer le GOTO

        # ---- WAITING_PICKUP --------------------------------------------
        elif self._state == "WAITING_PICKUP":
            timeout = (now - self._step_start_time) > PICKUP_TIMEOUT_S
            if action_done or timeout:
                if timeout and not action_done:
                    print(f"[strategy] PICKUP timeout — on avance")
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            return self._current_cmd

        # ---- WAITING_DROP ----------------------------------------------
        elif self._state == "WAITING_DROP":
            timeout = (now - self._step_start_time) > DROP_TIMEOUT_S
            if action_done or timeout:
                if timeout and not action_done:
                    print(f"[strategy] DROP timeout — on avance")
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            return self._current_cmd

        # ---- WAITING_CURSOR --------------------------------------------
        elif self._state == "WAITING_CURSOR":
            timeout = (now - self._step_start_time) > GOTO_TIMEOUT_S
            if action_done or timeout:
                self._state = "IDLE"
                self.plan_idx += 1
                return self._next_step(world)
            return self._current_cmd

        return None

    # ------------------------------------------------------------------
    # Gestion de l'évitement adversaire
    # ------------------------------------------------------------------

    def _handle_avoidance(self, world, now: float) -> Optional[Command]:
        """
        Appelé uniquement pendant WAITING_GOTO.

        Retourne :
          - une Command (GOTO sur place ou vers détour) si évitement actif
          - None si tout est normal → laisser le GOTO continuer
        """
        # Position robot réel (vision ArUco)
        robot = world.robot.get("us", None)
        if robot is None:
            return None   # pas de localisation → on ne peut pas évaluer

        rx, ry = robot.x_mm, robot.y_mm

        # Position adversaire réel (vision ArUco)
        opp = world.opponent.get("enemy", None)
        if opp is None:
            # Pas d'adversaire visible → réinitialiser et continuer
            if self._avoid_state != "NORMAL":
                print("[avoid] adversaire perdu de vue → reprise NORMAL")
                self._avoid_state = "NORMAL"
                self._avoid_detour_target = None
                self._avoid_resume_target = None
            return None

        ox, oy = opp.x_mm, opp.y_mm

        # ---- Vitesse + position future de l'adversaire ----
        LOOKAHEAD_S = 1.0
        vx_opp, vy_opp = 0.0, 0.0
        if self._opp_prev_x is not None:
            dt_opp = now - self._opp_prev_t
            if dt_opp > 1e-3:
                vx_opp = (ox - self._opp_prev_x) / dt_opp
                vy_opp = (oy - self._opp_prev_y) / dt_opp

        ox_pred = ox + vx_opp * LOOKAHEAD_S
        oy_pred = oy + vy_opp * LOOKAHEAD_S
        opp_speed = math.hypot(vx_opp, vy_opp)
        opp_is_moving = opp_speed > OPP_STATIC_SPEED_MM_S

        self._opp_prev_x = ox
        self._opp_prev_y = oy
        self._opp_prev_t = now

        # Cible GOTO courante
        if self._avoid_state == "AVOID_DETOURING" and self._avoid_resume_target:
            tx, ty = self._avoid_resume_target
        elif self._current_cmd is not None and self._current_cmd.x_mm is not None:
            tx, ty = self._current_cmd.x_mm, self._current_cmd.y_mm
        else:
            return None

        # Distance : couloir si mobile, point si statique
        if opp_is_moving:
            dist_to_path = _segments_min_distance(rx, ry, tx, ty,
                                                   ox, oy, ox_pred, oy_pred)
        else:
            dist_to_path, _, _ = _dist_point_to_segment(ox, oy, rx, ry, tx, ty)

        opp_on_path = dist_to_path < DANGER_RADIUS_MM

        # ---- NORMAL ---------------------------------------------------
        if self._avoid_state == "NORMAL":
            if now < self._avoid_cooldown_until:
                return None

            if opp_on_path:
                print(f"[avoid] ⚠ Adversaire sur le chemin "
                      f"(dist={dist_to_path:.0f}mm) → STOP")
                self._avoid_state      = "AVOID_STOPPED"
                self._avoid_stop_start = now
                self._avoid_resume_target = (tx, ty)
            return None

        # ---- AVOID_STOPPED --------------------------------------------
        elif self._avoid_state == "AVOID_STOPPED":
            if not opp_on_path:
                print("[avoid] ✓ Voie libre → reprise GOTO")
                self._avoid_state = "NORMAL"
                self._avoid_resume_target = None
                return None

            elapsed = now - self._avoid_stop_start
            if elapsed >= STOP_TIMEOUT_S:
                # ---- 3 cas selon l'adversaire ----

                if not opp_is_moving:
                    # CAS 1 : arrêté → contournement classique
                    dtx, dty = _compute_detour_waypoint(
                        rx, ry, tx, ty, ox, oy,
                        DANGER_RADIUS_MM, DETOUR_MARGIN_MM,
                    )
                    dtx = max(MAP_X_MIN, min(MAP_X_MAX, dtx))
                    dty = max(MAP_Y_MIN, min(MAP_Y_MAX, dty))
                    print(f"[avoid] CAS 1 (arrêté) → contournement ({dtx:.0f},{dty:.0f})")
                    self._avoid_state         = "AVOID_DETOURING"
                    self._avoid_detour_target = (dtx, dty)
                    return Command(kind="GOTO", x_mm=dtx, y_mm=dty)

                # Mobile : fonce-t-il sur nous ?
                dx_to_us = rx - ox
                dy_to_us = ry - oy
                dist_to_us = math.hypot(dx_to_us, dy_to_us)
                if dist_to_us > 1e-6:
                    cos_angle = (vx_opp * dx_to_us + vy_opp * dy_to_us) / (opp_speed * dist_to_us)
                else:
                    cos_angle = 0.0

                if cos_angle > FRONTAL_COS_THRESHOLD:
                    # CAS 3 : fonce sur nous → écart latéral (sur position prédite)
                    dtx, dty = _compute_detour_waypoint(
                        rx, ry, tx, ty, ox_pred, oy_pred,
                        DANGER_RADIUS_MM, DETOUR_MARGIN_MM,
                    )
                    dtx = max(MAP_X_MIN, min(MAP_X_MAX, dtx))
                    dty = max(MAP_Y_MIN, min(MAP_Y_MAX, dty))
                    print(f"[avoid] CAS 3 (fonce dessus) → écart latéral ({dtx:.0f},{dty:.0f})")
                    self._avoid_state         = "AVOID_DETOURING"
                    self._avoid_detour_target = (dtx, dty)
                    return Command(kind="GOTO", x_mm=dtx, y_mm=dty)
                else:
                    # CAS 2 : s'éloigne ou passe à côté → on attend
                    print(f"[avoid] CAS 2 (s'éloigne, cos={cos_angle:.2f}) → attente")
                    return Command(kind="GOTO", x_mm=rx, y_mm=ry)
            else:
                return Command(kind="GOTO", x_mm=rx, y_mm=ry)

        # ---- AVOID_DETOURING ------------------------------------------
        elif self._avoid_state == "AVOID_DETOURING":
            dtx, dty = self._avoid_detour_target
            d_detour = math.hypot(dtx - rx, dty - ry)

            if d_detour < DETOUR_ARRIVAL_TOL:
                print("[avoid] ✓ Détour atteint → reprise GOTO vers cible")
                self._avoid_state         = "NORMAL"
                self._avoid_detour_target = None
                self._avoid_resume_target = None
                self._avoid_cooldown_until = now + AVOID_COOLDOWN_S
                return None

            return Command(kind="GOTO", x_mm=dtx, y_mm=dty)

        return None

    # ------------------------------------------------------------------
    # Étape suivante du plan
    # ------------------------------------------------------------------

    def _next_step(self, world) -> Optional[Command]:
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
            self.plan_idx += 1
            return self._next_step(world)

    # ------------------------------------------------------------------
    def _start_goto(self, step: dict, world) -> Optional[Command]:
        tx = float(step["x_mm"])
        ty = float(step["y_mm"])

        robot = world.robot.get("us", None)
        if robot is not None:
            d = math.hypot(tx - robot.x_mm, ty - robot.y_mm)
            if d < ARRIVAL_TOL:
                print(f"[strategy] GOTO ({tx:.0f},{ty:.0f}) déjà atteint, étape suivante")
                self.plan_idx += 1
                return self._next_step(world)

        cmd = Command(kind="GOTO", x_mm=tx, y_mm=ty)
        self._current_cmd     = cmd
        self._state           = "WAITING_GOTO"
        self._step_start_time = time.time()
        # Réinitialiser l'évitement à chaque nouveau GOTO
        self._avoid_state         = "NORMAL"
        self._avoid_detour_target = None
        self._avoid_resume_target = None
        print(f"[strategy] → GOTO ({tx:.0f}, {ty:.0f})")
        return cmd

    def _start_pickup(self, world) -> Optional[Command]:
        robot = world.robot.get("us", None)
        if robot is None:
            print("[strategy] PICKUP : robot non localisé, skip")
            self.plan_idx += 1
            return self._next_step(world)

        if len(self._carried_ids) >= MAX_CARRIED:
            print("[strategy] PICKUP : bras pleins, skip")
            self.plan_idx += 1
            return self._next_step(world)

        best, best_dist = None, float("inf")
        for cid, caisse in world.caisses.items():
            if caisse.status in ("carried", "delivered"):
                continue
            d = math.hypot(caisse.x_mm - robot.x_mm, caisse.y_mm - robot.y_mm)
            if d < best_dist:
                best_dist = d
                best = (cid, caisse.x_mm, caisse.y_mm)

        if best is None:
            print("[strategy] PICKUP : aucune caisse visible, skip")
            self.plan_idx += 1
            return self._next_step(world)

        cid, cx, cy = best
        print(f"[strategy] → PICKUP caisse {cid} à {best_dist:.0f}mm")
        cmd = Command(kind="PICKUP", crate_id=cid)
        self._current_cmd     = cmd
        self._state           = "WAITING_PICKUP"
        self._step_start_time = time.time()
        self._carried_ids.append(cid)
        return cmd

    def _start_drop(self, step: dict) -> Command:
        zone = step.get("zone_name", "our_nest")
        dropped = list(self._carried_ids)
        self._carried_ids.clear()
        cmd = Command(kind="DROP_ALL", zone_name=zone)
        self._current_cmd     = cmd
        self._state           = "WAITING_DROP"
        self._step_start_time = time.time()
        print(f"[strategy] → DROP_ALL zone='{zone}' (caisses: {dropped})")
        return cmd

    def _start_cursor(self, step: dict) -> Command:
        tx = float(step["x_mm"])
        ty = float(step.get("y_mm", 0.0))
        fx = float(step.get("from_x_mm", tx))
        fy = float(step.get("from_y_mm", ty))
        cmd = Command(kind="MOVE_CURSOR", x_mm=tx, y_mm=ty, from_x_mm=fx, from_y_mm=fy)
        self._current_cmd     = cmd
        self._state           = "WAITING_CURSOR"
        self._step_start_time = time.time()
        print(f"[strategy] → MOVE_CURSOR ({fx:.0f},{fy:.0f}) → ({tx:.0f},{ty:.0f})")
        return cmd

    # ------------------------------------------------------------------
    def _check_goto_arrived(self, world, maman_state: dict) -> bool:
        if maman_state.get("action_done", False):
            return True
        robot = world.robot.get("us", None)
        if robot is not None and self._current_cmd is not None:
            tx, ty = self._current_cmd.x_mm, self._current_cmd.y_mm
            if tx is not None and ty is not None:
                if math.hypot(tx - robot.x_mm, ty - robot.y_mm) < ARRIVAL_TOL:
                    return True
        return False

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "plan_idx":     self.plan_idx,
            "plan_total":   len(self.plan),
            "state":        self._state,
            "avoid_state":  self._avoid_state,
            "current_cmd":  repr(self._current_cmd),
            "carried":      list(self._carried_ids),
            "detour_target": self._avoid_detour_target,
        }
