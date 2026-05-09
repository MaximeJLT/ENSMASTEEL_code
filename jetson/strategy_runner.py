import math
import time
from typing import Optional, Tuple
from sim_core import WorldState, Command

# -----------------------------------------------------------------------
# Param�tres d'�vitement
# -----------------------------------------------------------------------

DANGER_RADIUS_MM   = 450.0   # distance robot\u2194adversaire d�clenchant le stop
STOP_TIMEOUT_S     = 0.1     # attente avant de tenter le contournement
DETOUR_MARGIN_MM   = 100.0   # marge suppl�mentaire au-del� du rayon de danger
DETOUR_ARRIVAL_TOL = 60.0    # tol�rance arriv�e au waypoint de d�tour
ARRIVAL_TOL        = 40.0    # tol�rance arriv�e destination normale
AVOID_COOLDOWN_S   = 4.0     # apr�s un d�tour, d�lai avant de re-checker l'adversaire
# Vitesse adversaire en dessous de laquelle on le considère immobile (mm/s)
OPP_STATIC_SPEED_MM_S = 80.0

# Limites de la map (à ajuster selon ton zones.json)
MAP_X_MIN, MAP_X_MAX = -1450.0, 1450.0
MAP_Y_MIN, MAP_Y_MAX =  -950.0,  950.0


# -----------------------------------------------------------------------
# Helpers g�om�triques
# -----------------------------------------------------------------------

def _dist_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float, float]:
    """
    Distance du point P au segment AB.
    Retourne (distance, closest_x, closest_y).
    """
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
    """
    Calcule un waypoint de contournement perpendiculaire au segment RT,
    du c�t� oppos� � l'adversaire.
    """
    dx, dy = tx - rx, ty - ry
    seg_len = math.hypot(dx, dy)

    if seg_len < 1e-6:
        return tx, ty

    ux, uy = dx / seg_len, dy / seg_len
    perp_l = (-uy,  ux)
    perp_r = ( uy, -ux)

    # cross > 0 \u2192 adversaire � gauche \u2192 on passe � droite
    cross = ux * (oy - ry) - uy * (ox - rx)
    perp = perp_r if cross > 0 else perp_l

    _, cpx, cpy = _dist_point_to_segment(ox, oy, rx, ry, tx, ty)

    offset = danger_r + margin
    return cpx + perp[0] * offset, cpy + perp[1] * offset
# -----------------------------------------------------------------------
# StrategyRunner avec �vitement adversaire
# -----------------------------------------------------------------------

class StrategyRunner:
    """
    Ex�cute s�quentiellement les �tapes de world.strategy_plan.

    �tats internes d'�vitement :
        NORMAL    \u2192 ex�cution normale du plan
        STOPPED   \u2192 adversaire bloque le chemin, on attend STOP_TIMEOUT_S
        DETOURING \u2192 on se dirige vers un waypoint de contournement
    """

    def __init__(self):
        self.plan_idx: int = 0
        self.arrival_tol: float = ARRIVAL_TOL

        self._avoid_state: str             = "NORMAL"
        self._stop_start_time: float       = 0.0
        self._detour_target: Optional[Tuple[float, float]] = None
        self._resume_target: Optional[Tuple[float, float]] = None
        self._cooldown_until: float        = 0.0   # world.t_s en dessous duquel on ignore l'adversaire
        self._opp_prev_x: Optional[float] = None
        self._opp_prev_y: Optional[float] = None
        self._opp_prev_t: float = 0.0

    # ------------------------------------------------------------------
    def step(self, world: WorldState) -> Optional[Command]:

        if world.match_finished:
            return None

        if world.robot.action_state != "idle":
            return None

        plan = getattr(world, "strategy_plan", None)

        if not plan:
            return self._step_legacy(world)

        if self.plan_idx >= len(plan):
            return None

        step = plan[self.plan_idx]
        kind = step.get("type", "WAIT")

        # ---- �vitement adversaire (pendant GOTO ou si d�j� en �vitement) ----
        if kind == "GOTO" or self._avoid_state != "NORMAL":
            cmd = self._handle_avoidance(world, step)
            if cmd is not None:
                return cmd
            # None = reprendre plan normal

        # ---- GOTO ----
        if kind == "GOTO":
            return self._exec_goto(world, step)

        elif kind == "PICKUP_NEARBY":
            self.plan_idx += 1
            return Command(kind="PICKUP_NEARBY")

        elif kind == "DROP_ALL":
            zone = step.get("zone_name", "our_nest")
            self.plan_idx += 1
            return Command(kind="DROP_ALL", zone_name=zone)

        elif kind == "MOVE_CURSOR":
            from_x = float(step.get("from_x_mm", step["x_mm"]))
            from_y = float(step.get("from_y_mm", step["y_mm"]))
            tx = float(step["x_mm"])
            ty = float(step["y_mm"])
            self.plan_idx += 1
            return Command(kind="MOVE_CURSOR",
                           from_x_mm=from_x, from_y_mm=from_y,
                           x_mm=tx, y_mm=ty)

        else:
            self.plan_idx += 1
            return None
        

    # ------------------------------------------------------------------
    def _handle_avoidance(self, world: WorldState, step: dict) -> Optional[Command]:
        """
        Retourne une Command si �vitement actif, None sinon (plan reprend).
        """
        opp = world.opponent
        rx, ry = world.robot.x_mm, world.robot.y_mm

        if self._avoid_state == "DETOURING" and self._resume_target:
            tx, ty = self._resume_target
        else:
            tx = float(step.get("x_mm", rx))
            ty = float(step.get("y_mm", ry))

        if opp is None:
            self._avoid_state = "NORMAL"
            return None

        ox, oy = opp.x_mm, opp.y_mm

        # ---- Vitesse + position future de l'adversaire ----
        LOOKAHEAD_S = 1.0
        vx_opp, vy_opp = 0.0, 0.0
        if self._opp_prev_x is not None:
            dt_opp = world.t_s - self._opp_prev_t
            if dt_opp > 1e-3:
                vx_opp = (ox - self._opp_prev_x) / dt_opp
                vy_opp = (oy - self._opp_prev_y) / dt_opp

        ox_pred = ox + vx_opp * LOOKAHEAD_S
        oy_pred = oy + vy_opp * LOOKAHEAD_S
        opp_speed = math.hypot(vx_opp, vy_opp)
        opp_is_moving = opp_speed > OPP_STATIC_SPEED_MM_S

        self._opp_prev_x = ox
        self._opp_prev_y = oy
        self._opp_prev_t = world.t_s

        # On vérifie si notre trajectoire est gênée
        # Pour mobile : on regarde si on croise le "couloir" entre sa pos actuelle et future
        # Pour statique : on regarde juste sa position actuelle
        if opp_is_moving:
            # Distance min entre notre segment robot→cible et son segment pos→pos_future
            dist_to_opp = _segments_min_distance(rx, ry, tx, ty,
                                                ox, oy, ox_pred, oy_pred)
        else:
            dist_to_opp, _, _ = _dist_point_to_segment(ox, oy, rx, ry, tx, ty)

        opp_on_path = dist_to_opp < DANGER_RADIUS_MM

        # ---- NORMAL ----
        if self._avoid_state == "NORMAL":
            # Cooldown actif apr�s un d�tour \u2192 on ignore l'adversaire temporairement
            if world.t_s < self._cooldown_until:
                return None

            if opp_on_path:
                print(f"[avoid] \u26a0 Adversaire détecté (dist={dist_to_opp:.0f}mm) \u2192 STOP")
                self._avoid_state     = "STOPPED"
                self._stop_start_time = world.t_s
                self._resume_target   = (tx, ty)
            return None

        # ---- STOPPED ----
        elif self._avoid_state == "STOPPED":
            if not opp_on_path:
                print("[avoid] \u2713 Voie libre \u2192 reprise NORMAL")
                self._avoid_state   = "NORMAL"
                self._resume_target = None
                return None

            elapsed = world.t_s - self._stop_start_time
            remaining = max(0.0, STOP_TIMEOUT_S - elapsed)
            if elapsed >= STOP_TIMEOUT_S:
                # ---- 3 cas selon l'adversaire ----

                if not opp_is_moving:
                    # CAS 1 : adversaire arrêté → contournement classique
                    dtx, dty = _compute_detour_waypoint(
                        rx, ry, tx, ty, ox, oy,
                        DANGER_RADIUS_MM, DETOUR_MARGIN_MM,
                    )
                    dtx = max(MAP_X_MIN, min(MAP_X_MAX, dtx))
                    dty = max(MAP_Y_MIN, min(MAP_Y_MAX, dty))
                    print(f"[avoid] CAS 1 (arrêté) → contournement ({dtx:.0f},{dty:.0f})")
                    self._avoid_state   = "DETOURING"
                    self._detour_target = (dtx, dty)
                    return Command(kind="GOTO", x_mm=dtx, y_mm=dty)

                # Adversaire mobile → fonce sur nous ou s'éloigne ?
                # Vecteur adversaire → nous
                dx_to_us = rx - ox
                dy_to_us = ry - oy
                dist_to_us = math.hypot(dx_to_us, dy_to_us)

                if dist_to_us > 1e-6:
                    # Produit scalaire normalisé entre vitesse adv et direction vers nous
                    cos_angle = (vx_opp * dx_to_us + vy_opp * dy_to_us) / (opp_speed * dist_to_us)
                else:
                    cos_angle = 0.0

                if cos_angle > 0.3:
                    # CAS 3 : il fonce sur nous → s'écarter sur le côté (avec prédiction)
                    dtx, dty = _compute_detour_waypoint(
                        rx, ry, tx, ty, ox_pred, oy_pred,
                        DANGER_RADIUS_MM, DETOUR_MARGIN_MM,
                    )
                    dtx = max(MAP_X_MIN, min(MAP_X_MAX, dtx))
                    dty = max(MAP_Y_MIN, min(MAP_Y_MAX, dty))
                    print(f"[avoid] CAS 3 (fonce dessus) → écart latéral ({dtx:.0f},{dty:.0f})")
                    self._avoid_state   = "DETOURING"
                    self._detour_target = (dtx, dty)
                    return Command(kind="GOTO", x_mm=dtx, y_mm=dty)
                else:
                    # CAS 2 : il s'éloigne ou passe à côté → on attend
                    print(f"[avoid] CAS 2 (s'éloigne, cos={cos_angle:.2f}) → attente")
                    return Command(kind="GOTO", x_mm=rx, y_mm=ry)
            else:
                return Command(kind="GOTO", x_mm=rx, y_mm=ry)

        # ---- DETOURING ----
        elif self._avoid_state == "DETOURING":
            dtx, dty = self._detour_target
            d_detour = math.hypot(dtx - rx, dty - ry)

            if d_detour < DETOUR_ARRIVAL_TOL:
                print("[avoid] \u2713 D�tour atteint \u2192 reprise plan normal")
                self._avoid_state    = "NORMAL"
                self._detour_target  = None
                self._resume_target  = None
                self._cooldown_until = world.t_s + AVOID_COOLDOWN_S
                print(f"[avoid]   cooldown {AVOID_COOLDOWN_S}s activ� jusqu'� t={self._cooldown_until:.1f}s")
                return None

            return Command(kind="GOTO", x_mm=dtx, y_mm=dty)

        return None

    # ------------------------------------------------------------------
    def _exec_goto(self, world: WorldState, step: dict) -> Optional[Command]:
        tx = float(step["x_mm"])
        ty = float(step["y_mm"])
        rx, ry = world.robot.x_mm, world.robot.y_mm
        d = math.hypot(tx - rx, ty - ry)

        if d < self.arrival_tol:
            self.plan_idx += 1
            return None
        else:
            return Command(kind="GOTO", x_mm=tx, y_mm=ty)

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

    # ------------------------------------------------------------------
    def avoid_status(self) -> dict:
        return {
            "state":           self._avoid_state,
            "detour_target":   self._detour_target,
            "resume_target":   self._resume_target,
            "cooldown_until":  self._cooldown_until,
        }