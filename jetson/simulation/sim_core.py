import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Moteur physique de la simulation, etat du monde, fonctions de chargement, scoring.

# -------------------- Geometry --------------------

@dataclass
class RectZone:
    name: str
    x_mm: float
    y_mm: float
    width_mm: float
    height_mm: float

    def contains(self, x: float, y: float) -> bool:
        return (self.x_mm <= x <= self.x_mm + self.width_mm) and \
               (self.y_mm <= y <= self.y_mm + self.height_mm)

# -------------------- World --------------------

@dataclass
class RobotState:
    x_mm: float
    y_mm: float
    theta_rad: float = 0.0

    # 2 bras x 4 caisses = 8 max
    carried_ids: List[int] = field(default_factory=list)
    max_carried: int = 8
    arm_capacity: int = 4
    n_arms: int = 2

    # Machine a etats : idle | picking | dropping | dragging_cursor
    action_state: str = "idle"
    action_timer_s: float = 0.0

    # Drag curseur
    cursor_drag_target_x: Optional[float] = None

@dataclass
class CrateState:
    id: int
    x_mm: float
    y_mm: float
    color: str = "blue"       # "blue" (notre equipe) | "yellow" (adversaire)
    carried: bool = False
    delivered: bool = False
    delivered_zone: Optional[str] = None

@dataclass
class CheckpointState:
    id: int
    x_mm: float
    y_mm: float

@dataclass
class StartMarkerState:
    name: str
    x_mm: float
    y_mm: float

@dataclass
class CursorState:
    x_mm: float
    y_mm: float

@dataclass
class OpponentState:
    x_mm: float
    y_mm: float
    theta_rad: float = 0.0
    radius_mm: float = 180.0   # rayon physique du robot adverse (encombrement)

@dataclass
class WorldState:
    t_s: float = 0.0
    match_duration_s: float = 100.0
    match_finished: bool = False

    robot: RobotState = field(default_factory=lambda: RobotState(0.0, 0.0, 0.0))
    opponent: Optional["OpponentState"] = None      # None = pas d'adversaire simulé
    crates: Dict[int, CrateState] = field(default_factory=dict)
    checkpoints: Dict[int, CheckpointState] = field(default_factory=dict)
    zones: Dict[str, RectZone] = field(default_factory=dict)
    cursor: Optional[CursorState] = None
    strategy_path: List[Tuple[float, float]] = field(default_factory=list)
    strategy_plan: List[dict] = field(default_factory=list)
    start_markers: List[StartMarkerState] = field(default_factory=list)

    score: int = 0
    score_breakdown: Dict[str, int] = field(default_factory=dict)
    # score_breakdown keys:
    #   "nest"           -> pts caisses dans le nid (2 pts/caisse, max 6)
    #   "pts_pantry_N"   -> pts caisses dans le garde-manger N (3 pts/caisse)
    #   "bonus_pantry_N" -> bonus majorite dans le garde-manger N (5 pts)
    #   "cursor"         -> pts thermometre (0/1/3/5/10)
    #   "return"         -> pts retour au nid (5 partiel, 10 total)

# -------------------- Loaders --------------------

def load_zones(zones_path: str) -> Dict[str, RectZone]:
    with open(zones_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    zones = {}
    for z in data.get("zones", []):
        zones[z["name"]] = RectZone(
            name=z["name"],
            x_mm=z["x_mm"],
            y_mm=z["y_mm"],
            width_mm=z["width_mm"],
            height_mm=z["height_mm"],
        )
    return zones

def load_scenario(world: WorldState, scenario_path: str) -> None:
    with open(scenario_path, "r", encoding="utf-8") as f:
        sc = json.load(f)

    rs = sc.get("robot_start", {"x_mm": 0.0, "y_mm": 0.0, "theta_rad": 0.0})
    world.robot = RobotState(rs["x_mm"], rs["y_mm"], rs.get("theta_rad", 0.0))

    cursor = sc.get("cursor", None)
    if cursor is not None:
        world.cursor = CursorState(cursor["x_mm"], cursor["y_mm"])

    world.crates.clear()
    for c in sc.get("crates", []):
        cid = int(c["id"])
        color = c.get("color", "blue")
        world.crates[cid] = CrateState(id=cid, x_mm=float(c["x_mm"]), y_mm=float(c["y_mm"]), color=color)

    world.checkpoints.clear()
    for cp in sc.get("checkpoints", []):
        cid = int(cp["id"])
        world.checkpoints[cid] = CheckpointState(id=cid, x_mm=float(cp["x_mm"]), y_mm=float(cp["y_mm"]))

    world.strategy_path = []
    for p in sc.get("strategy_path", []):
        world.strategy_path.append((float(p["x_mm"]), float(p["y_mm"])))

    world.start_markers = []
    for sm in sc.get("start_markers", []):
        world.start_markers.append(
            StartMarkerState(name=sm["name"], x_mm=float(sm["x_mm"]), y_mm=float(sm["y_mm"]))
        )

    world.strategy_plan = sc.get("strategy_plan", [])

# -------------------- Thermometre --------------------

# Plages X -> points du thermometre
# x=1250 = position depart curseur  -> 0 pts
# x=800  = position arrivee curseur -> 10 pts (max)
THERMO_BANDS = [
    (   50,  250,  0),
    (  250,  400,  1),
    (  400,  550,  3),
    (  550,  700,  5),
    (  700,  900, 10),   # zone max : x=800 ici
    (  900, 1050,  5),
    ( 1050, 1200,  3),
    ( 1200, 1500,  0),   # zone depart : x=1250 ici
]

def thermo_score(x_mm: float) -> int:
    for x_min, x_max, pts in THERMO_BANDS:
        if x_min <= x_mm < x_max:
            return pts
    return 0

# -------------------- Engine --------------------

@dataclass
class Command:
    kind: str       # GOTO | PICKUP_NEARBY | DROP_ALL | MOVE_CURSOR | WAIT
    x_mm: Optional[float] = None
    y_mm: Optional[float] = None
    from_x_mm: Optional[float] = None
    from_y_mm: Optional[float] = None
    crate_id: Optional[int] = None
    zone_name: Optional[str] = None


class SimEngine:
    def __init__(self):
        # Cinematique holonome, vitesse realiste competition
        self.v_max = 500.0          # mm/s

        # Tolerance d'arrivee
        self.arrival_eps = 30.0

        # Rayon de detection pour ramassage (portee des bras)
        self.pickup_eps = 150.0

        # Timings mecaniques
        self.pickup_duration_s = 1.0   # 2 bras en parallele
        self.drop_duration_s   = 0.8   # ouverture bras

        # Vitesse drag curseur (precision)
        self.v_cursor_drag = 300.0

    # ------------------------------------------------------------------
    def step(self, world: WorldState, cmd: Optional[Command], dt: float) -> None:

        # 1) Avancer le temps
        world.t_s += dt

        # 2) Fin de match
        if world.t_s >= world.match_duration_s and not world.match_finished:
            self._finalize_match(world)
            return

        if world.match_finished:
            return

        # 3) Actions physiques en cours
        if world.robot.action_state != "idle":
            if world.robot.action_state == "dragging_cursor":
                tx = world.robot.cursor_drag_target_x
                if tx is not None:
                    rx = world.robot.x_mm
                    dx = tx - rx
                    dist = abs(dx)
                    if dist > self.arrival_eps:
                        move = min(self.v_cursor_drag * dt, dist)
                        world.robot.x_mm = rx + move * (1.0 if dx > 0 else -1.0)
                        world.robot.theta_rad = 0.0 if dx > 0 else math.pi
                        if world.cursor is not None:
                            world.cursor.x_mm = world.robot.x_mm
                    else:
                        # Arrive a destination : lacher le curseur
                        world.robot.x_mm = tx
                        if world.cursor is not None:
                            world.cursor.x_mm = tx
                        # Scoring thermometre
                        pts = thermo_score(world.cursor.x_mm)
                        world.score_breakdown["cursor"] = pts
                        print(f"  Actn : lache le curseur en ({world.cursor.x_mm:.0f}, {world.cursor.y_mm:.0f}) -> {pts} pts thermometre")
                        world.robot.cursor_drag_target_x = None
                        world.robot.action_state = "idle"
            else:
                # Timer fixe pour picking / dropping
                world.robot.action_timer_s -= dt
                if world.robot.action_timer_s <= 0.0:
                    world.robot.action_timer_s = 0.0
                    world.robot.action_state = "idle"
            # Recalcul permanent (retour nid + curseur visibles en temps reel)
            self._recompute_score(world)
            return

        # 4) Traiter la commande \u2014 recalcul systematique pour affichage temps reel
        self._recompute_score(world)

        if cmd is None:
            return

        if cmd.kind == "GOTO":
            self._cmd_goto(world, cmd, dt)
        elif cmd.kind == "PICKUP_NEARBY":
            self._cmd_pickup_nearby(world)
        elif cmd.kind == "DROP_ALL":
            self._cmd_drop_all(world, cmd)
        elif cmd.kind == "MOVE_CURSOR":
            self._cmd_move_cursor(world, cmd)

        # Recalcul permanent du score (retour nid mis a jour en temps reel)
        self._recompute_score(world)

    # ------------------------------------------------------------------
    def _cmd_goto(self, world: WorldState, cmd: Command, dt: float) -> None:
        rx, ry = world.robot.x_mm, world.robot.y_mm
        dx, dy = cmd.x_mm - rx, cmd.y_mm - ry
        dist = math.hypot(dx, dy)

        if dist > 1e-6:
            move = min(self.v_max * dt, dist)
            world.robot.x_mm = rx + move * (dx / dist)
            world.robot.y_mm = ry + move * (dy / dist)
            world.robot.theta_rad = math.atan2(dy, dx)

        self._update_carried_crates(world)

    # ------------------------------------------------------------------
    def _cmd_pickup_nearby(self, world: WorldState) -> None:
        robot = world.robot
        if len(robot.carried_ids) >= robot.max_carried:
            print("  Actn : bras pleins, ramassage ignore")
            return

        slots_free = robot.max_carried - len(robot.carried_ids)

        candidates = []
        for c in world.crates.values():
            if c.carried or c.delivered or c.color != "blue":
                continue
            d = math.hypot(c.x_mm - robot.x_mm, c.y_mm - robot.y_mm)
            if d <= self.pickup_eps:
                candidates.append((d, c))

        candidates.sort(key=lambda t: t[0])

        picked = 0
        for _, c in candidates:
            if picked >= slots_free:
                break
            c.carried = True
            robot.carried_ids.append(c.id)
            picked += 1

        if picked > 0:
            print(f"  Actn : ramasse {picked} caisse(s) [IDs: {[c.id for _, c in candidates[:picked]]}]"
                  f" | porte={len(robot.carried_ids)}/{robot.max_carried}")
            robot.action_state = "picking"
            robot.action_timer_s = self.pickup_duration_s
        else:
            print("  Actn : aucune caisse a portee")

    # ------------------------------------------------------------------
    def _cmd_drop_all(self, world: WorldState, cmd: Command) -> None:
        robot = world.robot
        if not robot.carried_ids:
            print("  Actn : rien a deposer")
            return

        zone_name = cmd.zone_name
        if zone_name is None or zone_name not in world.zones:
            print(f"  Actn : zone '{zone_name}' introuvable")
            return

        zone = world.zones[zone_name]
        if not zone.contains(robot.x_mm, robot.y_mm):
            print(f"  Actn : robot hors zone '{zone_name}' \u2014 depose ignoree")
            return

        ids_dropped = list(robot.carried_ids)
        for cid in ids_dropped:
            if cid in world.crates:
                c = world.crates[cid]
                c.carried = False
                c.delivered = True
                c.delivered_zone = zone_name
                c.x_mm = robot.x_mm
                c.y_mm = robot.y_mm

        robot.carried_ids.clear()
        print(f"  Actn : depose {len(ids_dropped)} caisse(s) dans '{zone_name}' [IDs: {ids_dropped}]")

        self._recompute_score(world)

        robot.action_state = "dropping"
        robot.action_timer_s = self.drop_duration_s

    # ------------------------------------------------------------------
    def _cmd_move_cursor(self, world: WorldState, cmd: Command) -> None:
        if world.cursor is None:
            return
        if cmd.x_mm is None:
            return

        print(f"  Actn : tire le curseur en ({world.cursor.x_mm:.0f}, {world.cursor.y_mm:.0f})")
        world.robot.cursor_drag_target_x = cmd.x_mm
        world.robot.action_state = "dragging_cursor"

    # ------------------------------------------------------------------
    def _finalize_match(self, world: WorldState) -> None:
        world.match_finished = True

        # Caisses encore portees => non comptabilisees
        for cid in world.robot.carried_ids:
            if cid in world.crates:
                world.crates[cid].carried = False
                world.crates[cid].delivered = False
                world.crates[cid].delivered_zone = None
        world.robot.carried_ids.clear()

        # Points retour au nid
        if "our_nest" in world.zones:
            nest = world.zones["our_nest"]
            rx, ry = world.robot.x_mm, world.robot.y_mm
            robot_radius = 150.0
            in_total = nest.contains(rx, ry)
            in_partial = (
                nest.x_mm - robot_radius <= rx <= nest.x_mm + nest.width_mm + robot_radius and
                nest.y_mm - robot_radius <= ry <= nest.y_mm + nest.height_mm + robot_radius
            )
            if in_total:
                world.score_breakdown["return"] = 10
            elif in_partial:
                world.score_breakdown["return"] = 5
            else:
                world.score_breakdown["return"] = 0

        self._recompute_score(world)

        print(f"\n=== MATCH TERMINE a t={world.t_s:.1f}s ===")
        print(f"Score total : {world.score} pts")
        for k, v in sorted(world.score_breakdown.items()):
            if v > 0:
                print(f"  {k:<25} : {v:>3} pts")

    # ------------------------------------------------------------------
    def _update_carried_crates(self, world: WorldState) -> None:
        for cid in world.robot.carried_ids:
            if cid in world.crates:
                world.crates[cid].x_mm = world.robot.x_mm
                world.crates[cid].y_mm = world.robot.y_mm

    def _recompute_score(self, world: WorldState) -> None:
        # Garde-mangers : 3 pts/caisse + 5 pts bonus majorite
        pantry_names = [z for z in world.zones if z.startswith("pantry_")]
        for pname in pantry_names:
            crates_here = [c for c in world.crates.values()
                           if c.delivered_zone == pname]
            if crates_here:
                world.score_breakdown[f"pts_{pname}"] = len(crates_here) * 3
                world.score_breakdown[f"bonus_{pname}"] = 5
            else:
                world.score_breakdown.pop(f"pts_{pname}", None)
                world.score_breakdown.pop(f"bonus_{pname}", None)

        # Nid : 2 pts/caisse, max 6 caisses
        crates_nest = [c for c in world.crates.values()
                       if c.delivered_zone == "our_nest"]
        world.score_breakdown["nest"] = min(len(crates_nest), 6) * 2

        # Retour au nid : calcule en temps reel (affichage pendant le match)
        # Mis a jour aussi dans _finalize_match pour le score officiel
        if "our_nest" in world.zones:
            nest = world.zones["our_nest"]
            rx, ry = world.robot.x_mm, world.robot.y_mm
            robot_radius = 150.0
            in_total = nest.contains(rx, ry)
            in_partial = (
                nest.x_mm - robot_radius <= rx <= nest.x_mm + nest.width_mm + robot_radius and
                nest.y_mm - robot_radius <= ry <= nest.y_mm + nest.height_mm + robot_radius
            )
            if in_total:
                world.score_breakdown["return"] = 10
            elif in_partial:
                world.score_breakdown["return"] = 5
            else:
                world.score_breakdown["return"] = 0

        world.score = sum(world.score_breakdown.values())

# -------------------- Opponent Simulator --------------------

class OpponentSimulator:
    """
    Simule un robot adverse se déplaçant selon une liste de waypoints (patrouille).
    Si aucun waypoint n'est fourni, l'adversaire reste immobile.

    Usage dans la boucle principale :
        opp_sim = OpponentSimulator(world, waypoints=[...], speed_mm_s=400)
        # à chaque tick :
        opp_sim.step(world, dt)
    """

    def __init__(
        self,
        world: WorldState,
        start_x: float = 1200.0,
        start_y: float = 600.0,
        waypoints: Optional[List[Tuple[float, float]]] = None,
        speed_mm_s: float = 400.0,
        loop: bool = True,
    ):
        world.opponent = OpponentState(x_mm=start_x, y_mm=start_y)
        self.waypoints: List[Tuple[float, float]] = waypoints or []
        self.wp_idx: int = 0
        self.speed: float = speed_mm_s
        self.loop: bool = loop
        self._arrival_tol: float = 40.0

    # ------------------------------------------------------------------
    def step(self, world: WorldState, dt: float) -> None:
        opp = world.opponent
        if opp is None or world.match_finished:
            return

        if not self.waypoints:
            return   # immobile

        # Cible courante
        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - opp.x_mm, ty - opp.y_mm
        dist = math.hypot(dx, dy)

        if dist < self._arrival_tol:
            # Waypoint atteint → waypoint suivant
            next_idx = self.wp_idx + 1
            if next_idx >= len(self.waypoints):
                if self.loop:
                    self.wp_idx = 0
                # else : adversaire s'arrête au dernier waypoint
            else:
                self.wp_idx = next_idx
            return

        # Déplacement
        move = min(self.speed * dt, dist)
        opp.x_mm += move * (dx / dist)
        opp.y_mm += move * (dy / dist)
        opp.theta_rad = math.atan2(dy, dx)

# -------------------- Opponent Strategy (adversaire strat�gique) --------------------

class OpponentStrategy:
    """
    Adversaire qui ex�cute son propre plan d'actions (GOTO / PICKUP / DROP),
    comme notre robot. Il joue les caisses jaunes et d�pose dans les pantries.
    
    Le plan est charge depuis un fichier JSON s�par� (opponent_scenario.json).
    """

    def __init__(
        self,
        world: WorldState,
        scenario_path: str = "opponent_scenario.json",
        speed_mm_s: float = 450.0,
    ):
        # Charger le plan adverse depuis le JSON
        with open(scenario_path, "r", encoding="utf-8") as f:
            sc = json.load(f)

        # Position de depart
        rs = sc.get("robot_start", {"x_mm": -1200, "y_mm": 800})
        world.opponent = OpponentState(x_mm=rs["x_mm"], y_mm=rs["y_mm"])

        # Plan d'actions
        self.plan: List[dict] = sc.get("strategy_plan", [])
        self.plan_idx: int = 0

        # Caisses portees par l'adversaire
        self.carried_ids: List[int] = []

        # Param�tres physiques
        self.speed: float = speed_mm_s
        self.arrival_eps: float = 30.0
        self.pickup_eps: float = 150.0

        # Timer pour les actions instantanees (PICKUP / DROP) \u2014 l'adversaire 
        # met une seconde pour ramasser ou deposer, comme nous
        self.action_timer: float = 0.0
        self.action_state: str = "idle"  # idle | picking | dropping

    # ------------------------------------------------------------------
    def step(self, world: WorldState, dt: float) -> None:
        """� appeler � chaque tick (comme OpponentSimulator.step)."""
        opp = world.opponent
        if opp is None or world.match_finished:
            return

        # Si une action en cours (pickup ou drop) \u2192 decompter le timer
        if self.action_state in ("picking", "dropping"):
            self.action_timer -= dt
            if self.action_timer <= 0.0:
                self.action_state = "idle"
            else:
                return  # encore en train de faire l'action

        # Plan termin� \u2192 on ne bouge plus
        if self.plan_idx >= len(self.plan):
            return

        step = self.plan[self.plan_idx]
        kind = step.get("type", "")

        # ---- GOTO ----
        if kind == "GOTO":
            tx = float(step["x_mm"])
            ty = float(step["y_mm"])
            dx, dy = tx - opp.x_mm, ty - opp.y_mm
            dist = math.hypot(dx, dy)

            if dist < self.arrival_eps:
                # Arrive \u2192 etape suivante
                self.plan_idx += 1
            else:
                # Avancer
                move = min(self.speed * dt, dist)
                opp.x_mm += move * (dx / dist)
                opp.y_mm += move * (dy / dist)
                opp.theta_rad = math.atan2(dy, dx)

        # ---- PICKUP_NEARBY ----
        elif kind == "PICKUP_NEARBY":
            picked = 0
            for c in world.crates.values():
                if c.carried or c.delivered:
                    continue
                if c.color != "yellow":  # l'adversaire ne joue que les jaunes
                    continue
                d = math.hypot(c.x_mm - opp.x_mm, c.y_mm - opp.y_mm)
                if d <= self.pickup_eps:
                    c.carried = True
                    self.carried_ids.append(c.id)
                    picked += 1

            if picked > 0:
                print(f"  Opp  : ramasse {picked} caisse(s) jaune(s)")
            self.action_state = "picking"
            self.action_timer = 1.0
            self.plan_idx += 1

        # ---- DROP_ALL ----
        elif kind == "DROP_ALL":
            zone_name = step.get("zone_name", "")
            if zone_name in world.zones and self.carried_ids:
                zone = world.zones[zone_name]
                if zone.contains(opp.x_mm, opp.y_mm):
                    for cid in self.carried_ids:
                        if cid in world.crates:
                            c = world.crates[cid]
                            c.carried = False
                            c.delivered = True
                            c.delivered_zone = zone_name
                            c.x_mm = opp.x_mm
                            c.y_mm = opp.y_mm
                    print(f"  Opp  : d�pose {len(self.carried_ids)} caisse(s) dans '{zone_name}'")
                    self.carried_ids.clear()
                    self.action_state = "dropping"
                    self.action_timer = 0.8
            self.plan_idx += 1

        else:
            # Type inconnu \u2192 on ignore et on passe
            self.plan_idx += 1

        # Garder les caisses portees collees à l'adversaire
        for cid in self.carried_ids:
            if cid in world.crates:
                world.crates[cid].x_mm = opp.x_mm
                world.crates[cid].y_mm = opp.y_mm
