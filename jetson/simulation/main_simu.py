import time
from sim_core import WorldState, load_zones, load_scenario, SimEngine, OpponentSimulator, OpponentStrategy
from strategy_runner import StrategyRunner
from sim_render import Renderer

OPPONENT_PROFILE = "C"

# -----------------------------------------------------------------------
# ADVERSAIRE \u2014 Trajectoire conue pour gner notre robot plusieurs fois
#
# Notre trajectoire (rappel) :
#   (1150,800)\u2192(1150,200)\u2192(1150,-600)\u2192(1250,-950)\u2192(800,-900)
#   \u2192(400,-700)\u2192(700,-200)\u2192(350,-100)\u2192(250,450)\u2192(1150,800)
#
# L'adversaire part de (-1200, 0) et remonte la table de gauche � droite
# en coupant deliberement notre chemin à 3 endroits :
#
#   CROISEMENT 1 : (1150, 500)  \u2192 coupe seg 1  (notre descente initiale)
#   CROISEMENT 2 : (550, -450)  \u2192 coupe seg 6  (notre mont�e vers pantry_6)
#   CROISEMENT 3 : (700, 625)   \u2192 coupe seg 9  (notre retour au nid)
#
# Puis il repart coté gauche pour boucler.
# -----------------------------------------------------------------------

OPP_WAYPOINTS = [
    # D�part c�t� gauche, il monte vers notre chemin
    (-1200,    0),
    # CROISEMENT 1 : taille notre descente initiale (seg 1)
    ( 1150,  500),
    # Redescend vers le bas du terrain
    ( 1300, -800),
    # CROISEMENT 2 : coupe notre trajet vers pantry_6 (seg 6)
    (  550, -450),
    # Remonte vers le haut
    ( -500,  300),
    # CROISEMENT 3 : coupe notre retour au nid (seg 9)
    (  700,  625),
    # Repart � gauche pour recommencer
    (-1200,    0),
]

OPP_START_X   = -1200   # d�marre loin de nous
OPP_START_Y   =     0
OPP_SPEED     =   420   # mm/s \u2014 légèrement plus lent que nous (500 mm/s)

# False = d�sactive l'adversaire pour voir la stratégie pure
ENABLE_OPPONENT = True


def main():
    world = WorldState()
    world.zones = load_zones("zones.json")
    load_scenario(world, "scenario.json")

    engine = SimEngine()
    runner = StrategyRunner()
    render = Renderer(world)

    opp_sim = None
    if ENABLE_OPPONENT:
        opp_sim = OpponentStrategy(
            world,
            scenario_path=f"opponent_scenario{OPPONENT_PROFILE}.json",
            speed_mm_s=450.0,
        )
        print(f"[main] Adversaire stratégique {OPPONENT_PROFILE} activé — il joue son propre plan")
    else:
        print("[main] Adversaire désactivé — stratégie pure")
    dt        = 0.05   # 20 Hz
    real_time = True

    print("\nSim démarrée. Ctrl+C pour arréter.")
    try:
        while True:
            cmd = runner.step(world)
            engine.step(world, cmd, dt)

            if opp_sim is not None:
                opp_sim.step(world, dt)

            render.update(runner)

            if real_time:
                time.sleep(dt)

    except KeyboardInterrupt:
        print("\nArr�t�.")


if __name__ == "__main__":
    main()
