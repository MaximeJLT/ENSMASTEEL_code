# test_individual.py
# Génère un individu aléatoire et le fait jouer dans la simu.

from sim_core import WorldState, load_zones, load_scenario, SimEngine, OpponentStrategy
from strategy_runner import StrategyRunner
from sim_render import Renderer
from genetic import load_world_data, generate_individual
import time

# -----------------------------------------------------------------------
# Paramètres
# -----------------------------------------------------------------------
DT_S            = 0.05
ENABLE_OPPONENT = True
OPPONENT_PROFILE = "A"   # "A" | "B" | "C"
REAL_TIME       = True   # True = vitesse réelle, False = aussi vite que possible


def main():
    # 1) Générer un individu aléatoire
    world_data = load_world_data()
    individu = generate_individual(world_data)

    print(f"\n=== Individu généré ({len(individu)} actions) ===")
    for i, action in enumerate(individu):
        print(f"  [{i}] {action}")
    print()

    # 2) Charger le monde comme d'habitude
    world = WorldState()
    world.zones = load_zones("zones.json")
    load_scenario(world, "scenario.json")

    # 3) Remplacer le plan par notre individu
    world.strategy_plan = individu

    # 4) Adversaire stratégique
    opp_sim = None
    if ENABLE_OPPONENT:
        opp_sim = OpponentStrategy(
            world,
            scenario_path=f"opponent_scenario{OPPONENT_PROFILE}.json",
            speed_mm_s=450.0,
        )
        print(f"[test] Adversaire {OPPONENT_PROFILE} chargé")

    # 5) Boucle de simu (copiée de main_simu.py)
    engine = SimEngine()
    runner = StrategyRunner()
    render = Renderer(world)

    try:
        while not world.match_finished:
            cmd = runner.step(world)
            engine.step(world, cmd, DT_S)
            if opp_sim is not None:
                opp_sim.step(world, DT_S)
            render.update(runner)
            if REAL_TIME:
                time.sleep(DT_S)
    except KeyboardInterrupt:
        print("\n[test] interrompu")

    # 6) Score final
    print(f"\n=== Score final : {world.score} ===")


if __name__ == "__main__":
    main()