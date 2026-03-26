import time
from sim_core import WorldState, load_zones, load_scenario, SimEngine
from strategy_runner import StrategyRunner
from sim_render import Renderer

#Moteur de la simulation, boucle principale, etc. 
#Charge le monde, le scénario, lance la boucle de contrôle et d'affichage.
#Le monde contient l'état du robot, des caisses, des zones, le  temps et le score.

def main():
    world = WorldState()
    world.zones = load_zones("zones.json")
    load_scenario(world, "scenario.json")

    engine = SimEngine()
    runner = StrategyRunner()

    render = Renderer(world)

    dt = 0.05  # 20 Hz
    real_time = True

    print("Sim started. Ctrl+C to stop.")
    try:
        while True:
            cmd = runner.step(world)
            engine.step(world, cmd, dt)
            render.update()

            if real_time:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()