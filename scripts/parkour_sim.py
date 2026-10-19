"""Launch the external unitree_mujoco viewer using this project's parkour config.

Only adapter glue lives here; the simulator, robot XML and meshes stay external.
"""
import argparse
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mujoco-root", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--port", type=int, default=17200)
    args = parser.parse_args()
    if args.domain <= 0 or not 1024 <= args.port <= 65535:
        parser.error("Use a positive simulation domain and port 1024..65535")
    sim = args.mujoco_root.resolve()/"simulate_python"
    if not (sim/"unitree_mujoco.py").is_file():
        parser.error("Missing external simulate_python/unitree_mujoco.py")
    if not (sim/"parkour_observation.py").is_file():
        parser.error("Use wangsy1999/unitree_mujoco with its parkour_observation.py camera bridge")
    sys.path.insert(0, str(sim))
    import config
    import parkour_observation
    project_config = Path(__file__).resolve().parents[1]/"config/g1_parkour.json"
    original = parkour_observation.ParkourObservationPublisher

    class Publisher(original):
        def __init__(self, model, data, port=args.port, config_path=None):
            super().__init__(model, data, port, project_config)

    parkour_observation.ParkourObservationPublisher = Publisher
    config.ROBOT = "g1"
    config.ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
    config.DOMAIN_ID, config.INTERFACE = args.domain, "lo"
    config.USE_JOYSTICK = 0
    config.PARKOUR_OBSERVATIONS = True
    config.PARKOUR_OBSERVATION_PORT = args.port
    config.PARKOUR_DEPTH_PREVIEW = False
    runpy.run_path(str(sim/"unitree_mujoco.py"), run_name="__main__")


if __name__ == "__main__":
    main()
