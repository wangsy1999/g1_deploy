"""Start the shared G1 29DOF MuJoCo environment for DWaQ or Parkour.

Both controllers use DDS on lo. Depth observations are enabled by default so
Parkour can use the same environment; --no-depth selects DDS-only simulation.
The physics engine, scene and dependencies remain in unitree_mujoco.
"""
import argparse
import os
from pathlib import Path
import runpy
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mujoco-root", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--port", type=int, default=17200, help="Parkour observation UDP port")
    parser.add_argument("--depth", action=argparse.BooleanOptionalAction, default=True,
                        help="Publish Parkour depth observations (DWaQ ignores them)")
    parser.add_argument("--window-backend", choices=("auto", "x11", "wayland"), default="auto",
                        help="Window backend; auto prefers X11/XWayland when DISPLAY is available")
    args = parser.parse_args()
    if args.domain <= 0 or not 1024 <= args.port <= 65535:
        parser.error("Use a positive simulation domain and port 1024..65535")

    root = args.mujoco_root.resolve()
    simulation = root / "simulate_python"
    simulator = simulation / "unitree_mujoco.py"
    scene = root / "unitree_robots/g1/scene_29dof.xml"
    if not simulator.is_file():
        parser.error("Missing external simulate_python/unitree_mujoco.py")
    if not scene.is_file():
        parser.error("Missing external unitree_robots/g1/scene_29dof.xml")
    if args.depth and not (simulation / "parkour_observation.py").is_file():
        parser.error("Depth mode requires simulate_python/parkour_observation.py; use --no-depth for DWaQ only")

    previous_path, previous_argv = sys.path[:], sys.argv
    previous_backend = os.environ.get("PYGLFW_LIBRARY_VARIANT")
    window_backend = args.window_backend
    if window_backend == "auto":
        window_backend = previous_backend or ("x11" if sys.platform == "linux" and os.environ.get("DISPLAY") else "auto")
    # Select the GLFW wheel library before importing the simulator or camera bridge.
    if window_backend != "auto":
        os.environ["PYGLFW_LIBRARY_VARIANT"] = window_backend
    bridge = original_publisher = None
    try:
        sys.path.insert(0, str(simulation))
        import config

        if args.depth:
            import parkour_observation as bridge
            original_publisher = bridge.ParkourObservationPublisher
            project_config = PROJECT_ROOT / "apps/parkour/config/g1_parkour.json"

            class Publisher(original_publisher):
                def __init__(self, model, data, port=args.port, config_path=None):
                    super().__init__(model, data, port, project_config)

            bridge.ParkourObservationPublisher = Publisher

        config.ROBOT = "g1"
        config.ROBOT_SCENE = str(scene)
        config.PHYSICS_BACKEND = "mujoco"
        config.DOMAIN_ID, config.INTERFACE = args.domain, "lo"
        config.USE_JOYSTICK = 0
        config.PARKOUR_OBSERVATIONS = args.depth
        config.PARKOUR_OBSERVATION_PORT = args.port
        config.PARKOUR_DEPTH_PREVIEW = False
        print(f"Shared G1 simulation: DDS domain {args.domain}, interface lo, "
              f"depth {'on' if args.depth else 'off'}, window backend {window_backend}", flush=True)
        sys.argv = [str(simulator)]
        runpy.run_path(str(simulator), run_name="__main__")
    finally:
        if bridge is not None and original_publisher is not None:
            bridge.ParkourObservationPublisher = original_publisher
        sys.path[:] = previous_path
        sys.argv = previous_argv
        if previous_backend is None:
            os.environ.pop("PYGLFW_LIBRARY_VARIANT", None)
        else:
            os.environ["PYGLFW_LIBRARY_VARIANT"] = previous_backend


if __name__ == "__main__":
    main()
