"""Start the shared G1 29DOF MuJoCo environment for DWaQ or Parkour.

Both controllers use DDS on lo. Depth observations are enabled by default so
Parkour can use the same environment; --no-depth selects DDS-only simulation.
The physics engine, scene and dependencies remain in unitree_mujoco.
"""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import runpy
import sys
from tempfile import TemporaryDirectory


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mujoco-root", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--depth", action=argparse.BooleanOptionalAction, default=True,
                        help="Publish Parkour depth observations (DWaQ ignores them)")
    parser.add_argument("--depth-preview", action="store_true",
                        help="Show raw and model-input depth images in the MuJoCo viewer's top-right corner")
    parser.add_argument("--follow", action="store_true",
                        help="Start with a camera tracking the robot torso")
    parser.add_argument("--terrain", default="original",
                        choices=("original", "mixed", "platforms", "stones", "gaps", "stairs", "hurdles"),
                        help="Obstacle course; mixed places categories in parallel lanes (default: original external scene)")
    parser.add_argument("--window-backend", choices=("auto", "x11", "wayland"), default="x11",
                        help="Window backend (default: x11); auto prefers X11/XWayland when DISPLAY is available")
    args = parser.parse_args()
    if args.domain <= 0:
        parser.error("Use a positive simulation domain")
    if args.depth_preview and not args.depth:
        parser.error("--depth-preview requires depth; remove --no-depth")

    root = args.mujoco_root.resolve()
    simulation = root / "simulate_python"
    simulator = simulation / "unitree_mujoco.py"
    scene = root / "unitree_robots/g1/scene_29dof.xml"
    if not simulator.is_file():
        parser.error("Missing external simulate_python/unitree_mujoco.py")
    if not scene.is_file():
        parser.error("Missing external unitree_robots/g1/scene_29dof.xml")

    previous_path, previous_argv = sys.path[:], sys.argv
    previous_backend = os.environ.get("PYGLFW_LIBRARY_VARIANT")
    window_backend = args.window_backend
    if window_backend == "auto":
        window_backend = previous_backend or ("x11" if sys.platform == "linux" and os.environ.get("DISPLAY") else "auto")
    # Select the GLFW wheel library before importing the simulator or camera bridge.
    if window_backend != "auto":
        os.environ["PYGLFW_LIBRARY_VARIANT"] = window_backend
    bridge = original_publisher = None
    temporary_scene = None
    try:
        sys.path.insert(0, str(simulation))
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        if args.terrain != "original":
            from terrain import generate_scene
            temporary_scene = TemporaryDirectory(prefix="g1-terrain-")
            scene = Path(temporary_scene.name) / "scene.xml"
            sections = generate_scene(root, args.terrain, scene)
            for category, start, end, y in sections:
                print(f"Terrain {category}: x={start:.2f}..{end:.2f} m, lane center y={y:.2f} m, along +X", flush=True)
        import config

        if args.depth:
            import parkour_observation as bridge
            original_publisher = bridge.ParkourObservationPublisher
            project_config = PROJECT_ROOT / "apps/parkour/config/g1_parkour.json"

            class Publisher(original_publisher):
                def __init__(self, model, data, port=None, config_path=None):
                    super().__init__(model, data, config_path=project_config)

            bridge.ParkourObservationPublisher = Publisher

        config.ROBOT = "g1"
        config.ROBOT_SCENE = str(scene)
        config.PHYSICS_BACKEND = "mujoco"
        config.DOMAIN_ID, config.INTERFACE = args.domain, "lo"
        config.USE_JOYSTICK = 0
        config.PARKOUR_OBSERVATIONS = args.depth
        config.PARKOUR_OBSERVATION_PORT = None # 外部启动接口的旧参数，不再使用网络
        config.PARKOUR_DEPTH_PREVIEW = args.depth_preview
        print(f"Shared G1 simulation: DDS domain {args.domain}, interface lo, "
              f"depth {'on' if args.depth else 'off'}, window backend {window_backend}", flush=True)
        sys.argv = [str(simulator)]
        with ExitStack() as extensions:
            if args.follow:
                from viewer_camera import follow_robot
                extensions.enter_context(follow_robot())
            if args.depth_preview:
                from viewer_depth_overlay import install_depth_overlay
                extensions.enter_context(install_depth_overlay())
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
        if temporary_scene is not None:
            temporary_scene.cleanup()


if __name__ == "__main__":
    main()
