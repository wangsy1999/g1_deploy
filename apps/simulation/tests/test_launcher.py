"""Check launcher configuration without graphics, DDS or simulator dependencies."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = PROJECT_ROOT / "apps/simulation/sim.py"
LEGACY = PROJECT_ROOT / "apps/parkour/parkour_sim.py"


class LauncherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        simulation = self.root / "simulate_python"
        simulation.mkdir()
        scene = self.root / "unitree_robots/g1/scene_29dof.xml"
        scene.parent.mkdir(parents=True)
        scene.touch()
        self.config = simulation / "config.py"
        self.config.write_text("# External configuration remains unchanged.\n")
        self.bridge = simulation / "parkour_observation.py"
        self.bridge.write_text('''class ParkourObservationPublisher:
    def __init__(self, model, data, port, config_path):
        self.port, self.config_path = port, str(config_path)
''')
        (simulation / "unitree_mujoco.py").write_text('''import config, json, os, sys
result = dict(robot=config.ROBOT, scene=config.ROBOT_SCENE, domain=config.DOMAIN_ID,
              interface=config.INTERFACE, depth=config.PARKOUR_OBSERVATIONS,
              preview=config.PARKOUR_DEPTH_PREVIEW, backend=config.PHYSICS_BACKEND,
              joystick=config.USE_JOYSTICK, argv=sys.argv,
              window_backend=os.environ.get("PYGLFW_LIBRARY_VARIANT"))
if config.PARKOUR_OBSERVATIONS:
    from parkour_observation import ParkourObservationPublisher
    camera = ParkourObservationPublisher(None, None, config.PARKOUR_OBSERVATION_PORT, "wrong.json")
    result.update(port=camera.port, profile=camera.config_path)
print(json.dumps(result))
''')

    def launch(self, *args, launcher=LAUNCHER, env=None):
        environment = os.environ.copy()
        environment.pop("PYGLFW_LIBRARY_VARIANT", None)
        environment.pop("DISPLAY", None)
        environment.update(env or {})
        return subprocess.run([sys.executable, str(launcher), "--mujoco-root", str(self.root), *args],
                              cwd=self.root, capture_output=True, text=True, env=environment)

    def configuration(self, *args, launcher=LAUNCHER, env=None):
        result = self.launch(*args, launcher=launcher, env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.config.read_text(), "# External configuration remains unchanged.\n")
        return json.loads(result.stdout.splitlines()[-1])

    def test_shared_scene_and_project_camera_config(self):
        result = self.configuration("--domain", "7", "--port", "18000")
        self.assertEqual((result["robot"], result["domain"], result["interface"]), ("g1", 7, "lo"))
        self.assertEqual(result["scene"], str(self.root / "unitree_robots/g1/scene_29dof.xml"))
        self.assertTrue(result["depth"])
        self.assertFalse(result["preview"])
        self.assertEqual(result["backend"], "mujoco")
        self.assertEqual(result["joystick"], 0)
        self.assertEqual(result["port"], 18000)
        self.assertEqual(result["profile"], str(PROJECT_ROOT / "apps/parkour/config/g1_parkour.json"))
        self.assertEqual(result["argv"], [str(self.root / "simulate_python/unitree_mujoco.py")])

    def test_dwaq_without_camera_bridge(self):
        self.bridge.unlink()
        result = self.configuration("--no-depth")
        self.assertFalse(result["depth"])
        self.assertNotIn("profile", result)

    def test_legacy_entry_matches_shared_entry(self):
        self.assertEqual(self.configuration(), self.configuration(launcher=LEGACY))

    def test_window_backend_selection(self):
        cases = [
            ((), {"DISPLAY": ":0", "XDG_SESSION_TYPE": "wayland"}, "x11"),
            ((), {}, None),
            ((), {"DISPLAY": ":0", "PYGLFW_LIBRARY_VARIANT": "wayland"}, "wayland"),
            (("--window-backend", "x11"), {"PYGLFW_LIBRARY_VARIANT": "wayland"}, "x11"),
            (("--window-backend", "wayland"), {"DISPLAY": ":0"}, "wayland"),
        ]
        for args, env, expected in cases:
            with self.subTest(args=args, env=env):
                self.assertEqual(self.configuration(*args, env=env)["window_backend"], expected)

    def test_missing_bridge_is_reported(self):
        self.bridge.unlink()
        result = self.launch()
        self.assertEqual(result.returncode, 2)
        self.assertIn("--no-depth", result.stderr)

    def test_invalid_domain_and_port_are_rejected(self):
        for args in [("--domain", "0"), ("--port", "10"), ("--domain", "-1")]:
            with self.subTest(args=args):
                self.assertEqual(self.launch(*args).returncode, 2)

    def test_missing_scene_is_reported(self):
        (self.root / "unitree_robots/g1/scene_29dof.xml").unlink()
        result = self.launch()
        self.assertEqual(result.returncode, 2)
        self.assertIn("scene_29dof.xml", result.stderr)


if __name__ == "__main__":
    unittest.main()
