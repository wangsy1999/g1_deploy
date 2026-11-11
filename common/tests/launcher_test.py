"""Verify source/package launch commands using fake controllers, without DDS."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class LauncherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="g1 launcher ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        shutil.copy(ROOT / "g1", self.root / "g1")
        for app, script in (("parkour", "parkour.sh"), ("dwaq", "g1.sh")):
            destination = self.root / "apps" / app
            destination.mkdir(parents=True)
            shutil.copy(ROOT / "apps" / app / script, destination / script)
        self.fake = self.root / "fake-controller"
        self.fake.write_text('#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
        self.fake.chmod(0o755)
        for layout in ("build", "bin"):
            (self.root / layout).mkdir()
            for app in ("dwaq", "parkour"):
                shutil.copy(self.fake, self.root / layout / f"g1_{app}")

    def launch(self, *args, entry="g1"):
        return subprocess.run(["bash", str(self.root / entry), *args],
                              capture_output=True, text=True,
                              env={**os.environ, "PYTHON_BIN": str(self.fake)})

    def arguments(self, *args, **kwargs):
        result = self.launch(*args, **kwargs)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_source_and_installed_commands(self):
        for source in (True, False):
            marker = self.root / "CMakeLists.txt"
            if source:
                marker.touch()
            else:
                marker.unlink()
                shutil.copy(ROOT / "apps/parkour/parkour.sh", self.root / "parkour")
                (self.root / "parkour").chmod(0o755)
            for app in ("dwaq", "parkour"):
                with self.subTest(source=source, app=app):
                    check = self.arguments(app, "check", "--config", "custom config.json")
                    self.assertIn("--check-policy", check)
                    self.assertEqual(check[-2:], ["--config", "custom config.json"])
                    sim = self.arguments(app, "sim", "--domain", "7")
                    self.assertIn("--sim", sim)
                    self.assertEqual(sim[-2:], ["--domain", "7"])
                    self.assertEqual(sim[sim.index("--interface") + 1], "lo")

    def test_hardware_dispatch_and_camera(self):
        interfaces = [p.name for p in Path("/sys/class/net").iterdir() if p.name != "lo"]
        if not interfaces:
            self.skipTest("No non-loopback interface for launcher validation")
        interface = interfaces[0]
        for app in ("dwaq", "parkour"):
            options = ["--no-auto-camera"] if app == "parkour" else []
            args = self.arguments(app, "real", interface, *options, "--config", "custom config.json")
            self.assertNotIn("--reset-seconds", args)
            self.assertIn("--hardware", args)
            self.assertEqual(args[args.index("--interface") + 1], interface)
            self.assertEqual(args[-2:], ["--config", "custom config.json"])
        self.assertEqual(self.launch("dwaq", "real", interface, "remote").returncode, 2)
        self.assertEqual(self.launch("parkour", "real", interface, "remote").returncode, 2)
        for removed in ("--model", "--model-dir", "--limits", "--unitree-remote", "--reset-seconds"):
            self.assertEqual(self.launch("parkour", "real", interface, removed, "value").returncode, 2)
        camera = self.arguments("parkour", "real", interface, "--port", "18001")
        self.assertTrue(camera[0].endswith("parkour_camera.py"))
        self.assertEqual(camera[camera.index("--depth-port") + 1], "18001")

    def test_help_invalid_interface_and_legacy_entry(self):
        for app in ("dwaq", "parkour"):
            self.assertEqual(self.launch(app, "help").returncode, 0)
            self.assertEqual(self.launch(app, "real", "lo").returncode, 2)
            self.assertEqual(self.launch(app, "real").returncode, 2)
        self.assertEqual(self.arguments("check"), self.arguments("dwaq", "check"))
        self.assertEqual(self.arguments("dwaq", "sim"),
                         self.arguments("sim", entry="apps/dwaq/g1.sh"))


if __name__ == "__main__":
    unittest.main()
