"""Load generated MJCF in MuJoCo and ray-test walking surfaces and real holes."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("terrain", Path(__file__).resolve().parents[1] / "terrain.py")
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)


class TerrainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import mujoco
            import numpy as np
        except ImportError as error:
            raise unittest.SkipTest(str(error))
        cls.mujoco, cls.np = mujoco, np

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        robot = self.root / "unitree_robots/g1"
        (robot / "meshes").mkdir(parents=True)
        (robot / "meshes/marker.obj").write_text(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
        (robot / "g1_29dof.xml").write_text('''<mujoco>
          <compiler meshdir="meshes" angle="radian"/>
          <asset><mesh name="marker" file="marker.obj"/></asset>
          <worldbody><body name="torso_link" pos="0 0 1"><freejoint/>
            <geom type="sphere" size=".1" mass="1"/>
            <body pos="0 0 -.2"><joint name="motor_joint"/><geom size=".05" mass=".1"/></body>
          </body></worldbody>
          <actuator><motor joint="motor_joint"/></actuator>
        </mujoco>''')
        (robot / "scene_29dof.xml").write_text('''<mujoco>
          <include file="g1_29dof.xml"/><include file="old_terrain.xml"/>
          <worldbody><geom name="floor" type="plane" size="0 0 .05"/></worldbody>
        </mujoco>''')
        self.settings = json.loads(terrain.SETTINGS.read_text())

    def load(self, selection):
        # A different directory exercises absolute asset paths in generated scenes.
        destination = self.root / "generated"
        destination.mkdir(exist_ok=True)
        path = destination / "scene.xml"
        self.sections = terrain.generate_scene(self.root, selection, path)
        self.model = self.mujoco.MjModel.from_xml_path(str(path))
        self.data = self.mujoco.MjData(self.model)
        self.mujoco.mj_forward(self.model, self.data)
        self.assertEqual(self.model.nu, 1, "Robot actuators were changed")
        self.assertEqual(self.model.nmesh, 1, "Robot mesh assets were lost")

    def surface_height(self, x, y=0):
        hit = self.np.zeros(1, dtype=self.np.int32)
        distance = self.mujoco.mj_ray(self.model, self.data,
                                    self.np.array([x, y, 2.0]), self.np.array([0., 0., -1.]),
                                    None, 1, -1, hit)
        self.assertGreaterEqual(distance, 0)
        return 2.0 - distance

    def test_all_profiles_compile_and_spawn_on_flat_ground(self):
        for selection in terrain.TERRAINS[1:]:
            with self.subTest(selection=selection):
                self.load(selection)
                self.assertAlmostEqual(self.surface_height(0, 1), 0)
                for _ in range(20):
                    self.mujoco.mj_step(self.model, self.data)
                self.assertTrue(self.np.isfinite(self.data.qpos).all())

    def test_gaps_have_no_hidden_support_plane(self):
        self.load("gaps")
        x = self.settings["start_length"]
        cfg = self.settings["gaps"]
        for width in cfg["widths"]:
            for y in (-1.5, 0, 1.5):
                self.assertAlmostEqual(self.surface_height(x + width / 2, y), -self.settings["pit_depth"])
            self.assertAlmostEqual(self.surface_height(x + width + .1), 0)
            x += width + cfg["landing_length"]

    def test_mixed_lanes_are_parallel_with_connected_start(self):
        self.load("mixed")
        self.assertEqual(len(self.sections), 5)
        self.assertEqual(len({y for _, _, _, y in self.sections}), 5)
        lanes = {name: y for name, _, _, y in self.sections}
        for _, start, _, y in self.sections:
            self.assertEqual(start, self.settings["start_length"])
            self.assertAlmostEqual(self.surface_height(-1, y), 0)
        # The starting plaza also covers spaces between lanes, so they can be selected by walking.
        self.assertAlmostEqual(self.surface_height(0, self.settings["lane_spacing"] / 2), 0)
        start = self.settings["start_length"]
        self.assertAlmostEqual(self.surface_height(start + .1, lanes["gaps"]), -self.settings["pit_depth"])
        self.assertAlmostEqual(self.surface_height(start + .1, lanes["platforms"]), .1)
        stones = self.settings["stones"]
        self.assertAlmostEqual(self.surface_height(start + stones["stride"] / 2,
                                                   lanes["stones"] - stones["lateral_offset"]), .1)

    def test_platform_and_stone_top_heights(self):
        self.load("platforms")
        x = self.settings["start_length"]
        cfg = self.settings["platforms"]
        for height in cfg["heights"]:
            self.assertAlmostEqual(self.surface_height(x + cfg["length"] / 2), height)
            x += cfg["length"] + cfg["spacing"]
        self.load("stones")
        cfg = self.settings["stones"]
        x = self.settings["start_length"]
        for index, height in enumerate(cfg["heights"]):
            y = cfg["lateral_offset"] * (1 if index % 2 else -1)
            self.assertAlmostEqual(self.surface_height(x + cfg["stride"] / 2, y), height)
            self.assertAlmostEqual(self.surface_height(x + cfg["stride"] / 2, 1.5), -self.settings["pit_depth"])
            x += cfg["stride"]


if __name__ == "__main__":
    unittest.main()
