"""Verify camera tracking with MuJoCo scene updates, without opening a window."""
from contextlib import nullcontext
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "viewer_camera", Path(__file__).resolve().parents[1] / "viewer_camera.py")
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


class FollowCameraTest(unittest.TestCase):
    def test_camera_tracks_motion_and_restores_launch(self):
        try:
            import mujoco
            import mujoco.viewer
        except ImportError as error:
            self.skipTest(str(error))
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="torso_link" pos="0 0 1"><freejoint/>
            <geom type="sphere" size=".1" mass="1"/>
          </body></worldbody></mujoco>''')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        viewer = SimpleNamespace(cam=mujoco.MjvCamera(), lock=nullcontext)
        callback = lambda key: None
        with patch.object(mujoco.viewer, "launch_passive", return_value=viewer) as launch:
            with self.assertRaisesRegex(RuntimeError, "test failure"):
                with camera.follow_robot():
                    self.assertIs(mujoco.viewer.launch_passive(model, data, key_callback=callback), viewer)
                    launch.assert_called_once_with(model, data, key_callback=callback)
                    self.assertEqual(viewer.cam.type, mujoco.mjtCamera.mjCAMERA_TRACKING)
                    self.assertEqual(viewer.cam.trackbodyid, model.body("torso_link").id)
                    self.assertEqual(viewer.cam.distance, 3.0)
                    initial_x = viewer.cam.lookat[0]
                    scene = mujoco.MjvScene(model, maxgeom=10)
                    data.qpos[0] = 5
                    mujoco.mj_forward(model, data)
                    for _ in range(30):
                        mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None,
                                              viewer.cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
                    self.assertGreater(viewer.cam.lookat[0], initial_x + 1)
                    raise RuntimeError("test failure")
            self.assertIs(mujoco.viewer.launch_passive, launch)


if __name__ == "__main__":
    unittest.main()
