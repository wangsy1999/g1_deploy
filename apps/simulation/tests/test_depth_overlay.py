"""Exercise image placement and preview hooks without opening a viewer."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


MODULE = Path(__file__).resolve().parents[1] / "viewer_depth_overlay.py"
spec = importlib.util.spec_from_file_location("viewer_depth_overlay", MODULE)
overlay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(overlay)


class DepthOverlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import cv2
            import mujoco
            import mujoco.viewer
            import numpy as np
        except ImportError as error:
            raise unittest.SkipTest(str(error))
        cls.cv, cls.mujoco, cls.np = cv2, mujoco, np

    def test_pixels_resize_and_top_right_placement(self):
        np, mujoco = self.np, self.mujoco
        images = []
        viewer = SimpleNamespace(viewport=mujoco.MjrRect(40, 20, 1200, 800),
                                 is_running=lambda: True,
                                 set_images=images.append, clear_images=images.clear)
        preprocessing = ModuleType("depth_image")
        preprocessing.preprocess_depth = lambda raw: raw[18:, 16:48].clip(0, 2.5) / 2.5
        with patch.dict(sys.modules, {"depth_image": preprocessing}), \
             patch.object(self.cv, "namedWindow", side_effect=AssertionError("Must not open another window")):
            preview = overlay.DepthOverlay(viewer)
            depth = np.full((36, 64), 2.5, dtype=np.float32)
            depth[:, :16] = np.inf
            depth[18:, 16:48] = 0
            preview.update(depth, 1.0)
            rect, pixels = images[-1]
            self.assertEqual((pixels.dtype, pixels.shape), (np.dtype("uint8"), (rect.height, rect.width, 3)))
            self.assertEqual(rect.left + rect.width, 40 + 1200 - 8)
            self.assertEqual(rect.bottom + rect.height, 20 + 800 - 8)
            self.assertGreater(rect.width, 480, "Preview should be larger than the previous size")
            self.assertLessEqual(rect.height, 800 // 2)
            self.assertTrue((pixels[35, 5] == 0).all(), "No-hit rays must be black")
            self.assertFalse((pixels[35, 8 + (rect.width - 12) // 2] == 0).all(), "Model depth must be colored")
            preview.update(depth, 1.0)
            self.assertEqual(len(images), 1, "Preview must be throttled")
            viewer.viewport = mujoco.MjrRect(10, 15, 400, 300)
            preview.next_update = 0
            preview.update(depth, 2.0)
            small, _ = images[-1]
            self.assertLess(small.width, rect.width)
            self.assertEqual(small.left + small.width, 10 + 400 - 8)
            self.assertEqual(small.bottom + small.height, 15 + 300 - 8)
            preview.close()
            self.assertEqual(images, [])

    def test_external_preview_hook_is_restored_on_error(self):
        import mujoco.viewer
        viewer = object()
        existing = ModuleType("depth_preview")
        with patch.object(mujoco.viewer, "launch_passive", return_value=viewer) as launch, \
             patch.dict(sys.modules, {"depth_preview": existing}), \
             patch.object(overlay, "DepthOverlay", return_value="overlay") as factory:
            with self.assertRaisesRegex(RuntimeError, "test failure"):
                with overlay.install_depth_overlay():
                    self.assertIs(mujoco.viewer.launch_passive("model", "data"), viewer)
                    self.assertEqual(sys.modules["depth_preview"].DepthPreview(), "overlay")
                    factory.assert_called_once_with(viewer)
                    raise RuntimeError("test failure")
            self.assertIs(mujoco.viewer.launch_passive, launch)
            self.assertIs(sys.modules["depth_preview"], existing)


if __name__ == "__main__":
    unittest.main()
