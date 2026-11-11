"""Show sampled depth in the passive viewer, without another window or renderer."""
from contextlib import contextmanager
import sys
import time
from types import ModuleType


class DepthOverlay:
    def __init__(self, viewer):
        import cv2
        import mujoco
        import numpy as np
        from depth_image import preprocess_depth

        self.viewer = viewer
        self.cv = cv2
        self.mujoco = mujoco
        self.np = np
        self.preprocess = preprocess_depth
        self.next_update = 0.0

    def update(self, depth, sim_time):
        now = time.monotonic()
        if depth is None or now < self.next_update or not self.viewer.is_running():
            return
        self.next_update = now + 1 / 15
        viewport = self.viewer.viewport
        if viewport is None:
            return
        # Larger preview: up to 960 px / 75% of the scene width, capped at half its height.
        width = min(960, viewport.width * 3 // 4,
                    2 * max(0, viewport.height // 2 - 30) * 64 // 36 + 12)
        tile_width = (width - 12) // 2
        tile_height = tile_width * 36 // 64
        height = tile_height + 30
        if tile_width < 32 or height > viewport.height - 16:
            self.viewer.clear_images()
            return

        cv, np = self.cv, self.np
        raw = np.asarray(depth, dtype=np.float32)
        processed = self.preprocess(raw)

        def color(values, no_hit=None):
            level = np.rint((1 - np.clip(values, 0, 1)) * 255).astype(np.uint8)
            pixels = cv.applyColorMap(level, cv.COLORMAP_TURBO)
            if no_hit is not None:
                pixels[no_hit] = 0
            return cv.resize(pixels, (tile_width, tile_height), interpolation=cv.INTER_NEAREST)

        raw_image = color(np.nan_to_num(raw / 2.5, posinf=1.), ~np.isfinite(raw))
        cv.rectangle(raw_image, (tile_width // 4, tile_height // 2),
                     (tile_width * 3 // 4 - 1, tile_height - 1), (255, 255, 255), 1)
        canvas = np.full((height, width, 3), 24, dtype=np.uint8)
        canvas[26:26 + tile_height, 4:4 + tile_width] = raw_image
        canvas[26:26 + tile_height, 8 + tile_width:8 + 2 * tile_width] = color(processed)
        for label, x in (("Raw 36x64", 4), ("Model 18x32", 8 + tile_width)):
            font_scale = min(.65, max(.3, tile_width / 600))
            cv.putText(canvas, label, (x, 18), cv.FONT_HERSHEY_SIMPLEX, font_scale,
                       (240, 240, 240), 1, cv.LINE_AA)
        rectangle = self.mujoco.MjrRect(
            viewport.left + viewport.width - width - 8,
            viewport.bottom + viewport.height - height - 8, width, height)
        # MuJoCo expects top-down RGB; Handle.set_images handles the OpenGL flip.
        self.viewer.set_images((rectangle, cv.cvtColor(canvas, cv.COLOR_BGR2RGB)))

    def close(self):
        if self.viewer.is_running():
            self.viewer.clear_images()


@contextmanager
def install_depth_overlay():
    """Adapt the external simulator's preview hook; restore imports on exit."""
    import mujoco.viewer

    if not hasattr(mujoco.viewer.Handle, "set_images"):
        raise RuntimeError("Depth overlay requires a MuJoCo viewer with Handle.set_images (installed version is too old)")
    original_launch = mujoco.viewer.launch_passive
    previous_preview = sys.modules.get("depth_preview")
    viewers = []

    def launch_with_overlay(*args, **kwargs):
        viewer = original_launch(*args, **kwargs)
        viewers.append(viewer)
        return viewer

    def create_preview():
        if not viewers:
            raise RuntimeError("The MuJoCo viewer must be opened before the depth preview")
        return DepthOverlay(viewers[-1])

    preview = ModuleType("depth_preview")
    preview.DepthPreview = create_preview
    mujoco.viewer.launch_passive = launch_with_overlay
    sys.modules["depth_preview"] = preview
    try:
        yield
    finally:
        mujoco.viewer.launch_passive = original_launch
        if previous_preview is None:
            sys.modules.pop("depth_preview", None)
        else:
            sys.modules["depth_preview"] = previous_preview
