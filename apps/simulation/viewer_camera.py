"""Configure the external simulator's viewer without changing its scene files."""
from contextlib import contextmanager


@contextmanager
def follow_robot():
    import mujoco
    import mujoco.viewer

    original_launch = mujoco.viewer.launch_passive

    def launch_tracking(model, data, *args, **kwargs):
        # Resolve before launching so an invalid scene does not leave an open window.
        torso = model.body("torso_link").id
        viewer = original_launch(model, data, *args, **kwargs)
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = torso
            viewer.cam.lookat[:] = data.subtree_com[torso]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
        return viewer

    mujoco.viewer.launch_passive = launch_tracking
    try:
        yield
    finally:
        mujoco.viewer.launch_passive = original_launch
