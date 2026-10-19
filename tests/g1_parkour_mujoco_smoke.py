"""External MuJoCo camera/state -> new parkour executable -> DDS -> PD.

No simulator resources are bundled. Uses an isolated positive DDS domain.
"""
import argparse
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mujoco-root", type=Path, required=True)
parser.add_argument("--controller", type=Path, default=PROJECT/"build/g1_parkour")
parser.add_argument("--domain", type=int, default=192)
parser.add_argument("--stale", action="store_true")
parser.add_argument("--ground", action="store_true")
parser.add_argument("--stand-seconds", type=float, default=5)
args = parser.parse_args()
if args.domain <= 0 or args.stale and args.ground or not np.isfinite(args.stand_seconds) or args.stand_seconds <= 0:
    parser.error("Use positive domain/duration, and separate --stale and --ground")
root = args.mujoco_root.resolve()
sys.path.insert(0, str(root/"simulate_python"))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import config
config.ROBOT = "g1"
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
from parkour_observation import ParkourObservationPublisher

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
model = mujoco.MjModel.from_xml_path(str(root/"unitree_robots/g1/scene_29dof.xml"))
model.opt.timestep = .002
data = mujoco.MjData(model)
ChannelFactoryInitialize(args.domain, "lo")
bridge = UnitreeSdk2Bridge(model, data)
observations = ParkourObservationPublisher(model, data, port, PROJECT/"config/g1_parkour.json")
log = tempfile.TemporaryFile(mode="w+t")
controller = subprocess.Popen([str(args.controller.resolve()), "--sim", "--domain", str(args.domain),
    "--port", str(port), "--reset-seconds", ".5", "--model-dir", str(PROJECT/"models/parkour"),
    "--config", str(PROJECT/"config/g1_parkour.json")], stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
events = [(1.,b'p'),(1.1,b'r'),(1.2,b'9'),(1.3,b'p'),(2.6,b'r'),
          (2.8,b'w'),(2.9,b'q'),(3.1,b'x'),(3.3,b'p'),(4.5,b'r'),(5.1,b' ')]
if args.ground:
    events = [(1.2,b'9'),(1.3,b'p'),(2.6,b'r'),(2.7+args.stand_seconds,b' ')]
started = time.monotonic()
policy_changed = grounded = False
min_height, max_drift = float("inf"), 0.
ground_start = None
try:
    while controller.poll() is None and time.monotonic()-started < (7+args.stand_seconds if args.ground else 9):
        before = time.monotonic()
        elapsed = before-started
        while events and elapsed >= events[0][0]:
            _,key = events.pop(0)
            controller.stdin.write(key);controller.stdin.flush()
        if not args.ground or elapsed < 2.7:
            data.qpos[:7] = [0,0,1.2,1,0,0,0]
            data.qvel[:6] = 0
        elif not grounded:
            data.qpos[:7] = [0,0,.78,1,0,0,0]
            data.qvel[:6] = 0
            grounded = True
            ground_start = data.time
        mujoco.mj_forward(model,data)
        if not args.stale or elapsed < 4.7:
            observations.update()
        bridge.UpdateMotorControl()
        adapter = observations.adapter
        data.ctrl[adapter.actuators] = np.clip(data.ctrl[adapter.actuators],-adapter.limits,adapter.limits)
        mujoco.mj_step(model,data)
        if args.ground and grounded:
            min_height = min(min_height,float(data.xpos[adapter.torso,2]))
            max_drift = max(max_drift,float(np.linalg.norm(data.qpos[:2])))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.ctrl).all():
            raise RuntimeError("Non-finite simulation")
        if elapsed < 1.3 and np.any(bridge.motor_command[2:]):
            raise RuntimeError("Dry-run or enable sent commands before ordered reset")
        if 2.7 < elapsed < 3.2:
            neutral = np.empty(29)
            neutral[adapter.actuators] = adapter.policy.default
            policy_changed |= np.max(np.abs(bridge.motor_command[0]-neutral)) > .01
        time.sleep(max(0,.002-(time.monotonic()-before)))
    controller.wait(timeout=3)
    time.sleep(.05)
    log.seek(0);output = log.read();print(output)
    assert controller.returncode == (1 if args.stale else 0), controller.returncode
    assert output.count("Parkour policy ON.") == (1 if args.ground else 2), "Policy entry failed"
    assert policy_changed,"Policy targets never changed"
    assert not np.any(bridge.motor_command[2:]),"Exit did not clear PD/tau"
    if args.stale:
        assert "stale; stopping" in output,"Camera watchdog not triggered"
    if args.ground:
        assert min_height > .65 and max_drift < .5,(min_height,max_drift)
        print(f"Standing {data.time-ground_start:.3f}s simulated, min torso {min_height:.3f}m, max drift {max_drift:.3f}m")
    print("PASS: external MuJoCo depth/state -> parkour double ONNX -> DDS/PD")
finally:
    if controller.poll() is None:
        controller.terminate()
        try:
            controller.wait(timeout=3)
        except subprocess.TimeoutExpired:
            controller.kill();controller.wait()
    observations.close()
    for thread in (bridge.lowStateThread,bridge.HighStateThread,bridge.WirelessControllerThread):
        thread.Wait(1)
    log.close()
