"""Publish synchronized simulation observations through local shared memory."""
from pathlib import Path
import json
from g1_shared_memory import SharedFeed
import struct
import threading



def joint_signature(names, signs):
    value = 2166136261
    for byte in ''.join(f'{name}:{int(sign)}\n' for name, sign in zip(names, signs)).encode():
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


class ParkourObservationPublisher:
    def __init__(self, model, data, config_path=None):
        from mujoco_adapter import MujocoAdapter
        self.config_path = str(config_path)
        cfg = json.loads(Path(config_path or Path(__file__).with_name('g1.json')).read_text())
        self.adapter = MujocoAdapter(model, data, cfg)
        self.signature = joint_signature(cfg['joint_names'], cfg['joint_sign'])
        self.data = data
        self.period = cfg['step_dt']
        self.next_time = -1.
        self.last_time = -1.
        self.sequence = 0
        self.preview_lock = threading.Lock()
        self.latest_depth = None
        self.latest_depth_time = 0.
        self.feed = SharedFeed("observation", 32 + (29 + 29 + 3 + 3 + 1 + 36 * 64) * 4)

    def update(self):
        import numpy as np
        now = self.data.time
        if now < self.last_time:
            self.next_time = now
        self.last_time = now
        if now + 1e-9 < self.next_time:
            return
        self.next_time = now + self.period
        q, dq, omega, gravity, depth = self.adapter.observation()
        with self.preview_lock:
            self.latest_depth = depth.copy()
            self.latest_depth_time = now
        height = self.data.xpos[self.adapter.torso, 2]
        payload = np.concatenate([q, dq, omega, gravity, [height], depth.ravel()]).astype('<f4')
        header = struct.pack('<8sQdII', b'G1PKR001', self.sequence, now, self.signature, len(payload))
        self.sequence += 1
        self.feed.publish(header + payload.tobytes())

    def close(self):
        self.feed.close()

    def preview_frame(self):
        with self.preview_lock:
            return (None if self.latest_depth is None else self.latest_depth.copy(),
                    self.latest_depth_time)
