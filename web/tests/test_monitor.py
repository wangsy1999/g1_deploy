import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joint_monitor import JointMonitor


def command(q, enabled=0):
    return SimpleNamespace(motor_cmd=[SimpleNamespace(q=q, dq=0, kp=20, kd=1, tau=0, mode=enabled) for _ in range(29)])


def state():
    return SimpleNamespace(mode_machine=5,
        motor_state=[SimpleNamespace(q=0.1, dq=0.2, tau_est=0.3) for _ in range(29)],
        imu_state=SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0, 0, 0], accelerometer=[0, 0, 9.81]))


class MonitorSmokeTest(unittest.TestCase):
    def setUp(self):
        self.monitor = JointMonitor('127.0.0.1', 0)
        self.monitor.start()
        self.url = 'http://127.0.0.1:' + str(self.monitor.server.server_address[1])
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def tearDown(self):
        self.monitor.close()

    def capture(self):
        self.monitor.next_capture = 0
        self.monitor.on_state(state())
        with self.http.open(self.url + '/api/joints', timeout=3) as response:
            return json.load(response)

    def test_static_page_api_and_sse(self):
        with self.http.open(self.url, timeout=3) as response:
            self.assertEqual(response.read(), (Path(__file__).resolve().parents[1] / 'static/joint_monitor.html').read_bytes())
        data = self.capture()
        self.assertEqual(len(data['joints']), 29)
        self.assertEqual(data['imu']['rpy'], [0, 0, 0])
        with self.http.open(self.url + '/api/stream', timeout=3) as response:
            line = response.readline().decode()
            self.assertTrue(line.startswith('data: '))
            self.assertEqual(json.loads(line[6:])['sample_id'], data['sample_id'])
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.http.open(self.url + '/missing', timeout=3)
        self.assertEqual(caught.exception.code, 404)

    def test_dry_targets_and_real_enable_priority(self):
        self.monitor.on_command(command(0.2))
        self.monitor.on_dry_target(command(0.7))
        self.monitor.on_enable(command(0, enabled=0))
        data = self.capture()
        self.assertTrue(data['dry_run'])
        self.assertIs(data['output_enabled'], False)
        self.assertEqual(data['joints'][0]['q_target'], 0.7)
        self.monitor.on_target(command(0.4))
        self.monitor.on_enable(command(0, enabled=1))
        data = self.capture()
        self.assertFalse(data['dry_run'])
        self.assertIs(data['output_enabled'], True)
        self.assertEqual(data['joints'][0]['q_target'], 0.4)

    def test_expired_preview_does_not_remain_active(self):
        self.monitor.on_dry_target(command(0.7))
        self.monitor.dry_target_at -= 1
        data = self.capture()
        self.assertFalse(data['dry_run'])
        self.assertFalse(data['joints'][0]['target_available'])


if __name__ == '__main__':
    unittest.main()
