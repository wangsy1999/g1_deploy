"""CSV persistence and HTTP recording controls; no DDS SDK or robot required."""
import csv
import io
import json
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from apps.monitor.csv_recorder import CsvRecorder
from apps.monitor.joint_monitor import JOINT_NAMES, JointMonitor, read_motor_temperature


def sample(number=1):
    return dict(sample_id=number, simulation_time=number/50, age_ms=0,
                output_enabled=False, dry_run=True,
                imu=dict(rpy=[.1, .2, .3], gyroscope=[1, 2, 3],
                         accelerometer=[4, 5, 6], quaternion=[1, 0, 0, 0]),
                torso_imu=dict(age_ms=150, fresh=False, valid=True, samples=9,
                               data=dict(projected_gravity=[0, 0, -1])),
                joints=[dict(index=i, name=name, enabled=False, active=False,
                             target_available=i == 0, q=-.123456789, dq=.5, tau=2,
                             temperature_0_c=42, temperature_1_c=45,
                             q_target=.25, dq_target=0, error=.373456789, kp=20, kd=1)
                        for i, name in enumerate(JOINT_NAMES)])


class TemperatureTests(unittest.TestCase):
    def test_two_channels_and_missing_values(self):
        cases = [(SimpleNamespace(temperature=[41, 46]), [41.0, 46.0]),
                 (SimpleNamespace(), [None, None]),
                 (SimpleNamespace(temperature=None), [None, None]),
                 (SimpleNamespace(temperature=[0]), [0.0, None]),
                 (SimpleNamespace(temperature=[float('nan'), float('inf')]), [None, None]),
                 (SimpleNamespace(temperature=[None, 'bad']), [None, None])]
        for motor, expected in cases:
            with self.subTest(motor=motor):
                self.assertEqual(read_motor_temperature(motor), expected)


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.recorder = CsvRecorder(self.temp.name, JOINT_NAMES)

    def tearDown(self):
        self.recorder.stop()
        self.temp.cleanup()

    def test_complete_rows_survive_stop_and_new_recorder(self):
        recorder = self.recorder
        recorder.start()
        with self.assertRaises(ValueError):
            recorder.start()
        for number in range(1, 21):
            recorder.submit(sample(number))
        recorder.stop()
        state = recorder.status()
        self.assertEqual(state['rows'], 20)
        self.assertFalse(state['saving'])
        self.assertEqual(state['lost_samples'], 0)
        path = recorder.download_path(state['filename'])
        with path.open(encoding='utf-8-sig', newline='') as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 20)
        self.assertEqual(rows[-1]['sample_id'], '20')
        prefix = 'joint_0_left_hip_pitch_joint_'
        self.assertEqual(rows[0][prefix+'q_rad'], '-0.123456789')
        self.assertEqual(rows[0][prefix+'q_target_rad'], '0.25')
        self.assertEqual(rows[0][prefix+'enabled'], '0')
        self.assertEqual(rows[0][prefix+'temperature_0_c'], '42')
        self.assertEqual(rows[0][prefix+'temperature_1_c'], '45')
        self.assertEqual(rows[0]['joint_1_left_hip_roll_joint_q_target_rad'], '')
        self.assertEqual(rows[0]['torso_fresh'], '0')
        self.assertEqual(rows[0]['torso_projected_gravity_z'], '-1')
        self.assertEqual(rows[0]['pelvis_roll_rad'], '0.1')
        self.assertIsNone(rows[0].get(None))  # Every row has exactly the header's width.
        fresh = CsvRecorder(self.temp.name, JOINT_NAMES)
        self.assertEqual(fresh.status()['files'][0]['name'], path.name)
        self.assertEqual(fresh.download_path(path.name), path)
        recorder.start()
        recorder.stop()
        self.assertEqual(len(recorder.status()['files']), 2)

    def test_queue_overflow_stops_and_reports_missing_sample(self):
        entered, release = threading.Event(), threading.Event()
        original = self.recorder._row
        def blocked_row(*args):
            entered.set()
            if not release.wait(5):
                raise OSError('test timed out')
            return original(*args)
        self.recorder._row = blocked_row
        self.recorder.start()
        self.recorder.submit(sample(1))
        self.assertTrue(entered.wait(2))
        try:
            with self.recorder.lock:
                self.recorder.pending = queue.Queue(maxsize=1)
            self.recorder.submit(sample(2))
            self.recorder.submit(sample(3))
            self.assertFalse(self.recorder.status()['recording'])
        finally:
            release.set()
        self.recorder.stop()
        state = self.recorder.status()
        self.assertEqual(state['rows'], 2)
        self.assertEqual(state['lost_samples'], 1)
        self.assertIn('队列溢出', state['error'])

    def test_writer_error_is_reported(self):
        def fail(*_):
            raise OSError('disk full')
        self.recorder._row = fail
        self.recorder.start()
        self.recorder.submit(sample())
        self.recorder.stop()
        state = self.recorder.status()
        self.assertIn('disk full', state['error'])
        self.assertEqual(state['lost_samples'], 1)
        self.assertFalse(state['saving'])

    def test_start_refuses_low_disk_space(self):
        with patch('apps.monitor.csv_recorder.shutil.disk_usage',
                   return_value=SimpleNamespace(free=10 * 1024 * 1024)):
            with self.assertRaises(OSError):
                self.recorder.start()
        self.assertFalse(self.recorder.status()['recording'])
        self.assertEqual(self.recorder.status()['files'], [])

    def test_download_rejects_paths_and_symlinks(self):
        for name in ('../secret.csv', '/tmp/a.csv', 'g1-monitor-\r\n.csv'):
            with self.assertRaises(FileNotFoundError):
                self.recorder.download_path(name)
        outside = Path(self.temp.name) / 'outside.csv'
        outside.touch()
        link = Path(self.temp.name) / 'g1-monitor-20260101T000000Z-12345678.csv'
        link.symlink_to(outside)
        with self.assertRaises(FileNotFoundError):
            self.recorder.download_path(link.name)


class HttpRecordingTests(unittest.TestCase):
    def test_record_without_browser_then_download(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = JointMonitor('127.0.0.1', 0, record_dir=directory)
            monitor.start()
            base = 'http://127.0.0.1:'+str(monitor.server.server_port)
            def request(path, method='GET', origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin:
                    headers['Origin'] = origin
                return urlopen(Request(base+path, data=b'{}' if method == 'POST' else None,
                                       method=method, headers=headers), timeout=5)
            try:
                with request('/api/recordings/start', 'POST') as response:
                    name = json.load(response)['filename']
                with self.assertRaises(HTTPError) as error:
                    request('/api/recordings/download?name='+name)
                self.assertEqual(error.exception.code, 409)
                error.exception.close()
                # No SSE/browser connection exists while the DDS callback receives a frame.
                motor = SimpleNamespace(q=.1, dq=.2, tau_est=.3, temperature=[42, 45])
                imu = SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0]*3, accelerometer=[0]*3)
                msg = SimpleNamespace(motor_state=[motor]*29, mode_machine=5, imu_state=imu)
                monitor.on_state(msg)
                with request('/api/recordings') as response:
                    self.assertTrue(json.load(response)['recording'])
                with request('/api/recordings/stop', 'POST') as response:
                    state = json.load(response)
                self.assertEqual(state['rows'], 1)
                self.assertFalse(state['saving'])
                with request('/api/recordings/download?name='+name) as response:
                    rows = list(csv.DictReader(io.StringIO(response.read().decode('utf-8-sig'))))
                    self.assertIn(name, response.headers['Content-Disposition'])
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]['joint_28_right_wrist_yaw_joint_q_rad'], '0.1')
                self.assertEqual(rows[0]['joint_28_right_wrist_yaw_joint_temperature_0_c'], '42.0')
                self.assertEqual(rows[0]['joint_28_right_wrist_yaw_joint_temperature_1_c'], '45.0')
                with self.assertRaises(HTTPError) as error:
                    request('/api/recordings/start', 'POST', 'https://other.example')
                self.assertEqual(error.exception.code, 403)
                error.exception.close()
            finally:
                monitor.close()


if __name__ == '__main__':
    unittest.main()
