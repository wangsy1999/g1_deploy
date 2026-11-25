"""Bounded, asynchronous CSV recording for the read-only DDS monitor."""
import csv
from datetime import datetime, timezone
import os
from pathlib import Path
import queue
import re
import shutil
import threading
import time
import uuid


IMU_FIELDS = (('rpy', ('roll_rad', 'pitch_rad', 'yaw_rad')),
              ('gyroscope', ('gx_rad_s', 'gy_rad_s', 'gz_rad_s')),
              ('accelerometer', ('ax_m_s2', 'ay_m_s2', 'az_m_s2')),
              ('quaternion', ('qw', 'qx', 'qy', 'qz')))
JOINT_FIELDS = (('enabled', 'enabled'), ('active', 'active'),
                ('target_available', 'target_available'), ('q_rad', 'q'),
                ('dq_rad_s', 'dq'), ('tau_Nm', 'tau'),
                ('temperature_0_c', 'temperature_0_c'), ('temperature_1_c', 'temperature_1_c'),
                ('q_target_rad', 'q_target'),
                ('dq_target_rad_s', 'dq_target'), ('error_rad', 'error'), ('kp', 'kp'), ('kd', 'kd'))
TARGET_FIELDS = {'q_target', 'dq_target', 'error', 'kp', 'kd'}


def imu_columns(prefix, gravity=False):
    columns = [prefix + '_' + name for _, names in IMU_FIELDS for name in names]
    return columns + ([prefix + '_projected_gravity_' + axis for axis in 'xyz'] if gravity else [])


def imu_values(imu, gravity=False):
    imu = imu or {}
    values = [(imu.get(key) or [None] * len(names))[i]
              for key, names in IMU_FIELDS for i in range(len(names))]
    return values + (list(imu.get('projected_gravity') or [None] * 3) if gravity else [])


class CsvRecorder:
    def __init__(self, directory, joint_names):
        self.directory = Path(directory).expanduser().resolve()
        self.names = tuple(joint_names)
        self.lock = threading.Lock()
        self.control = threading.Lock()
        self.thread = None
        self.active = False
        self.path = None
        self.rows = 0
        self.error = None
        self.accepted = 0
        self.overflow = 0

    def status(self):
        with self.lock:
            thread, path = self.thread, self.path
            state = dict(recording=self.active, saving=bool(thread and thread.is_alive()),
                         filename=path.name if path else None, rows=self.rows, error=self.error,
                         lost_samples=self.overflow + (max(0, self.accepted-self.rows)
                                      if thread and not thread.is_alive() else 0))
        state['directory'] = str(self.directory)
        state['files'] = []
        if self.directory.is_dir():
            for file in sorted(self.directory.glob('g1-monitor-*.csv'), reverse=True):
                if file.is_file() and not file.is_symlink():
                    state['files'].append(dict(name=file.name, bytes=file.stat().st_size))
        return state

    def start(self):
        with self.control:
            if self.thread and self.thread.is_alive():
                raise ValueError('已有记录正在进行或保存中')
            self.directory.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(self.directory).free < 100 * 1024 * 1024:
                raise OSError('磁盘可用空间不足 100 MB，无法开始记录')
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            path = self.directory / f'g1-monitor-{stamp}-{uuid.uuid4().hex[:8]}.csv'
            handle = path.open('x', encoding='utf-8-sig', newline='')
            writer = csv.writer(handle)
            header = ['received_at_utc', 'sample_id', 'monitor_time_s', 'state_age_ms',
                      'output_enabled', 'dry_run', *imu_columns('pelvis'),
                      'torso_age_ms', 'torso_fresh', 'torso_valid', 'torso_samples',
                      *imu_columns('torso', True)]
            header += [f'joint_{index}_{name}_{column}'
                       for index, name in enumerate(self.names) for column, _ in JOINT_FIELDS]
            try:
                writer.writerow(header)
                handle.flush()
                os.fsync(handle.fileno())
            except BaseException:
                handle.close()
                raise
            with self.lock:
                self.path, self.rows, self.error = path, 0, None
                self.accepted, self.overflow = 0, 0
                self.pending = queue.Queue(maxsize=2000)
                self.stop_event = threading.Event()
                self.thread = threading.Thread(target=self._write, args=(handle, writer),
                                               name='monitor-csv', daemon=True)
                self.active = True
                self.thread.start()

    def submit(self, state):
        # Called from DDS: only enqueue references to immutable snapshot data.
        with self.lock:
            if not self.active:
                return
            item = (datetime.now(timezone.utc).isoformat(), state)
            try:
                self.pending.put_nowait(item)
                self.accepted += 1
            except queue.Full:
                self.overflow += 1
                self.error = '磁盘写入跟不上采样，记录已停止；队列溢出丢失 1 帧'
                self.active = False
                self.stop_event.set()

    def _row(self, received_at, state):
        torso = state.get('torso_imu') or {}
        row = [received_at, state['sample_id'], state['simulation_time'], state.get('age_ms'),
               state.get('output_enabled'), state.get('dry_run'), *imu_values(state.get('imu')),
               torso.get('age_ms'), torso.get('fresh'), torso.get('valid'), torso.get('samples'),
               *imu_values(torso.get('data'), True)]
        joints = {joint['index']: joint for joint in state['joints']}
        for index, name in enumerate(self.names):
            joint = joints.get(index, {})
            for _, key in JOINT_FIELDS:
                row.append(joint.get(key) if joint.get('name') == name and
                           (key not in TARGET_FIELDS or joint.get('target_available')) else None)
        return [int(value) if isinstance(value, bool) else value for value in row]

    def _write(self, handle, writer):
        synced_at = time.monotonic()
        try:
            while not self.stop_event.is_set() or not self.pending.empty():
                try:
                    received_at, state = self.pending.get(timeout=0.2)
                except queue.Empty:
                    state = None
                if state is not None:
                    writer.writerow(self._row(received_at, state))
                    with self.lock:
                        self.rows += 1
                if time.monotonic() - synced_at >= 1:
                    handle.flush()
                    os.fsync(handle.fileno())
                    if shutil.disk_usage(self.directory).free < 10 * 1024 * 1024:
                        raise OSError('磁盘可用空间低于 10 MB，记录已停止')
                    synced_at = time.monotonic()
            handle.flush()
            os.fsync(handle.fileno())
        except (OSError, ValueError) as error:
            with self.lock:
                self.error = f'CSV 写入失败：{error}'
        finally:
            try:
                handle.close()
            except OSError as error:
                with self.lock:
                    self.error = f'CSV 关闭失败：{error}'
            with self.lock:
                self.active = False

    def stop(self):
        with self.control:
            with self.lock:
                self.active = False
                thread = self.thread
                if thread:
                    self.stop_event.set()
            if thread:
                thread.join()

    def download_path(self, name):
        if not name or not re.fullmatch(r'g1-monitor-[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}\.csv', name):
            raise FileNotFoundError('记录文件不存在')
        path = self.directory / name
        if path.is_symlink() or not path.is_file():
            raise FileNotFoundError('记录文件不存在')
        with self.lock:
            if path == self.path and self.thread and self.thread.is_alive():
                raise ValueError('请先停止记录并等待保存完成')
        return path
