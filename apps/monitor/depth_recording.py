"""Compact, lossless float32 depth recording. Also reads/exports recordings."""
if __name__ == '__main__':
    from pathlib import Path as _Path
    import runpy as _runpy
    _runpy.run_path(str(_Path(__file__).resolve().parents[1] / 'python_runtime.py'))['ensure_runtime'](__file__)

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import select
import subprocess
import tempfile
import struct
import threading
import time
import uuid
import zlib

MAGIC = b'G1DREC01'
FRAME = struct.Struct('<QQddB3xII')
NAME = re.compile(r'g1-depth-[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}\.(?:g1depth|mp4)')


def encode_depth(raw, previous):
    # Temporal XOR plus float-byte shuffling exposes repeated high-order bits.
    delta = bytes(a ^ b for a, b in zip(raw, previous))
    return zlib.compress(b''.join(delta[offset::4] for offset in range(4)), 9)


def read_recording(path, recover=False):
    """Yield metadata, then frames with exact float32 bytes. recover keeps complete
    frames from a recording interrupted during its final write."""
    with Path(path).open('rb') as handle:
        if handle.read(8) != MAGIC:
            raise ValueError('Not a G1 depth recording')
        size = handle.read(4)
        if len(size) != 4:
            raise ValueError('Incomplete recording metadata')
        length = struct.unpack('<I', size)[0]
        if length > 65536:
            raise ValueError('Invalid metadata length')
        metadata = json.loads(handle.read(length))
        if metadata.get('codec') != 'float32-xor-shuffle-zlib-v1':
            raise ValueError('Unsupported depth codec')
        width, height = metadata['width'], metadata['height']
        if not (isinstance(width, int) and isinstance(height, int) and 1 <= width <= 64 and 1 <= height <= 36):
            raise ValueError('Invalid depth dimensions')
        count = width * height * 4
        previous = bytes(count)
        yield metadata
        while True:
            header = handle.read(FRAME.size)
            if not header:
                return
            if len(header) != FRAME.size:
                if recover: return
                raise ValueError('Incomplete final frame header')
            session, sequence, captured, received, source, length, checksum = FRAME.unpack(header)
            if length > count + 1024 or source not in (0, 1):
                raise ValueError('Invalid depth frame header')
            payload = handle.read(length)
            if len(payload) != length:
                if recover: return
                raise ValueError('Incomplete final frame payload')
            decoder = zlib.decompressobj()
            shuffled = decoder.decompress(payload, count + 1)
            if len(shuffled) != count or not decoder.eof or decoder.unused_data:
                raise ValueError('Invalid compressed depth frame')
            delta = bytearray(count)
            pixels = count // 4
            for offset in range(4):
                delta[offset::4] = shuffled[offset*pixels:(offset+1)*pixels]
            raw = bytes(a ^ b for a, b in zip(delta, previous))
            if zlib.crc32(raw) != checksum:
                raise ValueError('Depth frame checksum mismatch')
            previous = raw
            yield dict(session=str(session), sequence=sequence, captured_at=captured,
                       received_at_unix=received, source='simulation' if source else 'realsense', data=raw)


def video_pixels(frame, color):
    options = frame['options']
    low, high = (0, 1) if frame['units'] == 'normalized' else (options['minimum'], options['maximum'])
    pixels = bytearray()
    for value in frame['image']:
        unit = max(0., min(1., (value-low)/(high-low)))
        if color == 'color':
            pixels.extend(round(255*max(0., min(1., 1.5-abs(4*unit-shift)))) for shift in (3, 2, 1))
        else:
            pixels.append(round(unit*255))
    return bytes(pixels)


class VideoWriter:
    def __init__(self, path, width, height, fps, color):
        self.error_log = tempfile.TemporaryFile()
        self.path = path
        self.color = color
        self.process = subprocess.Popen([
            shutil.which('ffmpeg'), '-hide_banner', '-loglevel', 'error', '-nostdin',
            '-f', 'rawvideo', '-pixel_format', 'rgb24' if color == 'color' else 'gray',
            '-video_size', f'{width}x{height}', '-framerate', str(fps), '-i', 'pipe:0',
            '-an', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '32', '-threads', '1', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', '-y', str(path)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.error_log)
        os.set_blocking(self.process.stdin.fileno(), False)

    def write(self, frame):
        data = memoryview(video_pixels(frame, self.color))
        deadline = time.monotonic()+2
        while data:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                raise OSError('视频编码阻塞，录制已停止')
            try:
                count = os.write(self.process.stdin.fileno(), data)
                data = data[count:]
            except BlockingIOError:
                continue

    def close(self):
        try:
            self.process.stdin.close()
            try:
                code = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
                raise OSError('视频编码器结束超时') from None
            if code:
                self.error_log.seek(0)
                raise OSError('视频编码失败：'+self.error_log.read().decode(errors='replace')[-1000:])
            if self.path.exists():
                with self.path.open('rb') as handle: os.fsync(handle.fileno())
        finally:
            self.error_log.close()


class DepthRecorder:
    def __init__(self, preview, directory):
        self.preview = preview
        self.directory = Path(directory)
        self.control = threading.Lock()
        self.lock = threading.Lock()
        self.thread = None
        self.path = None
        self.active = False
        self.frames = 0
        self.error = None
        self.metadata = None
        self.first_frame_at = self.last_frame_at = None

    def status(self):
        with self.lock:
            state = dict(recording=self.active, saving=bool(self.thread and self.thread.is_alive()),
                         filename=self.path.name if self.path else None, frames=self.frames,
                         error=self.error, metadata=self.metadata,
                         sampled_fps=((self.frames-1)/(self.last_frame_at-self.first_frame_at)
                                      if self.frames > 1 and self.last_frame_at > self.first_frame_at else 0))
        files = []
        if self.directory.is_dir():
            for path in sorted(self.directory.glob('g1-depth-*.g1depth'), reverse=True):
                try:
                    if NAME.fullmatch(path.name) and path.is_file() and not path.is_symlink():
                        video = path.with_suffix('.mp4')
                        files.append(dict(name=path.name, bytes=path.stat().st_size,
                                          video=video.name if video.is_file() and not video.is_symlink() else None,
                                          video_bytes=video.stat().st_size if video.is_file() and not video.is_symlink() else 0))
                except FileNotFoundError:
                    continue
        return dict(state, directory=str(self.directory), files=files)

    def start(self, settings, fps=5, video=False, color="gray"):
        with self.control:
            if self.thread and self.thread.is_alive():
                raise ValueError('已有深度录制正在进行或保存中')
            if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or not 5 <= fps <= 30:
                raise ValueError('录制帧率必须在 5～30 FPS 之间')
            if not isinstance(settings, dict):
                raise ValueError('录制参数必须为对象')
            if not isinstance(video, bool) or color not in ('gray', 'color'):
                raise ValueError('视频选项无效')
            if video:
                executable = shutil.which('ffmpeg')
                if executable is None:
                    raise ValueError('视频录制需要 ffmpeg，请运行 scripts/setup_robot.sh 安装依赖')
                encoders = subprocess.run([executable, '-hide_banner', '-encoders'], capture_output=True, text=True, timeout=5)
                if 'libx264' not in encoders.stdout:
                    raise ValueError('当前 ffmpeg 不支持 libx264 编码器')
            query = {key: [str(value)] for key, value in settings.items()}
            frame = self.preview.process(query)
            if frame['error'] or not frame['fresh'] or frame['image'] is None:
                raise ValueError(frame['error'] or '请等待新鲜深度画面后再录制')
            self.directory.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(self.directory).free < 100*1024*1024:
                raise OSError('磁盘可用空间不足 100 MB')
            now = datetime.now(timezone.utc)
            path = self.directory / f'g1-depth-{now:%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}.g1depth'
            metadata = dict(version=1, codec='float32-xor-shuffle-zlib-v1', dtype='<f4',
                            width=frame['width'], height=frame['height'], units=frame['units'],
                            options=frame['options'], fps=fps, created_at_utc=now.isoformat(),
                            description='Processed preview values, losslessly stored; video is a separate lossy visualization.',
                            video=path.with_suffix('.mp4').name if video else None, video_color=color if video else None)
            handle = path.open('xb')
            video_writer = None
            try:
                if video:
                    video_writer = VideoWriter(path.with_suffix('.mp4'), frame['width'], frame['height'], fps, color)
                encoded = json.dumps(metadata, separators=(',', ':')).encode()
                handle.write(MAGIC + struct.pack('<I', len(encoded)) + encoded)
                handle.flush(); os.fsync(handle.fileno())
            except BaseException:
                handle.close()
                if video_writer: video_writer.close()
                raise
            with self.lock:
                self.path, self.metadata = path, metadata
                self.frames, self.error, self.active = 0, None, True
                self.first_frame_at = self.last_frame_at = None
                self.stopping = threading.Event()
                self.thread = threading.Thread(target=self._write, args=(handle, query, fps, frame, video_writer),
                                               name='monitor-depth-record', daemon=True)
                self.thread.start()

    def _write(self, handle, query, fps, first, video_writer):
        previous = bytes(first['width']*first['height']*4)
        last_key = None
        synced = time.monotonic()
        frame = first
        next_frame = time.monotonic()
        try:
            while not self.stopping.is_set():
                key = (frame.get('session'), frame.get('sequence'))
                if frame['error']:
                    raise ValueError(frame['error'])
                if frame['fresh'] and frame['image'] is not None and key != last_key:
                    raw = struct.pack('<'+'f'*len(frame['image']), *frame['image'])
                    if len(raw) != len(previous):
                        raise ValueError('深度图尺寸发生变化')
                    packed = encode_depth(raw, previous)
                    received = time.time() - frame['age_ms']/1000
                    handle.write(FRAME.pack(int(frame['session']), frame['sequence'], frame['captured_at'],
                                            received, int(frame['source']=='simulation'), len(packed), zlib.crc32(raw)))
                    handle.write(packed)
                    if video_writer: video_writer.write(frame)
                    previous, last_key = raw, key
                    with self.lock:
                        now = time.monotonic()
                        if self.first_frame_at is None: self.first_frame_at = now
                        self.last_frame_at = now
                        self.frames += 1
                if time.monotonic()-synced >= 1:
                    handle.flush(); os.fsync(handle.fileno())
                    if shutil.disk_usage(self.directory).free < 10*1024*1024:
                        raise OSError('磁盘可用空间低于 10 MB，深度录制已停止')
                    synced = time.monotonic()
                next_frame += 1/fps
                if next_frame < time.monotonic(): next_frame = time.monotonic()
                self.stopping.wait(max(0, next_frame-time.monotonic()))
                if not self.stopping.is_set(): frame = self.preview.process(query)
            handle.flush(); os.fsync(handle.fileno())
        except Exception as error:
            with self.lock: self.error = f'深度录制失败：{error}'
        finally:
            if video_writer:
                try: video_writer.close()
                except (OSError, ValueError) as error:
                    with self.lock: self.error = f'视频保存失败：{error}'
            try: handle.close()
            except OSError as error:
                with self.lock: self.error = f'深度保存失败：{error}'
            with self.lock: self.active = False

    def stop(self):
        with self.control:
            with self.lock:
                self.active = False
                thread = self.thread
                if thread: self.stopping.set()
            if thread: thread.join()

    def download_path(self, name):
        if not name or not NAME.fullmatch(name): raise FileNotFoundError('深度记录文件不存在')
        path = self.directory / name
        if path.is_symlink() or not path.is_file(): raise FileNotFoundError('深度记录文件不存在')
        with self.lock:
            if self.path and path.stem == self.path.stem and self.thread and self.thread.is_alive():
                raise ValueError('请先停止深度录制并等待保存完成')
        return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--export', type=Path, help='Export NumPy .npz (requires NumPy)')
    parser.add_argument('--recover', action='store_true', help='Read complete frames from an interrupted recording')
    args = parser.parse_args()
    records = read_recording(args.recording, args.recover)
    metadata = next(records)
    if args.export:
        import numpy as np
        frames = list(records)
        images = np.stack([np.frombuffer(frame['data'], dtype='<f4').reshape(metadata['height'], metadata['width']) for frame in frames]) if frames else np.empty((0, metadata['height'], metadata['width']), dtype=np.float32)
        np.savez_compressed(args.export, depth=images, captured_at=np.array([f['captured_at'] for f in frames]),
                            received_at_unix=np.array([f['received_at_unix'] for f in frames]),
                            sequence=np.array([f['sequence'] for f in frames], dtype=np.uint64),
                            session=np.array([int(f['session']) for f in frames], dtype=np.uint64),
                            source=np.array([f['source'] for f in frames]), metadata=json.dumps(metadata))
        count = len(frames)
    else:
        count = sum(1 for _ in records)
    print(json.dumps(dict(metadata=metadata, frames=count, bytes=args.recording.stat().st_size), ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
