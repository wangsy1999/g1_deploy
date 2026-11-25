"""Receive low-rate controller depth copies; never import or open RealSense."""
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time


HEADER = struct.Struct('<8sQQdHHB3x')
MAGIC = b'G1DPV001'
WIDTH, HEIGHT = 64, 36
DEFAULT_PORT = 9871


def decode_packet(packet):
    if len(packet) != HEADER.size + WIDTH * HEIGHT * 4:
        raise ValueError('Invalid depth preview packet length')
    magic, session, sequence, captured_at, width, height, source = HEADER.unpack_from(packet)
    if (magic != MAGIC or (width, height) != (WIDTH, HEIGHT) or source not in (0, 1)
            or not math.isfinite(captured_at)):
        raise ValueError('Invalid depth preview header')
    values = struct.unpack_from(f'<{WIDTH * HEIGHT}f', packet, HEADER.size)
    if any(math.isnan(value) or value < 0 or (source == 0 and not math.isfinite(value)) for value in values):
        raise ValueError('Invalid depth preview values')
    return dict(session=str(session), sequence=sequence, captured_at=captured_at,
                width=width, height=height, source='simulation' if source else 'realsense',
                payload=packet[HEADER.size:])


class DepthPreview:
    def __init__(self, port=DEFAULT_PORT, profile=None):
        self.port = port
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = None
        self.socket = None
        self.frame = None
        self.received_at = None
        self.error = None
        self.cv = self.np = None
        self.defaults = dict(crop=True, inpaint=True, filter=False, normalize=True,
                             top=18, bottom=0, left=16, right=16,
                             threshold=0.2, radius=3.0, kernel=3, sigma=1.0,
                             minimum=0.0, maximum=2.5)
        if profile and Path(profile).is_file():
            settings = json.loads(Path(profile).read_text()).get('preprocessing', {})
            self.defaults.update(zip(('top', 'bottom', 'left', 'right'), settings.get('crop', [18, 0, 16, 16])))
            self.defaults.update(threshold=settings.get('inpaint_below_m', 0.2),
                                 radius=settings.get('inpaint_radius', 3),
                                 filter=bool(settings.get('gaussian_kernel', 0)),
                                 kernel=settings.get('gaussian_kernel') or 3,
                                 sigma=settings.get('gaussian_sigma', 1.0))
            self.defaults.update(zip(('minimum', 'maximum'), settings.get('depth_range', [0, 2.5])))

    def start(self):
        if self.port == 0:
            self.error = '深度预览已禁用（UDP 端口为 0）'
            return
        try:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket = receiver
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
            receiver.bind(('127.0.0.1', self.port))
            receiver.settimeout(0.2)
            self.thread = threading.Thread(target=self._receive, name='monitor-depth', daemon=True)
            self.thread.start()
        except OSError as error:
            self.error = f'无法接收深度预览：{error}'
            if self.socket:
                self.socket.close()
                self.socket = None

    def _receive(self):
        while not self.stopping.is_set():
            try:
                packet, _ = self.socket.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = decode_packet(packet)
            except ValueError:
                continue
            with self.lock:
                if (self.frame and frame['session'] == self.frame['session']
                        and frame['sequence'] <= self.frame['sequence']):
                    continue
                self.frame = frame
                self.received_at = time.monotonic()

    def close(self):
        self.stopping.set()
        if self.socket:
            self.socket.close()
        if self.thread:
            self.thread.join(timeout=1)

    def config(self):
        return dict(port=self.port, defaults=self.defaults, width=WIDTH, height=HEIGHT)

    def options(self, query):
        options = dict(self.defaults)
        for key in ('crop', 'inpaint', 'filter', 'normalize'):
            value = query.get(key, [str(int(options[key]))])[0]
            if value not in ('0', '1'):
                raise ValueError(f'{key} 必须为 0 或 1')
            options[key] = value == '1'
        limits = dict(top=(0, HEIGHT-1), bottom=(0, HEIGHT-1), left=(0, WIDTH-1), right=(0, WIDTH-1),
                      threshold=(0, 100), radius=(0.1, 10), kernel=(1, 15), sigma=(0.01, 10),
                      minimum=(0, 100), maximum=(0.01, 100))
        for key, (low, high) in limits.items():
            try:
                value = float(query.get(key, [options[key]])[0])
            except (ValueError, TypeError):
                raise ValueError(f'{key} 必须为有效数字') from None
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{key} 必须在 {low} 到 {high} 之间')
            if key in ('top', 'bottom', 'left', 'right', 'kernel'):
                if not value.is_integer():
                    raise ValueError(f'{key} 必须为整数')
                value = int(value)
            options[key] = value
        if options['crop'] and (options['top'] + options['bottom'] >= HEIGHT or
                                options['left'] + options['right'] >= WIDTH):
            raise ValueError('裁剪后必须至少保留一个像素')
        if options['kernel'] % 2 == 0:
            raise ValueError('滤波核大小必须为奇数')
        if options['minimum'] >= options['maximum']:
            raise ValueError('深度上限必须大于下限')
        return options

    def process(self, query):
        options = self.options(query)
        with self.lock:
            frame, received_at = self.frame, self.received_at
        age = None if received_at is None else (time.monotonic()-received_at)*1000
        result = dict(available=frame is not None, fresh=age is not None and age < 1000,
                      age_ms=age, error=self.error, options=options, image=None)
        if frame is None:
            return result
        result.update({key: value for key, value in frame.items() if key != 'payload'})
        if self.np is None:
            try:
                import cv2
                import numpy
                cv2.setNumThreads(1)
                self.cv, self.np = cv2, numpy
            except ImportError:
                result['error'] = '预览需要 numpy 和 opencv-python-headless，请运行依赖配置脚本'
                return result
        np, cv = self.np, self.cv
        image = np.frombuffer(frame['payload'], dtype='<f4').reshape(HEIGHT, WIDTH).copy()
        # Simulation uses +inf for rays without a hit; display these at the far plane.
        np.nan_to_num(image, copy=False, posinf=options['maximum'])
        if options['crop']:
            image = image[options['top']:HEIGHT-options['bottom'],
                          options['left']:WIDTH-options['right']].copy()
        if options['inpaint']:
            image = cv.inpaint(image, (image < options['threshold']).astype(np.uint8),
                               options['radius'], cv.INPAINT_NS)
        if options['filter']:
            image = cv.GaussianBlur(image, (options['kernel'], options['kernel']), options['sigma'])
        if options['normalize']:
            image = (np.clip(image, options['minimum'], options['maximum']) - options['minimum']) / (
                options['maximum'] - options['minimum'])
        result.update(width=int(image.shape[1]), height=int(image.shape[0]),
                      units='normalized' if options['normalize'] else 'm', image=image.ravel().tolist())
        return result
