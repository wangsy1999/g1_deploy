"""Local depth telemetry and independent preview processing; no camera or DDS."""
import json
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen

from apps.monitor.depth_preview import HEADER, HEIGHT, MAGIC, WIDTH, DepthPreview, decode_packet
from apps.monitor.joint_monitor import JointMonitor

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


def packet(sequence=1, session=1, source=0, values=None):
    values = [1.0] * (WIDTH * HEIGHT) if values is None else values
    return HEADER.pack(MAGIC, session, sequence, 1.25, WIDTH, HEIGHT, source) + struct.pack(f'<{WIDTH*HEIGHT}f', *values)


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_for(predicate):
    end = time.monotonic()+3
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Preview did not arrive')


class ProtocolTests(unittest.TestCase):
    def test_validation_and_session(self):
        frame = decode_packet(packet(session=2**63+123))
        self.assertEqual(frame['session'], str(2**63+123))
        self.assertEqual(frame['sequence'], 1)
        self.assertEqual(frame['source'], 'realsense')
        for bad in (packet()[:-1], b'WRONGMAG'+packet()[8:], packet(values=[float('nan')]*(WIDTH*HEIGHT)),
                    packet(values=[-1.0]*(WIDTH*HEIGHT)), packet(values=[float('inf')]*(WIDTH*HEIGHT))):
            with self.assertRaises(ValueError):
                decode_packet(bad)
        self.assertEqual(decode_packet(packet(source=1, values=[float('inf')]*(WIDTH*HEIGHT)))['source'], 'simulation')

    def test_invalid_options_and_missing_frame(self):
        preview = DepthPreview()
        for query in ({'top':['36']}, {'left':['32'], 'right':['32']}, {'kernel':['2']},
                      {'radius':['nan']}, {'normalize':['yes']}, {'minimum':['3'], 'maximum':['2']},
                      {'kernel':['3.5']}):
            with self.assertRaises(ValueError):
                preview.options(query)
        state = preview.process({})
        self.assertFalse(state['available'])
        self.assertIsNone(state['image'])

    def test_busy_or_disabled_preview_port_is_reported_without_crashing(self):
        disabled = DepthPreview(0)
        disabled.start()
        self.assertIn('禁用', disabled.process({})['error'])
        disabled.close()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('127.0.0.1', 0))
            preview = DepthPreview(occupied.getsockname()[1])
            preview.start()
            self.assertIn('无法接收', preview.process({})['error'])
            self.assertFalse(preview.process({})['available'])
            preview.close()

    def test_receiver_restart_stale_frame_and_malformed_packets(self):
        preview = DepthPreview(free_port())
        preview.start()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                address = ('127.0.0.1', preview.port)
                sender.sendto(b'bad', address)
                sender.sendto(packet(sequence=10), address)
                wait_for(lambda: preview.frame is not None)
                sender.sendto(packet(sequence=9), address)
                time.sleep(.03)
                self.assertEqual(preview.frame['sequence'], 10)
                sender.sendto(packet(sequence=1, session=2), address)
                wait_for(lambda: preview.frame['session']=='2')
            with preview.lock:
                preview.received_at -= 2
            self.assertFalse(preview.process({})['fresh'])
        finally:
            preview.close()


@unittest.skipUnless(np is not None, 'NumPy/OpenCV needed for image processing tests')
class ProcessingTests(unittest.TestCase):
    def make_preview(self):
        image = np.linspace(0, 3, WIDTH*HEIGHT, dtype=np.float32).reshape(HEIGHT, WIDTH)
        image[21:23, 25:27] = 0
        preview = DepthPreview()
        preview.frame = decode_packet(packet(values=image.ravel().tolist()))
        preview.received_at = time.monotonic()
        return preview, image

    def test_all_four_stages_match_opencv_and_do_not_mutate_raw_data(self):
        preview, raw = self.make_preview()
        payload = preview.frame['payload']
        for crop in (False, True):
            for inpaint in (False, True):
                for blur in (False, True):
                    for normalize in (False, True):
                        with self.subTest(crop=crop, inpaint=inpaint, filter=blur, normalize=normalize):
                            expected = raw.copy()
                            if crop:
                                expected = expected[18:, 16:-16].copy()
                            if inpaint:
                                expected = cv2.inpaint(expected, (expected < .2).astype(np.uint8), 3, cv2.INPAINT_NS)
                            if blur:
                                expected = cv2.GaussianBlur(expected, (3, 3), 1)
                            if normalize:
                                expected = np.clip(expected, 0, 2.5)/2.5
                            query = {key:[str(int(value))] for key,value in dict(crop=crop,inpaint=inpaint,filter=blur,normalize=normalize).items()}
                            state = preview.process(query)
                            np.testing.assert_allclose(np.array(state['image']).reshape(expected.shape), expected, rtol=1e-6)
                            self.assertEqual((state['width'],state['height']), (expected.shape[1], expected.shape[0]))
                            self.assertEqual(preview.frame['payload'], payload)
                            json.dumps(state, allow_nan=False)

    def test_custom_crop_range_and_simulation_infinity(self):
        preview, _ = self.make_preview()
        state = preview.process(dict(top=['1'],bottom=['2'],left=['3'],right=['4'],minimum=['.5'],maximum=['2']))
        self.assertEqual((state['width'], state['height']), (57,33))
        self.assertTrue(all(0 <= value <= 1 for value in state['image']))
        preview.frame = decode_packet(packet(source=1, values=[float('inf')]*(WIDTH*HEIGHT)))
        state = preview.process({'inpaint':['0']})
        self.assertEqual(set(state['image']), {1.0})


class CppPublisherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):
            raise unittest.SkipTest('C++ compiler unavailable')
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        source = Path(cls.temp.name)/'sender.cpp'
        source.write_text('''#include "g1_depth_preview.h"
int main() {
    g1_camera::DepthPreviewPublisher preview;
    auto raw = std::make_shared<std::vector<float>>(480*270);
    for (int i=0; i<480*270; ++i) (*raw)[i] = i*.001f;
    preview.Offer(raw, nullptr, 7, 1.25);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::array<float,64*36> small{};small.fill(1.75f);
    preview.Offer(nullptr, &small, 8, 2.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}
''')
        cls.binary = Path(cls.temp.name)/'sender'
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pthread',
                        '-I',str(Path(__file__).resolve().parents[1]/'common/include'),str(source),'-o',str(cls.binary)],check=True)

    def test_real_and_simulated_packets_decode_in_python(self):
        import os
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1',0))
            receiver.settimeout(3)
            env = dict(os.environ,G1_DEPTH_PREVIEW_PORT=str(receiver.getsockname()[1]))
            sender = subprocess.Popen([str(self.binary)],env=env)
            try:
                real = decode_packet(receiver.recv(65536))
                simulated = decode_packet(receiver.recv(65536))
                self.assertEqual((real['sequence'],real['source']), (7,'realsense'))
                self.assertEqual((simulated['sequence'],simulated['source']), (8,'simulation'))
                value = struct.unpack_from('<f',real['payload'],(WIDTH+1)*4)[0]
                self.assertAlmostEqual(value,(7*480+7)*.001,places=5)
                self.assertEqual(struct.unpack_from('<f',simulated['payload'])[0],1.75)
                self.assertEqual(real['session'],simulated['session'])
            finally:
                sender.wait(timeout=5)
            self.assertEqual(sender.returncode,0)


@unittest.skipUnless(np is not None, 'NumPy/OpenCV needed for preview HTTP tests')
class HttpPreviewTests(unittest.TestCase):
    def test_preview_http_does_not_change_controller_data(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = JointMonitor('127.0.0.1',0,record_dir=directory,preview_port=free_port())
            monitor.start()
            base = 'http://127.0.0.1:'+str(monitor.server.server_port)
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
                    sender.sendto(packet(),('127.0.0.1',monitor.depth_preview.port))
                wait_for(lambda:monitor.depth_preview.frame is not None)
                with urlopen(base+'/api/depth?crop=0&inpaint=0&filter=0&normalize=0') as response:
                    raw = json.load(response)
                self.assertEqual((raw['width'],raw['height']),(64,36))
                self.assertEqual(raw['units'],'m')
                with urlopen(base+'/api/depth?crop=1&inpaint=1&filter=1&normalize=1') as response:
                    cropped = json.load(response)
                self.assertEqual((cropped['width'],cropped['height']),(32,18))
                self.assertEqual(raw['sequence'],cropped['sequence'])
                self.assertEqual(monitor.depth_preview.frame['payload'],packet()[HEADER.size:])
                with self.assertRaises(HTTPError) as error:
                    urlopen(base+'/api/depth?kernel=2')
                self.assertEqual(error.exception.code,400)
                error.exception.close()
            finally:
                monitor.close()


if __name__ == '__main__':
    unittest.main()
