import importlib.util
import json
from pathlib import Path
import threading
import unittest
from urllib.request import urlopen

import numpy as np

MODULE = Path(__file__).resolve().parents[1]/"realsense_web.py"
spec = importlib.util.spec_from_file_location("realsense_web", MODULE)
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


class DepthTests(unittest.TestCase):
    def test_metric_depth_invalid_returns_and_packet(self):
        profile = dict(width=64,height=36,horizontal_fov_deg=90,vertical_fov_deg=90)
        depth = np.full((36,64),1.75,np.float32)
        depth[20,40] = 0
        # Match pixel centers so resampling is an identity transform.
        image, coverage = camera.project_depth(depth,dict(fx=32,fy=18,ppx=31.5,ppy=17.5),profile)
        self.assertEqual(coverage,1)
        self.assertTrue(np.isinf(image[20,40]))
        self.assertEqual(image[2,3],1.75)
        packet = camera.depth_packet(image,7,12.5)
        self.assertEqual(len(packet),9248)
        self.assertEqual(camera.HEADER.unpack_from(packet),(b"G1DEP001",7,12.5,64,36))
        recovered = np.frombuffer(packet, dtype="<f4", offset=32).reshape(36,64)
        np.testing.assert_array_equal(recovered,image)

    def test_uncovered_fov_is_far_without_stretching(self):
        profile = dict(width=64,height=36,horizontal_fov_deg=120,vertical_fov_deg=100)
        image, coverage = camera.project_depth(np.ones((36,64),np.float32),dict(fx=50,fy=50,ppx=32,ppy=18),profile)
        self.assertLess(coverage,.5)
        self.assertTrue(np.isinf(image[0,0]))
        self.assertEqual(image[18,32],1)


class HttpTests(unittest.TestCase):
    def test_readonly_status_page_and_mjpeg(self):
        frames = camera.Frames()
        frames.update(camera.jpeg(np.zeros((12,16,3),np.uint8)),camera.jpeg(np.zeros((12,16,3),np.uint8)),
                      dict(source="SYNTHETIC TEST"))
        server = camera.make_server(frames,"127.0.0.1",0)
        worker = threading.Thread(target=server.serve_forever,daemon=True)
        worker.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            with urlopen(base+"/",timeout=2) as response:
                self.assertIn(b"/color.mjpg",response.read())
            with urlopen(base+"/api/status",timeout=2) as response:
                status = json.load(response)
                self.assertTrue(status["ready"])
                self.assertEqual(status["source"],"SYNTHETIC TEST")
                self.assertLess(status["age_ms"],2000)
            with urlopen(base+"/depth.mjpg",timeout=2) as response:
                data = response.read(100)
                self.assertIn(b"Content-Type: image/jpeg",data)
                self.assertIn(b"\xff\xd8",data)
            frames.fail("camera unplugged")
            with urlopen(base+"/api/status",timeout=2) as response:
                status=json.load(response)
                self.assertFalse(status["ready"])
                self.assertIn("unplugged",status["error"])
        finally:
            frames.stopped=True
            with frames.changed:
                frames.changed.notify_all()
            server.shutdown();server.server_close();worker.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
