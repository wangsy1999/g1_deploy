"""Read-only RealSense RGB/depth web test and optional local parkour depth feed.

No DDS imports, robot commands, Torch or GUI dependencies. --mock is explicitly
synthetic and intended for HTTP/protocol checks without a connected camera.
"""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import socket
import struct
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np

HEADER = struct.Struct("<8sQdII")
ROOT = Path(__file__).resolve().parents[1]


def project_depth(depth_m, intrinsics, profile):
    """Sample optical-axis Z using the training pinhole rays, without stretching FOV.

    Zero/invalid returns and rays outside the measured FOV are far/no-hit. This
    resamples intrinsics only; it cannot correct a different physical camera pose.
    """
    w, h = profile["width"], profile["height"]
    if (w, h) != (64, 36):
        raise ValueError("Parkour expects a 64x36 depth image")
    if not all(math.isfinite(float(intrinsics[k])) for k in ("fx", "fy", "ppx", "ppy")):
        raise ValueError("Non-finite depth intrinsics")
    if intrinsics["fx"] <= 0 or intrinsics["fy"] <= 0:
        raise ValueError("Invalid depth focal length")
    horizontal, vertical = profile["horizontal_fov_deg"], profile["vertical_fov_deg"]
    if not (0 < horizontal < 180 and 0 < vertical < 180):
        raise ValueError("Invalid training camera field of view")
    u, v = np.meshgrid(np.arange(w) + .5, np.arange(h) + .5)
    x = np.rint((u-w/2)/(w/2)*np.tan(np.deg2rad(horizontal)/2)*intrinsics["fx"] + intrinsics["ppx"]).astype(int)
    y = np.rint((v-h/2)/(h/2)*np.tan(np.deg2rad(vertical)/2)*intrinsics["fy"] + intrinsics["ppy"]).astype(int)
    covered = (x >= 0) & (x < depth_m.shape[1]) & (y >= 0) & (y < depth_m.shape[0])
    image = np.full((h, w), np.inf, dtype=np.float32)
    samples = depth_m[y[covered], x[covered]]
    image[covered] = np.where(np.isfinite(samples) & (samples > 0), samples, np.inf)
    return image, float(covered.mean())


def depth_packet(image, sequence, captured_at):
    if image.shape != (36, 64) or np.isnan(image).any() or (image < 0).any():
        raise ValueError("Invalid parkour depth image")
    return HEADER.pack(b"G1DEP001", sequence, captured_at, 64, 36) + image.astype("<f4").tobytes()


class Frames:
    def __init__(self, token=""):
        self.changed = threading.Condition()
        self.sequence = 0
        self.color = self.depth = None
        self.status = {"ready": False, "error": None}
        self.stopped = False
        self.token = token

    def update(self, color, depth, status):
        with self.changed:
            self.color, self.depth = color, depth
            self.sequence += 1
            self.status = {**status, "ready": True, "error": None, "updated_at": time.monotonic()}
            self.changed.notify_all()

    def fail(self, error):
        with self.changed:
            self.status = {**self.status, "ready": False, "error": str(error)}
            self.changed.notify_all()


def jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.tobytes()


def make_server(frames, host, port):
    html = (Path(__file__).parent / "static/realsense.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            # Optional Basic auth for networks beyond the robot LAN. A query
            # parameter is deliberately not used, to keep tokens out of URLs.
            if frames.token:
                import base64
                wanted = "Basic " + base64.b64encode(("camera:"+frames.token).encode()).decode()
                if not secrets.compare_digest(self.headers.get("Authorization", ""), wanted):
                    self.send_response(401)
                    self.send_header("WWW-Authenticate", 'Basic realm="RealSense"')
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            path = urlsplit(self.path).path
            if path in ("/color.mjpg", "/depth.mjpg"):
                self.connection.settimeout(3)
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seen = -1
                try:
                    while not frames.stopped:
                        with frames.changed:
                            frames.changed.wait_for(lambda: (frames.sequence != seen and frames.status.get("ready")) or frames.stopped, timeout=1)
                            if frames.sequence == seen or not frames.status.get("ready"):
                                continue
                            seen = frames.sequence
                            data = frames.color if path == "/color.mjpg" else frames.depth
                        if data:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                             str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                return
            if path == "/api/status":
                with frames.changed:
                    status = dict(frames.status)
                    updated = status.pop("updated_at", None)
                    captured = status.get("capture_time_monotonic", updated)
                    status["age_ms"] = (None if captured is None else
                                        round((time.monotonic()-captured)*1000, 1))
                    status["sequence"] = frames.sequence
                data, mime = json.dumps(status, allow_nan=False).encode(), "application/json"
            elif path == "/":
                data, mime = html, "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer((host, port), Handler)


def capture(args, frames, stopping):
    pipeline = None
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.depth_port else None
    if sender:
        sender.setblocking(False)
    try:
        profile = json.loads(args.profile.read_text())
        if args.mock:
            info = {"source": "SYNTHETIC TEST", "serial": "mock", "fps": args.fps}
            intrinsics = dict(fx=args.width/2, fy=args.height/2, ppx=args.width/2, ppy=args.height/2)
        else:
            import pyrealsense2 as rs
            pipeline = rs.pipeline()
            config = rs.config()
            if args.serial:
                config.enable_device(args.serial)
            config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
            config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
            active = pipeline.start(config)
            device = active.get_device()
            depth_scale = device.first_depth_sensor().get_depth_scale()
            intr = active.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            intrinsics = {k: getattr(intr, k) for k in ("fx", "fy", "ppx", "ppy")}
            # Require rectified optical-axis depth. Do not align depth to color:
            # policy depth stays in the depth camera frame.
            info = {"source": "RealSense", "serial": device.get_info(rs.camera_info.serial_number),
                    "name": device.get_info(rs.camera_info.name), "fps": args.fps,
                    "depth_scale_m": depth_scale, "intrinsics": intrinsics,
                    "distortion_model": str(intr.model)}
            if any(abs(float(v)) > 1e-8 for v in intr.coeffs):
                raise RuntimeError("Depth stream has nonzero distortion; select a rectified depth profile before policy sampling")
        sequence, last_frame = 0, None
        sensor_origin = host_origin = previous_sensor_time = None
        while not stopping.is_set():
            before = time.monotonic()
            if args.mock:
                color = np.zeros((args.height, args.width, 3), np.uint8)
                cv2.putText(color, "SYNTHETIC TEST", (15, args.height//2), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
                depth = np.tile(np.linspace(.3, 2.5, args.width, dtype=np.float32), (args.height, 1))
                captured_at = time.monotonic()
            else:
                sample = pipeline.wait_for_frames(1000)
                df, cf = sample.get_depth_frame(), sample.get_color_frame()
                if not df or not cf or df.get_frame_number() == last_frame:
                    continue
                sensor_time = float(df.get_timestamp())*.001
                if not math.isfinite(sensor_time) or (previous_sensor_time is not None and sensor_time <= previous_sensor_time):
                    raise RuntimeError("RealSense capture clock restarted or stopped")
                previous_sensor_time = sensor_time
                if sensor_origin is None:
                    sensor_origin, host_origin = sensor_time, time.monotonic()
                captured_at = host_origin + sensor_time-sensor_origin
                last_frame = df.get_frame_number()
                # Keep ownership independent of SDK frame buffers.
                depth = np.asanyarray(df.get_data()).astype(np.float32)*depth_scale
                color = np.asanyarray(cf.get_data()).copy()
            policy_depth, coverage = project_depth(depth, intrinsics, profile)
            if sender:
                try:
                    sender.sendto(depth_packet(policy_depth, sequence, captured_at), ("127.0.0.1", args.depth_port))
                except BlockingIOError:
                    pass
            sequence += 1
            visual = cv2.applyColorMap((np.clip(depth, 0, 2.5)/2.5*255).astype(np.uint8), cv2.COLORMAP_TURBO)
            visual[depth <= 0] = 0
            frames.update(jpeg(color), jpeg(visual), {**info, "policy_fov_coverage": coverage,
                          "capture_time_monotonic": captured_at, "depth_udp_port": args.depth_port,
                          "valid_depth_fraction": float(np.mean(np.isfinite(depth) & (depth > 0))),
                          "camera_mount_verification": "operator confirmed; not measurable by this script"})
            stopping.wait(max(0, 1/args.fps-(time.monotonic()-before)))
    except Exception as error:
        frames.fail(error)
        print(f"Camera stopped: {error}", flush=True)
        stopping.set()
    finally:
        if pipeline:
            pipeline.stop()
        if sender:
            sender.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--serial", default="")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth-port", type=int, default=0, help="Optional loopback parkour feed; 0 disables")
    parser.add_argument("--profile", type=Path, default=ROOT/"config/realsense_parkour.json")
    parser.add_argument("--token", default="", help="Optional HTTP Basic password (user: camera)")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    if args.fps <= 0 or args.width <= 0 or args.height <= 0 or not 1 <= args.port <= 65535:
        parser.error("Invalid size, FPS or HTTP port")
    if args.depth_port and not 1024 <= args.depth_port <= 65535:
        parser.error("Depth UDP port must be 1024..65535")
    if args.mock and args.depth_port:
        parser.error("Synthetic images cannot be forwarded to a robot policy")
    frames, stopping = Frames(args.token), threading.Event()
    server = make_server(frames, args.host, args.port)
    worker = threading.Thread(target=capture, args=(args, frames, stopping), daemon=True)
    worker.start()
    print(f"RealSense read-only web test: http://{args.host}:{args.port}/ (no robot commands)", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        frames.stopped = True
        with frames.changed:
            frames.changed.notify_all()
        server.server_close()
        worker.join(timeout=3)


if __name__ == "__main__":
    main()
