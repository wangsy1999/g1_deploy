"""Read-only RealSense RGB/depth web test and optional local parkour depth feed.

No DDS imports, robot commands, Torch or GUI dependencies. --mock is explicitly
synthetic and intended for HTTP/protocol checks without a connected camera.
"""
import argparse
import fcntl
import mmap
import os
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
CAPTURE_FREQUENCY_CHECK_INTERVAL = 500


class FrameClock:
    """Map increasing camera timestamps into this host's monotonic clock.

    The lower envelope of receive offsets corrects forward clock drift without
    stamping delayed frames as newly captured. A late receive cannot increase
    the offset, so transport/processing stalls remain visible as frame age.
    """
    def __init__(self):
        self.offset = None
        self.last_sensor = None
        self.last_received = None

    def map(self, sensor_time, received_at):
        if not math.isfinite(sensor_time) or not math.isfinite(received_at):
            raise RuntimeError("Non-finite RealSense capture clock")
        if self.last_sensor is not None and sensor_time <= self.last_sensor:
            raise RuntimeError("RealSense capture clock restarted or stopped")
        if self.last_received is not None and received_at < self.last_received:
            raise RuntimeError("Host monotonic clock moved backwards")
        candidate = received_at - sensor_time
        self.offset = candidate if self.offset is None else min(self.offset, candidate)
        self.last_sensor, self.last_received = sensor_time, received_at
        return min(received_at, sensor_time + self.offset)


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


def instinct_depth(depth_m, profile):
    """Match ParkourAgent.refresh_depth_frame, using this model's configured parameters."""
    p = profile['preprocessing']
    if not np.isfinite(depth_m).all() or (depth_m < 0).any():
        raise ValueError('Invalid source depth')
    image = cv2.resize(depth_m, tuple(p['resize']), interpolation=cv2.INTER_NEAREST)
    top,bottom,left,right = p['crop']
    image = image[top:image.shape[0]-bottom, left:image.shape[1]-right].copy()
    image = cv2.inpaint(image, (image < p['inpaint_below_m']).astype(np.uint8), p['inpaint_radius'], cv2.INPAINT_NS)
    if 'blind_spot_crop' in p:
        top,bottom,left,right = p['blind_spot_crop']
        image[:top,:]=0;image[image.shape[0]-bottom:,:]=0
        image[:,:left]=0;image[:,image.shape[1]-right:]=0
    if p.get('gaussian_kernel'):
        k=p['gaussian_kernel'];sigma=p['gaussian_sigma']
        image = cv2.GaussianBlur(image, (k,k), sigma, sigma)
    lo,hi=p['depth_range'];out_lo,out_hi=p['output_range']
    image = (np.clip(image,lo,hi)-lo)/(hi-lo)*(out_hi-out_lo)+out_lo
    if image.shape != (18,32) or not np.isfinite(image).all():
        raise ValueError('Model requires a finite 18x32 depth image')
    return image.astype(np.float32)


def depth_packet(image, sequence, captured_at, normalized=False):
    if normalized:
        if image.shape != (18,32) or not np.isfinite(image).all() or (image<0).any() or (image>1).any():
            raise ValueError("Invalid normalized model depth")
        return HEADER.pack(b"G1DEP002", sequence, captured_at, 32, 18) + image.astype("<f4").tobytes()
    if image.shape != (36, 64) or np.isnan(image).any() or (image < 0).any():
        raise ValueError("Invalid parkour depth image")
    return HEADER.pack(b"G1DEP001", sequence, captured_at, 64, 36) + image.astype("<f4").tobytes()


class RawDepthFeed:
    """Latest raw depth in local shared memory; UDP carries only notifications."""
    def __init__(self, port, width, height):
        if (width, height) != (480, 270):
            raise ValueError("Raw policy feed requires 480x270 depth")
        self.path = Path(f"/dev/shm/g1-depth-{os.getuid()}-{port}")
        self.owner = os.fdopen(os.open(self.path.with_suffix('.owner'), os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600), 'r+b')
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.owner.close()
            raise RuntimeError(f"Another camera owns depth port {port}")
        self.file = os.fdopen(os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600), 'r+b')
        self.file.truncate(HEADER.size + width*height*4)
        self.memory = mmap.mmap(self.file.fileno(), 0)

    def publish(self, depth, sequence, captured_at):
        header = HEADER.pack(b"G1DEP003", sequence, captured_at, 480, 270)
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        try:
            self.memory[HEADER.size:] = depth.astype('<f4', copy=False).tobytes()
            self.memory[:HEADER.size] = header
        finally:
            fcntl.flock(self.file, fcntl.LOCK_UN)
        return header

    def close(self):
        self.memory.close()
        self.file.close()
        self.path.unlink(missing_ok=True)
        self.owner.close()


class Frames:
    def __init__(self, token=""):
        self.changed = threading.Condition()
        self.sequence = 0
        self.color = self.depth = self.comparison = None
        self.status = {"ready": False, "error": None}
        self.stopped = False
        self.token = token

    def update(self, color, depth, status, comparison=None):
        with self.changed:
            self.color, self.depth, self.comparison = color, depth, comparison
            self.sequence += 1
            self.status = {**self.status, **status}
            self.changed.notify_all()

    def captured(self, status):
        with self.changed:
            self.status = {**self.status, **status, "ready": True, "error": None,
                           "updated_at": time.monotonic()}

    def fail(self, error):
        with self.changed:
            self.status = {**self.status, "ready": False, "error": str(error)}
            self.changed.notify_all()


def jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.tobytes()


class PreviewWorker:
    """One replaceable pending frame; encoding never holds the capture thread."""
    def __init__(self, args, frames, render_depth):
        self.args, self.frames, self.render_depth = args, frames, render_depth
        self.changed = threading.Condition()
        self.pending = None
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self.run, name="camera-preview", daemon=True)
        self.worker.start()

    def submit(self, depth, color, depth_frame, captured_at):
        # Arrays are owned by capture and never mutated after submission. Keep
        # the SDK frame alive for legacy preview filtering in the worker.
        with self.changed:
            self.pending = (depth, color, depth_frame, captured_at)
            self.changed.notify()

    def close(self):
        self.closed.set()
        with self.changed:
            self.pending = None
            self.changed.notify_all()
        self.worker.join(timeout=2)

    def run(self):
        next_at = 0.
        while not self.closed.is_set():
            with self.changed:
                self.changed.wait_for(lambda: self.pending is not None or self.closed.is_set())
            if self.closed.wait(max(0., next_at-time.monotonic())):
                return
            with self.changed:
                if self.closed.is_set():
                    return
                depth, color, depth_frame, captured_at = self.pending
                self.pending = None
            next_at = time.monotonic() + 1/self.args.preview_fps
            try:
                processed_depth = self.render_depth(depth, depth_frame)
                processed_visual = cv2.applyColorMap(
                    (np.clip(processed_depth, 0, 2.5)/2.5*255).astype(np.uint8), cv2.COLORMAP_TURBO)
                processed_visual[processed_depth <= 0] = 0
                visual = cv2.applyColorMap(
                    (np.clip(depth, 0, 2.5)/2.5*255).astype(np.uint8), cv2.COLORMAP_TURBO)
                visual[depth <= 0] = 0
                self.frames.update(jpeg(color), jpeg(visual), {
                    "preview_capture_time_monotonic": captured_at,
                    "preview_error": None,
                    "valid_depth_fraction": float(np.mean(np.isfinite(depth) & (depth > 0))),
                    "processed_valid_depth_fraction": float(np.mean(np.isfinite(processed_depth) & (processed_depth > 0)))},
                    comparison=jpeg(np.concatenate((visual, processed_visual), axis=1)))
            except Exception as error:
                # Preview failures must not stop the independent policy feed.
                with self.frames.changed:
                    self.frames.status["preview_error"] = str(error)


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
            if path in ("/color.mjpg", "/depth.mjpg", "/depth_compare.mjpg"):
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
                            data = {"/color.mjpg": frames.color, "/depth.mjpg": frames.depth,
                                    "/depth_compare.mjpg": frames.comparison}[path]
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
                    preview_at = status.get("preview_capture_time_monotonic")
                    status["preview_age_ms"] = (None if preview_at is None else
                                                round((time.monotonic()-preview_at)*1000, 1))
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
    pipeline_started = False
    preview = None
    raw_feed = None
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
            if args.color:
                config.enable_stream(rs.stream.color, args.color_width, args.color_height, rs.format.bgr8, args.color_fps)
            active = pipeline.start(config)
            pipeline_started = True
            # Filters persist across frames so the temporal filter retains history.
            to_disparity = rs.disparity_transform(True)
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            spatial.set_option(rs.option.filter_smooth_delta, 20)
            temporal = rs.temporal_filter()
            temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
            temporal.set_option(rs.option.filter_smooth_delta, 20)
            to_depth = rs.disparity_transform(False)
            fill = rs.hole_filling_filter(1)
            device = active.get_device()
            depth_scale = device.first_depth_sensor().get_depth_scale()
            intr = active.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            intrinsics = {k: getattr(intr, k) for k in ("fx", "fy", "ppx", "ppy")}
            # Require rectified optical-axis depth. Do not align depth to color:
            # policy depth stays in the depth camera frame.
            info = {"source": "RealSense", "serial": device.get_info(rs.camera_info.serial_number),
                    "name": device.get_info(rs.camera_info.name), "fps": args.fps,
                    "depth_width": args.width, "depth_height": args.height,
                    "color_width": args.color_width, "color_height": args.color_height,
                    "color_fps": args.color_fps if args.color else 0, "color_enabled": args.color,
                    "camera_mode": args.camera_mode, "depth_protocol": "G1DEP003" if args.camera_mode=="instinct" else "G1DEP001",
                    "depth_scale_m": depth_scale, "intrinsics": intrinsics,
                    "distortion_model": str(intr.model)}
            if any(abs(float(v)) > 1e-8 for v in intr.coeffs):
                raise RuntimeError("Depth stream has nonzero distortion; select a rectified depth profile before policy sampling")
        disabled_color=np.zeros((240,424,3),np.uint8)
        cv2.putText(disabled_color,"RGB disabled (depth only)",(12,120),cv2.FONT_HERSHEY_SIMPLEX,.6,(180,180,180),1)
        def render_depth(depth, depth_frame):
            if args.camera_mode == "instinct":
                return cv2.resize(instinct_depth(depth, profile)*2.5, (args.width, args.height), interpolation=cv2.INTER_NEAREST)
            if args.mock:
                return depth
            filtered = to_disparity.process(depth_frame)
            filtered = spatial.process(filtered)
            filtered = temporal.process(filtered)
            filtered = to_depth.process(filtered)
            filtered = fill.process(filtered)
            return np.asanyarray(filtered.get_data()).astype(np.float32)*depth_scale

        if sender and args.camera_mode == "instinct":
            raw_feed = RawDepthFeed(args.depth_port, args.width, args.height)
        preview = PreviewWorker(args, frames, render_depth)
        sequence, last_frame = 0, None
        frame_clock = FrameClock()
        frequency_started_at = None
        frequency_count = 0
        capture_fps = None
        while not stopping.is_set():
            before = time.monotonic()
            if args.mock:
                color = np.zeros((args.height, args.width, 3), np.uint8)
                cv2.putText(color, "SYNTHETIC TEST", (15, args.height//2), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
                depth = np.tile(np.linspace(.3, 2.5, args.width, dtype=np.float32), (args.height, 1))
                captured_at = time.monotonic()
            else:
                sample = pipeline.wait_for_frames(1000)
                received_at = time.monotonic()
                df, cf = sample.get_depth_frame(), sample.get_color_frame()
                if not df or (args.color and not cf) or df.get_frame_number() == last_frame:
                    continue
                sensor_time = float(df.get_timestamp())*.001
                captured_at = frame_clock.map(sensor_time, received_at)
                last_frame = df.get_frame_number()
                # Keep ownership independent of SDK frame buffers.
                depth = np.asanyarray(df.get_data()).astype(np.float32)*depth_scale
                color = np.asanyarray(cf.get_data()).copy() if args.color else disabled_color
            # Measure delivered frames with the host clock, excluding startup.
            if frequency_started_at is None:
                frequency_started_at = time.monotonic()
            else:
                frequency_count += 1
                if frequency_count == CAPTURE_FREQUENCY_CHECK_INTERVAL:
                    measured_at = time.monotonic()
                    capture_fps = frequency_count / (measured_at-frequency_started_at)
                    print(f"Camera capture running at {capture_fps:.4f} Hz.", flush=True)
                    frequency_count = 0
                    frequency_started_at = measured_at
            coverage = None
            if sender:
                if raw_feed is not None:
                    packet = raw_feed.publish(depth, sequence, captured_at)
                else:
                    policy_depth, coverage = project_depth(depth, intrinsics, profile)
                    packet = depth_packet(policy_depth, sequence, captured_at)
                if packet is not None:
                    try:
                        sender.sendto(packet, ("127.0.0.1", args.depth_port))
                    except BlockingIOError:
                        pass
            sequence += 1
            frames.captured({**info, "policy_fov_coverage": coverage,
                             "capture_time_monotonic": captured_at,
                             "depth_udp_port": args.depth_port,
                             "capture_fps": capture_fps,
                             "preview_fps": args.preview_fps,
                          "postprocessing_preview": ("Instinct model input: nearest resize/crop/inpaint/" + ("Gaussian/" if profile["preprocessing"].get("gaussian_kernel") else "") + "normalize") if args.camera_mode=="instinct" else "disparity → spatial → temporal → depth → hole filling" if not args.mock else "synthetic; filters bypassed",
                          "policy_depth_source": "raw 480x270 shared memory; preprocessing on policy side" if args.camera_mode=="instinct" else "legacy raw depth",
                          "camera_mount_verification": "operator confirmed; not measurable by this script"})
            preview.submit(depth, color, df if not args.mock else None, captured_at)
            # RealSense arrival drives capture, as in instinct_onboard. Only
            # synthetic frames need software pacing.
            if args.mock:
                stopping.wait(max(0, 1/args.fps-(time.monotonic()-before)))
    except Exception as error:
        frames.fail(error)
        print(f"Camera stopped: {error}", flush=True)
        stopping.set()
    finally:
        if preview is not None:
            preview.close()
        if raw_feed is not None:
            raw_feed.close()
        if pipeline_started:
            pipeline.stop()
        if sender:
            sender.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--serial", default="")
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--height", type=int, default=270)
    parser.add_argument("--fps", type=int, default=60, help="Depth stream FPS")
    parser.add_argument("--preview-fps", type=int, default=15, help="Maximum web preview FPS; independent of depth feed")
    parser.add_argument("--camera-mode", choices=("instinct","legacy"), default="instinct")
    parser.add_argument("--color", action="store_true", help="Optional RGB preview; disabled for depth-only reference setup")
    parser.add_argument("--color-width", type=int, default=424)
    parser.add_argument("--color-height", type=int, default=240)
    parser.add_argument("--color-fps", type=int, default=30)
    parser.add_argument("--depth-port", type=int, default=0, help="Optional loopback parkour feed; 0 disables")
    parser.add_argument("--profile", type=Path, default=ROOT/"config/realsense_parkour.json")
    parser.add_argument("--token", default="", help="Optional HTTP Basic password (user: camera)")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    if min(args.preview_fps, args.fps, args.width, args.height, args.color_fps, args.color_width, args.color_height) <= 0 or not 1 <= args.port <= 65535:
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
