"""Own a preview camera only when needed; never enable robot output."""
import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import build_opener, ProxyHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--depth-port', type=int, default=17201)
    parser.add_argument('--web-port', type=int, default=8766)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not all(1024 <= p <= 65535 for p in (args.depth_port, args.web_port)):
        parser.error('A controller command and ports in 1024..65535 are required')
    opener = build_opener(ProxyHandler({}))
    url = f'http://127.0.0.1:{args.web_port}/api/status'
    camera = controller = log = None
    stopping = False

    def stop_requested(*_):
        nonlocal stopping
        stopping = True
        if controller is not None and controller.poll() is None:
            controller.send_signal(signal.SIGINT)

    def status():
        try:
            with opener.open(url, timeout=1) as response:
                return json.load(response)
        except HTTPError as error:
            raise RuntimeError(f'Camera status returned HTTP {error.code}; cannot reuse service') from error
        except URLError as error:
            if isinstance(error.reason, ConnectionRefusedError):
                return None
            raise RuntimeError(f'Cannot check camera service: {error}') from error
        except (ValueError, TimeoutError) as error:
            raise RuntimeError(f'Invalid or unresponsive service on port {args.web_port}') from error

    def check(value):
        if value.get('error'):
            raise RuntimeError(f'Camera error: {value["error"]}')
        if value.get('ready'):
            if value.get('source') != 'RealSense' or value.get('depth_udp_port') != args.depth_port:
                raise RuntimeError(
                    f'Existing camera does not forward real depth to UDP {args.depth_port}. '
                    f'Stop it, then rerun Parkour, or restart it with --depth-port {args.depth_port}.')
            if value.get('camera_mode')!='instinct' or value.get('depth_protocol')!='G1DEP003' or value.get('fps')!=60 or (value.get('depth_width'),value.get('depth_height'))!=(480,270) or value.get('color_enabled'):
                raise RuntimeError('Existing camera is not the Instinct 480x270/60 depth-only setup; stop it and rerun Parkour')
            return True
        return False

    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)
    try:
        initial = status()
        if initial is None:
            launcher = args.root / 'realsense'
            if not launcher.is_file():
                launcher = args.root / 'scripts/realsense.sh'
            logs = args.root / 'logs'
            logs.mkdir(exist_ok=True)
            log_path = logs / 'parkour-camera.log'
            log = log_path.open('ab', buffering=0)
            camera = subprocess.Popen(
                ['bash', str(launcher), '--host', '0.0.0.0', '--port', str(args.web_port),
                 '--depth-port', str(args.depth_port)],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            print(f'Starting camera + UDP {args.depth_port}; log: {log_path}', flush=True)
        else:
            check(initial)
            print(f'Checking existing camera on port {args.web_port}; it will be left running.', flush=True)
        deadline = time.monotonic() + 20
        previous_sequence = None
        while not stopping:
            if camera is not None and camera.poll() is not None:
                raise RuntimeError(f'Camera exited; see {log_path}')
            value = status()
            if value is not None and check(value):
                sequence = value.get('sequence')
                if sequence is not None and previous_sequence is not None and sequence > previous_sequence:
                    break
                previous_sequence = sequence
            if time.monotonic() >= deadline:
                raise RuntimeError('Camera did not produce fresh frames within 20 seconds; controller was not started')
            time.sleep(.1)
        if stopping:
            return 130
        print('Camera ready. Starting Parkour in its normal dry-run mode.', flush=True)
        controller = subprocess.Popen(command)
        while controller.poll() is None:
            if camera is not None and camera.poll() is not None:
                raise RuntimeError('Camera exited while Parkour was running')
            time.sleep(.1)
        return controller.returncode
    except (RuntimeError, OSError) as error:
        print(f'Parkour camera startup: {error}', file=sys.stderr, flush=True)
        return 1
    finally:
        # Never stop a camera service that this invocation did not start.
        for proc in (controller, camera):
            if proc is not None and proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    continue
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print(f'Process {proc.pid} has not exited; check it manually.', file=sys.stderr)
        if log is not None:
            log.close()


if __name__ == '__main__':
    sys.exit(main())
