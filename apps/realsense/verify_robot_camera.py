"""Read-only RealSense capture and UDP verification; never launches a controller."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import urlopen


def probe(port, depth_port):
    url = 'http://127.0.0.1:{}/api/status'.format(port)
    with urlopen(url, timeout=3) as response:
        first = json.load(response)
    assert first.get('ready') and first.get('source') == 'RealSense', first
    assert first.get('depth_protocol') == 'G1DEP003', first
    assert (first.get('depth_width'), first.get('depth_height'), first.get('fps')) == (480, 270, 60), first
    assert first.get('depth_udp_port') == depth_port, first
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', depth_port))
        receiver.settimeout(3)
        packet, _ = receiver.recvfrom(1024)
        assert len(packet) == 32 and packet.startswith(b'G1DEP003'), packet
    time.sleep(1)
    with urlopen(url, timeout=3) as response:
        latest = json.load(response)
    assert latest['sequence'] > first['sequence'], latest
    assert latest.get('age_ms') is not None and latest['age_ms'] < 500, latest
    with urlopen('http://127.0.0.1:{}/'.format(port), timeout=3) as response:
        assert len(response.read()) > 1000
    print('Camera web page, fresh 480x270 depth frames and G1DEP003 UDP: OK', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--web-port', type=int, default=18766)
    parser.add_argument('--depth-port', type=int, default=17291)
    parser.add_argument('--probe', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.probe:
        probe(args.web_port, args.depth_port)
        return 0
    command = [sys.executable, str(args.root / 'apps/parkour/parkour_camera.py'),
               '--root', str(args.root), '--web-port', str(args.web_port),
               '--depth-port', str(args.depth_port), '--',
               sys.executable, str(Path(__file__).resolve()), '--probe',
               '--web-port', str(args.web_port), '--depth-port', str(args.depth_port)]
    return subprocess.call(command)


if __name__ == '__main__':
    sys.exit(main())
