"""Run the native C++ capture check; no DDS or robot commands."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    binary = args.root / ('build' if (args.root / 'CMakeLists.txt').is_file() else 'bin') / 'g1_camera_check'
    if not binary.is_file():
        parser.error(f'Missing native camera check: {binary}; build the project first')
    return subprocess.call([str(binary)])


if __name__ == '__main__':
    sys.exit(main())
