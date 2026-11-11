#!/usr/bin/env bash
# Configure an already-copied ARM64 robot checkout. Never transfers project files.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cache="$root/.deploy-cache/robot-arm64"
jobs=4
deps_only=0
verify_camera=0
install_system_deps=0
usage() {
    cat <<'HELP'
Usage: bash scripts/setup_robot.sh [options]
  --cache DIR             Offline dependency cache (default: .deploy-cache/robot-arm64)
  --jobs N                Parallel build jobs (default: 4)
  --deps-only             Prepare and verify dependencies without building controllers
  --verify-camera         Test real depth capture and UDP; no robot controller is started
  --install-system-deps   Install missing Ubuntu build packages using sudo apt-get
  --help                  Show this help
Copy the project and offline cache yourself before running. No SSH, SCP, rsync,
robot motion, time changes, system glibc replacement, or auto-start is performed.
HELP
}
while (($#)); do
    case "$1" in
        --cache) [[ $# -ge 2 ]] || { usage; exit 2; }; cache="$2"; shift 2 ;;
        --jobs) [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || { usage; exit 2; }; jobs="$2"; shift 2 ;;
        --deps-only) deps_only=1; shift ;;
        --verify-camera) verify_camera=1; shift ;;
        --install-system-deps) install_system_deps=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
done
[[ "$(uname -m)" == aarch64 ]] || { echo 'This cache supports Linux ARM64/aarch64 only.' >&2; exit 1; }
[[ -f "$cache/SHA256SUMS" ]] || { echo "Missing offline cache: $cache. Copy .deploy-cache/robot-arm64 with the project." >&2; exit 1; }
cache="$(cd -- "$cache" && pwd)"
(cd "$cache" && sha256sum -c SHA256SUMS)
if ((install_system_deps)); then
    required=(build-essential cmake libboost-dev libyaml-cpp-dev libopencv-dev libusb-1.0-0 pkg-config)
    missing=()
    for package in "${required[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then missing+=("$package"); fi
    done
    if ((${#missing[@]})); then sudo apt-get update; sudo apt-get install -y "${missing[@]}"; fi
fi
for tool in python3 gcc g++ cmake make tar dpkg-deb sha256sum; do
    command -v "$tool" >/dev/null || { echo "Missing $tool. Install build tools or rerun with --install-system-deps." >&2; exit 1; }
done
# Do not replace a controller binary while an operator is using it.
python3 - "$root" <<'PY'
import os, pathlib, sys
root = pathlib.Path(sys.argv[1]).resolve()
for process in pathlib.Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        executable = pathlib.Path(os.readlink(str(process / 'exe')).removesuffix(' (deleted)') if hasattr(str, 'removesuffix') else os.readlink(str(process / 'exe')).replace(' (deleted)', ''))
    except (OSError, PermissionError):
        continue
    if executable.name in ('g1_dwaq', 'g1_parkour') and root in executable.parents:
        raise SystemExit('Controller is running (PID {}). Exit it before setup.'.format(process.name))
PY
deps="$root/.deps"
mkdir -p "$deps/setup-logs"
extract() {
    local archive="$1" destination="$2" sentinel="$3"
    if [[ ! -e "$sentinel" ]]; then
        mkdir -p "$destination"
        tar --warning=no-timestamp -xzf "$cache/$archive" -C "$destination"
    fi
}
extract cpython-arm64.tar.gz "$deps/runtime" "$deps/runtime/python/bin/python3"
extract unitree-sdk2-arm64.tar.gz "$deps/unitree-sdk2" "$deps/unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
extract cyclonedds-0.10.2-arm64.tar.gz "$deps/cyclonedds" "$deps/cyclonedds/include/idl/retcode.h"
extract onnxruntime-1.18.1-arm64.tar.gz "$deps/onnxruntime" "$deps/onnxruntime/lib/libonnxruntime.so"
if [[ ! -e "$deps/glibc-2.35/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1" ]]; then
    mkdir -p "$deps/glibc-2.35"
    dpkg-deb -x "$cache/libc6_2.35-0ubuntu3.15_arm64.deb" "$deps/glibc-2.35"
fi
if [[ ! -x "$deps/python/bin/python3" ]]; then "$deps/runtime/python/bin/python3" -m venv "$deps/python"; fi
python="$deps/python/bin/python3"
"$python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Expected project Python 3.10"'
if [[ ! -e "$deps/python/lib/libpython3.10.so.1.0" ]]; then
    ln -s ../../runtime/python/lib/libpython3.10.so.1.0 "$deps/python/lib/libpython3.10.so.1.0"
fi
cp "$root/apps/realsense/python3-realsense.sh" "$deps/python/bin/python3-realsense"
chmod +x "$deps/python/bin/python3-realsense"
"$python" -m pip install --no-index --find-links="$cache" \
    'pip==25.0.1' 'setuptools==75.9.1' 'wheel==0.48.0' \
    'numpy==1.26.4' 'rich-click==1.9.9' 'opencv-python-headless==4.11.0.86' 'pyrealsense2==2.55.1.6486'
if ! "$python" -c 'import importlib.metadata as m; assert m.version("cyclonedds")=="0.10.2"' 2>/dev/null; then
    CFLAGS="-I$deps/runtime/python/include/python3.10 ${CFLAGS:-}" \
    CYCLONEDDS_HOME="$deps/cyclonedds" \
        "$python" -m pip install --no-index --find-links="$cache" --no-build-isolation "$cache/cyclonedds-0.10.2.tar.gz" \
        > "$deps/setup-logs/cyclonedds.log" 2>&1 || { tail -n 45 "$deps/setup-logs/cyclonedds.log"; exit 1; }
fi
mkdir -p "$deps/sdk-python"
tar --warning=no-timestamp -xzf "$cache/unitree_sdk2_python.tar.gz" -C "$deps/sdk-python" --strip-components=1
# Camera OpenCV is supplied by the headless package; the SDK monitor uses DDS/NumPy only.
"$python" -m pip install --no-index --no-build-isolation --no-deps "$deps/sdk-python"
LD_LIBRARY_PATH="$deps/cyclonedds/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$python" -c \
    'from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_,IMUState_; print("Monitor dependencies OK")'
"$deps/python/bin/python3-realsense" -c 'import cv2,pyrealsense2; print("Camera dependencies and private glibc OK")'
"$python" -m pip freeze > "$deps/setup-logs/python-packages.txt"
if ((!deps_only)); then
    cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release \
        -DONNXRUNTIME_ROOT="$deps/onnxruntime" \
        -Dunitree_sdk2_DIR="$deps/unitree-sdk2/lib/cmake/unitree_sdk2" \
        -DCMAKE_INSTALL_PREFIX="$root/install"
    cmake --build "$root/build" -j"$jobs"
    (cd "$root/build" && ctest --output-on-failure)
    if [[ -d "$root/install" ]]; then
        mkdir -p "$root/deployment-backups"
        tar -czf "$root/deployment-backups/install-$(date +%Y%m%d-%H%M%S)-$$.tar.gz" -C "$root" install
    fi
    cmake --install "$root/build"
    # Launchers copied from Windows must remain usable even without Git checkout conversion.
    "$python" - "$root/install" <<'PY'
from pathlib import Path
import sys
for name in ('g1', 'monitor', 'realsense', 'parkour'):
    path = Path(sys.argv[1]) / name
    path.write_bytes(path.read_bytes().replace(b'\r\n', b'\n'))
    path.chmod(path.stat().st_mode | 0o111)
PY
    (cd "$root/install" && ./g1 check && ./g1 parkour check)
fi
if ((verify_camera)); then
    "$python" "$root/apps/realsense/verify_robot_camera.py" --root "$root"
fi
printf '\nSetup complete. No controller was started.\n'
printf 'Run: cd "%s/install"\n' "$root"
printf './g1 monitor eth0 0.0.0.0 8765\n./g1 parkour real eth0\n'
