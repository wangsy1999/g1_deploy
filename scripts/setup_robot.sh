#!/usr/bin/env bash
# Configure an already-copied ARM64 robot checkout. Never transfers project files.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deps="${G1_DEPS_DIR:-$HOME/.local/share/g1-deploy/$(uname -m)}"
cache="${G1_CACHE_DIR:-$HOME/.cache/g1-deploy/robot-arm64}"
jobs=4
deps_only=0
verify_camera=0
install_system_deps=1
offline=0
usage() {
    cat <<'HELP'
Usage: bash scripts/setup_robot.sh [options]
  --offline               Disable dependency downloads and apt-get
  --cache DIR             Dependency download/offline cache (default: ~/.cache/g1-deploy/robot-arm64)
  --jobs N                Parallel build jobs (default: 4)
  --deps-only             Prepare and verify dependencies without building controllers
  --verify-camera         Test native C++ depth capture; no robot controller is started
  --install-system-deps   Install missing Ubuntu build packages (default)
  --no-system-deps        Check dependencies without invoking apt-get
  --help                  Show this help
Missing dependencies are downloaded automatically; --offline requires installed
dependencies or an offline cache. Dependencies persist outside the checkout in
~/.local/share/g1-deploy/aarch64 (override with G1_DEPS_DIR).
Copy the project yourself before running. No SSH, SCP, rsync,
robot motion, time changes, system glibc replacement, or auto-start is performed.
HELP
}
while (($#)); do
    case "$1" in
        --offline) offline=1; shift ;;
        --cache) [[ $# -ge 2 ]] || { usage; exit 2; }; cache="$2"; shift 2 ;;
        --jobs) [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || { usage; exit 2; }; jobs="$2"; shift 2 ;;
        --deps-only) deps_only=1; shift ;;
        --verify-camera) verify_camera=1; shift ;;
        --install-system-deps) install_system_deps=1; shift ;;
        --no-system-deps) install_system_deps=0; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
done
[[ "$(uname -m)" == aarch64 ]] || { echo 'This cache supports Linux ARM64/aarch64 only.' >&2; exit 1; }
if ((offline)); then install_system_deps=0; fi
source "$root/scripts/robot_online_deps.sh"
# Dependency installations survive deletion of the entire checkout.
cache_verified=0
require_cache() {
    if ((cache_verified)); then return; fi
    [[ -f "$cache/SHA256SUMS" ]] || {
        echo "Missing offline cache: $cache. Copy it to prepare missing dependencies; installed dependencies are retained outside the checkout." >&2
        exit 1
    }
    cache="$(cd -- "$cache" && pwd)"
    (cd "$cache" && sha256sum -c SHA256SUMS)
    cache_verified=1
}
if ((install_system_deps)); then
    required=(python3 curl git ca-certificates build-essential cmake libboost-dev libyaml-cpp-dev libopencv-dev libusb-1.0-0-dev libssl-dev pkg-config)
    missing=()
    for package in "${required[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then missing+=("$package"); fi
    done
    if ((${#missing[@]})); then
        echo "Installing missing system packages: ${missing[*]}"
        privilege=()
        if ((EUID != 0)); then privilege=(sudo); fi
        "${privilege[@]}" apt-get update
        "${privilege[@]}" apt-get install -y "${missing[@]}"
    else
        echo 'System build packages already installed; skipping apt-get.'
    fi
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
    if executable.name in ('g1_dwaq', 'g1_parkour', 'g1_hybrid') and root in executable.parents:
        raise SystemExit('Controller is running (PID {}). Exit it before setup.'.format(process.name))
PY
mkdir -p "$deps/setup-logs" "$cache"
deps="$(cd -- "$deps" && pwd)"
cache="$(cd -- "$cache" && pwd)"
extract() {
    local archive="$1" destination="$2" sentinel="$3"
    if [[ ! -e "$sentinel" ]]; then
        if [[ -f "$cache/$archive" && -f "$cache/SHA256SUMS" ]]; then
            require_cache
            mkdir -p "$destination"
            tar --warning=no-timestamp -xzf "$cache/$archive" -C "$destination"
        elif ((offline)); then
            echo "Missing offline dependency: $cache/$archive" >&2; exit 1
        else
            online_native "$archive" "$destination"
        fi
    fi
}
extract cpython-arm64.tar.gz "$deps/runtime" "$deps/runtime/python/bin/python3"
extract unitree-sdk2-arm64.tar.gz "$deps/unitree-sdk2" "$deps/unitree-sdk2/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
extract cyclonedds-0.10.2-arm64.tar.gz "$deps/cyclonedds" "$deps/cyclonedds/include/idl/retcode.h"
extract onnxruntime-1.18.1-arm64.tar.gz "$deps/onnxruntime" "$deps/onnxruntime/lib/libonnxruntime.so"
if [[ ! -e "$deps/glibc-2.35/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1" ]]; then
    if [[ -f "$cache/libc6_2.35-0ubuntu3.15_arm64.deb" && -f "$cache/SHA256SUMS" ]]; then
        require_cache
        mkdir -p "$deps/glibc-2.35"
        dpkg-deb -x "$cache/libc6_2.35-0ubuntu3.15_arm64.deb" "$deps/glibc-2.35"
    elif ((offline)); then
        echo 'Missing private glibc in offline cache.' >&2; exit 1
    else
        online_glibc
    fi
fi
if [[ ! -x "$deps/python/bin/python3" ]]; then "$deps/runtime/python/bin/python3" -m venv "$deps/python"; fi
python="$deps/python/bin/python3"
"$python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Expected project Python 3.10"'
if [[ ! -e "$deps/python/lib/libpython3.10.so.1.0" ]]; then
    ln -s ../../runtime/python/lib/libpython3.10.so.1.0 "$deps/python/lib/libpython3.10.so.1.0"
fi
cp "$root/apps/realsense/python3-realsense.sh" "$deps/python/bin/python3-realsense"
chmod +x "$deps/python/bin/python3-realsense"
python_packages=('pip==25.0.1' 'setuptools==75.9.1' 'wheel==0.48.0'
    'numpy==1.26.4' 'rich-click==1.9.9' 'opencv-python-headless==4.11.0.86' 'pyrealsense2==2.55.1.6486')
if ! "$python" - "${python_packages[@]}" <<'PYTHON'
import importlib.metadata as metadata
import sys
for requirement in sys.argv[1:]:
    name, version = requirement.split('==')
    try:
        if metadata.version(name) != version:
            sys.exit(1)
    except metadata.PackageNotFoundError:
        sys.exit(1)
PYTHON
then
    if ((offline)); then
        require_cache
        "$deps/python/bin/python3-realsense" -m pip install --no-index --find-links="$cache" "${python_packages[@]}"
    else
        # The camera wheel needs glibc 2.35 even when the robot runs Ubuntu 20.04.
        "$deps/python/bin/python3-realsense" -m pip install "${python_packages[@]}"
    fi
else
    echo 'Pinned Python packages already installed; skipping pip install.'
fi
if ! "$python" -c 'import importlib.metadata as m; assert m.version("cyclonedds")=="0.10.2"' 2>/dev/null; then
    cyclone_package='cyclonedds==0.10.2'
    pip_source=()
    if ((offline)); then
        require_cache
        pip_source=(--no-index --find-links="$cache")
        cyclone_package="$cache/cyclonedds-0.10.2.tar.gz"
    fi
    CFLAGS="-I$deps/runtime/python/include/python3.10 ${CFLAGS:-}" \
    CYCLONEDDS_HOME="$deps/cyclonedds" \
        "$python" -m pip install "${pip_source[@]}" --no-build-isolation "$cyclone_package" \
        > "$deps/setup-logs/cyclonedds.log" 2>&1 || { tail -n 45 "$deps/setup-logs/cyclonedds.log"; exit 1; }
fi
if ! "$python" -c 'import importlib.metadata as m; m.distribution("unitree-sdk2py")' 2>/dev/null; then
    if [[ -f "$cache/unitree_sdk2_python.tar.gz" && -f "$cache/SHA256SUMS" ]]; then
        require_cache
        mkdir -p "$deps/sdk-python"
        tar --warning=no-timestamp -xzf "$cache/unitree_sdk2_python.tar.gz" -C "$deps/sdk-python" --strip-components=1
    elif ((offline)); then
        echo 'Missing Unitree Python SDK in offline cache.' >&2; exit 1
    else
        source_tree sdk-python 'https://github.com/unitreerobotics/unitree_sdk2_python/archive/refs/heads/master.tar.gz'
        mkdir -p "$deps/sdk-python"
        cp -a "$deps/sources/sdk-python/." "$deps/sdk-python/"
    fi
    # Camera OpenCV is supplied by the headless package; the monitor uses DDS/NumPy only.
    "$python" -m pip install --no-index --no-build-isolation --no-deps "$deps/sdk-python"
else
    echo 'Unitree Python SDK already installed; skipping installation.'
fi
LD_LIBRARY_PATH="$deps/cyclonedds/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$python" -c \
    'from unitree_sdk2py.core.channel import ChannelSubscriber; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_,IMUState_; print("Monitor dependencies OK")'
"$deps/python/bin/python3-realsense" -c 'import cv2,pyrealsense2; print("Camera dependencies and private glibc OK")'
"$python" -m pip freeze > "$deps/setup-logs/python-packages.txt"
ensure_native_camera
if ((!deps_only)); then
    cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release \
        -DG1_REQUIRE_REALSENSE=ON \
        -DG1_DEPS_DIR="$deps" \
        -DCMAKE_PREFIX_PATH="$deps/realsense" \
        -DONNXRUNTIME_ROOT="$deps/onnxruntime" \
        -Dunitree_sdk2_DIR="$deps/unitree-sdk2/lib/cmake/unitree_sdk2" \
        -DCMAKE_INSTALL_PREFIX="$root/install"
    cmake --build "$root/build" -j"$jobs"
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
    [[ -x "$root/build/g1_camera_check" ]] || { echo 'Build the native camera check before --verify-camera.' >&2; exit 1; }
    "$root/build/g1_camera_check"
fi
printf '\nSetup complete. No controller was started.\n'
printf 'Run: cd "%s/install"\n' "$root"
printf './g1 monitor eth0 0.0.0.0 8765\n./g1 parkour real eth0\n'
