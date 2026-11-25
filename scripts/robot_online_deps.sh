#!/usr/bin/env bash
# Helpers sourced by setup_robot.sh. Downloads stay outside build/.
download() {
    local url="$1" output="$2"
    if [[ ! -s "$output" ]]; then
        command -v curl >/dev/null || { echo "Missing curl. Install it or rerun without --no-system-deps." >&2; return 1; }
        mkdir -p "$(dirname -- "$output")"
        echo "Downloading $url"
        curl --fail --location --retry 3 --connect-timeout 30 \
            --proto '=https' --proto-redir '=https' "$url" -o "$output.part"
        mv "$output.part" "$output"
    fi
}
source_tree() {
    local name="$1" url="$2"
    download "$url" "$cache/downloads/$name.tar.gz"
    mkdir -p "$deps/sources/$name"
    tar -xzf "$cache/downloads/$name.tar.gz" -C "$deps/sources/$name" --strip-components=1
}
build_dependency() {
    local name="$1" prefix="$2"; shift 2
    cmake -S "$deps/sources/$name" -B "$deps/build/$name" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib "$@"
    cmake --build "$deps/build/$name" -j"$jobs"
    cmake --install "$deps/build/$name"
}
online_native() {
    local archive="$1" destination="$2"
    case "$archive" in
        cpython-arm64.tar.gz)
            download 'https://github.com/astral-sh/python-build-standalone/releases/download/20241016/cpython-3.10.15%2B20241016-aarch64-unknown-linux-gnu-install_only.tar.gz' "$cache/downloads/cpython.tar.gz"
            mkdir -p "$destination"
            tar -xzf "$cache/downloads/cpython.tar.gz" -C "$destination"
            ;;
        unitree-sdk2-arm64.tar.gz)
            source_tree unitree-sdk2 'https://github.com/unitreerobotics/unitree_sdk2/archive/refs/heads/main.tar.gz'
            build_dependency unitree-sdk2 "$destination" -DBUILD_EXAMPLES=OFF
            ;;
        cyclonedds-0.10.2-arm64.tar.gz)
            source_tree cyclonedds 'https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.10.2.tar.gz'
            build_dependency cyclonedds "$destination" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SECURITY=OFF
            ;;
        onnxruntime-1.18.1-arm64.tar.gz)
            download 'https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-aarch64-1.18.1.tgz' "$cache/downloads/onnxruntime.tgz"
            mkdir -p "$destination"
            tar -xzf "$cache/downloads/onnxruntime.tgz" -C "$destination" --strip-components=1
            ;;
        *) echo "No online source for $archive" >&2; return 1 ;;
    esac
}
online_glibc() {
    # Read Ubuntu's current Jammy package filename and digest instead of relying
    # on an obsolete security-update URL. Extract privately, never dpkg -i.
    download 'https://ports.ubuntu.com/ubuntu-ports/dists/jammy-updates/main/binary-arm64/Packages.gz' "$cache/downloads/jammy-Packages.gz"
    python3 - "$cache/downloads" <<'PY'
import gzip, hashlib, pathlib, sys, urllib.request
root = pathlib.Path(sys.argv[1])
for paragraph in gzip.decompress((root / 'jammy-Packages.gz').read_bytes()).decode().split('\n\n'):
    fields = dict(line.split(': ', 1) for line in paragraph.splitlines() if ': ' in line and not line.startswith(' '))
    if fields.get('Package') == 'libc6' and fields.get('Architecture') == 'arm64':
        url = 'https://ports.ubuntu.com/ubuntu-ports/' + fields['Filename']
        target = root / 'libc6-jammy-arm64.deb'
        if not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest() != fields['SHA256']:
            print('Downloading ' + url, flush=True)
            with urllib.request.urlopen(url, timeout=120) as response:
                data = response.read()
            if hashlib.sha256(data).hexdigest() != fields['SHA256']:
                raise SystemExit('Ubuntu libc6 checksum mismatch')
            target.write_bytes(data)
        break
else:
    raise SystemExit('ARM64 libc6 not found in Ubuntu Jammy package index')
PY
    mkdir -p "$deps/glibc-2.35"
    dpkg-deb -x "$cache/downloads/libc6-jammy-arm64.deb" "$deps/glibc-2.35"
}
ensure_native_camera() {
    # Probe CMake's system search paths as well as the project-local SDK.
    mkdir -p "$deps/build/realsense-probe"
    cat > "$deps/build/realsense-probe/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(realsense_probe LANGUAGES CXX)
find_package(realsense2 CONFIG REQUIRED)
CMAKE
    if cmake -S "$deps/build/realsense-probe" -B "$deps/build/realsense-probe/build" \
        -DCMAKE_PREFIX_PATH="$deps/realsense" > "$deps/setup-logs/realsense-probe.log" 2>&1; then
        return
    fi
    if ((offline)); then
        echo "Missing native RealSense SDK. Provide $deps/realsense or install librealsense2-dev." >&2
        return 1
    fi
    source_tree realsense 'https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.55.1.tar.gz'
    build_dependency realsense "$deps/realsense" -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_WITH_DDS=OFF -DBUILD_UNIT_TESTS=OFF -DIMPORT_DEPTH_CAM_FW=OFF
}
