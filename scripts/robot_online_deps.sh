#!/usr/bin/env bash
# Helpers sourced by setup_robot.sh. Downloads stay outside build/.
download_matches() {
    local file="$1" checksum="$2" actual
    [[ -s "$file" ]] || return 1
    [[ -n "$checksum" ]] || return 0
    actual=$(sha256sum < "$file") || return
    [[ "${actual%% *}" == "$checksum" ]]
}
download() {
    local url="$1" output="$2" mirror_url="${3:-}" checksum="${4:-}" candidate
    if download_matches "$output" "$checksum"; then return 0; fi
    command -v curl >/dev/null || { echo "Missing curl. Install it or rerun without --no-system-deps." >&2; return 1; }
    mkdir -p "$(dirname -- "$output")" || return

    local -a candidates=("$url")
    if [[ "$url" == https://github.com/* ]]; then
        local mirror="${G1_GITHUB_MIRROR:-}"
        if [[ -n "$mirror" ]]; then
            candidates=("${mirror%/}/$url")
        else
            candidates=("https://gh-proxy.org/$url" "https://ghproxy.net/$url")
            if [[ -n "$mirror_url" ]]; then
                candidates=("$mirror_url" "${candidates[@]}")
            fi
        fi
    elif [[ -n "$mirror_url" ]]; then
        candidates+=("$mirror_url")
    fi

    rm -f -- "$output.part" || return
    for candidate in "${candidates[@]}"; do
        echo "Downloading $candidate"
        if curl --fail --location --retry 3 --connect-timeout 30 \
            --proto '=https' --proto-redir '=https' "$candidate" -o "$output.part"; then
            if download_matches "$output.part" "$checksum"; then
                mv -- "$output.part" "$output"
                return
            fi
            echo "Downloaded file is empty or its SHA-256 does not match: $candidate" >&2
        fi
        rm -f -- "$output.part" || return
    done
    echo "Download failed for $url" >&2
    return 1
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
            local asset='20241016/cpython-3.10.15%2B20241016-aarch64-unknown-linux-gnu-install_only.tar.gz'
            download "https://github.com/astral-sh/python-build-standalone/releases/download/$asset" \
                "$cache/downloads/cpython.tar.gz" "https://registry.npmmirror.com/-/binary/python-build-standalone/$asset"
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
            local asset='onnxruntime-linux-aarch64-1.18.1.tgz'
            download "https://registry.npmmirror.com/-/binary/onnxruntime/v1.18.1/$asset" \
                "$cache/downloads/onnxruntime.tgz" \
                "https://hf-mirror.com/csukuangfj/onnxruntime-libs/resolve/b83286b3dcbeab2f42e646ad29b3edffa1cfe5b5/$asset" \
                'c1dcd8ab29e8d227d886b6ee415c08aea893956acf98f0758a42a84f27c02851' || return
            mkdir -p "$destination"
            tar -xzf "$cache/downloads/onnxruntime.tgz" -C "$destination" --strip-components=1
            ;;
        *) echo "No online source for $archive" >&2; return 1 ;;
    esac
}
jammy_libc_package() {
    python3 - "$1" <<'PY'
import gzip, re, sys

try:
    with gzip.open(sys.argv[1], 'rt', encoding='utf-8') as index:
        paragraphs = index.read().split('\n\n')
except (OSError, EOFError, UnicodeError) as error:
    raise SystemExit('Cannot read Ubuntu Jammy package index: {}'.format(error))

for paragraph in paragraphs:
    if not paragraph.startswith('Package: libc6\n'):
        continue
    fields = dict(line.split(': ', 1) for line in paragraph.splitlines() if ': ' in line and not line.startswith(' '))
    filename, checksum = fields.get('Filename', ''), fields.get('SHA256', '')
    if (fields.get('Architecture') == 'arm64'
            and re.fullmatch(r'pool/main/g/glibc/libc6_2\.35-[A-Za-z0-9.+~_-]+_arm64\.deb', filename)
            and re.fullmatch(r'[0-9a-f]{64}', checksum)):
        print(filename, checksum)
        sys.exit(0)
raise SystemExit('ARM64 libc6 2.35 entry not found in Ubuntu Jammy package index')
PY
}
online_glibc() (
    local index="$cache/downloads/jammy-Packages.gz" package="$cache/downloads/libc6-jammy-arm64.deb"
    local destination="$deps/glibc-2.35" entry filename checksum mirror fresh_index
    # Reuse a verified cache. Otherwise fetch the index and package from the same mirror.
    if [[ -s "$index" && -s "$package" ]] && entry=$(jammy_libc_package "$index"); then
        read -r filename checksum <<< "$entry"
        if download_matches "$package" "$checksum"; then
            mkdir -p "$destination" || return
            dpkg-deb -x "$package" "$destination"
            return
        fi
    fi

    mkdir -p "$cache/downloads" || return
    fresh_index=$(mktemp "$cache/downloads/jammy-Packages.XXXXXX") || return
    trap 'rm -f -- "$fresh_index" "$fresh_index.part"' EXIT
    for mirror in https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports \
        https://mirrors.ustc.edu.cn/ubuntu-ports https://mirrors.aliyun.com/ubuntu-ports \
        https://ports.ubuntu.com/ubuntu-ports; do
        rm -f -- "$fresh_index" || return
        download "$mirror/dists/jammy-updates/main/binary-arm64/Packages.gz" "$fresh_index" || continue
        entry=$(jammy_libc_package "$fresh_index") || continue
        read -r filename checksum <<< "$entry"
        if download "$mirror/$filename" "$package" '' "$checksum"; then
            mv -- "$fresh_index" "$index" || return
            mkdir -p "$destination" || return
            dpkg-deb -x "$package" "$destination"
            return
        fi
    done
    echo 'Could not download a verified Ubuntu Jammy ARM64 libc6 package from any Ubuntu Ports mirror.' >&2
    return 1
)
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
    local json_archive="$cache/downloads/nlohmann-json-3.11.3.tar.gz"
    local json_checksum='0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406'
    download 'https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz' "$json_archive" '' \
        "$json_checksum" || return
    # Use the cached archive while keeping the source path used by RealSense's rsutils.
    cat > "$deps/sources/realsense/CMake/json-download.cmake.in" <<'CMAKE' || return
cmake_minimum_required(VERSION 3.10...3.24)
project(nlohmann-json-download NONE)
include(ExternalProject)
ExternalProject_Add(nlohmann_json
    PREFIX .
    URL "@G1_REALSENSE_JSON_ARCHIVE@"
    URL_HASH "SHA256=@G1_REALSENSE_JSON_SHA256@"
    SOURCE_DIR "${CMAKE_BINARY_DIR}/third-party/json"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
CMAKE
    build_dependency realsense "$deps/realsense" -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_WITH_DDS=OFF -DBUILD_UNIT_TESTS=OFF -DIMPORT_DEPTH_CAM_FW=OFF \
        "-DG1_REALSENSE_JSON_ARCHIVE:FILEPATH=$json_archive" \
        "-DG1_REALSENSE_JSON_SHA256=$json_checksum"
}
