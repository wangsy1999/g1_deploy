#!/usr/bin/env bash
# Source checkout and installed package entry point. Never enters control mode without an explicit run command.
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$package_dir/src/main.cpp" ]]; then
    package_dir="$(cd -- "$package_dir/../.." && pwd)"
    controller="$package_dir/build/g1_dwaq"
else
    controller="$package_dir/bin/g1_dwaq"
fi
config="$package_dir/apps/dwaq/config/g1_locomotion.json"
usage() {
    cat <<'HELP'
Usage:
  ./g1 check                         Offline ONNX inference, no DDS
  ./g1 interfaces                    Show local network interfaces and route to control PC
  ./g1 monitor INTERFACE [HOST] [PORT] Start the read-only web monitor (Python dependencies needed)
  ./g1 realsense [options]           Read-only RGB/depth web test, optional local depth feed
  ./g1 parkour check|sim|real ...     Dual-network parkour inference (separate executable)
  ./g1 joystick [DEVICE]             Read-only F710 input check, no DDS
  ./g1 read-only ROBOT_INTERFACE      Subscribe to hardware state, no commands or mode release
  ./g1 run ROBOT_INTERFACE [remote]   Hardware control, keyboard or Unitree remote
  ./g1 run ROBOT_INTERFACE remote     Unitree: Start enables, X reset, A policy, Select stop; no R1 gate

Example interface: eth0 (use the actual onboard interface, never lo).
run starts in dry run: p -> wait -> r previews targets without actuator commands.
9/Start enables real output and clears preview; then p -> wait -> r is required again.
Default model/config are loaded from this package, not the build machine.
HELP
}
if (($# == 0)); then usage; exit 0; fi
mode="$1"; shift
if [[ "$mode" == help || "$mode" == --help ]]; then usage; exit 0; fi
if [[ "$mode" == interfaces ]]; then
    (($# == 0)) || { usage >&2; exit 2; }
    uname -m
    ip -br address
    ip route get 192.168.123.161 || true
    exit 0
fi
export LD_LIBRARY_PATH="$package_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ "$mode" == monitor ]]; then
    if [[ -x "$package_dir/monitor" ]]; then exec "$package_dir/monitor" "$@"; fi
    exec bash "$package_dir/apps/monitor/monitor.sh" "$@"
fi
if [[ "$mode" == realsense || "$mode" == parkour ]]; then
    if [[ -x "$package_dir/$mode" ]]; then exec "$package_dir/$mode" "$@"; fi
    exec bash "$package_dir/apps/$mode/$mode.sh" "$@"
fi
[[ -x "$controller" ]] || { echo "Missing executable: $controller" >&2; exit 1; }
case "$mode" in
    check)
        (($# == 0)) || { usage >&2; exit 2; }
        exec "$controller" --check-policy "$config"
        ;;
    joystick)
        (($# <= 1)) || { usage >&2; exit 2; }
        exec "$controller" --check-joystick --joystick "${1:-/dev/input/js0}"
        ;;
    read-only|run)
        if [[ "$mode" == read-only ]]; then max_args=1; else max_args=2; fi
        (($# >= 1 && $# <= max_args)) || { usage >&2; exit 2; }
        robot_interface="$1"; shift
        [[ -n "$robot_interface" && "$robot_interface" != lo && "$robot_interface" != -* && "$robot_interface" != */* ]] || {
            echo 'Specify the actual non-loopback robot network interface.' >&2; exit 2;
        }
        [[ -d "/sys/class/net/$robot_interface" ]] || { echo "No such interface: $robot_interface" >&2; exit 2; }
        if [[ "$mode" == read-only ]]; then
            exec "$controller" --hardware-read-only 3 "$robot_interface" 0 "$config"
        fi
        if (($#)); then
            if [[ "$1" == remote ]]; then
                exec "$controller" --hardware 3 "$robot_interface" 0 "$config" --unitree-remote
            fi
            echo "Hardware input must be remote or omitted for keyboard; F710 is simulation-only." >&2
            exit 2
        fi
        exec "$controller" --hardware 3 "$robot_interface" 0 "$config"
        ;;
    *) usage >&2; exit 2 ;;
esac
