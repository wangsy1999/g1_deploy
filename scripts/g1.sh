#!/usr/bin/env bash
# Installed package entry point. Never enters control mode without an explicit run command.
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
controller="$package_dir/bin/g1_dwaq"
model="$package_dir/models/policy.onnx"
config="$package_dir/config/g1_locomotion.yaml"
usage() {
    cat <<'HELP'
Usage:
  ./g1 check                         Offline ONNX inference, no DDS
  ./g1 interfaces                    Show local network interfaces and route to control PC
  ./g1 joystick [DEVICE]             Read-only F710 input check, no DDS
  ./g1 read-only ROBOT_INTERFACE      Subscribe to hardware state, no commands or mode release
  ./g1 run ROBOT_INTERFACE [DEVICE]   Hardware control, optional F710 jsdev path

Example interface: eth0 (use the actual onboard interface, never lo).
run starts disabled; keyboard 9 -> p -> wait -> r, or F710 Start -> A -> wait -> Y.
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
[[ -x "$controller" ]] || { echo "Missing executable: $controller" >&2; exit 1; }
case "$mode" in
    check)
        (($# == 0)) || { usage >&2; exit 2; }
        exec "$controller" --check-policy "$model" "$config"
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
            exec "$controller" --hardware 3 "$robot_interface" 0 "$config" "$model" --joystick "$1"
        fi
        exec "$controller" --hardware 3 "$robot_interface" 0 "$config" "$model"
        ;;
    *) usage >&2; exit 2 ;;
esac
