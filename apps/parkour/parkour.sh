#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
application="${G1_CONTROL_APP:-parkour}"
case "$application" in parkour|hybrid) ;; *) echo 'Unknown control application' >&2; exit 2 ;; esac
if [[ -x "$script_dir/bin/g1_$application" ]]; then
    root_dir="$script_dir"
    controller="$root_dir/bin/g1_$application"
else
    root_dir="$(cd -- "$script_dir/../.." && pwd)"
    controller="$root_dir/build/g1_$application"
fi
usage() {
    echo "Usage: $0 check [options] | sim [options] | real ROBOT_INTERFACE [options]"
    echo 'real opens RealSense directly in the C++ controller; no Python camera process.'
    echo 'real starts in dry-run with Unitree remote input. Press Start for real output handover.'
}
(($#)) || { usage; exit 0; }
mode="$1"; shift
# Reject obsolete overrides before starting the controller.
for argument in "$@"; do
    case "$argument" in
        --model|--model-dir|--limits|--unitree-remote|--reset-seconds|--port|--channel|--depth-port|--depth-channel|--no-auto-camera|--camera-web-port)
            echo "Removed option: $argument; transport is automatic; other settings use the program JSON configuration." >&2; exit 2 ;;
    esac
done
if [[ "$mode" == help || "$mode" == --help ]]; then
    usage
    if [[ -x "$controller" ]]; then "$controller" --help; fi
    exit 0
fi
[[ -x "$controller" ]] || { echo "Missing executable: $controller" >&2; exit 1; }
export LD_LIBRARY_PATH="$root_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
common=(--config "$root_dir/apps/$application/config/g1_$application.json")
case "$mode" in
    check) exec "$controller" "${common[@]}" --check-policy "$@" ;;
    sim) exec "$controller" "${common[@]}" --sim --domain 1 --interface lo "$@" ;;
    real|run)
        (($#)) || { usage >&2; exit 2; }
        interface="$1"; shift
        if [[ "${1:-}" == remote ]]; then
            echo 'The remote argument was removed; hardware uses the Unitree remote by default.' >&2; exit 2
        fi
        [[ "$interface" != lo && -d "/sys/class/net/$interface" ]] || {
            echo "Specify the actual robot network interface." >&2; exit 2;
        }
        exec "$controller" "${common[@]}" --hardware --domain 0 --interface "$interface" "$@"
        ;;
    *) usage >&2; exit 2 ;;
esac
