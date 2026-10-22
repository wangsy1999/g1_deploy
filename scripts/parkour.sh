#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$script_dir/bin/g1_parkour" ]]; then
    root_dir="$script_dir"
    controller="$root_dir/bin/g1_parkour"
else
    root_dir="$(cd -- "$script_dir/.." && pwd)"
    controller="$root_dir/build/g1_parkour"
fi
usage() {
    echo "Usage: $0 check | sim [controller options] | real ROBOT_INTERFACE [controller options]"
    echo 'real automatically starts/reuses camera + depth UDP; --no-auto-camera disables this.'
    echo 'real starts in dry-run with Unitree remote input. Real output needs --hardware-calibrated and Start.'
}
(($#)) || { usage; exit 0; }
mode="$1"; shift
[[ -x "$controller" ]] || { echo "Missing executable: $controller" >&2; exit 1; }
export LD_LIBRARY_PATH="$root_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
common=(--model-dir "$root_dir/models/parkour" --config "$root_dir/config/g1_parkour.json" --limits "$root_dir/config/g1_locomotion.yaml")
case "$mode" in
    check) exec "$controller" "${common[@]}" --check-policy "$@" ;;
    sim) exec "$controller" "${common[@]}" --sim --domain 1 --interface lo "$@" ;;
    real)
        (($#)) || { usage >&2; exit 2; }
        interface="$1"; shift
        auto_camera=1
        web_port=8766
        depth_port=17201
        options=()
        while (($#)); do
            case "$1" in
                --no-auto-camera) auto_camera=0; shift ;;
                --help|--check-policy) auto_camera=0; options+=("$1"); shift ;;
                --camera-web-port)
                    (($# >= 2)) || { echo 'Missing camera web port' >&2; exit 2; }
                    web_port="$2"; shift 2 ;;
                --port)
                    (($# >= 2)) || { echo 'Missing depth UDP port' >&2; exit 2; }
                    depth_port="$2"; options+=("$1" "$2"); shift 2 ;;
                *) options+=("$1"); shift ;;
            esac
        done
        [[ "$interface" != lo && -d "/sys/class/net/$interface" ]] || {
            echo "Specify the actual robot network interface." >&2; exit 2;
        }
        if (( ! auto_camera )); then
            exec "$controller" "${common[@]}" --hardware --domain 0 --interface "$interface" "${options[@]}"
        fi
        python="${PYTHON_BIN:-python3}"
        for candidate in "$root_dir/.deps/python/bin/python3" "$root_dir/../.deps/python/bin/python3"; do
            if [[ -z "${PYTHON_BIN:-}" && -x "$candidate" ]]; then python="$candidate"; break; fi
        done
        exec "$python" "$root_dir/camera/parkour_camera.py" --root "$root_dir" \
            --depth-port "$depth_port" --web-port "$web_port" -- \
            "$controller" "${common[@]}" --hardware --domain 0 --interface "$interface" "${options[@]}"
        ;;
    *) usage >&2; exit 2 ;;
esac
