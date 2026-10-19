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
        exec "$controller" "${common[@]}" --hardware --domain 0 --interface "$interface" "$@"
        ;;
    *) usage >&2; exit 2 ;;
esac
