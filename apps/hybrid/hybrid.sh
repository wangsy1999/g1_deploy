#!/usr/bin/env bash
# Share the camera/controller lifecycle launcher with Parkour.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/apps/parkour/parkour_camera.py" ]]; then
    root_dir="$script_dir"
else
    root_dir="$(cd -- "$script_dir/../.." && pwd)"
fi
export G1_CONTROL_APP=hybrid
if [[ -x "$root_dir/parkour" ]]; then exec bash "$root_dir/parkour" "$@"; fi
exec bash "$root_dir/apps/parkour/parkour.sh" "$@"
