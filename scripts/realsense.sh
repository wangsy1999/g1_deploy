#!/usr/bin/env bash
# Source checkout and installed package share the same read-only camera service.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/camera/realsense_web.py" ]]; then
    root_dir="$script_dir"
else
    root_dir="$(cd -- "$script_dir/.." && pwd)"
fi
if [[ -d "$root_dir/.python-deps" ]]; then
    export PYTHONPATH="$root_dir/.python-deps${PYTHONPATH:+:$PYTHONPATH}"
fi
exec "${PYTHON_BIN:-python3}" -u "$root_dir/camera/realsense_web.py" "$@"
