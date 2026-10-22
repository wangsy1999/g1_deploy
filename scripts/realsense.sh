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
python="${PYTHON_BIN:-python3}"
for dependency_root in "$root_dir/.deps" "$root_dir/../.deps"; do
    if [[ -z "${PYTHON_BIN:-}" && -x "$dependency_root/python/bin/python3" ]]; then
        python="$dependency_root/python/bin/python3"
        export LD_LIBRARY_PATH="$dependency_root/realsense/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        break
    fi
done
exec "$python" -u "$root_dir/camera/realsense_web.py" "$@"
