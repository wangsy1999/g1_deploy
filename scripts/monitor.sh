#!/usr/bin/env bash
# Read-only monitor; works in the source checkout and in the CMake install tree.
set -euo pipefail
if (($# < 1 || $# > 3)); then
    echo "Usage: $0 ROBOT_INTERFACE [HOST=127.0.0.1] [PORT=8765]" >&2
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/web/joint_monitor.py" ]]; then
    root_dir="$script_dir"
else
    root_dir="$(cd -- "$script_dir/.." && pwd)"
fi
interface="$1"
[[ -n "$interface" && "$interface" != -* && "$interface" != */* && -d "/sys/class/net/$interface" ]] || {
    echo "Specify an existing network interface." >&2
    exit 2
}
[[ -f "$root_dir/web/joint_monitor.py" ]] || { echo "Missing web/joint_monitor.py" >&2; exit 1; }
python="${PYTHON_BIN:-python3}"
if [[ -d "$root_dir/.python-deps" ]]; then
    export PYTHONPATH="$root_dir/.python-deps${PYTHONPATH:+:$PYTHONPATH}"
fi
# Match the native SDK used by the controller, without changing the system config.
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$python" -u "$root_dir/web/joint_monitor.py" \
    --domain 0 --interface "$interface" --host "${2:-127.0.0.1}" --port "${3:-8765}" --rate 50
