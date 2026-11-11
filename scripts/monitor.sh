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
for candidate in "$root_dir/.deps/python/bin/python3" "$root_dir/../.deps/python/bin/python3"; do
    if [[ -z "${PYTHON_BIN:-}" && -x "$candidate" ]]; then python="$candidate"; break; fi
done
if [[ -d "$root_dir/.python-deps" ]]; then
    export PYTHONPATH="$root_dir/.python-deps${PYTHONPATH:+:$PYTHONPATH}"
fi
# Match the native SDK used by the controller, without changing the system config.
dds_library_dir=/usr/local/lib
for dependency_root in "$root_dir/.deps" "$root_dir/../.deps"; do
    if [[ -f "$dependency_root/cyclonedds/lib/libddsc.so" ]]; then
        dds_library_dir="$dependency_root/cyclonedds/lib"
        break
    fi
done
export LD_LIBRARY_PATH="$dds_library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$python" -u "$root_dir/web/joint_monitor.py" \
    --domain 0 --interface "$interface" --host "${2:-127.0.0.1}" --port "${3:-8765}" --rate 50
