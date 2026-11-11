#!/usr/bin/env bash
set -euo pipefail
dependency_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$dependency_root/glibc-2.35/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1" \
    --library-path "$dependency_root/glibc-2.35/lib/aarch64-linux-gnu:$dependency_root/realsense/lib:$dependency_root/runtime/python/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$dependency_root/python/bin/python3" "$@"
