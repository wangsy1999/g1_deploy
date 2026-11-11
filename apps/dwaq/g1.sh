#!/usr/bin/env bash
# Compatibility entry point; the shared launcher lives at the repository root.
set -euo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$root_dir/g1" "$@"
