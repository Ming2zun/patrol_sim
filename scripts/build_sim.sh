#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
pack=simulation/runtime/AlpinePatrol.pck
python3 scripts/repack_runtime.py "$pack" simulation/godot/scripts/main.gd "$pack.next"
mv -- "$pack.next" "$pack"
echo '运行包已更新；重启仿真生效。'
