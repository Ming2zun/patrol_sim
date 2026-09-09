#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -f simulation/runtime/AlpinePatrol.x86_64 || ! -f simulation/runtime/AlpinePatrol.pck ]]; then
  echo '缺少 Linux 运行包。下载网盘资源后执行：./scripts/install_resources.sh 网盘资源文件夹路径'; exit 1
fi
exec ./simulation/runtime/start.sh "$@"
