#!/usr/bin/env sh
set -eu
cd -- "$(dirname -- "$0")"
exec ./AlpinePatrol.x86_64 "$@"
