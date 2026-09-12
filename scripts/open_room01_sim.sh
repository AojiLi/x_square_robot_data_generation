#!/usr/bin/env bash
set -e
ROOM01_LAUNCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOM01_LAUNCH_ROOT/scripts/isaac_env.sh"
exec python "$ROOM01_LAUNCH_ROOT/scripts/preview_room01_sim.py" "$@"
