#!/usr/bin/env bash
# Source this file to use the existing pinned runtime with this project's settings.
set -e
ROOM01_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOM01_ISAAC_RUNTIME="${ROOM01_ISAAC_RUNTIME:-/home/aoji/x_square_copy/isaac-sim-scenes}"
source "$ROOM01_ISAAC_RUNTIME/scripts/runtime_env.sh"
export XDG_CONFIG_HOME="$ROOM01_ROOT/tools/room01_sim_runtime/config"
export XDG_DATA_HOME="$ROOM01_ROOT/tools/room01_sim_runtime/data"
export KIT_PORTABLE_ROOT="$ROOM01_ROOT/tools/room01_sim_runtime/kit"
export PYTHONPATH="$ROOM01_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$XDG_CONFIG_HOME" "$XDG_DATA_HOME" "$KIT_PORTABLE_ROOT"
