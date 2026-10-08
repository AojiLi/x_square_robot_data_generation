#!/usr/bin/env bash
# Source this file to use the existing pinned runtime with this project's settings.
set -e
ROOM01_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOM01_ISAAC_RUNTIME="${ROOM01_ISAAC_RUNTIME:-/home/aoji/x_square_copy/isaac-sim-scenes}"
if [[ -n "${ROOM01_ISAAC_VENV:-}" ]]; then
    if [[ ! -f "$ROOM01_ISAAC_VENV/bin/activate" || ! -f "${ISAACLAB_ROOT:-}/isaaclab.sh" ]]; then
        echo "Set ROOM01_ISAAC_VENV to an installed Python environment and ISAACLAB_ROOT to its Isaac Lab checkout." >&2
        return 1
    fi
    source "$ROOM01_ISAAC_VENV/bin/activate"
    export ISAACLAB_ROOT PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
elif [[ -f "$ROOM01_ISAAC_RUNTIME/scripts/runtime_env.sh" ]]; then
    source "$ROOM01_ISAAC_RUNTIME/scripts/runtime_env.sh"
else
    echo "Isaac runtime not found. See docs/COLLEAGUE_SETUP.md for runtime configuration." >&2
    return 1
fi
export XDG_CONFIG_HOME="$ROOM01_ROOT/tools/room01_sim_runtime/config"
export XDG_DATA_HOME="$ROOM01_ROOT/tools/room01_sim_runtime/data"
export KIT_PORTABLE_ROOT="$ROOM01_ROOT/tools/room01_sim_runtime/kit"
export PYTHONPATH="$ROOM01_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$XDG_CONFIG_HOME" "$XDG_DATA_HOME" "$KIT_PORTABLE_ROOT"
