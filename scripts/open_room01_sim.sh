#!/usr/bin/env bash
set -e
ROOM01_LAUNCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOM01_LAUNCH_ROOT/scripts/isaac_env.sh"
if [[ "${1:-}" == "--legacy" ]]; then
    shift
    exec python "$ROOM01_LAUNCH_ROOT/scripts/preview_room01_sim.py" "$@"
fi
if [[ -f "$ROOM01_LAUNCH_ROOT/assets/room01/battery_task/room01_battery_task.usda" ]]; then
    exec python "$ROOM01_LAUNCH_ROOT/scripts/preview_room01_sim.py" \
        --scene "$ROOM01_LAUNCH_ROOT/assets/room01/battery_task/room01_battery_task.usda" \
        --config "$ROOM01_LAUNCH_ROOT/assets/room01/battery_task/task_config.json" \
        --output "$ROOM01_LAUNCH_ROOT/reports/room01_battery_task/preview" "$@"
fi
exec python "$ROOM01_LAUNCH_ROOT/scripts/preview_room01_sim.py" "$@"
