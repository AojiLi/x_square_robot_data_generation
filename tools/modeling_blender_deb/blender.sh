#!/usr/bin/env bash
set -e
pilot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="$pilot_root/root/usr/lib/x86_64-linux-gnu:$pilot_root/root/usr/lib:$pilot_root/root/usr/lib/x86_64-linux-gnu/blas:$pilot_root/root/usr/lib/x86_64-linux-gnu/lapack${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export BLENDER_SYSTEM_SCRIPTS="$pilot_root/root/usr/share/blender/scripts"
export BLENDER_SYSTEM_DATAFILES="$pilot_root/root/usr/share/blender/datafiles"
export BLENDER_USER_CONFIG="$pilot_root/config"
export PYTHONPATH="$pilot_root/../../.venv-usd/lib/python3.12/site-packages"
exec "$pilot_root/root/usr/bin/blender" "$@"
