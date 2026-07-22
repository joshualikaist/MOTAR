#!/bin/sh
# Double-click this file (or pin it in the IDE) to open the NavRL 3-D setup window.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$APP_DIR"
PY="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHONNOUSERSITE=1
PY_DIR=$(dirname -- "$PY")
export PATH="$PY_DIR:$PATH"
if [ "$#" -eq 0 ] && /usr/bin/python3 -c "import gi; gi.require_version('Gtk', '3.0')" 2>/dev/null; then
    exec /usr/bin/python3 aerial_gym/apps/navrl_3d_gtk_launcher.py --runtime-python "$PY"
fi
exec "${PY}" aerial_gym/apps/navrl_3d.py "$@"
