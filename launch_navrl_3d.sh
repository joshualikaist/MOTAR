#!/bin/sh
# Double-click this file (or pin it in the IDE) to open the NavRL 3-D setup window.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$APP_DIR"
PY="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHONNOUSERSITE=1
PY_DIR=$(dirname -- "$PY")
export PATH="$PY_DIR:$PATH"
exec "${PY}" aerial_gym/apps/navrl_3d.py "$@"
