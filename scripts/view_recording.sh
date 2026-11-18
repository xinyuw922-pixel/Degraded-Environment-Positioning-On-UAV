#!/usr/bin/env bash
# Serve an offline trajectory using the installed viser environment.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export OPENBLAS_NUM_THREADS=1
exec "${VISER_PYTHON:-$HOME/odin1_ws/.venv-viser/bin/python}" -B -m tools.recording_viewer "$@"
