#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export OPENBLAS_NUM_THREADS=1
exec "${PYTHON_BIN:-$HOME/positioning_data/environments/lio-modern/bin/python}" -B -m tools.python_lio_replay "$@"
