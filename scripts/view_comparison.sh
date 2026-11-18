#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${project_root}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${VISER_PYTHON:-$HOME/odin1_ws/.venv-viser/bin/python}" -B -m tools.comparison_viewer "$@"
