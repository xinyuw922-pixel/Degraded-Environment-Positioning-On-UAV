#!/usr/bin/env bash
# Use the existing ROS and viser environments, then delegate to tools.
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
exec "${VISER_PYTHON:-$HOME/odin1_ws/.venv-viser/bin/python}" -B -m tools.fastlio_viewer "$@"
