#!/usr/bin/env bash
# Use existing ROS and delegate the diagnostic to tools.
set -eo pipefail
source /opt/ros/noetic/setup.bash --extend
source "${LIVOX_SETUP:-$HOME/livox_ws/devel/setup.bash}" --extend
set -u
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export OPENBLAS_NUM_THREADS=1
exec "${PYTHON_BIN:-python3}" -B -m tools.floor_alignment "$@"
