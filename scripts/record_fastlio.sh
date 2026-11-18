#!/usr/bin/env bash
# Configure ROS, then let tools own all recording and event logic.
set -eo pipefail
source /opt/ros/noetic/setup.bash --extend
source "${LIVOX_SETUP:-$HOME/livox_ws/devel/setup.bash}" --extend
set -u
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${PYTHON_BIN:-python3}" -B -m tools.experiment_recorder "$@"
