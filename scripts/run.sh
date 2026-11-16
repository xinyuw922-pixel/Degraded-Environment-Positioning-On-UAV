#!/usr/bin/env bash
# Resolve this checkout and forward arguments to the tools entry point.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

exec "${PYTHON_BIN:-python3}" -B -m tools "$@"
