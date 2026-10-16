#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_common.sh"
if (($#==0)); then
  echo 'usage: bash scripts/calibrate.sh --port DEVICE --group all [--calibrate-directions]' >&2
  echo 'Requires external support; direction calibration moves joints.' >&2
  exit 2
fi
python_bin="$PROJECT_ROOT/software/deployment/.venv/bin/python"
require_python "$python_bin"
cd "$PROJECT_ROOT/software/deployment"
exec "$python_bin" deploy.py calibrate "$@"
