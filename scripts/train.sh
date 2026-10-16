#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_common.sh"
if [[ "${1:-}" != '--run' ]]; then
  echo 'usage: bash scripts/train.sh --run [mjlab training options]' >&2
  echo 'Starts real RL training; no implicit smoke test.' >&2
  exit 2
fi
shift
python_bin="$PROJECT_ROOT/software/training/.venv/bin/python"
require_python "$python_bin"
cd "$PROJECT_ROOT/software/training"
exec "$python_bin" -m mjlab.scripts.train Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck "$@"
