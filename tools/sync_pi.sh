#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "usage: bash tools/sync_pi.sh pi@PI_IP /absolute/remote/deployment" >&2
  exit 2
fi
root="$(cd "$(dirname "$0")/.." && pwd)"
rsync -av --progress --exclude '.venv/' --exclude 'logs/' --exclude '__pycache__/' \
  --exclude '*.pyc' --exclude '*_calibration.json' --exclude '*.bak' \
  --exclude '*.before_*' \
  "$root/software/deployment/" "$1:$2/"
echo "Pi calibration preserved. Install requirements in the remote deployment .venv separately."
