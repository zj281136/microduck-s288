#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_common.sh"
if (($#==0)); then
  echo 'usage: bash scripts/export_onnx.sh --checkpoint-file FILE --onnx-file NEW_FILE' >&2
  echo 'Creates an environment for export; manual execution, pending end-to-end validation.' >&2
  exit 2
fi
python_bin="$PROJECT_ROOT/software/training/.venv/bin/python"
require_python "$python_bin"
cd "$PROJECT_ROOT/software/training"
exec "$python_bin" scripts/export.py "$@"
