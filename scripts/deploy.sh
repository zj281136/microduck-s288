#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_common.sh"
python_bin="$PROJECT_ROOT/software/deployment/.venv/bin/python"
require_python "$python_bin"
cd "$PROJECT_ROOT/software/deployment"
# No arguments delegates to check, not movement. Motion retains --arm-ground-test.
exec "$python_bin" deploy.py "$@"
