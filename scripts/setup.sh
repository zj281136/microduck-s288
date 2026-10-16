#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_common.sh"
case "${1:-}" in
  deployment)
    exec bash "$PROJECT_ROOT/software/deployment/setup_pi.sh"
    ;;
  training)
    command -v uv >/dev/null || { echo 'uv is required for training setup' >&2; exit 2; }
    cd "$PROJECT_ROOT/software/training"
    exec uv sync --locked
    ;;
  *) echo 'usage: bash scripts/setup.sh deployment|training (installs dependencies; manual execution)' >&2; exit 2;;
esac
