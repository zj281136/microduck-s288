#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
require_python() {
  if [[ ! -x "$1" ]]; then
    echo "Missing environment: $1. Follow the documented setup first." >&2
    exit 2
  fi
}
