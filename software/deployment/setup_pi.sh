#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
python3 -m venv "$root/.venv"
"$root/.venv/bin/python" -m pip install -r "$root/teleop/scripts/requirements_pi_deploy.txt"
"$root/.venv/bin/python" "$root/deploy.py" check
echo "Installed deployment dependencies. Calibrate your own robot before motion."
