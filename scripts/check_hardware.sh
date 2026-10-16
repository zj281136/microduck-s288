#!/usr/bin/env bash
set -euo pipefail
# Device-node listing only: no serial/I2C open, bus packet or motor command.
echo 'System node listing only; no automatic hardware/safety acceptance.'
shopt -s nullglob
nodes=(/dev/serial/by-id/* /dev/i2c-*)
if ((${#nodes[@]})); then
  ls -l -- "${nodes[@]}"
else
  echo 'No matching by-id or I2C nodes found.'
fi
echo 'NOT CHECKED: pinout, voltage, motor IDs, zero/direction, IMU axes/rate, communication quality, safe stand.'
