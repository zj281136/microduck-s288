#!/usr/bin/env python3
"""S288 rotor zero calibration; default group=head, preserving other joints.

Match pure-rotor deployment. Put selected joints at model RESET q=0, not
HOME/STAND. Capture uses zero gains/torque and never commands a movement.
Existing per-joint directions are kept; an atomic backup precedes each save.
"""
import shutil
import sys
import time
from pathlib import Path
import calibrate_microduck_j288 as calibration

calibration.__doc__ = __doc__
calibration.DEFAULT_OUTPUT = Path(__file__).resolve().parent / "microduck_s288_calibration.json"
calibration.CALIBRATION_ROBOT = "microduck_s288"
calibration.DEFAULT_POSITION_SOURCE = "rotor"
calibration.DEFAULT_GROUP = "head"
_parse_args = calibration.parse_args
_write_json = calibration.write_json


def parse_rotor_args():
    args = _parse_args()
    if args.position_source != "rotor":
        sys.exit("S288 rotor calibration requires --position-source rotor")
    if args.output.expanduser().exists():
        document = calibration.load_document(args.output)
        if document.get("robot") != "microduck_s288":
            sys.exit("output calibration must belong to microduck_s288")
    return args


def write_with_backup(path, document):
    path = Path(path).expanduser().resolve()
    if path.exists():
        backup = path.with_name(path.name + f".before_rotor_{time.time_ns()}.bak")
        shutil.copy2(path, backup)
        print(f"Original calibration backed up: {backup}")
    _write_json(path, document)


calibration.parse_args = parse_rotor_args
calibration.write_json = write_with_backup


if __name__ == "__main__":
    raise SystemExit(calibration.main())
