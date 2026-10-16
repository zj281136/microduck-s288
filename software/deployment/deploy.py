#!/usr/bin/env python3
"""Public S288 rotor deployment launcher; default 'check' never opens hardware."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT / "teleop" / "scripts"
ENTRIES = {
    "stand": "deploy_microduck_s288_v0_backlash_dr_real.py",
    "keyboard": "deploy_microduck_s288_v0_backlash_dr_keyboard_real.py",
    "calibrate": "calibrate_microduck_s288_rotor.py",
    "probe": "probe_s288_bus.py",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        "stand/keyboard/calibrate/probe forward all remaining options to the "
        "original S288 entry. check only loads the local policy."
    ))
    parser.add_argument("mode", nargs="?", default="check",
                        choices=("check", *ENTRIES))
    args, remaining = parser.parse_known_args()
    sys.path[:0] = [str(SCRIPTS), str(ROOT / "teleop")]
    if args.mode == "check":
        if remaining:
            parser.error("check takes no hardware arguments")
        entry = importlib.import_module("deploy_microduck_s288_v0_backlash_dr_real")
        model = entry.deployment.DEFAULT_POLICY
        session, _, _, _, _ = entry.load_backlash_dr_policy(model)
        manifest = json.loads(model.with_suffix(".manifest.json").read_text())
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        if digest != manifest["onnx_sha256"]:
            raise RuntimeError("ONNX hash differs from model manifest")
        if entry.deployment.MicroDuckJ288.__name__ != "MicroDuckS288M1Rotor":
            raise RuntimeError("Expected the pure rotor hardware adapter")
        print(json.dumps({
            "policy": str(model.relative_to(ROOT)),
            "input_shape": session.get_inputs()[0].shape,
            "output_shape": session.get_outputs()[0].shape,
            "sha256": digest,
            "position_source": "rotor",
            "hardware_opened": False,
        }, indent=2))
        return
    script = SCRIPTS / ENTRIES[args.mode]
    sys.argv = [str(script), *remaining]
    # Delegate unchanged: calibration, motor commands and all guards remain
    # implemented in the already matched S288 deployment source.
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
