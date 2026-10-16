#!/usr/bin/env python3
"""Terminal keyboard entry for the independent V0 backlash DR model_9999.

Reuse the existing V0 keys and zero head commands. The matched DR loader,
rotor observation, M1 gains, motor guards and target processing are unchanged.
"""
import sys
import deploy_microduck_s288_v0_backlash_dr_real as baseline
from s288_v0_keyboard import TerminalKeyboard

deployment = baseline.deployment
deployment.COMMAND_SOURCE = TerminalKeyboard(enable_head=False)
deployment.POLICY_RAMP_MODE = "none"
_base_parse_args = deployment.parse_args


def parse_keyboard_args():
    args = _base_parse_args()
    if args.stand_ramp_seconds != 12.0:
        raise deployment.DeploymentAbort("keyboard entry uses a 12 s STAND transition")
    if any(arg.split("=")[0] in {
        "--vx", "--vy", "--wz", "--head-neck-pitch", "--head-pitch",
        "--head-yaw", "--head-roll",
    } for arg in sys.argv[1:]):
        raise deployment.DeploymentAbort("commands come from terminal keys")
    args.vx = args.vy = args.wz = 0.0
    args.head_neck_pitch = args.head_pitch = args.head_yaw = args.head_roll = 0.0
    if not any(arg.split("=")[0] == "--duration" for arg in sys.argv[1:]):
        args.duration = 0.0
    return args


deployment.parse_args = parse_keyboard_args


if __name__ == "__main__":
    raise SystemExit(deployment.main())
