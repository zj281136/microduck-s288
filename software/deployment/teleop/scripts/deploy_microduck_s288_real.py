#!/usr/bin/env python3
"""Deploy V0 revision 2: 61D rotor-only, rigid-transmission PD+M1 baseline.

Requires freshly trained, contract-tagged weights. Legacy V0 is incompatible.
OutPos remains in diagnostics/safety; policy position and speed are rotor-side.
"""
import sys
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(SCRIPT_DIR.parent))
import s288_real_common as common
from robot_control.arm.microduck_s288 import MicroDuckS288M1, scaled_gain_profile_m1
from s288_v0_policy import load_v0_policy

deployment = common.deployment
deployment.MicroDuckJ288 = MicroDuckS288M1
deployment.scaled_gain_profile = scaled_gain_profile_m1
deployment.DEFAULT_POLICY = SCRIPT_DIR / "models" / "microduck_s288_v0_rotor_pd_m1.onnx"
deployment.DEFAULT_LOG_DIR = SCRIPT_DIR / "logs" / "real_imu_s288_v0_rotor_pd_m1"
deployment.OBSERVATION_MODE = "rotor_only_61d"
deployment.DEFAULT_POLICY_POSITION_SOURCE = "rotor"
deployment.load_policy = load_v0_policy
deployment.DEFAULT_STAND_RAMP_SECONDS = 12.0
from s288_stand_gate import transition as logged_stand_transition


def stand_transition(*args, **kwargs):
    return logged_stand_transition(deployment, *args, require_ready=False, **kwargs)


deployment.STAND_TRANSITION_HANDLER = stand_transition


def parse_v0_args():
    args = _base_parse_args()
    # This entry point is a matched baseline, not a gain/action tuning harness.
    if (args.kp_scale != 1 or args.kd_scale != 1 or args.action_scale != 1
            or args.head_action_scale != 1 or args.policy_qdot_delay_steps != 0
            or args.max_target_speed is not None or args.max_policy_offset is not None
            or args.friction_ff_nm != 0 or args.friction_ff_profile is not None
            or any((args.head_neck_pitch, args.head_pitch, args.head_yaw, args.head_roll))):
        raise deployment.DeploymentAbort("V0 baseline requires trained gains/actions, zero head commands, and no extra qdot delay/filter/feedforward")
    return args


_base_parse_args = deployment.parse_args
deployment.parse_args = parse_v0_args


def main():
    return deployment.main()


if __name__ == "__main__":
    raise SystemExit(main())
