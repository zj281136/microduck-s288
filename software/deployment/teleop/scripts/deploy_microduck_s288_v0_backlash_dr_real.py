#!/usr/bin/env python3
"""Deploy the independently trained V0 2--4 degree backlash DR rotor-only policy.

Mechanical play is a simulation model; hardware uses measured raw rotor q/dq.
No synthetic passive state or OutPos signal enters the 61D observation.
Reuse V0's 50 Hz loop, full M1 wire gains, HOME, target processing and guards.
"""
from pathlib import Path
import deploy_microduck_s288_real as baseline
from s288_v0_policy import load_v0_policy
from robot_control.arm.microduck_s288 import MicroDuckS288M1Rotor

SCRIPT_DIR = Path(__file__).resolve().parent
CONTRACT_ID = "s288_v0_backlash_dr_rotor_pd_m1_61d_v1"
DYNAMICS_VARIANT = "v0_backlash_dr_rotor_pd_m1"
deployment = baseline.deployment
deployment.MicroDuckJ288 = MicroDuckS288M1Rotor
deployment.DEFAULT_POLICY = SCRIPT_DIR / "models" / "microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx"
deployment.DEFAULT_LOG_DIR = SCRIPT_DIR / "logs" / "real_imu_s288_v0_backlash_dr_rotor_pd_m1"


def load_backlash_dr_policy(path: Path):
    result = load_v0_policy(
        path, expected_kp=1.3545e-4, expected_kd=3.74e-6,
        expected_contract=CONTRACT_ID, expected_dynamics_variant=DYNAMICS_VARIANT,
    )
    metadata = result[0].get_modelmeta().custom_metadata_map
    if metadata.get("s288_backlash_half_width_deg") != "1.0,2.0":
        raise deployment.DeploymentAbort("V0 backlash DR training range mismatch")
    return result


deployment.load_policy = load_backlash_dr_policy


if __name__ == "__main__":
    raise SystemExit(deployment.main())
