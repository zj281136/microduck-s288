"""Hardware-side contract for newly trained V0 rotor-only policies."""
import math
from pathlib import Path
import deploy_microduck_j288_onnx as fixture
from robot_control.arm.microduck_s288 import S288_STAND_MODEL_POSITIONS

CONTRACT_ID = "s288_v0_rotor_pd_m1_61d_v2"
OBSERVATION_NAMES = (
    "base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,"
    "head_command,body_command"
)
_BASE_LOAD_POLICY = fixture.load_policy


def load_v0_policy(
    path: Path, *, expected_kp=1.3545e-4, expected_kd=3.74e-6,
    expected_contract=CONTRACT_ID,
    expected_dynamics_variant="rigid_rotor_pd_m1",
):
    result = _BASE_LOAD_POLICY(path, expected_obs_size=61, expected_contract=expected_contract)
    metadata = result[0].get_modelmeta().custom_metadata_map
    expected = {
        "observation_names": OBSERVATION_NAMES,
        "s288_dynamics_variant": expected_dynamics_variant,
        "s288_position_source": "rotor",
        "s288_qdot_extra_delay_steps": "0",
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise fixture.DeploymentAbort(f"V0 ONNX {key} mismatch")
    for key, value in (("s288_kp_sdk", expected_kp), ("s288_kd_sdk", expected_kd)):
        if not math.isclose(float(metadata.get(key, "nan")), value, rel_tol=1e-8):
            raise fixture.DeploymentAbort(f"V0 ONNX {key} mismatch")
    home = [float(v) for v in metadata.get("default_joint_pos", "").split(",") if v]
    expected_home = [S288_STAND_MODEL_POSITIONS[n] for n in fixture.POLICY_JOINT_ORDER]
    # mjlab base metadata serializes arrays to three decimal places.
    if len(home) != 14 or any(abs(a-b) > 0.00051 for a,b in zip(home, expected_home)):
        raise fixture.DeploymentAbort("V0 ONNX HOME mismatch")
    if result[3] != 1.0:
        raise fixture.DeploymentAbort("V0 ONNX action scale must be 1")
    return result
