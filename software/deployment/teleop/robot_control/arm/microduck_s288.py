"""Identified S288 profile for the shared MicroDuck hardware interface."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from .microduck_288 import (
    BUS_MOTOR_IDS,
    JOINT_TO_MOTOR_ID,
    MAX_KD_SDK,
    MAX_KP_SDK,
    MOTOR_ID_TO_JOINT,
    POLICY_JOINT_ORDER,
    POLICY_MOTOR_IDS,
    CalibrationError,
    CommandCycleTiming,
    CommunicationError,
    HardwareProfile,
    JointCalibration,
    JointState,
    MicroDuck288,
    MicroDuck288Error,
    MotorCounters,
    MotorSafetyError,
    MotorTransactionTiming,
    SafetyLimits,
    STAND_MODEL_POSITIONS as J288_STAND_MODEL_POSITIONS,
    load_calibration,
    scaled_gain_profile as _scaled_gain_profile,
    wrap_pi,
)


# The identified SDK Kp baseline is 9.03e-5. The current S288 velocity policy
# was trained at 3x Kp and 3.74e-6 Kd. The fitted M1 torque ratio belongs only
# to the simulation model and is not applied to these wire-level gains again.
IDENTIFIED_KP_SDK = 9.03e-5
KP_SCALE = 3.0
DEFAULT_KP_BY_ID = dict.fromkeys(BUS_MOTOR_IDS, IDENTIFIED_KP_SDK * KP_SCALE)
DEFAULT_KD_BY_ID = dict.fromkeys(BUS_MOTOR_IDS, 3.74e-6)
# S288 stand verified on hardware; it is the current policy's HOME pose.
S288_STAND_MODEL_POSITIONS = {
    **J288_STAND_MODEL_POSITIONS,
    "left_hip_pitch": -0.3023770375,
    "right_hip_pitch": 0.3023770375,
    "left_ankle": 0.3122262088,
    "right_ankle": -0.3122262088,
}
S288_PROFILE = HardwareProfile(
    name="s288",
    calibration_robot="microduck_s288",
    default_kp_by_id=DEFAULT_KP_BY_ID,
    default_kd_by_id=DEFAULT_KD_BY_ID,
    effort_limit_nm=0.60,
)
M1_KP_SCALE = 1.5
S288_M1_PROFILE = HardwareProfile(
    name="s288_m1_bam",
    calibration_robot="microduck_s288",
    default_kp_by_id=dict.fromkeys(
        BUS_MOTOR_IDS, IDENTIFIED_KP_SDK * M1_KP_SCALE
    ),
    default_kd_by_id=DEFAULT_KD_BY_ID,
    effort_limit_nm=0.60,
)


MicroDuckS288Error = MicroDuck288Error


def scaled_gain_profile(
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    uniform_kp: float | None = None,
    uniform_kd: float | None = None,
) -> tuple[dict[int, float], dict[int, float]]:
    return _scaled_gain_profile(
        S288_PROFILE, kp_scale, kd_scale, uniform_kp, uniform_kd
    )


def scaled_gain_profile_m1(
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    uniform_kp: float | None = None,
    uniform_kd: float | None = None,
) -> tuple[dict[int, float], dict[int, float]]:
    """Wire-level gains for policies trained with the S288 BAM M1 task."""
    return _scaled_gain_profile(
        S288_M1_PROFILE, kp_scale, kd_scale, uniform_kp, uniform_kd
    )


class MicroDuckS288(MicroDuck288):
    """S288 wrapper with an independent identified gain/effort profile."""

    PROFILE = S288_PROFILE

    def __init__(
        self,
        port: str,
        calibration_path: str | Path,
        *,
        timeout: float = 0.003,
        kp_by_id: Mapping[int, float] | None = None,
        kd_by_id: Mapping[int, float] | None = None,
        safety: SafetyLimits | None = None,
        motor_timeout_enabled: bool = False,
        position_target_source: str = "outpos",
    ):
        super().__init__(
            port,
            calibration_path,
            profile=self.PROFILE,
            timeout=timeout,
            kp_by_id=kp_by_id,
            kd_by_id=kd_by_id,
            safety=safety,
            motor_timeout_enabled=motor_timeout_enabled,
            position_target_source=position_target_source,
        )


class MicroDuckS288M1(MicroDuckS288):
    """Same S288 bus and calibration with the BAM M1 policy's 1.5x Kp."""

    PROFILE = S288_M1_PROFILE


class MicroDuckS288M1Rotor(MicroDuckS288M1):
    """M1 gains with rotor-frame commands, STAND feedback and following-error guard.

    Uses the existing joint sign/encoder-home map for BOTH observed rotor q
    and commanded rotor q. Never aligns a command using live OutPos.
    """

    def __init__(self, *args, **kwargs):
        kwargs["position_target_source"] = "rotor"
        super().__init__(*args, **kwargs)
