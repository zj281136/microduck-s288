"""J288 profile for the shared MicroDuck 288-series hardware interface."""

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
    STAND_MODEL_POSITIONS,
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
    load_calibration,
    scaled_gain_profile as _scaled_gain_profile,
    wrap_pi,
)


# Measured firmware-side profile used by the J288 training tasks. Training
# multiplies Kp (but not Kd) by 1.3 before converting through gear_ratio**2.
TRAINING_KP_SCALE = 1.3
BASE_KP_BY_ID = {
    0: 5.0e-5, 1: 5.0e-5, 2: 5.0e-5, 3: 5.0e-5,
    4: 5.0e-5, 5: 5.0e-5,
    6: 7.0e-5, 7: 6.5e-5, 8: 7.0e-5, 9: 6.5e-5,
    10: 7.0e-5, 11: 6.5e-5, 12: 7.0e-5, 13: 6.5e-5,
}
DEFAULT_KP_BY_ID = {
    motor_id: gain * TRAINING_KP_SCALE for motor_id, gain in BASE_KP_BY_ID.items()
}
DEFAULT_KD_BY_ID = {
    0: 4.0e-6, 1: 4.0e-6, 2: 3.0e-6, 3: 3.0e-6,
    4: 4.0e-6, 5: 4.0e-6, 6: 3.5e-6, 7: 4.0e-6,
    8: 4.0e-6, 9: 4.0e-6, 10: 3.5e-6, 11: 4.0e-6,
    12: 4.0e-6, 13: 4.0e-6,
}
J288_PROFILE = HardwareProfile(
    name="j288",
    calibration_robot="microduck_j288",
    default_kp_by_id=DEFAULT_KP_BY_ID,
    default_kd_by_id=DEFAULT_KD_BY_ID,
    effort_limit_nm=0.65,
)


MicroDuckJ288Error = MicroDuck288Error


def scaled_gain_profile(
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    uniform_kp: float | None = None,
    uniform_kd: float | None = None,
) -> tuple[dict[int, float], dict[int, float]]:
    return _scaled_gain_profile(
        J288_PROFILE, kp_scale, kd_scale, uniform_kp, uniform_kd
    )


class MicroDuckJ288(MicroDuck288):
    """J288-specific wrapper preserving the existing public API."""

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
    ):
        super().__init__(
            port,
            calibration_path,
            profile=J288_PROFILE,
            timeout=timeout,
            kp_by_id=kp_by_id,
            kd_by_id=kd_by_id,
            safety=safety,
            motor_timeout_enabled=motor_timeout_enabled,
        )
