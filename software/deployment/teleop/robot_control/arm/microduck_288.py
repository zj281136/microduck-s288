"""Shared 14-joint MicroDuck hardware interface for Unitree 288-series motors.

This module contains no ROS, IMU, policy, IK, or teleoperation dependencies.
It provides one synchronous bus cycle at a time; the caller owns loop timing.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .uniarm_l1_bus import MotorState, UnitreeMotorSDK


BUS_MOTOR_IDS = tuple(range(14))
# Software ceilings for explicit gain tuning, not recommended operating gains.
# Kp permits >5x the S288 identified baseline (9.03e-5).
MAX_KP_SDK = 5.0e-4
MAX_KD_SDK = 2.0e-5
POLICY_JOINT_ORDER = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)
JOINT_TO_MOTOR_ID = {
    "head_roll": 0,
    "head_yaw": 1,
    "head_pitch": 2,
    "neck_pitch": 3,
    "right_hip_yaw": 4,
    "left_hip_yaw": 5,
    "left_hip_roll": 6,
    "left_hip_pitch": 7,
    "left_knee": 8,
    "left_ankle": 9,
    "right_hip_roll": 10,
    "right_hip_pitch": 11,
    "right_knee": 12,
    "right_ankle": 13,
}
MOTOR_ID_TO_JOINT = {motor_id: name for name, motor_id in JOINT_TO_MOTOR_ID.items()}
POLICY_MOTOR_IDS = tuple(JOINT_TO_MOTOR_ID[name] for name in POLICY_JOINT_ORDER)
STAND_MODEL_POSITIONS = {
    "left_hip_yaw": 0.0,
    "left_hip_roll": -0.0907571211,
    "left_hip_pitch": -0.3896435001,
    "left_knee": 0.0418953063,
    "left_ankle": 0.4343992564,
    "neck_pitch": 0.1715470293,
    "head_pitch": 0.2636946273,
    "head_yaw": 0.0,
    "head_roll": 0.0,
    "right_hip_yaw": 0.0,
    "right_hip_roll": 0.0907571211,
    "right_hip_pitch": 0.3896435001,
    "right_knee": -0.0418953063,
    "right_ankle": -0.4343992564,
}

@dataclass(frozen=True)
class HardwareProfile:
    """Motor-specific values kept separate from shared bus/control behavior."""

    name: str
    calibration_robot: str
    default_kp_by_id: Mapping[int, float]
    default_kd_by_id: Mapping[int, float]
    effort_limit_nm: float

    def __post_init__(self) -> None:
        _validated_profile(self.default_kp_by_id, "default Kp", MAX_KP_SDK)
        _validated_profile(self.default_kd_by_id, "default Kd", MAX_KD_SDK)
        if self.effort_limit_nm <= 0.0:
            raise ValueError("effort_limit_nm must be positive")


class MicroDuck288Error(RuntimeError):
    """Base error raised by the shared MicroDuck 288-series hardware layer."""


class CalibrationError(MicroDuck288Error):
    """The calibration file is missing, malformed, or incomplete."""


class CommunicationError(MicroDuck288Error):
    """A motor exceeded the allowed consecutive communication failures."""


class MotorSafetyError(MicroDuck288Error):
    """Motor feedback exceeded a configured safety condition."""


@dataclass(frozen=True)
class JointCalibration:
    name: str
    motor_id: int
    model_home_rad: float
    encoder_home_rad: float
    direction: int


@dataclass(frozen=True)
class SafetyLimits:
    max_temperature_c: int = 70
    max_speed_rad_s: float = 7.0
    max_following_error_rad: float = 0.50
    max_consecutive_failures: int = 3

    def __post_init__(self) -> None:
        if self.max_temperature_c <= 0 or self.max_speed_rad_s <= 0:
            raise ValueError("temperature and speed limits must be positive")
        if self.max_following_error_rad <= 0 or self.max_consecutive_failures < 1:
            raise ValueError("following-error/failure limits must be positive")


@dataclass
class MotorCounters:
    success_count: int = 0
    timeout_count: int = 0
    short_frame_count: int = 0
    bad_header_count: int = 0
    bad_crc_count: int = 0
    id_mismatch_count: int = 0
    other_failure_count: int = 0
    consecutive_failures: int = 0
    max_speed_rad_s: float = 0.0
    max_torque_nm: float = 0.0
    max_temperature_c: int = 0
    max_following_error_rad: float = 0.0

    @property
    def failure_count(self) -> int:
        return (
            self.timeout_count
            + self.short_frame_count
            + self.bad_header_count
            + self.bad_crc_count
            + self.id_mismatch_count
            + self.other_failure_count
        )


@dataclass(frozen=True)
class JointState:
    name: str
    motor_id: int
    online: bool
    model_position_rad: float | None
    motor_position_rad: float | None
    model_velocity_rad_s: float | None
    model_torque_nm: float | None
    temperature_c: int | None
    voltage_v: float | None
    motor_error: int | None
    extra_flag: int | None
    read_error: str | None = None
    # Exact fields from one CRC-validated feedback packet. The legacy
    # ``motor_position_rad`` above contains OutPos, not rotor position.
    raw_pos: int | None = None
    raw_speed: int | None = None
    raw_outpos: int | None = None
    feedback_rx_s: float | None = None


@dataclass(frozen=True)
class MotorTransactionTiming:
    """Host-side timestamps for one motor request/response transaction."""

    motor_id: int
    command_tx_s: float
    feedback_rx_s: float


@dataclass(frozen=True)
class CommandCycleTiming:
    """Host-side timing for the most recent complete bus command cycle."""

    cycle_start_s: float
    cycle_end_s: float
    transactions: tuple[MotorTransactionTiming, ...]


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _validated_profile(
    values: Mapping[int, float], name: str, maximum: float
) -> dict[int, float]:
    if set(values) != set(BUS_MOTOR_IDS):
        raise ValueError(f"{name} must contain exactly motor IDs 0..13")
    result = {motor_id: float(values[motor_id]) for motor_id in BUS_MOTOR_IDS}
    if any(value < 0.0 or value > maximum for value in result.values()):
        raise ValueError(f"{name} values must be in [0, {maximum:g}]")
    return result


def scaled_gain_profile(
    profile: HardwareProfile,
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    uniform_kp: float | None = None,
    uniform_kd: float | None = None,
) -> tuple[dict[int, float], dict[int, float]]:
    if kp_scale <= 0 or kd_scale < 0:
        raise ValueError("kp_scale must be > 0 and kd_scale must be >= 0")
    base_kp = profile.default_kp_by_id if uniform_kp is None else dict.fromkeys(BUS_MOTOR_IDS, uniform_kp)
    base_kd = profile.default_kd_by_id if uniform_kd is None else dict.fromkeys(BUS_MOTOR_IDS, uniform_kd)
    kp = {motor_id: float(base_kp[motor_id]) * kp_scale for motor_id in BUS_MOTOR_IDS}
    kd = {motor_id: float(base_kd[motor_id]) * kd_scale for motor_id in BUS_MOTOR_IDS}
    return _validated_profile(kp, "Kp", MAX_KP_SDK), _validated_profile(
        kd, "Kd", MAX_KD_SDK
    )


def load_calibration(
    path: str | Path, *, expected_robot: str | None = None
) -> dict[int, JointCalibration]:
    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CalibrationError(f"calibration file not found: {resolved}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot read calibration file: {exc}") from exc
    calibration_robot = document.get("robot")
    if expected_robot is not None and calibration_robot != expected_robot:
        raise CalibrationError(
            f"calibration robot must be {expected_robot!r}, got {calibration_robot!r}"
        )
    if document.get("reference_pose") != "mujoco_reset_qpos_zero":
        raise CalibrationError(
            "calibration reference_pose must be 'mujoco_reset_qpos_zero'"
        )

    calibrations: dict[int, JointCalibration] = {}
    for item in document.get("joints", []):
        try:
            name = str(item["joint_name"])
            motor_id = int(item["motor_id"])
            encoder_home = item["encoder_home_rad"]
            direction = item["direction"]
            if JOINT_TO_MOTOR_ID.get(name) != motor_id:
                raise ValueError("joint name and motor ID do not match the fixed mapping")
            if encoder_home is None or direction not in (-1, 1):
                raise ValueError("encoder home or direction is incomplete")
            if motor_id in calibrations:
                raise ValueError(f"duplicate motor ID {motor_id}")
            calibrations[motor_id] = JointCalibration(
                name=name,
                motor_id=motor_id,
                model_home_rad=float(item["model_home_rad"]),
                encoder_home_rad=float(encoder_home) % (2.0 * math.pi),
                direction=int(direction),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"invalid calibration entry: {item!r} ({exc})") from exc
    if set(calibrations) != set(BUS_MOTOR_IDS):
        missing = sorted(set(BUS_MOTOR_IDS) - set(calibrations))
        extra = sorted(set(calibrations) - set(BUS_MOTOR_IDS))
        raise CalibrationError(f"expected IDs 0..13; missing={missing}, extra={extra}")
    return calibrations


class MicroDuck288:
    """Synchronous 14-motor interface using model-space joint coordinates."""

    def __init__(
        self,
        port: str,
        calibration_path: str | Path,
        *,
        profile: HardwareProfile,
        timeout: float = 0.003,
        kp_by_id: Mapping[int, float] | None = None,
        kd_by_id: Mapping[int, float] | None = None,
        safety: SafetyLimits | None = None,
        motor_timeout_enabled: bool = False,
        position_target_source: str = "outpos",
    ):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.profile = profile
        self.calibration_path = Path(calibration_path).expanduser().resolve()
        self.calibration = load_calibration(
            self.calibration_path, expected_robot=profile.calibration_robot
        )
        self.kp_by_id = _validated_profile(
            kp_by_id or profile.default_kp_by_id, "Kp", MAX_KP_SDK
        )
        self.kd_by_id = _validated_profile(
            kd_by_id or profile.default_kd_by_id, "Kd", MAX_KD_SDK
        )
        self.safety = safety or SafetyLimits()
        # The 288-series protocol timeout releases a stale command after
        # approximately one second.  Keep False as the historical default;
        # real robot deployment explicitly enables it.
        self.motor_timeout_enabled = bool(motor_timeout_enabled)
        if position_target_source not in ("outpos", "rotor"):
            raise ValueError("position_target_source must be outpos or rotor")
        self.position_target_source = position_target_source
        self.sdk = UnitreeMotorSDK(
            list(BUS_MOTOR_IDS), port, timeout=timeout,
            position_target_source=position_target_source,
        )
        self.sdk.capture_commands = position_target_source == "rotor"
        self.counters = {motor_id: MotorCounters() for motor_id in BUS_MOTOR_IDS}
        self.latest = {motor_id: self._offline_state(motor_id, "not_read") for motor_id in BUS_MOTOR_IDS}
        self.last_command_timing: CommandCycleTiming | None = None
        self.closed = False

    def __enter__(self) -> "MicroDuck288":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _offline_state(self, motor_id: int, reason: str) -> JointState:
        return JointState(
            name=MOTOR_ID_TO_JOINT[motor_id], motor_id=motor_id, online=False,
            model_position_rad=None, motor_position_rad=None,
            model_velocity_rad_s=None, model_torque_nm=None,
            temperature_c=None, voltage_v=None, motor_error=None, extra_flag=None,
            read_error=reason,
        )

    def model_to_motor(self, name: str, model_position_rad: float) -> float:
        motor_id = JOINT_TO_MOTOR_ID[name]
        item = self.calibration[motor_id]
        return item.encoder_home_rad + item.direction * (
            float(model_position_rad) - item.model_home_rad
        )

    def motor_to_model(self, motor_id: int, motor_position_rad: float) -> float:
        item = self.calibration[motor_id]
        return item.model_home_rad + item.direction * wrap_pi(
            float(motor_position_rad) - item.encoder_home_rad
        )

    def home_model_positions(self) -> dict[str, float]:
        """Return the MicroDuck HOME/STAND pose in model coordinates."""
        return dict(STAND_MODEL_POSITIONS)

    def calibration_reference_model_positions(self) -> dict[str, float]:
        """Return the model pose used while encoder references were captured."""
        return {item.name: item.model_home_rad for item in self.calibration.values()}

    def _state_from_feedback(self, state: MotorState) -> JointState:
        item = self.calibration[state.id]
        return JointState(
            name=item.name,
            motor_id=state.id,
            online=True,
            model_position_rad=self.motor_to_model(
                state.id, state.pos_rad if self.position_target_source == "rotor"
                else state.OutPos_rad,
            ),
            motor_position_rad=state.OutPos_rad,
            model_velocity_rad_s=item.direction * state.speed_rads,
            model_torque_nm=item.direction * state.torque_nm,
            temperature_c=max(state.temp_case, state.temp_winding),
            voltage_v=state.voltage,
            motor_error=state.MError,
            extra_flag=state.ExFlag,
            raw_pos=state.raw_pos,
            raw_speed=state.raw_speed,
            raw_outpos=state.raw_outpos,
            feedback_rx_s=state.last_update,
        )

    def _record_failure(self, motor_id: int) -> JointState:
        reason = self.sdk.last_read_error or "other"
        counters = self.counters[motor_id]
        field = {
            "timeout": "timeout_count",
            "short_frame": "short_frame_count",
            "bad_header": "bad_header_count",
            "bad_crc": "bad_crc_count",
            "id_mismatch": "id_mismatch_count",
        }.get(reason, "other_failure_count")
        setattr(counters, field, getattr(counters, field) + 1)
        counters.consecutive_failures += 1
        state = self._offline_state(motor_id, reason)
        self.latest[motor_id] = state
        try:
            self.sdk.ser.reset_input_buffer()
        except Exception:
            pass
        if counters.consecutive_failures >= self.safety.max_consecutive_failures:
            raise CommunicationError(
                f"ID {motor_id} ({MOTOR_ID_TO_JOINT[motor_id]}): "
                f"{counters.consecutive_failures} consecutive failures ({reason})"
            )
        return state

    def _record_success(
        self, state: MotorState, expected_motor_target: float | None
    ) -> JointState:
        motor_id = state.id
        counters = self.counters[motor_id]
        counters.success_count += 1
        counters.consecutive_failures = 0
        temperature = max(state.temp_case, state.temp_winding)
        counters.max_speed_rad_s = max(counters.max_speed_rad_s, abs(state.speed_rads))
        counters.max_torque_nm = max(counters.max_torque_nm, abs(state.torque_nm))
        counters.max_temperature_c = max(counters.max_temperature_c, temperature)
        if expected_motor_target is not None:
            feedback_pos = (state.pos_rad if self.position_target_source == "rotor"
                            else state.OutPos_rad)
            error = abs(wrap_pi(expected_motor_target - feedback_pos))
            counters.max_following_error_rad = max(counters.max_following_error_rad, error)
        else:
            error = 0.0

        if state.MError:
            raise MotorSafetyError(f"ID {motor_id}: MError=0x{state.MError:08X}")
        if state.ExFlag:
            raise MotorSafetyError(f"ID {motor_id}: ExFlag=0x{state.ExFlag:X}")
        if temperature > self.safety.max_temperature_c:
            raise MotorSafetyError(f"ID {motor_id}: temperature {temperature} C")
        if abs(state.speed_rads) > self.safety.max_speed_rad_s:
            raise MotorSafetyError(f"ID {motor_id}: speed {state.speed_rads:.3f} rad/s")
        if expected_motor_target is not None and error > self.safety.max_following_error_rad:
            raise MotorSafetyError(f"ID {motor_id}: following error {error:.3f} rad")

        converted = self._state_from_feedback(state)
        self.latest[motor_id] = converted
        return converted

    def _receive(self, motor_id: int, target: float | None) -> JointState:
        state = self.sdk.read(expected_motor_id=motor_id)
        if state is None:
            return self._record_failure(motor_id)
        return self._record_success(state, target)

    def _command_and_receive_with_retries(
        self,
        motor_id: int,
        target: float,
        torque: float,
    ) -> tuple[JointState, tuple[MotorTransactionTiming, ...]]:
        """Send one motor command, retrying transiently damaged replies.

        ``max_consecutive_failures`` used to be ineffective in position-command
        cycles: the first failed read returned an offline ``JointState`` and the
        deployment layer aborted before the next attempt.  Re-send the same
        command immediately so one short/CRC/header failure is tolerated while
        the existing counter still raises ``CommunicationError`` at the
        configured consecutive-failure threshold.
        """
        transactions: list[MotorTransactionTiming] = []
        while True:
            self.sdk.write_control_packet(
                motor_id=motor_id,
                pos=target,
                kp=self.kp_by_id[motor_id],
                kd=self.kd_by_id[motor_id],
                torque=torque,
                speed=0.0,
                status=1,
                timeout_enable=int(self.motor_timeout_enabled),
            )
            command_tx = time.perf_counter()
            state = self._receive(motor_id, target)
            feedback_rx = time.perf_counter()
            transactions.append(
                MotorTransactionTiming(motor_id, command_tx, feedback_rx)
            )
            if state.online:
                return state, tuple(transactions)

    def poll_zero_gain(self, retries: int = 1) -> dict[str, JointState]:
        """Read all states using non-actuating zero-gain request frames."""
        if retries < 1:
            raise ValueError("retries must be >= 1")
        result: dict[str, JointState] = {}
        for motor_id in BUS_MOTOR_IDS:
            state = None
            for _ in range(retries):
                self.sdk.write_control_packet(
                    motor_id=motor_id, pos=0.0, kp=0.0, kd=0.0,
                    torque=0.0, speed=0.0, status=1, timeout_enable=0,
                )
                state = self._receive(motor_id, None)
                if state.online:
                    break
            result[MOTOR_ID_TO_JOINT[motor_id]] = state
        return result

    def command_model_positions(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        order: Sequence[str] = POLICY_JOINT_ORDER,
        torque_feedforward: Mapping[str, float] | Sequence[float] | None = None,
    ) -> dict[str, JointState]:
        """Run one complete 14-motor command/feedback cycle.

        A sequence is interpreted in ``order`` (policy order by default). A
        mapping must contain every joint name. ``torque_feedforward`` is in
        model-space output-joint N.m; calibration direction is applied here
        before sending the motor-space command. Speed target remains zero.
        """
        if isinstance(positions, Mapping):
            if set(positions) != set(JOINT_TO_MOTOR_ID):
                raise ValueError("position mapping must contain all 14 joint names")
            model_targets = {name: float(positions[name]) for name in JOINT_TO_MOTOR_ID}
        else:
            if len(positions) != len(order) or set(order) != set(JOINT_TO_MOTOR_ID):
                raise ValueError("position sequence/order must describe all 14 joints")
            model_targets = {name: float(value) for name, value in zip(order, positions)}

        if torque_feedforward is None:
            model_torques = dict.fromkeys(JOINT_TO_MOTOR_ID, 0.0)
        elif isinstance(torque_feedforward, Mapping):
            if set(torque_feedforward) != set(JOINT_TO_MOTOR_ID):
                raise ValueError("torque-feedforward mapping must contain all 14 joint names")
            model_torques = {
                name: float(torque_feedforward[name]) for name in JOINT_TO_MOTOR_ID
            }
        else:
            if len(torque_feedforward) != len(order):
                raise ValueError("torque-feedforward sequence must match order")
            model_torques = {
                name: float(value)
                for name, value in zip(order, torque_feedforward)
            }
        if not all(math.isfinite(value) for value in model_torques.values()):
            raise ValueError("torque feed-forward must be finite")
        excessive_torque = {
            name: value
            for name, value in model_torques.items()
            if abs(value) > self.profile.effort_limit_nm
        }
        if excessive_torque:
            raise ValueError(
                f"torque feed-forward exceeds {self.profile.name} limit "
                f"{self.profile.effort_limit_nm:g} N.m: {excessive_torque}"
            )

        motor_targets = {
            JOINT_TO_MOTOR_ID[name]: self.model_to_motor(name, target)
            for name, target in model_targets.items()
        }
        result: dict[str, JointState] = {}
        transactions: list[MotorTransactionTiming] = []
        cycle_start = time.perf_counter()
        for motor_id in BUS_MOTOR_IDS:
            target = motor_targets[motor_id]
            item = self.calibration[motor_id]
            # Virtual work: q_motor = direction*q_model + constant, therefore
            # tau_motor = direction*tau_model.  This is the critical sign
            # conversion for mirrored/reversed installations.
            motor_torque = item.direction * model_torques[item.name]
            state, motor_transactions = self._command_and_receive_with_retries(
                motor_id, target, motor_torque
            )
            result[MOTOR_ID_TO_JOINT[motor_id]] = state
            transactions.extend(motor_transactions)
        self.last_command_timing = CommandCycleTiming(
            cycle_start, time.perf_counter(), tuple(transactions)
        )
        return result

    def states_in_policy_order(self) -> list[JointState]:
        return [self.latest[JOINT_TO_MOTOR_ID[name]] for name in POLICY_JOINT_ORDER]

    def safe_stop(self, rounds: int = 3) -> None:
        """Best-effort zero-gain/zero-torque command to every motor."""
        if self.closed:
            return
        for _ in range(rounds):
            for motor_id in BUS_MOTOR_IDS:
                try:
                    self.sdk.write_control_packet(
                        motor_id=motor_id, pos=0.0, kp=0.0, kd=0.0,
                        torque=0.0, speed=0.0, status=1, timeout_enable=0,
                    )
                    self.sdk.read(expected_motor_id=motor_id)
                except Exception:
                    try:
                        self.sdk.ser.reset_input_buffer()
                    except Exception:
                        pass

    def close(self) -> None:
        if self.closed:
            return
        self.safe_stop()
        self.sdk.close()
        self.closed = True
