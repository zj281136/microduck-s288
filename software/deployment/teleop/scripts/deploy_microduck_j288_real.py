#!/usr/bin/env python3
"""Run a 14-motor MicroDuck 288-series ONNX policy with a real IMU.

Hardware: Raspberry Pi OS Lite 64-bit, LSM6DSV16X on I2C, and one UniArmL1
USB motor controller.  IMU axes are assumed to equal robot axes: X forward,
Y left, Z up.  This remains experimental hardware control: use a fall-arrest
fixture and keep an operator at the power switch.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import signal
import sys
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(SCRIPT_DIR.parent))

from robot_control.arm.microduck_j288 import (  # noqa: E402
    JOINT_TO_MOTOR_ID,
    POLICY_JOINT_ORDER,
    MicroDuckJ288,
    MicroDuckJ288Error,
    SafetyLimits,
    STAND_MODEL_POSITIONS,
    scaled_gain_profile,
    wrap_pi,
)
from policy_qdot_delay import PolicyQdotDelay
from robot_control.arm.constants import GEAR_RATIO, K_POS, OUT_POS_RES  # noqa: E402
from robot_control.imu.lsm6dsv16x import ImuError, LSM6DSV16X  # noqa: E402
from deploy_microduck_j288_onnx import (  # noqa: E402
    ACTION_SIZE,
    JOINT_LIMITS,
    OBS_SIZE,
    POLICY_RATE_HZ,
    DeploymentAbort,
    clamp_targets,
    feedback_telemetry,
    feedback_velocity,
    load_policy,
    require_online_positions,
    smoothstep,
)


DEFAULT_CALIBRATION = SCRIPT_DIR / "microduck_j288_calibration.json"
DEFAULT_LOG_DIR = SCRIPT_DIR / "logs" / "real_imu"
DEFAULT_POLICY = SCRIPT_DIR / "models" / "microduck_j288_walk.onnx"
DEFAULT_STANDUP_POLICY: Path | None = (
    SCRIPT_DIR / "models" / "microduck_j288_standup.onnx"
)
HARDWARE_NAME = "J288"
# J288 keeps its established joint-action ramp. Hardware-specific wrappers may
# select command ramping or disable transition ramping entirely.
POLICY_RAMP_MODE = "action"
# Hardware-specific wrappers may raise this when their measured motor-speed
# envelope and training configuration support a higher target slew rate.
MAX_TARGET_SPEED_LIMIT = 7.0
DEFAULT_TARGET_SPEED: float | None = 7.0
# The original runtime filters final joint targets after home+scale*action.
# None is pass-through; a checkpoint must use the setting it was trained for.
HEAD_TARGET_LOWPASS: float | None = None
LEGS_TARGET_LOWPASS: float | None = None
# Raw policy action used as the hardware-specific transition endpoint and as
# the first last_action observation.  The default keeps the established J288
# HOME transition; an entry point may override it with a measured policy
# equilibrium.
POLICY_ENTRY_ACTION = (0.0,) * ACTION_SIZE
# Optional legacy target projection. The motor's own torque limit is separate
# from this target rewrite; hardware wrappers may leave these values at None.
PD_GAIN_OUTPUT_SCALE: float | None = None
PD_EFFORT_LIMIT_NM: float | None = None
PD_SATURATION_EFFORT_NM: float | None = None
PD_VELOCITY_LIMIT_RAD_S: float | None = None
OBSERVATION_MODE = "legacy_61"
DEFAULT_POLICY_POSITION_SOURCE = "outpos"
DEFAULT_STAND_RAMP_SECONDS = 8.0
STAND_TRANSITION_HANDLER = None
# Optional terminal command source used by a dedicated keyboard entry point.
# Existing fixed-command deployments leave this unset.
COMMAND_SOURCE = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyACM0")
    parser.add_argument("--onnx", type=Path, default=DEFAULT_POLICY)
    parser.add_argument(
        "--standup-onnx",
        type=Path,
        default=DEFAULT_STANDUP_POLICY,
        help=(
            "optional StandUp policy staged for recovery integration; "
            "disabled when omitted by a hardware-specific entry point"
        ),
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--i2c-bus", type=int, default=1)
    parser.add_argument("--i2c-address", type=lambda value: int(value, 0), default=0x6A)
    parser.add_argument("--imu-calibration-seconds", type=float, default=3.0)
    parser.add_argument("--imu-max-age", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=0.003)
    parser.add_argument("--duration", type=float, default=10.0, help="seconds; 0 runs until Ctrl+C")
    parser.add_argument("--stand-ramp-seconds", type=float, default=DEFAULT_STAND_RAMP_SECONDS)
    parser.add_argument("--policy-ramp-seconds", type=float, default=3.0)
    parser.add_argument("--action-scale", type=float, default=1.0)
    parser.add_argument(
        "--kp-scale",
        type=float,
        default=1.0,
        help="scale the current hardware Kp profile (default: 1.0)",
    )
    parser.add_argument(
        "--kd-scale",
        type=float,
        default=1.0,
        help="scale the current hardware Kd profile (default: 1.0)",
    )
    parser.add_argument(
        "--head-action-scale",
        type=float,
        default=1.0,
        help="Additional multiplier for ONNX neck/head actions (indices 5..8)",
    )
    parser.add_argument(
        "--max-policy-offset",
        type=float,
        default=None,
        help=(
            "optional extra action-offset clamp in rad; default disabled to "
            "match the original infer_policy.py action path"
        ),
    )
    parser.add_argument(
        "--max-target-speed",
        type=float,
        default=DEFAULT_TARGET_SPEED,
        help=(
            "optional commanded joint-target slew in rad/s "
            f"(default: {DEFAULT_TARGET_SPEED}, limit: {MAX_TARGET_SPEED_LIMIT:g})"
        ),
    )
    parser.add_argument(
        "--vx",
        type=float,
        default=0.05,
        help="forward velocity after STAND (default: 0.05 m/s)",
    )
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--head-neck-pitch", type=float, default=0.0)
    parser.add_argument("--head-pitch", type=float, default=0.0)
    parser.add_argument("--head-yaw", type=float, default=0.0)
    parser.add_argument("--head-roll", type=float, default=0.0)
    parser.add_argument("--max-start-travel", type=float, default=0.60)
    parser.add_argument("--max-temperature", type=int, default=70)
    parser.add_argument("--max-speed", type=float, default=7.0)
    parser.add_argument("--max-following-error", type=float, default=0.50)
    parser.add_argument("--max-consecutive-failures", type=int, default=3)
    parser.add_argument("--max-tilt-deg", type=float, default=35.0)
    parser.add_argument("--max-angular-speed", type=float, default=8.0)
    parser.add_argument("--max-inference-ms", type=float, default=12.0)
    parser.add_argument(
        "--friction-ff-nm",
        type=float,
        default=0.0,
        help=(
            "Coulomb-friction compensation magnitude per joint in output N.m; "
            "0 disables it until a supported identification test supplies a value"
        ),
    )
    parser.add_argument(
        "--friction-ff-velocity-width",
        type=float,
        default=0.10,
        help="tanh transition width for friction feed-forward (rad/s)",
    )
    parser.add_argument(
        "--friction-ff-profile",
        type=Path,
        help=(
            "optional JSON object mapping all 14 policy joint names to Coulomb "
            "compensation magnitudes in output N.m; overrides --friction-ff-nm"
        ),
    )
    parser.add_argument("--print-rate", type=float, default=1.0)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--policy-position-source", choices=("outpos", "rotor"),
        default=DEFAULT_POLICY_POSITION_SOURCE,
        help="diagnostic: policy q from output encoder or rotor position / gear; all other control unchanged",
    )
    parser.add_argument(
        "--arm-ground-test",
        action="store_true",
        help="required acknowledgement for real-IMU ground testing",
    )
    parser.add_argument("--policy-qdot-delay-steps", type=int, default=0,
                        help="extra qdot observation delay in policy ticks (50 Hz: 20 ms); motor feedback unchanged")
    args = parser.parse_args()
    if args.policy_qdot_delay_steps < 0:
        parser.error("--policy-qdot-delay-steps must be >= 0")

    if OBSERVATION_MODE in ("rotor_output_75d", "rotor_only_61d") and args.policy_position_source != "rotor":
        parser.error("rotor-side policy requires --policy-position-source rotor")

    if not args.arm_ground_test:
        parser.error("--arm-ground-test is required")
    if args.i2c_address not in (0x6A, 0x6B):
        parser.error("--i2c-address must be 0x6a or 0x6b")
    if args.timeout <= 0 or args.duration < 0 or args.print_rate <= 0:
        parser.error("timeout/print-rate must be positive and duration must be >= 0")
    if args.imu_calibration_seconds < 1.0 or not 0.02 <= args.imu_max_age <= 0.20:
        parser.error("IMU calibration must be >=1 s and max age in [0.02,0.20] s")
    if args.stand_ramp_seconds < 5.0:
        parser.error("stand ramp must be >=5 s")
    if POLICY_RAMP_MODE != "none" and args.policy_ramp_seconds < 1.0:
        parser.error("policy ramp must be >=1 s when ramping is enabled")
    if not 0.0 < args.action_scale <= 1.0:
        parser.error("--action-scale must be in (0,1]")
    if not 0.0 < args.kp_scale <= 2.0:
        parser.error("--kp-scale must be in (0,2.0]")
    max_kd_scale = 2.0 if HARDWARE_NAME == "S288" else 1.25
    if not 0.0 <= args.kd_scale <= max_kd_scale:
        parser.error(f"--kd-scale must be in [0,{max_kd_scale:g}]")
    if not 0.0 <= args.head_action_scale <= 1.0:
        parser.error("--head-action-scale must be in [0,1]")
    if args.max_policy_offset is not None and args.max_policy_offset <= 0.0:
        parser.error("--max-policy-offset must be positive when provided")
    if args.max_target_speed is not None and not 0.1 <= args.max_target_speed <= MAX_TARGET_SPEED_LIMIT:
        parser.error(
            "--max-target-speed must be in "
            f"[0.1,{MAX_TARGET_SPEED_LIMIT:g}] rad/s"
        )
    if not -0.30 <= args.vx <= 0.30 or not -0.20 <= args.vy <= 0.20:
        parser.error("command limits: vx +/-0.30 m/s, vy +/-0.20 m/s")
    if not -0.60 <= args.wz <= 0.60:
        parser.error("command limit: wz +/-0.60 rad/s")
    for value, limit, name in (
        (args.head_neck_pitch, 0.50, "head-neck-pitch"),
        (args.head_pitch, 0.50, "head-pitch"),
        (args.head_yaw, 0.70, "head-yaw"),
        (args.head_roll, 0.20, "head-roll"),
    ):
        if not -limit <= value <= limit:
            parser.error(f"--{name} must be within +/-{limit:.2f} rad")
    if not 10.0 <= args.max_tilt_deg <= 60.0:
        parser.error("--max-tilt-deg must be in [10,60]")
    if args.max_angular_speed <= 0 or args.max_inference_ms <= 0:
        parser.error("angular-speed and inference limits must be positive")
    if not 0.0 <= args.friction_ff_nm <= 0.20:
        parser.error("--friction-ff-nm must be in [0,0.20] N.m")
    if not 0.02 <= args.friction_ff_velocity_width <= 0.50:
        parser.error("--friction-ff-velocity-width must be in [0.02,0.50] rad/s")
    return args


def effective_policy_action(
    targets: np.ndarray, q_home: np.ndarray, model_action_scale: float
) -> np.ndarray:
    """Action reflected by final motor targets in training-action units."""
    return ((targets - q_home) / model_action_scale).astype(np.float32)


def lowpass_targets(
    targets: np.ndarray,
    previous: np.ndarray | None,
    head_alpha: float | None,
    legs_alpha: float | None,
) -> np.ndarray:
    """Original robotd control.rs target filter; first tick passes through."""
    if previous is None:
        return targets.copy()
    filtered = targets.copy()
    if head_alpha is not None:
        filtered[5:9] = head_alpha * targets[5:9] + (1.0 - head_alpha) * previous[5:9]
    if legs_alpha is not None:
        filtered[:5] = legs_alpha * targets[:5] + (1.0 - legs_alpha) * previous[:5]
        filtered[9:] = legs_alpha * targets[9:] + (1.0 - legs_alpha) * previous[9:]
    return filtered


def clamp_targets_by_pd_envelope(
    targets: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    feedforward: np.ndarray,
    stiffness: np.ndarray,
    damping: np.ndarray,
    effort_limit: float,
    saturation_effort: float,
    velocity_limit: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Limit position targets to the same PD torque envelope used in training."""
    targets = np.asarray(targets, dtype=np.float32)
    positions = np.asarray(positions, dtype=np.float32)
    velocities = np.asarray(velocities, dtype=np.float32)
    feedforward = np.asarray(feedforward, dtype=np.float32)
    stiffness = np.asarray(stiffness, dtype=np.float32)
    damping = np.asarray(damping, dtype=np.float32)
    if np.any(stiffness <= 0.0):
        raise DeploymentAbort("PD output stiffness must be positive")

    requested = stiffness * (targets - positions) - damping * velocities + feedforward
    corner_velocity = velocity_limit * (1.0 + effort_limit / saturation_effort)
    clipped_velocity = np.clip(velocities, -corner_velocity, corner_velocity)
    upper = np.minimum(
        saturation_effort * (1.0 - clipped_velocity / velocity_limit),
        effort_limit,
    )
    lower = np.maximum(
        saturation_effort * (-1.0 - clipped_velocity / velocity_limit),
        -effort_limit,
    )
    limited_effort = np.clip(requested, lower, upper)
    limited_targets = positions + (
        limited_effort - feedforward + damping * velocities
    ) / stiffness
    # The effort projection normally moves a target toward measured position,
    # but retain the exact MJCF hard limits as the final mechanical guard.
    for index, name in enumerate(POLICY_JOINT_ORDER):
        low, high = JOINT_LIMITS[name]
        limited_targets[index] = np.clip(limited_targets[index], low, high)
    count = int(np.count_nonzero(np.abs(requested - limited_effort) > 1.0e-6))
    return (
        limited_targets.astype(np.float32),
        requested.astype(np.float32),
        limited_effort.astype(np.float32),
        count,
    )


def friction_feedforward(
    target_velocity: np.ndarray,
    measured_velocity: np.ndarray,
    magnitude_nm: np.ndarray,
    velocity_width: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Compensate friction in the intended model-space direction of motion.

    While a joint is moving, compensation follows measured motion so a delayed
    actuator cannot receive a prematurely reversed friction torque.  Near
    standstill it follows desired motion to overcome breakaway friction.
    ``direction_velocity`` is returned for unambiguous logging.
    """
    velocity = np.asarray(target_velocity, dtype=np.float32)
    measured = np.asarray(measured_velocity, dtype=np.float32)
    magnitude = np.asarray(magnitude_nm, dtype=np.float32)
    moving = np.abs(measured) >= 0.03
    direction_velocity = np.where(moving, measured, velocity)
    torque = magnitude * np.tanh(direction_velocity / velocity_width)
    return torque.astype(np.float32), direction_velocity.astype(np.float32)


def load_friction_profile(args: argparse.Namespace) -> np.ndarray:
    if args.friction_ff_profile is None:
        return np.full(ACTION_SIZE, args.friction_ff_nm, dtype=np.float32)
    path = args.friction_ff_profile.expanduser().resolve()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentAbort(f"cannot read friction FF profile {path}: {exc}") from exc
    if not isinstance(document, dict) or set(document) != set(POLICY_JOINT_ORDER):
        raise DeploymentAbort("friction FF profile must contain exactly all 14 joint names")
    profile = np.asarray([document[name] for name in POLICY_JOINT_ORDER], dtype=np.float32)
    if not np.all(np.isfinite(profile)) or np.any(profile < 0.0) or np.any(profile > 0.20):
        raise DeploymentAbort("friction FF profile values must be finite and in [0,0.20] N.m")
    return profile


def raw_protocol_feedback(states: dict) -> dict[str, list[int | float | None]]:
    """Keep unscaled rotor/output fields together with their packet RX time.

    All four values for one motor come from the same CRC-validated feedback
    packet. The deployment observation continues to use its existing q/qdot.
    """
    return {
        "raw_pos": [states[name].raw_pos for name in POLICY_JOINT_ORDER],
        "raw_speed": [states[name].raw_speed for name in POLICY_JOINT_ORDER],
        "raw_outpos": [states[name].raw_outpos for name in POLICY_JOINT_ORDER],
        "raw_rx_s": [states[name].feedback_rx_s for name in POLICY_JOINT_ORDER],
    }


def s288_position_views(robot, states: dict) -> dict[str, list[float]]:
    """Convert packet fields to output-equivalent radians and calibrated model q."""
    rotor, rotor_speed, outpos, delta, rotor_model, outpos_model = ([] for _ in range(6))
    for name in POLICY_JOINT_ORDER:
        state = states[name]
        if state.raw_pos is None or state.raw_speed is None or state.raw_outpos is None:
            raise DeploymentAbort(f"{name}: missing raw rotor/output feedback")
        q_rotor = float(state.raw_pos) / K_POS / GEAR_RATIO
        dq_rotor = float(state.raw_speed) / 2.56 * 2.0 * math.pi / GEAR_RATIO
        q_out = float(state.raw_outpos) / OUT_POS_RES * 2.0 * math.pi
        rotor.append(q_rotor)
        rotor_speed.append(dq_rotor)
        outpos.append(q_out)
        delta.append(q_out - q_rotor)
        rotor_model.append(robot.motor_to_model(state.motor_id, q_rotor))
        # model_position_rad follows the selected control frame. Decode OutPos
        # explicitly so diagnostics still report output position in rotor mode.
        outpos_model.append(robot.motor_to_model(state.motor_id, q_out))
    return {
        "q_rotor_equiv_rad": rotor,
        "qdot_rotor_equiv_rad_s": rotor_speed,
        "q_out_rad": outpos,
        "delta_out_minus_rotor_rad": delta,
        "q_rotor_model_rad": rotor_model,
        "q_out_model_rad": outpos_model,
    }


def benchmark_policy(session, input_name: str, output_name: str, maximum_ms: float) -> float:
    observation = np.zeros((1, OBS_SIZE), dtype=np.float32)
    elapsed: list[float] = []
    for _ in range(30):
        started = time.perf_counter()
        result = session.run([output_name], {input_name: observation})[0]
        elapsed.append((time.perf_counter() - started) * 1000.0)
        if result.shape != (1, ACTION_SIZE) or not np.all(np.isfinite(result)):
            raise DeploymentAbort("ONNX benchmark returned an invalid action")
    p95 = float(np.percentile(np.asarray(elapsed[5:]), 95))
    if p95 > maximum_ms:
        raise DeploymentAbort(
            f"ONNX inference too slow on this Pi: p95={p95:.2f} ms > {maximum_ms:.2f} ms"
        )
    return p95


def validate_imu(sample, maximum_tilt_deg: float, maximum_angular_speed: float) -> float:
    gravity = np.asarray(sample.projected_gravity, dtype=np.float32)
    angular_velocity = np.asarray(sample.angular_velocity_rad_s, dtype=np.float32)
    if not np.all(np.isfinite(gravity)) or not np.all(np.isfinite(angular_velocity)):
        raise DeploymentAbort("non-finite IMU observation")
    gravity_norm = float(np.linalg.norm(gravity))
    if not 0.98 <= gravity_norm <= 1.02:
        raise DeploymentAbort(f"invalid projected-gravity norm {gravity_norm:.3f}")
    tilt_deg = math.degrees(math.acos(float(np.clip(-gravity[2], -1.0, 1.0))))
    if tilt_deg > maximum_tilt_deg:
        raise DeploymentAbort(f"body tilt {tilt_deg:.1f} deg exceeds limit")
    angular_speed = float(np.linalg.norm(angular_velocity))
    if angular_speed > maximum_angular_speed:
        raise DeploymentAbort(f"body angular speed {angular_speed:.2f} rad/s exceeds limit")
    return tilt_deg


def transition_to_stand_with_imu(
    robot: MicroDuckJ288,
    imu: LSM6DSV16X,
    q_start: np.ndarray,
    q_stand: np.ndarray,
    duration: float,
    imu_max_age: float,
    max_tilt_deg: float,
    max_angular_speed: float,
):
    q_stand = np.asarray(q_stand, dtype=np.float32)
    if q_stand.shape != (ACTION_SIZE,) or not np.all(np.isfinite(q_stand)):
        raise DeploymentAbort("invalid policy-entry stand target")
    travel = np.asarray(
        [wrap_pi(float(target - start)) for target, start in zip(q_stand, q_start)],
        dtype=np.float32,
    )
    started = next_tick = time.perf_counter()
    states = None
    while True:
        phase = min(1.0, (time.perf_counter() - started) / duration)
        validate_imu(imu.latest(imu_max_age), max_tilt_deg, max_angular_speed)
        states = robot.command_model_positions(
            (q_start + smoothstep(phase) * travel).tolist()
        )
        if phase >= 1.0:
            return states
        next_tick += 1.0 / POLICY_RATE_HZ
        time.sleep(max(0.0, next_tick - time.perf_counter()))


def run(args: argparse.Namespace) -> int:
    friction_magnitude = load_friction_profile(args)
    session, input_name, output_name, model_action_scale, onnx_path = load_policy(args.onnx)
    inference_p95 = benchmark_policy(
        session, input_name, output_name, args.max_inference_ms
    )
    standup_path = None
    standup_inference_p95 = None
    if args.standup_onnx is not None:
        # Validate the staged recovery model at startup even though automatic
        # fall recovery is intentionally not armed by this script yet.
        (
            standup_session,
            standup_input,
            standup_output,
            standup_action_scale,
            standup_path,
        ) = load_policy(args.standup_onnx)
        standup_inference_p95 = benchmark_policy(
            standup_session, standup_input, standup_output, args.max_inference_ms
        )
        if not math.isclose(
            standup_action_scale,
            model_action_scale,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise DeploymentAbort(
                "Velocity and StandUp ONNX action scales differ: "
                f"{model_action_scale} vs {standup_action_scale}"
            )
    log_path = args.log_file
    if log_path is None:
        log_path = DEFAULT_LOG_DIR / f"real_imu_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path = log_path.expanduser().resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    command = np.asarray([args.vx, args.vy, args.wz], dtype=np.float32)
    head_command = np.asarray(
        [args.head_neck_pitch, args.head_pitch, args.head_yaw, args.head_roll],
        dtype=np.float32,
    )
    body_command = np.zeros(6, dtype=np.float32)
    q_home = np.asarray(
        [STAND_MODEL_POSITIONS[name] for name in POLICY_JOINT_ORDER], dtype=np.float32
    )
    entry_action = np.asarray(POLICY_ENTRY_ACTION, dtype=np.float32)
    if entry_action.shape != (ACTION_SIZE,) or not np.all(np.isfinite(entry_action)):
        raise DeploymentAbort("invalid POLICY_ENTRY_ACTION")
    entry_applied_action = entry_action.copy()
    entry_applied_action[5:9] *= args.head_action_scale
    entry_offsets = entry_applied_action * model_action_scale * args.action_scale
    entry_targets, _ = clamp_targets(entry_offsets, args.max_policy_offset)
    safety = SafetyLimits(
        max_temperature_c=args.max_temperature,
        max_speed_rad_s=args.max_speed,
        max_following_error_rad=args.max_following_error,
        max_consecutive_failures=args.max_consecutive_failures,
    )
    try:
        kp_by_id, kd_by_id = scaled_gain_profile(args.kp_scale, args.kd_scale)
    except ValueError as exc:
        raise DeploymentAbort(f"invalid {HARDWARE_NAME} gain scaling: {exc}") from exc
    pd_envelope_values = (
        PD_GAIN_OUTPUT_SCALE,
        PD_EFFORT_LIMIT_NM,
        PD_SATURATION_EFFORT_NM,
        PD_VELOCITY_LIMIT_RAD_S,
    )
    if any(value is not None for value in pd_envelope_values) and not all(
        value is not None for value in pd_envelope_values
    ):
        raise DeploymentAbort("incomplete PD torque-envelope configuration")
    pd_envelope_enabled = all(value is not None for value in pd_envelope_values)
    if pd_envelope_enabled:
        assert PD_GAIN_OUTPUT_SCALE is not None
        pd_stiffness = np.asarray(
            [
                kp_by_id[JOINT_TO_MOTOR_ID[name]] * PD_GAIN_OUTPUT_SCALE
                for name in POLICY_JOINT_ORDER
            ],
            dtype=np.float32,
        )
        pd_damping = np.asarray(
            [
                kd_by_id[JOINT_TO_MOTOR_ID[name]] * PD_GAIN_OUTPUT_SCALE
                for name in POLICY_JOINT_ORDER
            ],
            dtype=np.float32,
        )
    else:
        pd_stiffness = pd_damping = np.zeros(ACTION_SIZE, dtype=np.float32)

    print("EXPERIMENTAL REAL-IMU DEPLOYMENT")
    print("Use a loose fall-arrest fixture; keep feet on the ground and power switch reachable.")
    print(f"ONNX: {onnx_path}")
    if standup_path is not None and standup_inference_p95 is not None:
        print(
            f"StandUp ONNX (validated, not auto-armed): {standup_path} "
            f"(p95={standup_inference_p95:.2f} ms)"
        )
    else:
        print("StandUp ONNX: disabled")
    print(f"ONNX benchmark p95: {inference_p95:.2f} ms")
    print(f"command vx/vy/wz = {args.vx:+.3f}/{args.vy:+.3f}/{args.wz:+.3f}")
    print(f"policy position source = {args.policy_position_source}")
    print(
        "policy-entry stand: "
        f"max offset={np.max(np.abs(entry_targets - q_home)):.3f} rad"
    )
    if POLICY_RAMP_MODE == "none":
        print("policy ramp = disabled")
    else:
        print(
            f"policy ramp = {POLICY_RAMP_MODE} over "
            f"{args.policy_ramp_seconds:.1f} s"
        )
    print(f"head action scale = {args.head_action_scale:.2f}")
    print(
        f"{HARDWARE_NAME} gain scales: "
        f"Kp={args.kp_scale:.3f}, Kd={args.kd_scale:.3f}"
    )
    if pd_envelope_enabled:
        print(
            "PD torque envelope: "
            f"continuous={PD_EFFORT_LIMIT_NM:.3f} N.m, "
            f"saturation={PD_SATURATION_EFFORT_NM:.3f} N.m, "
            f"velocity={PD_VELOCITY_LIMIT_RAD_S:.1f} rad/s"
        )
    print(
        f"friction FF: [{friction_magnitude.min():.4f}, "
        f"{friction_magnitude.max():.4f}] N.m, "
        f"velocity width={args.friction_ff_velocity_width:.3f} rad/s"
    )
    print("Keep the robot upright and completely still during IMU calibration.")

    imu = robot = None
    abort_snapshot = None
    imu_sample = None
    states = None
    feedback_completed = None
    previous_targets = None
    started = None
    try:
        imu = LSM6DSV16X(args.i2c_bus, args.i2c_address)
        imu.start(args.imu_calibration_seconds)
        time.sleep(0.05)
        first_imu = imu.latest(args.imu_max_age)
        imu_sample = first_imu
        initial_tilt = validate_imu(first_imu, 12.0, args.max_angular_speed)
        print(
            f"IMU ready: bias={imu.gyro_bias.tolist()}, initial tilt={initial_tilt:.1f} deg, "
            f"temperature={first_imu.temperature_c:.1f} C"
        )
        # --arm-ground-test is the one-shot operator acknowledgement.  Do not
        # stop for another terminal prompt: a solo operator needs both hands at
        # the robot and power switch once the process has been launched.
        print("Armed. Automatic STAND transition starts in 2 seconds...", flush=True)
        time.sleep(2.0)

        robot = MicroDuckJ288(
            args.port,
            args.calibration,
            timeout=args.timeout,
            kp_by_id=kp_by_id,
            kd_by_id=kd_by_id,
            safety=safety,
            motor_timeout_enabled=True,
        )
        print(f"position target source = {robot.position_target_source}", flush=True)
        states = robot.poll_zero_gain(retries=3)
        feedback_completed = time.monotonic()
        q_start = require_online_positions(states)
        entry_travel = np.asarray(
            [wrap_pi(float(target - start)) for target, start in zip(entry_targets, q_start)]
        )
        entry_index = int(np.argmax(np.abs(entry_travel)))
        if abs(entry_travel[entry_index]) > args.max_start_travel:
            raise DeploymentAbort(
                f"start-to-policy-entry travel exceeds {args.max_start_travel:.3f} rad: "
                f"{POLICY_JOINT_ORDER[entry_index]}={entry_travel[entry_index]:+.3f}"
            )
        print(
            "Moving smoothly to policy-entry STAND over "
            f"{args.stand_ramp_seconds:.1f} s..."
        )
        stand_handler = STAND_TRANSITION_HANDLER or transition_to_stand_with_imu
        stand_kwargs = {"log_path": log_path} if STAND_TRANSITION_HANDLER else {}
        states = stand_handler(
            robot,
            imu,
            q_start,
            entry_targets,
            args.stand_ramp_seconds,
            args.imu_max_age,
            args.max_tilt_deg,
            args.max_angular_speed,
            **stand_kwargs,
        )
        feedback_completed = time.monotonic()
        imu_sample = imu.latest(args.imu_max_age)
        validate_imu(imu_sample, args.max_tilt_deg, args.max_angular_speed)
        print(
            "STAND reached. Starting 50 Hz real-IMU policy automatically: "
            f"vx={args.vx:+.3f}, vy={args.vy:+.3f}, wz={args.wz:+.3f}."
        )
        if COMMAND_SOURCE is not None:
            COMMAND_SOURCE.start()

        # Training observes the previous raw network output, before target
        # scaling, slew limiting, joint limiting, or actuator dynamics.
        qdot_delay = PolicyQdotDelay(args.policy_qdot_delay_steps)
        log_path.with_suffix(".metadata.json").write_text(json.dumps({
            "arguments": vars(args), "joint_order": POLICY_JOINT_ORDER,
            "policy_frequency_hz": POLICY_RATE_HZ,
            "motor_command_cycle_frequency_hz": POLICY_RATE_HZ,
            "firmware_pd_frequency_hz": None,
            "gear_ratio": GEAR_RATIO, "kp_by_id": kp_by_id, "kd_by_id": kd_by_id,
            "position_target_source": robot.position_target_source,
            "checkpoint_path": str(args.onnx.resolve()),
            "onnx_sha256": hashlib.sha256(args.onnx.read_bytes()).hexdigest(),
            "training_checkpoint_path": session.get_modelmeta().custom_metadata_map.get("run_path"),
            "policy_qdot_delay_steps": args.policy_qdot_delay_steps,
            "max_speed": args.max_speed, "kp_scale": args.kp_scale, "kd_scale": args.kd_scale,
            "clock": "perf_counter seconds relative to policy start; per-joint RX/TX",
            "qdot_frame": "calibrated model rad/s; protocol scale and direction applied",
            "transmitted_target_note": (
                "q_wire_target_rotor_model is decoded from captured, quantized wire target"
                if robot.sdk.capture_commands else
                "q_target is submitted to bus; no wire-quantized target telemetry available"
            ),
        }, default=str, indent=2), encoding="utf-8")
        last_action = entry_action.copy()
        previous_targets = entry_targets.copy()
        previous_filter_targets = None
        started = next_tick = time.perf_counter()
        next_print = started + 1.0 / args.print_rate
        rounds = overruns = offset_clamps = limit_clamps = slew_clamps = 0
        effort_clamps = 0
        max_round_ms = max_inference_ms = 0.0
        fields = (
            "gyro_x", "gyro_y", "gyro_z", "gravity_x", "gravity_y", "gravity_z",
            "vx", "vy", "yaw_rate",
            "timestamp_imu", "timestamp_feedback_rx", "timestamp_policy_tick",
            "timestamp_inference_start", "timestamp_inference_end", "timestamp_command_tx",
            "qdot_rotor_raw_current", "qdot_rotor_policy_obs", "qdot_policy_source_rx_s",
            "last_action", "requested_q_target", "head_command", "body_command",
            "policy_qdot_delay_steps", "q_rotor", "reported_torque",
            "elapsed_s", "loop_start_s", "observation_ready_s",
            "inference_start_s", "inference_end_s", "command_cycle_start_s",
            "command_cycle_end_s", "q_feedback_rx_s", "command_tx_s", "feedback_rx_s",
            "round_ms", "inference_ms", "bus_ms", "tilt_deg", "imu_gyro",
            "projected_gravity", "q_feedback", "dq_feedback", "torque_feedback",
            "feedback_age_s", "q_feedback_after_command", "dq_feedback_after_command",
            "torque_feedback_after_command", "temperature_c", "voltage_v",
            "policy_command", "raw_action", "applied_action", "q_target", "target_velocity",
            "friction_direction_velocity", "friction_ff_nm",
            "estimated_requested_torque_nm", "estimated_limited_torque_nm",
            "raw_pos_feedback", "raw_speed_feedback", "raw_outpos_feedback",
            "raw_feedback_rx_s", "raw_pos_after_command",
            "raw_speed_after_command", "raw_outpos_after_command",
            "raw_after_command_rx_s",
            "policy_position_source", "q_policy_obs",
            "position_target_source", "wire_target_raw", "q_wire_target_rotor_model",
            "output_delta_obs",
            "q_rotor_equiv_rad", "qdot_rotor_equiv_rad_s", "q_out_rad",
            "delta_out_minus_rotor_rad", "q_rotor_model_rad", "q_out_model_rad",
        )
        with log_path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=fields)
            writer.writeheader()
            while args.duration == 0 or time.perf_counter() - started < args.duration:
                round_started = time.perf_counter()
                elapsed = round_started - started
                if COMMAND_SOURCE is not None:
                    keyboard_command = COMMAND_SOURCE.poll()
                    command[:] = (keyboard_command.vx, keyboard_command.vy,
                                  keyboard_command.wz)
                    head_command[:] = keyboard_command.head
                prior_timing = robot.last_command_timing
                imu_sample = imu.latest(args.imu_max_age)
                tilt_deg = validate_imu(
                    imu_sample, args.max_tilt_deg, args.max_angular_speed
                )
                q = require_online_positions(states)
                dq = feedback_velocity(states)
                torque, temperature, voltage = feedback_telemetry(states)
                protocol_before = raw_protocol_feedback(states)
                position_views = s288_position_views(robot, states)
                q_policy = (
                    np.asarray(position_views["q_rotor_model_rad"], dtype=np.float32)
                    if args.policy_position_source == "rotor" else q
                )
                output_delta_obs = (
                    np.asarray(position_views["q_out_model_rad"], dtype=np.float32)
                    - np.asarray(position_views["q_rotor_model_rad"], dtype=np.float32)
                )
                ramp = smoothstep(elapsed / args.policy_ramp_seconds)
                if POLICY_RAMP_MODE == "command":
                    policy_command = command * ramp
                    action_ramp = 1.0
                elif POLICY_RAMP_MODE == "action":
                    policy_command = command
                    action_ramp = ramp
                elif POLICY_RAMP_MODE == "none":
                    policy_command = command
                    action_ramp = 1.0
                else:
                    raise DeploymentAbort(f"invalid policy ramp mode: {POLICY_RAMP_MODE}")
                current_qdot_rx = [states[name].feedback_rx_s for name in POLICY_JOINT_ORDER]
                qdot_obs, qdot_source_rx = qdot_delay.sample(dq, current_qdot_rx)
                last_action_obs = last_action.copy()
                observation_parts = [
                    imu_sample.angular_velocity_rad_s,
                    imu_sample.projected_gravity,
                    q_policy - q_home,
                    qdot_obs,
                    last_action_obs,
                    policy_command,
                    head_command,
                    body_command,
                ]
                if OBSERVATION_MODE == "rotor_output_75d":
                    observation_parts.append(output_delta_obs)
                observation = np.concatenate(observation_parts).astype(np.float32)
                observation_ready = time.perf_counter()
                if observation.shape != (OBS_SIZE,) or not np.all(np.isfinite(observation)):
                    raise DeploymentAbort(f"invalid {OBS_SIZE}D observation")

                inference_started = time.perf_counter()
                action = session.run(
                    [output_name], {input_name: observation.reshape(1, OBS_SIZE)}
                )[0].reshape(-1).astype(np.float32)
                inference_ms = (time.perf_counter() - inference_started) * 1000.0
                inference_ended = time.perf_counter()
                max_inference_ms = max(max_inference_ms, inference_ms)
                if inference_ms > args.max_inference_ms:
                    raise DeploymentAbort(f"ONNX inference took {inference_ms:.2f} ms")
                if action.shape != (ACTION_SIZE,) or not np.all(np.isfinite(action)):
                    raise DeploymentAbort("invalid ONNX action")

                applied_action = action.copy()
                applied_action[5:9] *= args.head_action_scale
                raw_offsets = (
                    applied_action * model_action_scale * args.action_scale * action_ramp
                )
                requested_q_target = q_home + raw_offsets
                if args.max_policy_offset is not None:
                    offset_clamps += int(
                        np.count_nonzero(np.abs(raw_offsets) > args.max_policy_offset)
                    )
                    raw_offsets = np.clip(
                        raw_offsets, -args.max_policy_offset, args.max_policy_offset
                    )
                # robotd filters home+scale*action before the safety layer.
                filtered_targets = lowpass_targets(
                    q_home + raw_offsets, previous_filter_targets,
                    HEAD_TARGET_LOWPASS, LEGS_TARGET_LOWPASS,
                )
                previous_filter_targets = filtered_targets.copy()
                targets, clamped = clamp_targets(filtered_targets - q_home)
                limit_clamps += clamped
                if args.max_target_speed is not None:
                    max_step = args.max_target_speed / POLICY_RATE_HZ
                    step = targets - previous_targets
                    slew_clamps += int(np.count_nonzero(np.abs(step) > max_step))
                    targets = previous_targets + np.clip(step, -max_step, max_step)
                target_velocity = (targets - previous_targets) * POLICY_RATE_HZ
                friction_ff, friction_direction_velocity = friction_feedforward(
                    target_velocity,
                    dq,
                    friction_magnitude,
                    args.friction_ff_velocity_width,
                )
                estimated_requested_torque = np.zeros(ACTION_SIZE, dtype=np.float32)
                estimated_limited_torque = np.zeros(ACTION_SIZE, dtype=np.float32)
                if pd_envelope_enabled:
                    assert PD_EFFORT_LIMIT_NM is not None
                    assert PD_SATURATION_EFFORT_NM is not None
                    assert PD_VELOCITY_LIMIT_RAD_S is not None
                    (
                        targets,
                        estimated_requested_torque,
                        estimated_limited_torque,
                        clamped_efforts,
                    ) = clamp_targets_by_pd_envelope(
                        targets,
                        q,
                        dq,
                        friction_ff,
                        pd_stiffness,
                        pd_damping,
                        PD_EFFORT_LIMIT_NM,
                        PD_SATURATION_EFFORT_NM,
                        PD_VELOCITY_LIMIT_RAD_S,
                    )
                    effort_clamps += clamped_efforts
                    target_velocity = (targets - previous_targets) * POLICY_RATE_HZ
                states = robot.command_model_positions(
                    targets.tolist(), torque_feedforward=friction_ff.tolist()
                )
                feedback_completed = time.monotonic()
                command_timing = robot.last_command_timing
                if command_timing is None:
                    raise DeploymentAbort("missing command-cycle timing")
                q_after = require_online_positions(states)
                dq_after = feedback_velocity(states)
                torque_after, _, _ = feedback_telemetry(states)
                protocol_after = raw_protocol_feedback(states)
                wire_commands = getattr(robot.sdk, "last_control_commands", {})
                wire_raw = [
                    wire_commands.get(JOINT_TO_MOTOR_ID[name], {}).get("wire_target_raw")
                    for name in POLICY_JOINT_ORDER
                ]
                wire_q = [
                    robot.motor_to_model(JOINT_TO_MOTOR_ID[name], raw / K_POS / GEAR_RATIO)
                    if raw is not None else None
                    for name, raw in zip(POLICY_JOINT_ORDER, wire_raw)
                ]
                command_tx = {
                    item.motor_id: item.command_tx_s - started
                    for item in command_timing.transactions
                }
                feedback_rx = {
                    item.motor_id: item.feedback_rx_s - started
                    for item in command_timing.transactions
                }
                command_tx_policy = [
                    command_tx[JOINT_TO_MOTOR_ID[name]]
                    for name in POLICY_JOINT_ORDER
                ]
                feedback_rx_policy = [
                    feedback_rx[JOINT_TO_MOTOR_ID[name]]
                    for name in POLICY_JOINT_ORDER
                ]
                prior_feedback_rx = {
                    item.motor_id: item.feedback_rx_s
                    for item in prior_timing.transactions
                } if prior_timing is not None else {}
                feedback_age_policy = [
                    round_started - prior_feedback_rx[JOINT_TO_MOTOR_ID[name]]
                    for name in POLICY_JOINT_ORDER
                ]
                q_feedback_rx_policy = [
                    prior_feedback_rx[JOINT_TO_MOTOR_ID[name]] - started
                    for name in POLICY_JOINT_ORDER
                ]
                previous_targets = targets
                last_action = action.copy()

                round_ms = (time.perf_counter() - round_started) * 1000.0
                max_round_ms = max(max_round_ms, round_ms)
                rounds += 1
                writer.writerow(
                    {
                        "gyro_x": float(imu_sample.angular_velocity_rad_s[0]),
                        "gyro_y": float(imu_sample.angular_velocity_rad_s[1]),
                        "gyro_z": float(imu_sample.angular_velocity_rad_s[2]),
                        "gravity_x": float(imu_sample.projected_gravity[0]),
                        "gravity_y": float(imu_sample.projected_gravity[1]),
                        "gravity_z": float(imu_sample.projected_gravity[2]),
                        "vx": float(policy_command[0]), "vy": float(policy_command[1]),
                        "yaw_rate": float(policy_command[2]),
                        "timestamp_imu": imu_sample.timestamp - started,
                        "timestamp_feedback_rx": json.dumps(q_feedback_rx_policy),
                        "timestamp_policy_tick": elapsed,
                        "timestamp_inference_start": inference_started - started,
                        "timestamp_inference_end": inference_ended - started,
                        "timestamp_command_tx": json.dumps(command_tx_policy),
                        "qdot_rotor_raw_current": json.dumps(dq.tolist()),
                        "qdot_rotor_policy_obs": json.dumps(observation[20:34].tolist()),
                        "qdot_policy_source_rx_s": json.dumps((qdot_source_rx - started).tolist()),
                        "last_action": json.dumps(last_action_obs.tolist()),
                        "requested_q_target": json.dumps(requested_q_target.tolist()),
                        "head_command": json.dumps(head_command.tolist()),
                        "body_command": json.dumps(body_command.tolist()),
                        "policy_qdot_delay_steps": args.policy_qdot_delay_steps,
                        "q_rotor": json.dumps(position_views["q_rotor_model_rad"]),
                        "reported_torque": json.dumps(torque.tolist()),
                        "elapsed_s": f"{elapsed:.6f}",
                        "loop_start_s": f"{round_started - started:.6f}",
                        "observation_ready_s": f"{observation_ready - started:.6f}",
                        "inference_start_s": f"{inference_started - started:.6f}",
                        "inference_end_s": f"{inference_ended - started:.6f}",
                        "command_cycle_start_s": f"{command_timing.cycle_start_s - started:.6f}",
                        "command_cycle_end_s": f"{command_timing.cycle_end_s - started:.6f}",
                        "q_feedback_rx_s": json.dumps(q_feedback_rx_policy),
                        "command_tx_s": json.dumps(command_tx_policy),
                        "feedback_rx_s": json.dumps(feedback_rx_policy),
                        "round_ms": f"{round_ms:.4f}",
                        "inference_ms": f"{inference_ms:.4f}",
                        "bus_ms": f"{(command_timing.cycle_end_s-command_timing.cycle_start_s)*1000.0:.4f}",
                        "tilt_deg": f"{tilt_deg:.3f}",
                        "imu_gyro": json.dumps(imu_sample.angular_velocity_rad_s.tolist()),
                        "projected_gravity": json.dumps(imu_sample.projected_gravity.tolist()),
                        "q_feedback": json.dumps(q.tolist()),
                        "policy_position_source": args.policy_position_source,
                        "position_target_source": robot.position_target_source,
                        "wire_target_raw": json.dumps(wire_raw),
                        "q_wire_target_rotor_model": json.dumps(wire_q),
                        "q_policy_obs": json.dumps(q_policy.tolist()),
                        "output_delta_obs": json.dumps(output_delta_obs.tolist())
                        if OBSERVATION_MODE == "rotor_output_75d" else "",
                        "q_rotor_equiv_rad": json.dumps(position_views["q_rotor_equiv_rad"]),
                        "qdot_rotor_equiv_rad_s": json.dumps(position_views["qdot_rotor_equiv_rad_s"]),
                        "q_out_rad": json.dumps(position_views["q_out_rad"]),
                        "delta_out_minus_rotor_rad": json.dumps(position_views["delta_out_minus_rotor_rad"]),
                        "q_rotor_model_rad": json.dumps(position_views["q_rotor_model_rad"]),
                        "q_out_model_rad": json.dumps(position_views["q_out_model_rad"]),
                        "dq_feedback": json.dumps(dq.tolist()),
                        "torque_feedback": json.dumps(torque.tolist()),
                        "feedback_age_s": json.dumps(feedback_age_policy),
                        "q_feedback_after_command": json.dumps(q_after.tolist()),
                        "dq_feedback_after_command": json.dumps(dq_after.tolist()),
                        "torque_feedback_after_command": json.dumps(torque_after.tolist()),
                        "temperature_c": json.dumps(temperature.tolist()),
                        "voltage_v": json.dumps(voltage.tolist()),
                        "policy_command": json.dumps(policy_command.tolist()),
                        "raw_action": json.dumps(action.tolist()),
                        "applied_action": json.dumps(applied_action.tolist()),
                        "q_target": json.dumps(targets.tolist()),
                        "target_velocity": json.dumps(target_velocity.tolist()),
                        "friction_direction_velocity": json.dumps(
                            friction_direction_velocity.tolist()
                        ),
                        "friction_ff_nm": json.dumps(friction_ff.tolist()),
                        "estimated_requested_torque_nm": json.dumps(
                            estimated_requested_torque.tolist()
                        ),
                        "estimated_limited_torque_nm": json.dumps(
                            estimated_limited_torque.tolist()
                        ),
                        "raw_pos_feedback": json.dumps(protocol_before["raw_pos"]),
                        "raw_speed_feedback": json.dumps(protocol_before["raw_speed"]),
                        "raw_outpos_feedback": json.dumps(protocol_before["raw_outpos"]),
                        "raw_feedback_rx_s": json.dumps([
                            None if value is None else value - started
                            for value in protocol_before["raw_rx_s"]
                        ]),
                        "raw_pos_after_command": json.dumps(protocol_after["raw_pos"]),
                        "raw_speed_after_command": json.dumps(protocol_after["raw_speed"]),
                        "raw_outpos_after_command": json.dumps(protocol_after["raw_outpos"]),
                        "raw_after_command_rx_s": json.dumps([
                            None if value is None else value - started
                            for value in protocol_after["raw_rx_s"]
                        ]),
                    }
                )

                next_tick += 1.0 / POLICY_RATE_HZ
                remaining = next_tick - time.perf_counter()
                if remaining < 0:
                    overruns += 1
                    next_tick = time.perf_counter()
                else:
                    time.sleep(remaining)
                now = time.perf_counter()
                if now >= next_print:
                    log_file.flush()
                    temperatures = [states[name].temperature_c or 0 for name in POLICY_JOINT_ORDER]
                    print(
                        f"[REAL] t={elapsed:6.1f}s rate={rounds/max(now-started,1e-9):5.1f}Hz "
                        f"tilt={tilt_deg:4.1f}deg loop={round_ms:5.2f}ms "
                        f"infer={inference_ms:4.2f}ms temp={max(temperatures):d}C "
                        "clamps="
                        f"{offset_clamps}/{limit_clamps}/{slew_clamps}/{effort_clamps} "
                        f"overruns={overruns}"
                    )
                    next_print += 1.0 / args.print_rate

        print(
            f"Completed: rounds={rounds}, overruns={overruns}, "
            f"effort_clamps={effort_clamps}, max_loop={max_round_ms:.2f}ms, "
            f"max_inference={max_inference_ms:.2f}ms"
        )
        print(f"CSV log: {log_path}")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C: removing all motor gains immediately.")
        return 130
    except Exception as exc:
        # Cache existing references only: no extra bus reads or disk I/O before stop.
        abort_snapshot = {
            "reason": f"{type(exc).__name__}: {exc}",
            "timestamp_monotonic": time.monotonic(),
            "elapsed_s": None if started is None else time.perf_counter()-started,
            "imu": imu_sample,
            "motor_states": states,
            # _record_success can raise on the triggering feedback before the
            # completed-cycle ``states`` cache is updated. Snapshot the SDK's
            # last CRC-valid packet now, before close()/safe_stop mutates it.
            "protocol_feedback_at_abort": {
                str(motor_id): {
                    "raw_pos": state.raw_pos,
                    "raw_speed": state.raw_speed,
                    "raw_outpos": state.raw_outpos,
                    "feedback_rx_s": state.last_update,
                }
                for motor_id, state in robot.sdk.motor_states.items()
            } if robot is not None else None,
            "position_target_source": robot.position_target_source if robot is not None else None,
            "wire_commands_at_abort": {
                str(mid): dict(command)
                for mid, command in robot.sdk.last_control_commands.items()
            } if robot is not None else None,
            "feedback_cycle_completed_monotonic": feedback_completed,
            "last_completed_target_rad": previous_targets,
            "joint_order": POLICY_JOINT_ORDER,
            "arguments": vars(args),
            "kp_by_id": kp_by_id,
            "kd_by_id": kd_by_id,
            "note": "Motor feedback is from last completed cycle, not simultaneous with IMU. During stand transition cached states/IMU may predate the failure. No extra hardware read performed.",
        }
        raise
    finally:
        try:
            if robot is not None:
                print("Applying zero-gain/zero-torque state to all motors...")
                robot.close()
                print("Serial port closed; fall-arrest fixture must support the robot.")
        finally:
            try:
                if COMMAND_SOURCE is not None:
                    COMMAND_SOURCE.close()
                if imu is not None:
                    imu.close()
            finally:
                if abort_snapshot is not None:
                    def encode(value):
                        if is_dataclass(value):
                            return asdict(value)
                        if isinstance(value, np.ndarray):
                            return value.tolist()
                        if isinstance(value, np.generic):
                            return value.item()
                        if isinstance(value, Path):
                            return str(value)
                        raise TypeError(f"Unsupported snapshot value: {type(value)}")
                    try:
                        abort_path = log_path.with_name(log_path.stem + "_abort.json")
                        abort_path.write_text(json.dumps(abort_snapshot, default=encode, indent=2), encoding="utf-8")
                        print(f"Abort snapshot: {abort_path}", flush=True)
                    except Exception as log_exc:
                        print(f"Failed to save abort snapshot: {log_exc}", file=sys.stderr)


def main() -> int:
    def stop_on_signal(signum, frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_signal)
    signal.signal(signal.SIGHUP, stop_on_signal)
    args = parse_args()
    try:
        return run(args)
    except (DeploymentAbort, ImuError, MicroDuckJ288Error, OSError, RuntimeError) as exc:
        print(f"REAL DEPLOYMENT STOP: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
