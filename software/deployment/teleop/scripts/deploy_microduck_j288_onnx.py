#!/usr/bin/env python3
"""Fixture-only MicroDuck 288-series ONNX test with a fake upright IMU.

This is not a walking deployment.  It exists only to verify policy/joint
mapping while the robot is fully carried by a support fixture.  Ctrl+C and
all detected faults switch every motor to zero gain and zero torque.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent))

from robot_control.arm.microduck_j288 import (  # noqa: E402
    POLICY_JOINT_ORDER,
    MicroDuckJ288,
    MicroDuckJ288Error,
    SafetyLimits,
    STAND_MODEL_POSITIONS,
    wrap_pi,
)


DEFAULT_CALIBRATION = SCRIPT_DIR / "microduck_j288_calibration.json"
DEFAULT_LOG_DIR = SCRIPT_DIR / "logs" / "onnx_fixture"
DEFAULT_POLICY: Path | None = None
HARDWARE_NAME = "J288"
POLICY_RATE_HZ = 50.0
OBS_SIZE = 61
ACTION_SIZE = 14

# Mechanical limits from the current MicroDuck walk MJCF.  These exact hard
# ranges are the last line of defence, matching the original simulator model.
JOINT_LIMITS = {
    "left_hip_yaw": (-0.4363, 0.5236),
    "left_hip_roll": (-0.3840, 0.3840),
    "left_hip_pitch": (-1.5708, 1.5708),
    "left_knee": (-1.5708, 1.5708),
    "left_ankle": (-1.5708, 1.5708),
    "neck_pitch": (-1.5708, 1.0472),
    "head_pitch": (-1.5708, 1.5708),
    "head_yaw": (-2.9671, 2.9671),
    "head_roll": (-0.4363, 0.4363),
    "right_hip_yaw": (-0.5236, 0.4363),
    "right_hip_roll": (-0.3840, 0.3840),
    "right_hip_pitch": (-1.5708, 1.5708),
    "right_knee": (-1.5708, 1.5708),
    "right_ankle": (-1.5708, 1.5708),
}


class DeploymentAbort(RuntimeError):
    """Raised when the fixture test must enter zero-gain safe state."""


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--onnx",
        type=Path,
        default=DEFAULT_POLICY,
        required=DEFAULT_POLICY is None,
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--timeout", type=float, default=0.003)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--stand-ramp-seconds", type=float, default=8.0)
    parser.add_argument("--policy-ramp-seconds", type=float, default=3.0)
    parser.add_argument(
        "--action-scale",
        type=float,
        default=0.25,
        help="Fixture-test multiplier on ONNX actions; must be in (0, 1]",
    )
    parser.add_argument(
        "--max-policy-offset",
        type=float,
        default=None,
        help=(
            "Optional extra per-joint offset limit around STAND in rad. "
            "By default no extra offset clamp is applied, matching infer_policy.py; "
            "the original MJCF joint limits remain enforced."
        ),
    )
    parser.add_argument(
        "--max-target-speed",
        type=float,
        default=2.0,
        help="Maximum commanded joint-target slew in rad/s (default: 2.0)",
    )
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--head-neck-pitch", type=float, default=0.0)
    parser.add_argument("--head-pitch", type=float, default=0.0)
    parser.add_argument("--head-yaw", type=float, default=0.0)
    parser.add_argument("--head-roll", type=float, default=0.0)
    parser.add_argument("--max-start-travel", type=float, default=0.60)
    parser.add_argument("--max-temperature", type=int, default=70)
    parser.add_argument("--max-speed", type=float, default=5.0)
    parser.add_argument("--max-following-error", type=float, default=0.50)
    parser.add_argument("--max-consecutive-failures", type=int, default=3)
    parser.add_argument("--print-rate", type=float, default=1.0)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--arm-fixture-test",
        action="store_true",
        help="Required acknowledgement that the robot is fully supported",
    )
    args = parser.parse_args()

    if not args.arm_fixture_test:
        parser.error("--arm-fixture-test is required; fake IMU must never be used on the ground")
    if args.timeout <= 0 or args.duration <= 0 or args.print_rate <= 0:
        parser.error("timeout, duration and print-rate must be positive")
    if args.stand_ramp_seconds < 5.0 or args.policy_ramp_seconds < 1.0:
        parser.error("stand ramp must be >=5 s and policy ramp must be >=1 s")
    if not 0.0 < args.action_scale <= 1.0:
        parser.error("--action-scale must be in (0, 1]")
    if args.max_policy_offset is not None and args.max_policy_offset <= 0.0:
        parser.error("--max-policy-offset must be positive when provided")
    if not 0.1 <= args.max_target_speed <= 3.5:
        parser.error("--max-target-speed must be in [0.1, 3.5] rad/s")
    if not -0.15 <= args.vx <= 0.15 or not -0.10 <= args.vy <= 0.10:
        parser.error("fixture limits: vx ±0.15 m/s and vy ±0.10 m/s")
    if not -0.40 <= args.wz <= 0.40:
        parser.error("fixture limit: wz ±0.40 rad/s")
    head_limits = {
        "head_neck_pitch": 0.50,
        "head_pitch": 0.50,
        "head_yaw": 0.70,
        "head_roll": 0.20,
    }
    for argument, limit in head_limits.items():
        if not -limit <= getattr(args, argument) <= limit:
            parser.error(f"fixture limit: --{argument.replace('_', '-')} ±{limit:.2f} rad")
    if not 0.0 < args.max_start_travel <= 1.0:
        parser.error("--max-start-travel must be in (0, 1.0] rad")
    if (
        args.max_temperature <= 0
        or args.max_speed <= 0
        or args.max_following_error <= 0
        or args.max_consecutive_failures < 1
    ):
        parser.error("motor safety limits must be positive")
    return args


def require_online_positions(states) -> np.ndarray:
    values = []
    for name in POLICY_JOINT_ORDER:
        state = states[name]
        if not state.online or state.model_position_rad is None:
            raise DeploymentAbort(f"{name} offline ({state.read_error})")
        values.append(state.model_position_rad)
    result = np.asarray(values, dtype=np.float32)
    if not np.all(np.isfinite(result)):
        raise DeploymentAbort("non-finite joint position feedback")
    return result


def policy_positions(robot, states):
    """Default legacy policy view; hardware entry points may override it."""
    return require_online_positions(states)


def feedback_velocity(states) -> np.ndarray:
    values = []
    for name in POLICY_JOINT_ORDER:
        value = states[name].model_velocity_rad_s
        if value is None or not math.isfinite(value):
            raise DeploymentAbort(f"invalid velocity feedback for {name}")
        values.append(value)
    return np.asarray(values, dtype=np.float32)


def feedback_telemetry(states) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return model-frame torque, temperature and voltage in policy order."""
    torques = []
    temperatures = []
    voltages = []
    for name in POLICY_JOINT_ORDER:
        state = states[name]
        values = (state.model_torque_nm, state.temperature_c, state.voltage_v)
        if any(value is None or not math.isfinite(value) for value in values):
            raise DeploymentAbort(f"invalid telemetry feedback for {name}")
        torques.append(state.model_torque_nm)
        temperatures.append(state.temperature_c)
        voltages.append(state.voltage_v)
    return (
        np.asarray(torques, dtype=np.float32),
        np.asarray(temperatures, dtype=np.int16),
        np.asarray(voltages, dtype=np.float32),
    )


def validate_start(q_start: np.ndarray, maximum: float) -> None:
    q_home = np.asarray(
        [STAND_MODEL_POSITIONS[name] for name in POLICY_JOINT_ORDER], dtype=np.float32
    )
    travel = np.asarray(
        [wrap_pi(float(home - start)) for home, start in zip(q_home, q_start)]
    )
    index = int(np.argmax(np.abs(travel)))
    if abs(travel[index]) > maximum:
        raise DeploymentAbort(
            f"start-to-STAND travel exceeds {maximum:.3f} rad: "
            f"{POLICY_JOINT_ORDER[index]}={travel[index]:+.3f}"
        )


def transition_to_stand(
    robot: MicroDuckJ288,
    q_start: np.ndarray,
    duration: float,
) -> dict:
    q_home = np.asarray(
        [STAND_MODEL_POSITIONS[name] for name in POLICY_JOINT_ORDER], dtype=np.float32
    )
    travel = np.asarray(
        [wrap_pi(float(home - start)) for home, start in zip(q_home, q_start)],
        dtype=np.float32,
    )
    started = time.perf_counter()
    next_tick = started
    period = 1.0 / POLICY_RATE_HZ
    states = None
    while True:
        phase = min(1.0, (time.perf_counter() - started) / duration)
        target = q_start + smoothstep(phase) * travel
        states = robot.command_model_positions(target.tolist())
        if phase >= 1.0:
            return states
        next_tick += period
        time.sleep(max(0.0, next_tick - time.perf_counter()))


def load_policy(
    path: Path, *, expected_obs_size: int = OBS_SIZE,
    expected_contract: str | None = None,
):
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise DeploymentAbort(f"ONNX file not found: {resolved}")
    session = ort.InferenceSession(str(resolved), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or inputs[0].shape != [1, expected_obs_size]:
        raise DeploymentAbort(
            f"expected one [1,{expected_obs_size}] ONNX input, got {[x.shape for x in inputs]}"
        )
    if len(outputs) != 1 or outputs[0].shape != [1, ACTION_SIZE]:
        raise DeploymentAbort(
            f"expected one [1,14] ONNX output, got {[x.shape for x in outputs]}"
        )

    metadata = session.get_modelmeta().custom_metadata_map
    if expected_contract is not None and metadata.get("s288_observation_contract") != expected_contract:
        raise DeploymentAbort("S288 ONNX observation contract mismatch")
    expected_names = ",".join(POLICY_JOINT_ORDER)
    if metadata.get("joint_names") != expected_names:
        raise DeploymentAbort(
            "ONNX joint_names do not exactly match MicroDuck 288-series policy order"
        )
    try:
        model_action_scale = float(metadata.get("action_scale", "1.0"))
    except ValueError as exc:
        raise DeploymentAbort("invalid ONNX action_scale metadata") from exc
    return session, inputs[0].name, outputs[0].name, model_action_scale, resolved


def clamp_targets(
    offsets: np.ndarray, max_offset: float | None = None
) -> tuple[np.ndarray, int]:
    if max_offset is not None:
        offsets = np.clip(offsets, -max_offset, max_offset)
    targets = []
    limit_clamps = 0
    for name, offset in zip(POLICY_JOINT_ORDER, offsets):
        low, high = JOINT_LIMITS[name]
        requested = STAND_MODEL_POSITIONS[name] + float(offset)
        # Match the original MuJoCo inference path: do not shrink the MJCF
        # range.  Software clipping at the exact hard range prevents the real
        # actuator from being commanded through a mechanical stop.
        limited = min(high, max(low, requested))
        limit_clamps += int(limited != requested)
        targets.append(limited)
    return np.asarray(targets, dtype=np.float32), limit_clamps


def run(args: argparse.Namespace) -> int:
    session, input_name, output_name, model_action_scale, onnx_path = load_policy(args.onnx)
    log_path = args.log_file
    if log_path is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = DEFAULT_LOG_DIR / f"fixture_fake_imu_{stamp}.csv"
    log_path = log_path.expanduser().resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    safety = SafetyLimits(
        max_temperature_c=args.max_temperature,
        max_speed_rad_s=args.max_speed,
        max_following_error_rad=args.max_following_error,
        max_consecutive_failures=args.max_consecutive_failures,
    )
    command = np.asarray([args.vx, args.vy, args.wz], dtype=np.float32)
    # Training observation order: neck_pitch, head_pitch, head_yaw, head_roll.
    # These values are desired offsets from the nominal STAND pose, not absolute
    # motor positions.
    head_command = np.asarray(
        [args.head_neck_pitch, args.head_pitch, args.head_yaw, args.head_roll],
        dtype=np.float32,
    )
    body_command = np.zeros(6, dtype=np.float32)
    fake_ang_vel = np.zeros(3, dtype=np.float32)
    fake_gravity = np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
    q_home = np.asarray(
        [STAND_MODEL_POSITIONS[name] for name in POLICY_JOINT_ORDER], dtype=np.float32
    )

    print(
        f"DANGER: {HARDWARE_NAME} FAKE IMU FIXTURE TEST — "
        "the policy cannot detect body tilt or falls."
    )
    print("The robot must be completely carried by a fall-arrest fixture; feet off the ground.")
    print(f"ONNX: {onnx_path}")
    print(f"Command: vx={args.vx:+.3f}, vy={args.vy:+.3f}, wz={args.wz:+.3f}")
    print(
        "Head command (STAND-relative): "
        f"neck_pitch={args.head_neck_pitch:+.3f}, "
        f"head_pitch={args.head_pitch:+.3f}, "
        f"head_yaw={args.head_yaw:+.3f}, head_roll={args.head_roll:+.3f} rad"
    )
    print(
        f"Runtime action scale={args.action_scale:.2f}, "
        f"extra offset clamp={args.max_policy_offset if args.max_policy_offset is not None else 'disabled'}, "
        f"target slew≤{args.max_target_speed:.2f}rad/s, duration={args.duration:.1f}s"
    )
    input("Press Enter to arm motors: ")

    robot = None
    try:
        robot = MicroDuckJ288(
            args.port,
            args.calibration,
            timeout=args.timeout,
            safety=safety,
        )
        initial = robot.poll_zero_gain(retries=3)
        q_start = require_online_positions(initial)
        validate_start(q_start, args.max_start_travel)
        print(f"Moving smoothly to STAND over {args.stand_ramp_seconds:.1f}s...")
        states = transition_to_stand(robot, q_start, args.stand_ramp_seconds)
        print("STAND reached. Starting 50 Hz policy with fake upright IMU.")

        last_action = np.zeros(ACTION_SIZE, dtype=np.float32)
        started = time.perf_counter()
        next_tick = started
        next_print = started + 1.0 / args.print_rate
        rounds = overruns = offset_clamps = joint_limit_clamps = slew_clamps = 0
        max_round_ms = 0.0
        previous_targets = q_home.copy()
        fieldnames = (
            "elapsed_s",
            "round_ms",
            "vx",
            "vy",
            "wz",
            "head_neck_pitch",
            "head_pitch_command",
            "head_yaw_command",
            "head_roll_command",
            "ramp_scale",
            "q_feedback",
            "dq_feedback",
            "torque_feedback",
            "temperature_c",
            "voltage_v",
            "raw_action",
            "q_target",
        )
        with log_path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=fieldnames)
            writer.writeheader()
            while True:
                round_started = time.perf_counter()
                elapsed = round_started - started
                if elapsed >= args.duration:
                    break

                q = require_online_positions(states)
                dq = feedback_velocity(states)
                torque, temperature, voltage = feedback_telemetry(states)
                obs = np.concatenate(
                    (
                        fake_ang_vel,
                        fake_gravity,
                        policy_positions(robot, states) - q_home,
                        dq,
                        last_action,
                        command,
                        head_command,
                        body_command,
                    )
                ).astype(np.float32)
                if obs.shape != (OBS_SIZE,) or not np.all(np.isfinite(obs)):
                    raise DeploymentAbort("invalid 61D observation")

                action = session.run(
                    [output_name], {input_name: obs.reshape(1, OBS_SIZE)}
                )[0].reshape(-1).astype(np.float32)
                if action.shape != (ACTION_SIZE,) or not np.all(np.isfinite(action)):
                    raise DeploymentAbort("invalid ONNX action")

                ramp = smoothstep(elapsed / args.policy_ramp_seconds)
                raw_offsets = action * model_action_scale * args.action_scale * ramp
                if args.max_policy_offset is not None:
                    offset_clamps += int(
                        np.count_nonzero(np.abs(raw_offsets) > args.max_policy_offset)
                    )
                targets, clamped = clamp_targets(raw_offsets, args.max_policy_offset)
                joint_limit_clamps += clamped
                max_step = args.max_target_speed / POLICY_RATE_HZ
                target_step = targets - previous_targets
                slew_clamps += int(np.count_nonzero(np.abs(target_step) > max_step))
                targets = previous_targets + np.clip(target_step, -max_step, max_step)
                states = robot.command_model_positions(targets.tolist())
                previous_targets = targets
                last_action = action

                round_ms = (time.perf_counter() - round_started) * 1000.0
                max_round_ms = max(max_round_ms, round_ms)
                rounds += 1
                writer.writerow(
                    {
                        "elapsed_s": f"{elapsed:.6f}",
                        "round_ms": f"{round_ms:.4f}",
                        "vx": args.vx,
                        "vy": args.vy,
                        "wz": args.wz,
                        "head_neck_pitch": args.head_neck_pitch,
                        "head_pitch_command": args.head_pitch,
                        "head_yaw_command": args.head_yaw,
                        "head_roll_command": args.head_roll,
                        "ramp_scale": f"{ramp:.6f}",
                        "q_feedback": json.dumps(q.tolist(), separators=(",", ":")),
                        "dq_feedback": json.dumps(dq.tolist(), separators=(",", ":")),
                        "torque_feedback": json.dumps(
                            torque.tolist(), separators=(",", ":")
                        ),
                        "temperature_c": json.dumps(
                            temperature.tolist(), separators=(",", ":")
                        ),
                        "voltage_v": json.dumps(voltage.tolist(), separators=(",", ":")),
                        "raw_action": json.dumps(action.tolist(), separators=(",", ":")),
                        "q_target": json.dumps(targets.tolist(), separators=(",", ":")),
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
                    actual_hz = rounds / max(now - started, 1e-9)
                    temps = [states[name].temperature_c or 0 for name in POLICY_JOINT_ORDER]
                    print(
                        f"[POLICY] t={elapsed:5.1f}s rate={actual_hz:5.1f}Hz "
                        f"round={round_ms:5.2f}ms max={max_round_ms:5.2f}ms "
                        f"temp_max={max(temps):d}C "
                        f"clamps={offset_clamps}/{joint_limit_clamps}/{slew_clamps} "
                        f"overruns={overruns}"
                    )
                    next_print += 1.0 / args.print_rate

        print("Fixture policy test completed normally.")
        print(
            f"rounds={rounds}, overruns={overruns}, max_round={max_round_ms:.2f}ms, "
            f"offset_clamps={offset_clamps}, joint_limit_clamps={joint_limit_clamps}, "
            f"slew_clamps={slew_clamps}"
        )
        print(f"CSV log: {log_path}")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping policy and removing all motor gains.")
        return 130
    finally:
        if robot is not None:
            print("Applying zero-gain/zero-torque state to all motors...")
            robot.close()
            print("Serial port closed. The support fixture must carry the robot.")


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except (MicroDuckJ288Error, RuntimeError, OSError) as exc:
        print(f"FIXTURE TEST STOP: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
