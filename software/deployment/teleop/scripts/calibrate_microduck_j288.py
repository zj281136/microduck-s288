#!/usr/bin/env python3
"""Calibrate installed MicroDuck J288 encoder homes and joint directions.

The default mode is read-only in the mechanical sense: every request uses
zero torque, zero speed, zero Kp and zero Kd. Put the supported robot in the
MuJoCo RESET pose (all 14 model joints at q=0), then capture that pose as the
encoder reference.

Direction calibration is a separate, explicit mode.  It moves one motor at a
time by a small, smooth positive encoder offset and asks the operator whether
that motion matches the positive direction in MuJoCo.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent))

from robot_control.arm.uniarm_l1_bus import MotorState, UnitreeMotorSDK


JOINTS = (
    ("head_roll", 0, 0.0),
    ("head_yaw", 1, 0.0),
    ("head_pitch", 2, 0.0),
    ("neck_pitch", 3, 0.0),
    ("right_hip_yaw", 4, 0.0),
    ("left_hip_yaw", 5, 0.0),
    ("left_hip_roll", 6, 0.0),
    ("left_hip_pitch", 7, 0.0),
    ("left_knee", 8, 0.0),
    ("left_ankle", 9, 0.0),
    ("right_hip_roll", 10, 0.0),
    ("right_hip_pitch", 11, 0.0),
    ("right_knee", 12, 0.0),
    ("right_ankle", 13, 0.0),
)
MOTOR_IDS = [motor_id for _, motor_id, _ in JOINTS]
JOINT_BY_ID = {motor_id: name for name, motor_id, _ in JOINTS}
MOTOR_GROUPS = {
    "all": MOTOR_IDS,
    "head": [0, 1, 2, 3],
    "left": [5, 6, 7, 8, 9],
    "right": [4, 10, 11, 12, 13],
}
DEFAULT_OUTPUT = SCRIPT_DIR / "microduck_j288_calibration.json"
CALIBRATION_ROBOT = "microduck_j288"
DEFAULT_POSITION_SOURCE = "outpos"
DEFAULT_GROUP = "all"
GEAR_RATIO = 288.35


class CalibrationAbort(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=0.003)
    parser.add_argument("--capture-seconds", type=float, default=2.0)
    parser.add_argument("--position-source", choices=("outpos", "rotor"),
                        default=DEFAULT_POSITION_SOURCE,
                        help="Encoder whose q=0 position is captured; match the deployment target frame")
    parser.add_argument(
        "--group",
        choices=tuple(MOTOR_GROUPS),
        default=DEFAULT_GROUP,
        help=f"Motor group to capture or direction-test (default: {DEFAULT_GROUP})",
    )
    parser.add_argument(
        "--motor-id",
        type=int,
        default=None,
        help="Capture or direction-test only one motor ID; overrides --group",
    )
    parser.add_argument(
        "--calibrate-directions",
        action="store_true",
        help="Load captured homes and interactively determine all direction signs",
    )
    parser.add_argument("--test-offset", type=float, default=0.10, help="Direction-test offset in rad")
    parser.add_argument("--move-seconds", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--kp", type=float, default=1.0e-5, help="J288 SDK Kp for direction test")
    parser.add_argument("--kd", type=float, default=1.0e-6, help="J288 SDK Kd for direction test")
    parser.add_argument("--max-temperature", type=int, default=70)
    parser.add_argument("--max-speed", type=float, default=1.5)
    parser.add_argument("--max-consecutive-failures", type=int, default=3)
    args = parser.parse_args()

    if args.motor_id is not None and args.motor_id not in MOTOR_IDS:
        parser.error("--motor-id must be in 0..13")
    if args.timeout <= 0 or args.capture_seconds <= 0 or args.rate <= 0:
        parser.error("--timeout, --capture-seconds and --rate must be > 0")
    if not 0.0 < args.test_offset <= 0.30:
        parser.error("--test-offset must be > 0 and <= 0.30 rad")
    if args.move_seconds < 1.0:
        parser.error("--move-seconds must be at least 1.0 second")
    if not 0.0 < args.kp <= 5.0e-5 or not 0.0 <= args.kd <= 1.0e-5:
        parser.error("safe range: 0 < --kp <= 5e-5 and 0 <= --kd <= 1e-5")
    if args.max_temperature <= 0 or args.max_speed <= 0:
        parser.error("safety limits must be > 0")
    if args.max_consecutive_failures < 1:
        parser.error("--max-consecutive-failures must be >= 1")
    return args


def safe_request(sdk: UnitreeMotorSDK, motor_id: int) -> MotorState | None:
    sdk.write_control_packet(
        motor_id=motor_id,
        pos=0.0,
        kp=0.0,
        kd=0.0,
        torque=0.0,
        speed=0.0,
        status=1,
        timeout_enable=0,
    )
    return sdk.read(expected_motor_id=motor_id)


def safe_all(sdk: UnitreeMotorSDK, rounds: int = 3) -> None:
    for _ in range(rounds):
        for motor_id in MOTOR_IDS:
            try:
                safe_request(sdk, motor_id)
            except Exception as exc:
                print(f"Safety-stop warning, ID {motor_id}: {exc}", file=sys.stderr)
                try:
                    sdk.ser.reset_input_buffer()
                except Exception:
                    pass


def require_state(sdk: UnitreeMotorSDK, motor_id: int, attempts: int = 3) -> MotorState:
    for _ in range(attempts):
        state = safe_request(sdk, motor_id)
        if state is not None:
            if state.MError or state.ExFlag:
                raise CalibrationAbort(
                    f"ID {motor_id} fault: MError=0x{state.MError:08X}, ExFlag=0x{state.ExFlag:X}"
                )
            return state
        sdk.ser.reset_input_buffer()
    raise CalibrationAbort(f"ID {motor_id} communication failed ({sdk.last_read_error})")


def check_online(sdk: UnitreeMotorSDK, selected_ids: list[int]) -> None:
    print(f"Checking group motors {selected_ids} with zero gain and zero torque...")
    for motor_id in selected_ids:
        state = require_state(sdk, motor_id)
        print(
            f"  ID {motor_id:2d} {JOINT_BY_ID[motor_id]:>16s}: "
            f"q={state.OutPos_rad:+.6f} rad, temp={max(state.temp_case, state.temp_winding)} C"
        )


def circular_mean(samples: list[float]) -> tuple[float, float]:
    sin_mean = sum(math.sin(value) for value in samples) / len(samples)
    cos_mean = sum(math.cos(value) for value in samples) / len(samples)
    mean = math.atan2(sin_mean, cos_mean) % (2.0 * math.pi)
    concentration = math.hypot(sin_mean, cos_mean)
    circular_std = math.sqrt(max(0.0, -2.0 * math.log(max(concentration, 1.0e-12))))
    return mean, circular_std


def empty_calibration_document() -> dict:
    entries = []
    for name, motor_id, model_home in JOINTS:
        entries.append(
            {
                "joint_name": name,
                "motor_id": motor_id,
                "model_home_rad": model_home,
                "encoder_home_rad": None,
                "direction": None,
                "capture_std_rad": None,
                "sample_count": 0,
            }
        )
    return {
        "format_version": 1,
        "robot": CALIBRATION_ROBOT,
        "reference_pose": "mujoco_reset_qpos_zero",
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "position_mapping": (
            "q_model = model_home_rad + direction * "
            "wrap_pi(q_motor - encoder_home_rad)"
        ),
        "joints": entries,
    }


def merge_home_samples(path: Path, home_samples: dict[int, list[float]],
                       position_source: str = "outpos") -> dict:
    if path.expanduser().exists():
        document = load_document(path)
    else:
        document = empty_calibration_document()
    by_id = {item["motor_id"]: item for item in document["joints"]}
    for motor_id, samples in home_samples.items():
        encoder_home, sample_std = circular_mean(samples)
        by_id[motor_id]["encoder_home_rad"] = encoder_home
        by_id[motor_id]["capture_std_rad"] = sample_std
        by_id[motor_id]["sample_count"] = len(samples)
        by_id[motor_id]["position_source"] = position_source
        by_id[motor_id]["captured_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    document["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    return document


def write_json(path: Path, document: dict) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    print(f"Calibration saved: {path}")


def capture_home(
    sdk: UnitreeMotorSDK, selected_ids: list[int], args: argparse.Namespace
) -> None:
    print("\nHOME CAPTURE (no active motor command)")
    selection = f"motor ID {args.motor_id}" if args.motor_id is not None else f"group '{args.group}'"
    print(f"Selected {selection}: IDs {selected_ids}")
    print("Support the robot securely and place ONLY the selected joints in MuJoCo RESET (q=0).")
    print("The positions of joints outside this group are not recorded in this run.")
    print(f"Capturing {args.position_source} encoder zero; this is NOT the policy HOME/STAND pose.")
    input("Press Enter when the pose is ready: ")

    samples = {motor_id: [] for motor_id in selected_ids}
    failures = {motor_id: 0 for motor_id in selected_ids}
    started = time.perf_counter()
    while time.perf_counter() - started < args.capture_seconds:
        for motor_id in selected_ids:
            state = safe_request(sdk, motor_id)
            if state is None:
                failures[motor_id] += 1
                sdk.ser.reset_input_buffer()
                if failures[motor_id] >= args.max_consecutive_failures:
                    raise CalibrationAbort(
                        f"ID {motor_id}: repeated {sdk.last_read_error} during capture"
                    )
                continue
            failures[motor_id] = 0
            if state.MError or state.ExFlag:
                raise CalibrationAbort(
                    f"ID {motor_id} fault during capture: "
                    f"MError=0x{state.MError:08X}, ExFlag=0x{state.ExFlag:X}"
                )
            samples[motor_id].append(
                state.pos_rad if args.position_source == "rotor" else state.OutPos_rad
            )

    if any(not values for values in samples.values()):
        raise CalibrationAbort("one or more motors produced no valid home samples")
    document = merge_home_samples(args.output, samples, args.position_source)
    print("\nCaptured encoder homes for this group:")
    for joint in document["joints"]:
        if joint["motor_id"] not in selected_ids:
            continue
        print(
            f"  ID {joint['motor_id']:2d} {joint['joint_name']:>16s}: "
            f"encoder={joint['encoder_home_rad']:+.6f}, "
            f"model={joint['model_home_rad']:+.4f}, "
            f"std={joint['capture_std_rad']:.6f} rad"
        )
    write_json(args.output, document)
    missing = [item["motor_id"] for item in document["joints"] if item["encoder_home_rad"] is None]
    if missing:
        print(f"Still missing home capture for IDs: {missing}")
    else:
        print("All 14 homes are captured. Review the JSON before direction calibration.")


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def command_one(
    sdk: UnitreeMotorSDK,
    active_id: int,
    start: float,
    end: float,
    args: argparse.Namespace,
) -> None:
    # Interpolate along the shortest circular path.  This is essential when a
    # home position lies close to the 0/2pi encoder boundary.
    travel = (end - start + math.pi) % (2.0 * math.pi) - math.pi
    started = time.perf_counter()
    next_tick = started
    consecutive_failures = 0
    while True:
        elapsed = time.perf_counter() - started
        phase = min(1.0, elapsed / args.move_seconds)
        target = start + smoothstep(phase) * travel
        for motor_id in MOTOR_IDS:
            if motor_id == active_id:
                sdk.write_control_packet(
                    motor_id=motor_id,
                    pos=target,
                    kp=args.kp,
                    kd=args.kd,
                    torque=0.0,
                    speed=0.0,
                    status=1,
                    timeout_enable=0,
                )
                state = sdk.read(expected_motor_id=motor_id)
            else:
                state = safe_request(sdk, motor_id)

            if state is None:
                consecutive_failures += 1
                sdk.ser.reset_input_buffer()
                if consecutive_failures >= args.max_consecutive_failures:
                    raise CalibrationAbort(
                        f"ID {motor_id}: repeated communication failure ({sdk.last_read_error})"
                    )
                continue
            consecutive_failures = 0
            temperature = max(state.temp_case, state.temp_winding)
            if state.MError or state.ExFlag:
                raise CalibrationAbort(
                    f"ID {motor_id} fault: MError=0x{state.MError:08X}, ExFlag=0x{state.ExFlag:X}"
                )
            if temperature > args.max_temperature:
                raise CalibrationAbort(f"ID {motor_id}: temperature {temperature} C")
            if abs(state.speed_rads) > args.max_speed:
                raise CalibrationAbort(f"ID {motor_id}: speed {state.speed_rads:.3f} rad/s")
        if phase >= 1.0:
            return
        next_tick += 1.0 / args.rate
        time.sleep(max(0.0, next_tick - time.perf_counter()))


def load_document(path: Path) -> dict:
    try:
        document = json.loads(path.expanduser().read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CalibrationAbort(f"calibration file not found: {path}") from exc
    entries = document.get("joints", [])
    actual = {(item.get("joint_name"), item.get("motor_id")) for item in entries}
    expected = {(name, motor_id) for name, motor_id, _ in JOINTS}
    if actual != expected:
        raise CalibrationAbort("calibration file does not match the expected 14-joint mapping")
    return document


def calibrate_directions(
    sdk: UnitreeMotorSDK, selected_ids: list[int], args: argparse.Namespace
) -> None:
    document = load_document(args.output)
    by_id = {item["motor_id"]: item for item in document["joints"]}
    missing = [motor_id for motor_id in selected_ids if by_id[motor_id]["encoder_home_rad"] is None]
    if missing:
        raise CalibrationAbort(f"capture home positions before direction testing; missing IDs: {missing}")
    print("\nDIRECTION CALIBRATION")
    print("The robot must remain securely supported. Only one joint is actively driven at a time.")
    print("For each joint, compare the observed motion with +joint motion in the MuJoCo viewer.")
    input("Press Enter to enable the small direction tests: ")

    for name, motor_id, _ in JOINTS:
        if motor_id not in selected_ids:
            continue
        if by_id[motor_id].get("direction") in (-1, 1):
            print(
                f"\nID {motor_id}: {name}, already direction="
                f"{by_id[motor_id]['direction']:+d}; keeping it"
            )
            continue
        print(f"\nID {motor_id}: {name}")
        input(
            "Place/support this joint at a safe starting angle, then press Enter "
            "to move +encoder direction (Ctrl+C stops)... "
        )
        # A zero-gain joint may have moved under gravity while earlier joints
        # were being tested.  Always start from fresh feedback; commanding the
        # captured RESET encoder reference here could create a hidden position step.
        state = require_state(sdk, motor_id)
        start_position = (state.pos_rad if args.position_source == "rotor"
                          else state.OutPos_rad)
        try:
            command_one(
                sdk,
                motor_id,
                start_position,
                start_position + args.test_offset,
                args,
            )
            response = input(
                "Did that match the POSITIVE direction shown in MuJoCo? [y/n/q]: "
            ).strip().lower()
        except BaseException:
            # A safety fault or Ctrl+C must remove gain immediately, not try to
            # execute a return trajectory from an unsafe state.
            safe_all(sdk, rounds=1)
            raise

        # Only a normally completed test performs the smooth return.
        state = sdk.motor_states[motor_id]
        current = state.pos_rad if args.position_source == "rotor" else state.OutPos_rad
        command_one(sdk, motor_id, current, start_position, args)
        safe_all(sdk, rounds=1)

        if response == "q":
            print("Direction calibration stopped; saving completed entries.")
            write_json(args.output, document)
            return
        if response not in {"y", "n"}:
            raise CalibrationAbort("expected y, n, or q")
        by_id[motor_id]["direction"] = 1 if response == "y" else -1
        write_json(args.output, document)
        print(f"Recorded {name}: direction={by_id[motor_id]['direction']:+d}")

    remaining = [item["motor_id"] for item in document["joints"] if item["direction"] not in (-1, 1)]
    if not remaining:
        document["directions_completed_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    write_json(args.output, document)
    if remaining:
        print(f"Direction group complete. Still missing IDs: {remaining}")
    else:
        print("All 14 directions have been recorded.")


def main() -> int:
    global MOTOR_IDS
    args = parse_args()
    selected_ids = [args.motor_id] if args.motor_id is not None else MOTOR_GROUPS[args.group]
    # A replacement servo may be connected by itself on the bench. Restrict
    # communication and final zero-gain commands to the selected ID in this mode.
    MOTOR_IDS = selected_ids
    effective_kp = args.kp * GEAR_RATIO * GEAR_RATIO
    effective_kd = args.kd * GEAR_RATIO * GEAR_RATIO
    print(f"Port={args.port}, timeout={args.timeout:.4f} s, output={args.output}")
    if args.calibrate_directions:
        print(
            f"Direction-test gains: SDK kp={args.kp:g}, kd={args.kd:g} "
            f"(~physical Kp={effective_kp:.3f}, Kd={effective_kd:.3f})"
        )
    else:
        print("Capture mode sends zero torque, speed, Kp and Kd; it cannot hold the robot up.")

    sdk: UnitreeMotorSDK | None = None
    exit_code = 1
    try:
        sdk = UnitreeMotorSDK(MOTOR_IDS, args.port, timeout=args.timeout,
                             position_target_source=args.position_source)
        check_online(sdk, selected_ids)
        if args.calibrate_directions:
            calibrate_directions(sdk, selected_ids, args)
        else:
            capture_home(sdk, selected_ids, args)
        exit_code = 0
    except KeyboardInterrupt:
        print("\nCtrl+C: calibration stopped.")
    except CalibrationAbort as exc:
        print(f"\nCALIBRATION STOP: {exc}", file=sys.stderr)
    except Exception as exc:
        print(f"\nERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
    finally:
        if sdk is not None:
            print("Applying zero-gain/zero-torque state to all motors...")
            safe_all(sdk)
            sdk.close()
            print("Serial port closed.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
