#!/usr/bin/env python3
"""Microduck S288 velocity policy, Hybrid/M4, Kp15/Kd0, rotor 61D.

Joint-speed stop is 11 rad/s; body-angular-speed and following-error stops
are disabled at the user's request. Other guards
remain in the common runtime. Fixed 0.6 Nm is a training ceiling; this entry
uses direct firmware Hybrid PD and does not program a hardware torque cap.
No motor/IMU I/O on import.
"""
import sys
from contextlib import redirect_stdout
import json
import hashlib
from pathlib import Path
import s288_runtime as baseline
import deploy_microduck_j288_onnx as policy_io
from s288_policy_contract import load_matched_policy
from s288_keyboard import TerminalKeyboard, KeyboardCommands
from robot_control.arm.microduck_s288 import MicroDuckS288M1Rotor, BUS_MOTOR_IDS

class PolicyCommands(KeyboardCommands):
    def apply(self, key):
        key = key.lower()
        if key == 'w':
            self.vx = min(0.4, round(self.vx + 0.1, 2))
            return True
        if key == 's':
            self.vx = max(-0.4, round(self.vx - 0.1, 2))
            return True
        if key == 'a':
            self.wz = min(1.0, round(self.wz + 0.1, 2))
            return True
        if key == 'd':
            self.wz = max(-1.0, round(self.wz - 0.1, 2))
            return True
        return super().apply(key)


class PolicyKeyboard(TerminalKeyboard):
    def __init__(self):
        super().__init__(enable_head=False)
        self.commands = PolicyCommands()

    def start(self):
        import termios
        import tty
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard deployment needs a terminal on stdin')
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        print('\nW/S: vx +/-0.1 m/s per press (range -0.4..0.4); A/D: turn +/-0.1 rad/s per press (range -1..1); X/Space: stop; Ctrl+C: quit. Head commands remain zero.', flush=True)


SCRIPT_DIR = Path(__file__).resolve().parent
CONTRACT = 'microduck_s288_velocity_61d_v1'
DYNAMICS = 's288_hybrid_m4'
KP = 15 / 1280000
KD = 0.0
DEFAULT_STOP_SPEED_RAD_S = 11.0
DEFAULT_BODY_ANGULAR_SPEED_RAD_S = float('inf')
deployment = baseline.deployment
# Fail clearly if an old shared runtime silently ignores optimization flags.
if not hasattr(deployment, 'plan_policy_deadline') or not hasattr(policy_io, 'ONNX_INTRA_OP_THREADS'):
    raise RuntimeError('S288 velocity requires the synchronized 50 Hz runtime modules')
# Both firmware targets and safety/stand feedback must use the same rotor
# coordinate as this policy's training model. M1 alone defaults to OutPos.
deployment.MicroDuckJ288 = MicroDuckS288M1Rotor
_base_gain_profile = deployment.scaled_gain_profile
deployment.DEFAULT_POLICY = SCRIPT_DIR / 'models/microduck_s288_velocity.onnx'
deployment.DEFAULT_LOG_DIR = SCRIPT_DIR / 'logs/s288_velocity'
deployment.COMMAND_SOURCE = PolicyKeyboard()
deployment.POLICY_RAMP_MODE = 'none'
# Use this policy family's trained HOME, without an older policy pose offset.
ZERO_SPEED_STAND_MODEL_TARGETS = tuple(
    deployment.STAND_MODEL_POSITIONS[name] for name in deployment.POLICY_JOINT_ORDER
)
deployment.POLICY_ENTRY_ACTION = (0.0,) * 14


def matched_gain_profile(kp_scale=1., kd_scale=1.):
    if kp_scale != 1. or kd_scale != 1.:
        raise ValueError('M4 entry fixes protocol gains Kp=15/Kd=0; no extra scaling')
    return dict.fromkeys(BUS_MOTOR_IDS, KP), dict.fromkeys(BUS_MOTOR_IDS, KD)


def load_policy(path):
    result = load_matched_policy(path, expected_kp=KP, expected_kd=KD,
                           expected_contract=CONTRACT, expected_dynamics_variant=DYNAMICS)
    metadata = result[0].get_modelmeta().custom_metadata_map
    required = {
        's288_friction_model': 'm4', 's288_fit_actuator': 's288_hybrid12',
        's288_kp_wire': '15', 's288_kd_wire': '0',
        's288_output_torque_limit_nm': '0.6',
        's288_torque_limit_voltage_scaling': 'disabled',
        's288_voltage_randomization': 'disabled',
        's288_electrical_model': 'disabled', 's288_action_filter_mode': 'none',
        's288_fit_sha256': '2b149b92cded7e0bde1b5b277f27729637283aa957b59ef3ad9a738bd4c59895',
    }
    for key, expected in required.items():
        if metadata.get(key) != expected:
            raise deployment.DeploymentAbort(f'M4 metadata mismatch: {key}')
    if hashlib.sha256(Path(path).read_bytes()).hexdigest() != 'd0a8cad7b8a77ab8e20f35a680f59e2d50ac336a9f248f7d1b54013330e92eee':
        raise deployment.DeploymentAbort('This entry requires the published S288 velocity policy')
    print(f'Matched protocol gains Kp=15/Kd=0; SDK Kp={KP:.12g}, Kd={KD:.12g}')
    return result


def format_joint_comparison(elapsed, wire_targets, positions):
    lines = [f'[JOINTS] t={elapsed:.2f}s; rotor model angles [rad], error=cmd-pos',
             ' ID joint                    cmd       pos     error']
    for name, cmd, pos in zip(deployment.POLICY_JOINT_ORDER, wire_targets, positions):
        motor_id = deployment.JOINT_TO_MOTOR_ID[name]
        cmd = float('nan') if cmd is None else float(cmd)
        pos = float(pos)
        lines.append(f'{motor_id:3d} {name:20s} {cmd:+9.4f} {pos:+9.4f} {cmd-pos:+9.4f}')
    return '\n'.join(lines)


def print_joint_comparison(elapsed, wire_targets, positions):
    print(format_joint_comparison(elapsed, wire_targets, positions), flush=True)


# Batch-one small MLP: avoid default inference-worker contention with IMU,
# serial polling and logging. Hardware-side timing still needs Pi validation.
policy_io.ONNX_INTRA_OP_THREADS = 1
policy_io.ONNX_INTER_OP_THREADS = 1
deployment.PRESERVE_POLICY_CLOCK_PHASE = True
# Full per-frame joint diagnostics remain in CSV; omit the 14-line terminal
# table from the control process. Keep the concise periodic [REAL] status line.
deployment.JOINT_COMPARISON_PRINTER = None


def parse_args():
    args = baseline.parse_matched_args()
    if not any(arg.split('=')[0] == '--max-following-error' for arg in sys.argv[1:]):
        # Low-Kp position error is intentional torque demand. Keep recording
        # the error, but disable this stop by default for the matched policy.
        args.max_following_error = float('inf')
    if not any(arg.split('=')[0] == '--max-speed' for arg in sys.argv[1:]):
        args.max_speed = DEFAULT_STOP_SPEED_RAD_S
    if not any(arg.split('=')[0] == '--max-angular-speed' for arg in sys.argv[1:]):
        args.max_angular_speed = DEFAULT_BODY_ANGULAR_SPEED_RAD_S
    if args.stand_ramp_seconds != 12.:
        raise deployment.DeploymentAbort('Rebuilt-head keyboard entry uses 12s STAND')
    command_flags = {'--vx', '--vy', '--wz', '--head-neck-pitch', '--head-pitch', '--head-yaw', '--head-roll'}
    if any(arg.split('=')[0] in command_flags for arg in sys.argv[1:]):
        raise deployment.DeploymentAbort('Commands come from terminal keys; head commands remain zero')
    args.vx = args.vy = args.wz = 0.
    args.head_neck_pitch = args.head_pitch = args.head_yaw = args.head_roll = 0.
    if not any(arg.split('=')[0] == '--duration' for arg in sys.argv[1:]):
        args.duration = 0.
    return args


deployment.scaled_gain_profile = matched_gain_profile
deployment.load_policy = load_policy
deployment.parse_args = parse_args

class CommandOnlyConsole:
    """Keep startup prompts, then show commands only; leave stderr untouched."""

    def __init__(self, target):
        self.target = target
        self.pending = ''
        self.commands_only = False

    def write(self, text):
        self.pending += text
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            clean = line.lstrip('\r')
            if not self.commands_only or clean.startswith(('W:', 'vx=', 'ONNX POLICY ACTIVE:')):
                self.target.write(line + '\n')
            if clean.startswith('W:'):
                self.commands_only = True
        return len(text)

    def flush(self):
        self.target.flush()

    def __getattr__(self, name):
        return getattr(self.target, name)


def main():
    if any(arg in ('-h', '--help') for arg in sys.argv[1:]):
        return deployment.main()
    # Terminal output only: CSV logging, timing and stop handling are unchanged.
    with redirect_stdout(CommandOnlyConsole(sys.stdout)):
        return deployment.main()


if __name__ == '__main__':
    raise SystemExit(main())
