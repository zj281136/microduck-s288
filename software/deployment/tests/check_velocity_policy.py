"""Offline Pi checks only: no serial/I2C connection or robot.main invocation."""
from pathlib import Path
import sys, json, math, hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'teleop/scripts'),str(ROOT/'teleop')]
import deploy_microduck_s288_velocity as entry

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
policy=entry.load_policy(entry.deployment.DEFAULT_POLICY)
s, inp, out, scale, path=policy
assert s.get_session_options().intra_op_num_threads == 1
assert s.get_session_options().inter_op_num_threads == 1
assert entry.deployment.PRESERVE_POLICY_CLOCK_PHASE
assert entry.deployment.JOINT_COMPARISON_PRINTER is None
assert scale==1
assert s.get_inputs()[0].shape==[1,61] and s.get_outputs()[0].shape==[1,14]
for seed in range(16):
 x=np.random.default_rng(seed).normal(0,.3,(1,61)).astype(np.float32)
 assert np.isfinite(s.run([out],{inp:x})[0]).all()
sys.argv=['validate_pi.py','--port','/dev/ttyACM0','--arm-ground-test']
a=entry.parse_args()
assert a.max_speed == 11.0 and math.isinf(a.max_angular_speed)
assert math.isinf(a.max_following_error)
assert a.max_tilt_deg==35.0
assert a.max_temperature>0 and a.max_consecutive_failures>0
assert a.policy_qdot_delay_steps==0 and a.max_target_speed is None
assert a.vx==a.vy==a.wz==0
kp,kd=entry.matched_gain_profile()
assert len(kp)==len(kd)==14
assert set(kp.values())=={15/1280000} and set(kd.values())=={0.}
assert entry.deployment.MicroDuckJ288.__name__=='MicroDuckS288M1Rotor'
assert entry.deployment.HEAD_TARGET_LOWPASS is None
assert entry.deployment.LEGS_TARGET_LOWPASS is None
assert entry.deployment.POLICY_RAMP_MODE=='none'
assert entry.deployment.POLICY_ENTRY_ACTION==(0.,)*14
commands=entry.PolicyCommands();commands.apply('w');assert commands.vx==.1
commands.apply('w');assert commands.vx==.2
commands.apply('s');assert commands.vx==.1
for _ in range(20):commands.apply('w')
assert commands.vx==.4
for _ in range(20):commands.apply('s')
assert commands.vx==-.4
commands.apply('a');assert commands.wz==.1
for _ in range(20):commands.apply('a')
assert commands.wz==1.
for _ in range(30):commands.apply('d')
assert commands.wz==-1.
commands.apply(' ');assert commands.vx==commands.wz==0.
# Exercise quiet-console routing without invoking the real deployment loop.
import io
from contextlib import redirect_stdout, redirect_stderr
original_main = entry.deployment.main
stdout, stderr = io.StringIO(), io.StringIO()
def fake_main():
    print('EXPERIMENTAL REAL-IMU DEPLOYMENT')
    print('W: forward; Space: stop; Ctrl+C: quit')
    print('[REAL] rate=50Hz')
    print('\rvx=+0.25 vy=+0.00 wz=+0.00')
    print('REAL DEPLOYMENT STOP: test error', file=sys.stderr)
    return 7
try:
    entry.deployment.main = fake_main
    with redirect_stdout(stdout), redirect_stderr(stderr):
        assert entry.main() == 7
finally:
    entry.deployment.main = original_main
assert stdout.getvalue() == 'EXPERIMENTAL REAL-IMU DEPLOYMENT\nW: forward; Space: stop; Ctrl+C: quit\n\rvx=+0.25 vy=+0.00 wz=+0.00\n'
assert 'test error' in stderr.getvalue()
print('PASS: policy, gains, clock, guards, key increments and console; hardware not opened')
