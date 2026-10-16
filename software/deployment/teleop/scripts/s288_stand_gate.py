"""Logged rotor-side stand transition and bounded readiness gate. No policy calls."""
import csv
import json
import math
import time
from pathlib import Path
import numpy as np


def ready_metrics(q, dq, target, tilt, gyro):
    error = np.abs(np.asarray(target) - np.asarray(q))
    values = np.r_[error, dq, tilt, gyro]
    ready = (np.all(np.isfinite(values)) and np.max(error) <= math.radians(3)
             and np.max(np.abs(dq)) <= 0.2 and tilt <= 5.0 and np.linalg.norm(gyro) <= 0.3)
    return bool(ready), float(np.max(error)), int(np.argmax(error))


def transition(api, robot, imu, q_start, q_stand, duration, imu_max_age,
               max_tilt_deg, max_angular_speed, *, log_path, require_ready=True):
    """12 s default ramp; >=2 s hold; require 0.5 s ready, timeout at 5 s hold."""
    target = np.asarray(q_stand, dtype=np.float32)
    start = np.asarray(q_start, dtype=np.float32)
    travel = np.asarray([api.wrap_pi(float(t-s)) for t,s in zip(target,start)])
    path = Path(log_path).with_name(Path(log_path).stem + '_transition.csv')
    fields = ['elapsed_s','phase','sample_time','joint_order','target','rotor','outpos',
              'rotor_error','outpos_error','rotor_speed','torque','tilt_deg','gyro',
              'raw_pos','raw_speed','raw_outpos','feedback_rx_s','ready','ready_for_s']
    began = next_tick = time.perf_counter()
    ready_since = None
    print(f'Transition log: {path}',flush=True)
    with path.open('w',newline='',buffering=1) as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        while True:
            now=time.perf_counter(); elapsed=now-began
            phase='ramp' if elapsed < duration else 'hold'
            requested=start+api.smoothstep(min(1.,elapsed/duration))*travel if phase=='ramp' else target
            sample=imu.latest(imu_max_age)
            api.validate_imu(sample,max_tilt_deg,max_angular_speed)
            states=robot.command_model_positions(requested.tolist())
            sample=imu.latest(imu_max_age)
            tilt=api.validate_imu(sample,max_tilt_deg,max_angular_speed)
            views=api.s288_position_views(robot,states)
            q=np.asarray(views['q_rotor_model_rad']); out=np.asarray(views['q_out_model_rad'])
            dq=api.feedback_velocity(states)
            ready,err,index=ready_metrics(q,dq,target,tilt,sample.angular_velocity_rad_s)
            if phase=='hold' and ready:
                if ready_since is None: ready_since=time.perf_counter()
            else: ready_since=None
            stable=0. if ready_since is None else time.perf_counter()-ready_since
            row=dict(elapsed_s=time.perf_counter()-began,phase=phase,sample_time=sample.timestamp,
                     joint_order=list(api.POLICY_JOINT_ORDER),target=requested.tolist(),rotor=q.tolist(),
                     outpos=out.tolist(),rotor_error=(requested-q).tolist(),outpos_error=(requested-out).tolist(),
                     rotor_speed=dq.tolist(),torque=api.feedback_telemetry(states)[0].tolist(),
                     tilt_deg=tilt,gyro=np.asarray(sample.angular_velocity_rad_s).tolist(),ready=ready,ready_for_s=stable)
            for field in ('raw_pos','raw_speed','raw_outpos'):
                row[field]=[getattr(states[n],field) for n in api.POLICY_JOINT_ORDER]
            row['feedback_rx_s']=[states[n].feedback_rx_s for n in api.POLICY_JOINT_ORDER]
            writer.writerow({k:json.dumps(v) if isinstance(v,list) else v for k,v in row.items()})
            if not require_ready and elapsed >= duration:
                print('12 s STAND transition complete; entering policy without readiness gate.',flush=True)
                return states
            if phase=='hold' and elapsed-duration >=2.0 and stable>=0.5:
                print(f'STAND verified: max rotor error={math.degrees(err):.2f} deg, tilt={tilt:.2f} deg; policy handover.',flush=True)
                return states
            if elapsed >= duration+5.0:
                raise api.DeploymentAbort(f'STAND not ready; policy NOT started: {api.POLICY_JOINT_ORDER[index]} rotor error={math.degrees(err):.2f} deg, max speed={max(abs(dq)):.3f} rad/s, tilt={tilt:.2f} deg, gyro={np.linalg.norm(sample.angular_velocity_rad_s):.3f} rad/s. See {path}')
            next_tick+=1/api.POLICY_RATE_HZ
            time.sleep(max(0.,next_tick-time.perf_counter()))
