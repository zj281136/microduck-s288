#!/usr/bin/env python3
"""Zero-gain S288 bus feedback diagnostic; support the robot externally."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from robot_control.arm.uniarm_l1_bus import UnitreeMotorSDK


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--ids',type=int,nargs='+',default=list(range(15)),help="bus IDs to probe; default: all 15 servos, ID 0..14")
    p.add_argument('--samples',type=int,default=3)
    p.add_argument('--timeout',type=float,default=.01)
    args=p.parse_args()
    if args.samples<1 or args.timeout<=0 or any(i<0 or i>14 for i in args.ids):p.error('samples/timeout must be positive; IDs must be 0..14')
    sdk=UnitreeMotorSDK(args.ids,args.port,timeout=args.timeout,position_target_source='rotor')
    failures=0
    try:
        for _ in range(args.samples):
            for motor_id in args.ids:
                tx=time.perf_counter()
                sdk.write_control_packet(motor_id=motor_id,pos=0,kp=0,kd=0,torque=0,speed=0,status=1,timeout_enable=0)
                state=sdk.read(expected_motor_id=motor_id)
                rx=time.perf_counter()
                if state is None:
                    failures+=1;print(json.dumps(dict(id=motor_id,tx_s=tx,rx_s=rx,error=sdk.last_read_error)));continue
                print(json.dumps(dict(id=motor_id,tx_s=tx,rx_s=rx,raw_pos=state.raw_pos,raw_speed=state.raw_speed,raw_OutPos=state.raw_outpos,q_rotor_equiv_rad=state.pos_rad,qdot_rotor_equiv_rad_s=state.speed_rads,q_out_rad=state.OutPos_rad,torque_nm=state.torque_nm,voltage=state.voltage,motor_error=state.MError,extra_error=state.ExFlag)))
    finally:
        sdk.close()
    return 1 if failures else 0


if __name__=='__main__':raise SystemExit(main())
