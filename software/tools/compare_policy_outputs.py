#!/usr/bin/env python3
"""Manual offline checkpoint/ONNX comparison on supplied (N,61) observations.
Loads trusted checkpoint files only. Does not create an environment or hardware.
This entry requires training dependencies and is pending end-to-end validation.
"""
import argparse,json
from pathlib import Path
import numpy as np
import torch
import onnxruntime as ort
from tensordict import TensorDict
from rsl_rl.models import MLPModel


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--onnx',type=Path,required=True)
    p.add_argument('--observations',type=Path,required=True,help='Numpy .npy float observations in public61D order')
    p.add_argument('--tolerance',type=float,default=1e-5)
    args=p.parse_args()
    if args.tolerance<=0:p.error('tolerance must be positive')
    x=np.asarray(np.load(args.observations,allow_pickle=False),dtype=np.float32)
    if x.ndim!=2 or x.shape[1]!=61 or len(x)==0 or not np.isfinite(x).all():raise ValueError('observations must be finite nonempty (N,61)')
    # Reuse the selected recipe architecture, including EmpiricalNormalization.
    actor=MLPModel(TensorDict({'actor':torch.zeros(1,61)},batch_size=[1]),{'actor':['actor']},'actor',14,hidden_dims=[512,256,128],activation='elu',obs_normalization=True,distribution_cfg={'class_name':'GaussianDistribution','init_std':0.4,'std_type':'scalar'})
    checkpoint=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    infos=checkpoint.get('infos') or {}
    if infos.get('s288_v0_backlash_dr_contract')!='s288_v0_backlash_dr_rotor_pd_m1_61d_v1':raise ValueError('checkpoint contract mismatch')
    actor.load_state_dict(checkpoint['actor_state_dict']);actor.eval()
    session=ort.InferenceSession(str(args.onnx),providers=['CPUExecutionProvider'])
    if session.get_inputs()[0].shape!=[1,61] or session.get_outputs()[0].shape!=[1,14]:raise ValueError('expected61->14')
    meta=session.get_modelmeta().custom_metadata_map
    if meta.get('s288_observation_contract')!='s288_v0_backlash_dr_rotor_pd_m1_61d_v1':raise ValueError('ONNX contract mismatch')
    with torch.no_grad():a=actor(TensorDict({'actor':torch.tensor(x)},batch_size=[len(x)])).numpy()
    b=np.concatenate([session.run(None,{session.get_inputs()[0].name:row[None]})[0] for row in x])
    if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('non-finite output')
    error=float(np.max(abs(a-b)))
    print(json.dumps(dict(samples=len(x),max_abs_error=error,tolerance=args.tolerance,within_tolerance=error<=args.tolerance,closed_loop=False,hardware_accessed=False),indent=2))
    return 0 if error<=args.tolerance else 1


if __name__=='__main__':raise SystemExit(main())
