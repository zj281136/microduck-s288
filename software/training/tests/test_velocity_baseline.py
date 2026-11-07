"""Regression against the validated training configuration; no hardware I/O."""
import dataclasses,enum,hashlib,json
from pathlib import Path
import numpy as np
import torch
from mjlab.tasks.registry import load_env_cfg,load_rl_cfg
import mjlab_microduck.tasks
from mjlab_microduck.tasks.microduck_s288_velocity_env_cfg import TASK_ID
from mjlab_microduck.actuator.s288_bam_m4 import S288BamM4Actuator
def freeze(x):
 if dataclasses.is_dataclass(x):return {f.name:freeze(getattr(x,f.name)) for f in dataclasses.fields(x)}
 if isinstance(x,dict):return {str(k):freeze(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [freeze(y) for y in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,enum.Enum):return str(x)
 if callable(x):return x.__module__+'.'+x.__qualname__
 if isinstance(x,Path):return str(x)
 if x is None or isinstance(x,(str,int,float,bool)):return x
 return str(x)

def test_frozen_training_and_play_configuration():
    expected=json.loads((Path(__file__).parent/'fixtures/velocity_baseline.json').read_text())
    for key,play in (('env',False),('play',True)):
        cfg=freeze(load_env_cfg(TASK_ID,play=play))
        robot=cfg['scene']['entities']['robot']
        robot['spec_fn']='ROBOT_SPEC'
        robot['articulation']['actuators'][0]['params_path']='FIT_PARAMS'
        assert cfg==expected[key]
    assert freeze(load_rl_cfg(TASK_ID))==expected['agent']
    model=load_env_cfg(TASK_ID).scene.entities['robot'].build().spec.compile()
    assert model.nu==expected['nu'] and model.njnt==expected['njnt']
    for name,digest in expected['model'].items():
        assert hashlib.sha256(getattr(model,name).tobytes()).hexdigest()==digest,name
    assert np.isclose(model.body_mass.sum(),expected['mass'])


def test_m4_load_dependent_stribeck_budget():
    actuator=object.__new__(S288BamM4Actuator)
    actuator.params=json.loads((Path(__file__).parents[1]/'src/mjlab_microduck/actuator/params/s288_hybrid_m4.json').read_text())
    actuator.friction_scale=torch.tensor([[0.9,1.0,1.1]])
    motor=torch.tensor([[.2,-.3,0.]])
    external=torch.tensor([[-.4,.1,.2]])
    velocity=torch.tensor([[0.,.4,-2.]])
    p=actuator.params
    expected=[]
    for m,e,v,scale in zip(motor[0],external[0],velocity[0],actuator.friction_scale[0]):
        load=abs(float(e-m));s=np.exp(-(abs(float(v))/p['dtheta_stribeck'])**p['alpha'])
        expected.append((p['friction_base']+p['load_friction_base']*load+s*(p['friction_stribeck']+p['load_friction_stribeck']*load))*float(scale))
    np.testing.assert_allclose(actuator.friction_budget(motor,external,velocity).numpy()[0],expected,rtol=1e-6)
