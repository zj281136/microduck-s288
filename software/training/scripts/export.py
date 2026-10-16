"""Official mjlab runner export for the one published S288 rotor task."""
import argparse
from dataclasses import asdict
from pathlib import Path
import torch
import onnx
import mjlab_microduck.tasks  # register only the S288 release task
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg,load_rl_cfg,load_runner_cls
from mjlab.rl.exporter_utils import get_base_metadata,attach_metadata_to_onnx
from mjlab_microduck.tasks.microduck_s288_v0_backlash_dr_env_cfg import TASK_ID,METADATA
from mjlab_microduck.tasks.s288_v0_contract import ACTOR_TERMS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint-file',type=Path,required=True)
    p.add_argument('--onnx-file',type=Path,required=True)
    args=p.parse_args()
    device='cuda:0' if torch.cuda.is_available() else 'cpu'
    cfg=load_env_cfg(TASK_ID,play=True);cfg.scene.num_envs=1
    raw=ManagerBasedRlEnv(cfg=cfg,device=device)
    try:
        agent=load_rl_cfg(TASK_ID)
        env=RslRlVecEnvWrapper(raw,clip_actions=agent.clip_actions)
        runner=load_runner_cls(TASK_ID)(env,asdict(agent),device=device)
        runner.load(str(args.checkpoint_file),map_location=device)
        target=args.onnx_file.resolve();target.parent.mkdir(parents=True,exist_ok=True)
        # The runner exports actor(obs_normalizer(obs)); never hand-convert weights.
        runner.export_policy_to_onnx(str(target.parent),target.name)
        metadata=get_base_metadata(raw,run_path=args.checkpoint_file.name)
        assert tuple(metadata['observation_names'])==ACTOR_TERMS
        model=onnx.load(target)
        assert [d.dim_value for d in model.graph.input[0].type.tensor_type.shape.dim]==[1,61]
        assert [d.dim_value for d in model.graph.output[0].type.tensor_type.shape.dim]==[1,14]
        attach_metadata_to_onnx(str(target),{**metadata,**METADATA})
        print(target)
    finally:
        raw.close()


if __name__=='__main__':main()
