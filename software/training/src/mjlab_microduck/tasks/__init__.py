# S288 velocity dependency subset; see THIRD_PARTY_NOTICES.md.
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

class MicroduckOnPolicyRunner(VelocityOnPolicyRunner):
    def __init__(self, env, train_cfg: dict, log_dir=None, device="cpu", **kwargs):
        super().__init__(env, train_cfg, log_dir, device, **kwargs)
        # resolve_symmetry_config injects _env into train_cfg["algorithm"]["symmetry_cfg"]
        # in-place, sharing the same dict object with self.alg.symmetry.  Replace the
        # train_cfg reference with a copy that omits _env so dump_yaml can serialize the
        # config (MjSpec is not picklable), without touching the PPO's internal reference.
        alg = train_cfg.get("algorithm", {})
        sym = alg.get("symmetry_cfg") if isinstance(alg, dict) else None
        if isinstance(sym, dict) and "_env" in sym:
            alg["symmetry_cfg"] = {k: v for k, v in sym.items() if k != "_env"}

from mjlab.tasks.registry import register_mjlab_task
from .microduck_s288_velocity_env_cfg import (
    TASK_ID, make_microduck_s288_velocity_env_cfg, MicroduckS288VelocityRlCfg,
)
register_mjlab_task(
    task_id=TASK_ID, env_cfg=make_microduck_s288_velocity_env_cfg(),
    play_env_cfg=make_microduck_s288_velocity_env_cfg(play=True),
    rl_cfg=MicroduckS288VelocityRlCfg, runner_cls=MicroduckOnPolicyRunner,
)
