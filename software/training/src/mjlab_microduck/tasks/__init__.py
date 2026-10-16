# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
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

class MicroduckS288V0BacklashDRRunner(MicroduckOnPolicyRunner):
    """Independent rotor-only V0 experiment; preserve reference-run gains."""

    def __init__(self, env, train_cfg, log_dir=None, device="cpu", **kwargs):
        from .s288_v0_contract import ACTOR_TERMS
        if tuple(env.unwrapped.observation_manager.active_terms["actor"]) != ACTOR_TERMS:
            raise ValueError("V0 backlash DR actor term order mismatch")
        if env.get_observations()["actor"].shape[-1] != 61:
            raise ValueError("V0 backlash DR actor must be 61D")
        super().__init__(env, train_cfg, log_dir, device, **kwargs)

    def _contract(self):
        from .microduck_s288_v0_backlash_dr_env_cfg import CONTRACT_ID, METADATA
        return CONTRACT_ID, METADATA

    def save(self, path, infos=None):
        from mjlab.rl.exporter_utils import attach_metadata_to_onnx
        CONTRACT_ID, METADATA = self._contract()
        super().save(path, {**(infos or {}), "s288_v0_backlash_dr_contract": CONTRACT_ID})
        _, _, onnx_path = self._get_export_paths(path)
        if onnx_path.exists():
            attach_metadata_to_onnx(str(onnx_path), METADATA)

    def load(self, path, *args, **kwargs):
        import torch
        CONTRACT_ID, _ = self._contract()
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        if (checkpoint.get("infos") or {}).get("s288_v0_backlash_dr_contract") != CONTRACT_ID:
            raise ValueError("load a V0 backlash DR checkpoint; original rigid V0 stays separate")
        return super().load(path, *args, **kwargs)

from mjlab.tasks.registry import register_mjlab_task
from .microduck_s288_v0_backlash_dr_env_cfg import (
    TASK_ID, make_microduck_s288_v0_backlash_dr_env_cfg, MicroduckS288V0BacklashDRRlCfg,
)
register_mjlab_task(
    task_id=TASK_ID,
    env_cfg=make_microduck_s288_v0_backlash_dr_env_cfg(),
    play_env_cfg=make_microduck_s288_v0_backlash_dr_env_cfg(play=True),
    rl_cfg=MicroduckS288V0BacklashDRRlCfg,
    runner_cls=MicroduckS288V0BacklashDRRunner,
)
