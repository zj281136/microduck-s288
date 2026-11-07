# S288 velocity dependency subset; see THIRD_PARTY_NOTICES.md.
"""BAM actuator with per-env friction-magnitude domain randomization.

The canonical ``bam.mjlab.BamActuator`` exposes per-env gain scaling (kp/kd) but
no friction hook, and under BAM MuJoCo's ``dof_frictionloss`` is zeroed in
``edit_spec`` (BAM computes friction itself in ``compute()``). So the stock
``dr.dof_frictionloss`` is a no-op here.

This thin subclass adds a per-env ``friction_scale`` that multiplies BAM's
velocity-INDEPENDENT friction budget (Coulomb + Stribeck + load-dependent) inside
``_compute_friction_budget`` — the term that carries the dominant sim2real
friction uncertainty (stiction / gearbox). The viscous (velocity-proportional)
term is left at nominal; scale it too by overriding ``compute`` if ever needed.

Non-accumulating: ``friction_scale`` is reset to 1.0 then set to a fresh sample
each episode by the ``randomize_bam_friction`` event (see tasks/mdp.py).
"""

from __future__ import annotations

import torch

from bam.mjlab import BamActuator

class FrictionDRBamActuator(BamActuator):
    """BamActuator + per-env friction_scale on the BAM friction budget."""

    def initialize(self, mj_model, model, data, device) -> None:
        super().initialize(mj_model, model, data, device)
        # kp_scale is (num_envs, 1); mirror it for a per-env friction multiplier.
        self.friction_scale = torch.ones_like(self.kp_scale)
        self.default_friction_scale = self.friction_scale.clone()

    def _compute_friction_budget(
        self,
        motor_torque: torch.Tensor,
        external_torque: torch.Tensor,
        stribeck_coeff: torch.Tensor,
    ) -> torch.Tensor:
        base = super()._compute_friction_budget(
            motor_torque, external_torque, stribeck_coeff
        )
        fs = getattr(self, "friction_scale", None)
        return base if fs is None else base * fs  # (N, J) * (N, 1)

    def set_friction_scale(self, env_ids, friction_scale: torch.Tensor) -> None:
        self.friction_scale[env_ids] = friction_scale

    def reset_friction_scale(self, env_ids) -> None:
        self.friction_scale[env_ids] = self.default_friction_scale[env_ids]
