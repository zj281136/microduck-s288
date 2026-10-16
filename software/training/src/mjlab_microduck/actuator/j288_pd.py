# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
"""First-pass Unitree J288 actuator model for MicroDuck sim2real work.

This intentionally models only the parts backed by current measurements or
Unitree's published specifications: output-side PD gains, a linear stall-
torque/no-load-speed envelope, and the measured sub-cycle command delay.  It
is a v1 baseline, not a fitted friction or thermal model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco

import mujoco_warp as mjwarp

import numpy as np

from mjlab.actuator import DcMotorActuator, DcMotorActuatorCfg

from mjlab.utils.spec import create_motor_actuator

class J288PdActuator(DcMotorActuator):
    """DC-motor PD actuator with gains selected by MicroDuck joint name."""

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        """Reuse MicroDuck's XML actuators instead of adding duplicate names."""
        target_set = set(target_names)
        converted: set[str] = set()

        for actuator in spec.actuators:
            target = actuator.target
            target_name = (
                target.name
                if hasattr(target, "name")
                else (str(target) if target else None)
            )
            if target_name not in target_set:
                continue

            actuator.set_to_motor()
            actuator.forcelimited = True
            actuator.forcerange = (-self.cfg.effort_limit, self.cfg.effort_limit)
            actuator.gear = [1.0, 0, 0, 0, 0, 0]
            self._mjs_actuators.append(actuator)
            converted.add(target_name)

        for target_name in target_names:
            if target_name not in converted:
                actuator = create_motor_actuator(
                    spec,
                    target_name,
                    effort_limit=self.cfg.effort_limit,
                    armature=self.cfg.armature,
                    frictionloss=self.cfg.frictionloss,
                    viscous_damping=self.cfg.viscous_damping,
                    transmission_type=self.cfg.transmission_type,
                )
                self._mjs_actuators.append(actuator)

        for joint in spec.joints:
            if joint.name in target_set:
                joint.armature = self.cfg.armature
                joint.frictionloss = self.cfg.frictionloss
                joint.damping = np.full((3, 1), self.cfg.viscous_damping)

    def initialize(
        self,
        mj_model: mujoco.MjModel,
        model: mjwarp.Model,
        data: mjwarp.Data,
        device: str,
    ) -> None:
        super().initialize(mj_model, model, data, device)
        assert self.stiffness is not None
        assert self.damping is not None

        missing_kp = set(self._target_names) - set(self.cfg.stiffness_by_name)
        missing_kd = set(self._target_names) - set(self.cfg.damping_by_name)
        if missing_kp or missing_kd:
            raise ValueError(
                f"J288 gain profile incomplete: missing Kp={sorted(missing_kp)}, "
                f"missing Kd={sorted(missing_kd)}"
            )

        for column, name in enumerate(self._target_names):
            self.stiffness[:, column] = self.cfg.stiffness_by_name[name]
            self.damping[:, column] = self.cfg.damping_by_name[name]

        # Gain randomization must always restore to the per-joint J288 profile.
        self.default_stiffness = self.stiffness.clone()
        self.default_damping = self.damping.clone()

@dataclass(kw_only=True)
class J288PdActuatorCfg(DcMotorActuatorCfg):
    """Configuration for the measured per-joint J288 v1 PD profile."""

    stiffness_by_name: dict[str, float] = field(default_factory=dict)
    damping_by_name: dict[str, float] = field(default_factory=dict)

    def build(self, entity, target_ids, target_names) -> J288PdActuator:
        return J288PdActuator(self, entity, target_ids, target_names)
