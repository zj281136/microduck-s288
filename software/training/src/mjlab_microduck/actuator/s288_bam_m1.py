# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
"""mjlab adapter for BAM's fitted Unitree S288 M1 model.

The bundled bam.mjlab.BamActuator supports VoltageControlledActuator only.
S288's BAM actuator is firmware PD with voltage-scaled torque saturation, so
this adapter implements that control law and BAM M1 friction on mjlab tensors.
"""

from __future__ import annotations

import json

import math

from dataclasses import dataclass

from pathlib import Path

import mujoco

import mujoco_warp as mjwarp

import numpy as np

import torch

from mjlab.actuator.actuator import Actuator, ActuatorCfg, ActuatorCmd, TransmissionType

from mjlab.utils.spec import create_motor_actuator

from mjlab_microduck.actuator.friction_dr_bam import FrictionDRBamActuator

@dataclass(kw_only=True)
class S288BamM1ActuatorCfg(ActuatorCfg):
    params_path: str
    gear_ratio: float
    kp_sdk: float
    kd_sdk: float
    output_torque_limit_nm_12v: float
    # Optional, disabled until a measured S288 frequency response identifies it.
    torque_response_tau_s: float = 0.0
    torque_response_tau_range_s: tuple[float, float] | None = None
    kp_scale_range: tuple[float, float] | None = None
    kd_scale_range: tuple[float, float] | None = None
    motor_strength_scale_range: tuple[float, float] | None = None
    # (absolute output speed [rad/s], available torque fraction), starting at 0.
    torque_speed_curve: tuple[tuple[float, float], ...] | None = None
    vin: float = 12.0
    vin_range: tuple[float, float] | None = None
    vin_drop_gain_range: tuple[float, float] | None = None
    vin_min: float | None = None
    stiff_frictionloss: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.transmission_type != TransmissionType.JOINT:
            raise ValueError("S288 BAM M1 requires joint transmission")
        if min(self.gear_ratio, self.kp_sdk, self.kd_sdk, self.vin,
               self.output_torque_limit_nm_12v) <= 0:
            raise ValueError("S288 ratio, gains and voltage must be positive")
        if self.vin_range is not None and not 0 < self.vin_range[0] <= self.vin_range[1]:
            raise ValueError("invalid S288 vin_range")
        if self.vin_drop_gain_range is not None and not (
            0 <= self.vin_drop_gain_range[0] <= self.vin_drop_gain_range[1]
        ):
            raise ValueError("invalid S288 vin_drop_gain_range")
        if self.vin_min is not None and self.vin_min <= 0:
            raise ValueError("S288 vin_min must be positive")
        if self.vin_drop_gain_range is not None and self.vin_min is None:
            raise ValueError("S288 voltage sag requires vin_min")
        if self.torque_response_tau_s < 0:
            raise ValueError("S288 torque response time must be non-negative")
        for name in ("torque_response_tau_range_s", "kp_scale_range",
                     "kd_scale_range", "motor_strength_scale_range"):
            bounds = getattr(self, name)
            lower = 0 if name == "torque_response_tau_range_s" else 1e-9
            if bounds is not None and not (lower <= bounds[0] <= bounds[1]):
                raise ValueError(f"invalid S288 {name}")
        if self.torque_speed_curve is not None:
            points = self.torque_speed_curve
            if len(points) < 2 or points[0][0] != 0 or points[0][1] != 1:
                raise ValueError("S288 torque-speed curve must start at (0, 1)")
            if any(not (math.isfinite(v) and math.isfinite(f) and 0 <= f <= 1)
                   for v, f in points):
                raise ValueError("invalid S288 torque-speed point")
            if any(b[0] <= a[0] or b[1] > a[1]
                   for a, b in zip(points, points[1:])):
                raise ValueError("S288 torque-speed curve must decrease with speed")

    def build(self, entity, target_ids, target_names) -> S288BamM1Actuator:
        return S288BamM1Actuator(self, entity, target_ids, target_names)

class S288BamM1Actuator(FrictionDRBamActuator):
    """BAM M1 PD, voltage saturation and Coulomb/viscous friction."""

    cfg: S288BamM1ActuatorCfg

    def __init__(self, cfg, entity, target_ids, target_names) -> None:
        # bam.mjlab.BamActuator.__init__ rejects the S288's non-voltage-control
        # actuator. Use its friction-field and gain-DR methods, but initialize
        # the M1 parameters and mjlab base state here.
        Actuator.__init__(self, cfg, entity, target_ids, target_names)
        params = json.loads(Path(cfg.params_path).read_text(encoding="utf-8"))
        if (params.get("model"), params.get("actuator")) != ("m1", "s288"):
            raise ValueError("S288 actuator requires fitted s288/m1 JSON")
        self.ratio = float(params["ratio"])
        self.max_torque = float(params["max_torque"])
        # The fitted ceiling was never excited by the identification logs.
        # Keep it as fit metadata, but bound the simulated output separately.
        self.output_torque_limit_nm_12v = cfg.output_torque_limit_nm_12v
        self.armature = float(params["armature"])
        self.friction_base = float(params["friction_base"])
        self.friction_viscous = float(params["friction_viscous"])
        if min(self.ratio, self.max_torque, self.armature, self.friction_base) <= 0:
            raise ValueError("invalid fitted S288 M1 parameters")
        self._mjwarp_model = None
        self._dof_ids = None
        self._friction_fields_checked = False
        self._num_envs = 0
        self._physics_dt_s = None
        self._torque_state = None

    @property
    def command_field(self) -> str:
        """mjlab delays position targets before the firmware PD law."""
        return "position"

    def get_command(self, data) -> ActuatorCmd:
        return Actuator.get_command(self, data)

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        max_vin = max(self.cfg.vin_range) if self.cfg.vin_range else self.cfg.vin
        force_limit = self.output_torque_limit_nm_12v * max_vin / 12.0
        target_set = set(target_names)
        converted = set()
        for mjact in spec.actuators:
            target = mjact.target
            target_name = target.name if hasattr(target, "name") else str(target)
            if target_name not in target_set:
                continue
            mjact.set_to_motor()
            mjact.forcelimited = True
            mjact.forcerange = (-force_limit, force_limit)
            mjact.gear = [1.0, 0, 0, 0, 0, 0]
            self._mjs_actuators.append(mjact)
            converted.add(target_name)
        for target_name in target_names:
            if target_name not in converted:
                self._mjs_actuators.append(
                    create_motor_actuator(
                        spec, target_name, effort_limit=force_limit,
                        armature=self.armature, frictionloss=0.0,
                        transmission_type=self.cfg.transmission_type,
                    )
                )
        for joint in spec.joints:
            if joint.name in target_set:
                joint.armature = self.armature
                joint.frictionloss = 0.0
                joint.damping = np.zeros((3, 1))
                self._set_friction_stiffness(joint)

    def initialize(
        self, mj_model: mujoco.MjModel, model: mjwarp.Model,
        data: mjwarp.Data, device: str,
    ) -> None:
        Actuator.initialize(self, mj_model, model, data, device)
        self._mjwarp_model = model
        self._device = device
        self._num_envs = data.nworld
        self._physics_dt_s = float(mj_model.opt.timestep)
        self._dof_ids = torch.tensor(
            [mj_model.jnt_dofadr[self.entity.indexing.joint_ids[i].item()]
             for i in self._target_ids_list],
            dtype=torch.long, device=device,
        )
        shape = (data.nworld, len(self._target_ids_list))
        self.kp_scale = torch.ones(shape, device=device)
        self.kd_scale = torch.ones(shape, device=device)
        self.default_kp_scale = self.kp_scale.clone()
        self.default_kd_scale = self.kd_scale.clone()
        self.friction_scale = torch.ones(data.nworld, 1, device=device)
        self.default_friction_scale = self.friction_scale.clone()
        self.motor_strength_scale = torch.ones(shape, device=device)
        self.torque_response_tau = torch.full(
            (data.nworld, 1), self.cfg.torque_response_tau_s, device=device
        )
        self.vin_tensor = torch.full((data.nworld, 1), self.cfg.vin, device=device)
        self.vin_drop_gain = (torch.zeros(data.nworld, 1, device=device)
                              if self.cfg.vin_drop_gain_range else None)
        self._prev_motor_torque = torch.zeros(shape, device=device)
        self._torque_state = torch.zeros(shape, device=device)
        self._sample_episode_parameters(slice(None))

    def _sample_episode_parameters(self, env_ids) -> None:
        """Sample absolute factors, never multiply previous episode values."""
        for tensor, bounds, nominal in (
            (self.kp_scale, self.cfg.kp_scale_range, 1.0),
            (self.kd_scale, self.cfg.kd_scale_range, 1.0),
            (self.motor_strength_scale, self.cfg.motor_strength_scale_range, 1.0),
            (self.vin_tensor, self.cfg.vin_range, self.cfg.vin),
            (self.torque_response_tau, self.cfg.torque_response_tau_range_s,
             self.cfg.torque_response_tau_s),
            (self.vin_drop_gain, self.cfg.vin_drop_gain_range, 0.0),
        ):
            if tensor is None:
                continue
            if bounds is None:
                tensor[env_ids] = nominal
            else:
                tensor[env_ids] = torch.empty_like(tensor[env_ids]).uniform_(*bounds)

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        Actuator.reset(self, env_ids)
        selected = slice(None) if env_ids is None else env_ids
        if self._torque_state is not None:
            self._torque_state[selected] = 0.0
        if self._prev_motor_torque is not None:
            self._prev_motor_torque[selected] = 0.0
        if self._num_envs:
            self._sample_episode_parameters(selected)

    def _speed_torque_fraction(self, speed: torch.Tensor) -> torch.Tensor:
        points = self.cfg.torque_speed_curve
        if points is None:
            return torch.ones_like(speed)
        x = speed.abs()
        result = torch.full_like(x, points[-1][1])
        for (v0, f0), (v1, f1) in zip(points, points[1:]):
            segment = f0 + (f1 - f0) * (x - v0) / (v1 - v0)
            result = torch.where((x >= v0) & (x < v1), segment, result)
        return result

    def compute(self, cmd: ActuatorCmd) -> torch.Tensor:
        # Matches bam.unitree.actuator.UnitreeS288Actuator.compute_control:
        # ratio * gearbox^2 * (kp*(q_target-q) - kd*qdot), then voltage-scaled
        # output torque ceiling. M1 adds only Coulomb and viscous friction.
        vin = self.vin_tensor
        if self.vin_drop_gain is not None:
            vin = vin - self.vin_drop_gain * self._prev_motor_torque.abs().sum(
                dim=-1, keepdim=True
            )
            if self.cfg.vin_min is not None:
                vin = torch.clamp(vin, min=self.cfg.vin_min)
        scale = self.cfg.gear_ratio**2 * self.ratio
        requested = scale * (
            self.cfg.kp_sdk * self.kp_scale * (cmd.position_target - cmd.pos)
            - self.cfg.kd_sdk * self.kd_scale * cmd.vel
        )
        strength = getattr(self, "motor_strength_scale", None)
        if strength is not None:
            requested = requested * strength
        limit = (self.output_torque_limit_nm_12v * vin / 12.0
                 * self._speed_torque_fraction(cmd.vel))
        motor_torque = torch.clamp(requested, -limit, limit)
        if self._torque_state is not None:
            tau = self.torque_response_tau
            alpha = torch.where(
                tau > 0,
                1.0 - torch.exp(-self._physics_dt_s / torch.clamp(tau, min=1e-9)),
                torch.ones_like(tau),
            )
            self._torque_state = self._torque_state + alpha * (motor_torque - self._torque_state)
            motor_torque = torch.clamp(self._torque_state, -limit, limit)
            self._torque_state = motor_torque
        self._write_frictions(
            torch.full_like(motor_torque, self.friction_base) * self.friction_scale,
            self.friction_viscous,
        )
        self._prev_motor_torque = motor_torque.detach()
        return motor_torque
