# S288 velocity dependency subset; see THIRD_PARTY_NOTICES.md.
"""S288 Hybrid12 + BAM M4, using the identified output-shaft parameters.

Reference: bam/outputs/s288_11v5_hybrid_m1_m6_20261105/snapshots/{
 bam/model.py, bam/unitree/identification12/model.py, bam/unitree/actuator.py}.
Reuse the M1 mjlab adapter for model editing, gains/DR and friction writes.
No electrical model or voltage-dependent ceiling is used.
"""

from dataclasses import dataclass

import math

import torch

from .s288_bam_m1 import S288BamM1Actuator, S288BamM1ActuatorCfg

class S288BamM4Actuator(S288BamM1Actuator):
    fit_model = 'm4'
    fit_actuator = 's288_hybrid12'

    def __init__(self, cfg, entity, target_ids, target_names):
        super().__init__(cfg, entity, target_ids, target_names)
        import json
        from pathlib import Path
        self.params = json.loads(Path(cfg.params_path).read_text())
        for key, value in self.params.items():
            if key not in ('model', 'actuator') and not math.isfinite(float(value)):
                raise ValueError(f'Nonfinite M4 parameter: {key}')
        for key in ('friction_stribeck', 'load_friction_base',
                    'load_friction_stribeck', 'friction_viscous', 'command_delay'):
            if self.params[key] < 0:
                raise ValueError(f'Negative M4 parameter: {key}')
        if min(self.params['alpha'], self.params['dtheta_stribeck']) <= 0:
            raise ValueError('M4 Stribeck scale and exponent must be positive')
        if (cfg.torque_limit_scales_with_voltage or cfg.vin_range is not None
                or cfg.vin_drop_gain_range is not None
                or cfg.electrical_resistance_ohm is not None
                or cfg.torque_speed_curve is not None or cfg.torque_response_tau_s
                or cfg.torque_response_tau_range_s is not None
                or cfg.motor_strength_scale_range is not None):
            raise ValueError('Hybrid12 M4 does not use electrical/extra torque models')
        self._goal_history = None

    def initialize(self, mj_model, model, data, device):
        super().initialize(mj_model, model, data, device)
        self._data = data
        lag = self.params['command_delay'] / self._physics_dt_s
        self._delay_steps = int(math.floor(lag))
        self._delay_fraction = lag - self._delay_steps
        self._goal_history = torch.zeros(
            (self._delay_steps + 2, data.nworld, len(self._target_ids_list)),
            device=device)
        self._goal_ready = torch.zeros(data.nworld, dtype=torch.bool, device=device)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if self._goal_history is not None:
            self._goal_ready[slice(None) if env_ids is None else env_ids] = False

    def _delayed_goal(self, goal):
        # Same linear fractional-delay interpolation as BAM simulate.py.
        fresh = ~self._goal_ready
        self._goal_history[:, fresh] = goal[fresh]
        self._goal_ready[:] = True
        self._goal_history = torch.cat((goal.unsqueeze(0), self._goal_history[:-1]), dim=0)
        i, f = self._delay_steps, self._delay_fraction
        return (1 - f) * self._goal_history[i] + f * self._goal_history[i + 1]

    def friction_budget(self, motor, external, velocity):
        p = self.params
        s = torch.exp(-(velocity.abs() / p['dtheta_stribeck']) ** p['alpha'])
        load = (external - motor).abs()
        return (p['friction_base'] + p['load_friction_base'] * load
                + s * (p['friction_stribeck'] + p['load_friction_stribeck'] * load)
                ) * self.friction_scale

    def _update_frictions(self, cmd, motor_torque):
        # BAM mjlab's lagged external load: exclude our own dry constraint
        # before interpreting qfrc_constraint as load at the gearbox.
        bias = self._as_tensor(self._data.qfrc_bias)
        constraint = self._as_tensor(self._data.qfrc_constraint)
        own = self._dof_friction_force(bias.shape[-1])
        external = (-bias + constraint - own)[:, self._dof_ids]
        previous_motor = self._as_tensor(self._data.qfrc_actuator)[:, self._dof_ids]
        self._write_frictions(self.friction_budget(previous_motor, external, cmd.vel),
                              self.friction_viscous)

    def compute(self, cmd):
        goal = self._delayed_goal(cmd.position_target)
        requested = self.cfg.gear_ratio**2 * self.ratio * (
            self.cfg.kp_sdk * self.kp_scale * (goal - cmd.pos)
            - self.cfg.kd_sdk * self.kd_scale * cmd.vel)
        limit = self.output_torque_limit_nm_12v
        torque = requested.clamp(-limit, limit)
        self._update_frictions(cmd, torque)
        return torque

@dataclass(kw_only=True)
class S288BamM4ActuatorCfg(S288BamM1ActuatorCfg):
    torque_limit_scales_with_voltage: bool = False

    def build(self, entity, target_ids, target_names):
        return S288BamM4Actuator(self, entity, target_ids, target_names)
