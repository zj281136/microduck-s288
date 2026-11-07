# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bam.actuator import Actuator
from bam.parameter import Parameter
from bam.testbench import Testbench

if TYPE_CHECKING:
    from bam.actuator import ArrayLike


class S288Pendulum(Testbench):
    """Pendulum with independent tip-load radius and uniform-arm length."""

    def __init__(self, log: dict):
        self.mass = log["mass"]
        self.arm_mass = log["arm_mass"]
        self.load_radius = log.get("load_radius", log["length"])
        self.arm_length = log.get("arm_length", log["length"])
        # Versioned bench exports provide measured COM/pivot inertia. Legacy
        # logs retain their documented uniform-arm convention.
        self.arm_com = log.get("arm_com", self.arm_length / 2.0)
        self.arm_inertia = log.get(
            "arm_inertia", self.arm_mass * self.arm_length**2 / 3.0
        )
        self.fixtures_gravity_moment = log.get("fixtures_gravity_moment", 0.0)
        self.fixtures_inertia = log.get("fixtures_inertia", 0.0)

    def compute_mass(self, q: float, dq: float) -> float:
        return (
            self.mass * self.load_radius**2 + self.arm_inertia + self.fixtures_inertia
        )

    def compute_bias(self, q: float, dq: float) -> float:
        gravity_moment = self.mass * self.load_radius
        gravity_moment += self.arm_mass * self.arm_com + self.fixtures_gravity_moment
        return -9.80665 * gravity_moment * np.sin(q)


class UnitreeGo1Actuator(Actuator):
    def __init__(self, testbench_class: Testbench, damping=0.3):
        super().__init__(testbench_class)

        # Damping factor
        self.damping = damping

    def initialize(self):
        # Torque multiplier
        self.model.ratio = Parameter(1.0, 0.5, 2.0)

        # Maximum torque [N.m]
        self.model.max_torque = Parameter(23.7, 20, 30)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.00001, 2.0)

        # Adjusting upper bounds for identification
        self.model.max_friction_base = 5.0
        self.model.max_load_friction = 2.0
        self.model.max_viscous_friction = 1.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "damping" in log:
            self.damping = log["damping"]

    def control_unit(self) -> str:
        return "N.m"

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> ArrayLike | None:
        # Target velocity is assumed to be 0
        torque = (q_target - q) * self.kp * self.model.ratio.value + self.damping * (
            0.0 - dq
        )
        torque = self.backend.clamp(
            torque, -self.model.max_torque.value, self.model.max_torque.value
        )

        return torque

    def compute_torque(
        self,
        control: ArrayLike | None,
        torque_enable: bool,
        q: ArrayLike,
        dq: ArrayLike,
    ) -> ArrayLike:
        torques = control * torque_enable

        return torques

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class UnitreeS288Actuator(Actuator):
    """Output-side model of the S288 firmware PD controller.

    S288 gains on the wire are rotor-side quantities.  Position error, speed,
    and torque are converted through the 288.35:1 gearbox, so both PD gains
    scale by the square of the gear ratio at the output shaft.
    """

    GEAR_RATIO = 288.35

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)
        self.kp = 3.01e-5
        self.damping = 1.08e-6
        self.vin = 12.0
        self.voltage_scale = 1.0

    def initialize(self):
        self.model.ratio = Parameter(1.0, 0.5, 2.0)
        # The protocol can represent up to about 36.9 Nm at the output.  Keep
        # the physical limit identifiable because the datasheet does not state
        # an S288 output torque limit.
        self.model.max_torque = Parameter(1.0, 0.1, 36.9)
        self.model.armature = Parameter(0.0001, 0.000001, 0.1)
        self.model.max_friction_base = 2.0
        self.model.max_load_friction = 2.0
        self.model.max_viscous_friction = 1.0

    def load_log(self, log: dict):
        super().load_log(log)
        self.kp = log["kp"]
        self.damping = log["damping"]
        self.vin = log.get("vin", 12.0)
        measured_voltage = log.get("measured_voltage_mean", self.vin)
        self.voltage_scale = measured_voltage / 12.0

    def control_unit(self) -> str:
        return "N.m"

    def compute_control(self, q_target, q, dq, dt):
        scale = self.GEAR_RATIO**2 * self.model.ratio.value
        torque = scale * (self.kp * (q_target - q) - self.damping * dq)
        torque_limit = self.model.max_torque.value * self.voltage_scale
        return self.backend.clamp(torque, -torque_limit, torque_limit)

    def compute_torque(self, control, torque_enable, q, dq):
        return control * torque_enable

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
