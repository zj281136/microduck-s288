"""Reuse S288 mechanical parameter definitions, without voltage scaling."""
from bam.actuators import actuators
from bam.testbench import Pendulum
from bam.unitree.actuator import UnitreeS288Actuator


class S288Hybrid12(UnitreeS288Actuator):
    def __init__(self):
        super().__init__(Pendulum)

    def load_log(self, log):
        # Deliberately bypass the historical voltage-dependent load_log.
        self.testbench = self.testbench_class(log)
        self.kp = log["kp"]
        self.damping = log["damping"]
        self.voltage_scale = 1.0


def register():
    actuators["s288_hybrid12"] = S288Hybrid12
