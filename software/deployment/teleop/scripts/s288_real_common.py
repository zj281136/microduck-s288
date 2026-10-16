#!/usr/bin/env python3
"""Run the S288 velocity policy on hardware with the real IMU deployment loop."""

import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(SCRIPT_DIR.parent))

import deploy_microduck_j288_real as deployment
import deploy_microduck_j288_onnx as fixture
from robot_control.arm.microduck_s288 import (
    MicroDuckS288,
    MicroDuckS288Error,
    S288_STAND_MODEL_POSITIONS,
    scaled_gain_profile,
)


# The policy observation and joint layouts are shared by both 288-series
# robots.  Select only the S288-specific hardware and deployment artifacts.
deployment.MicroDuckJ288 = MicroDuckS288
deployment.MicroDuckJ288Error = MicroDuckS288Error
deployment.scaled_gain_profile = scaled_gain_profile
deployment.STAND_MODEL_POSITIONS = S288_STAND_MODEL_POSITIONS
# clamp_targets is defined in the fixture module and reads its own STAND map.
fixture.STAND_MODEL_POSITIONS = S288_STAND_MODEL_POSITIONS
deployment.DEFAULT_CALIBRATION = SCRIPT_DIR / "microduck_s288_calibration.json"
deployment.DEFAULT_POLICY = SCRIPT_DIR / "models" / "microduck_s288_velocity.onnx"
deployment.DEFAULT_STANDUP_POLICY = None
deployment.DEFAULT_LOG_DIR = SCRIPT_DIR / "logs" / "real_imu_s288"
deployment.HARDWARE_NAME = "S288"
# The 11 rad/s limit is a MOTOR torque-speed parameter in training, not a
# commanded-position slew cap.  Keep the mechanical joint-range guard.
deployment.MAX_TARGET_SPEED_LIMIT = 11.0
deployment.DEFAULT_TARGET_SPEED = None
# S288 model_4500 was trained with unfiltered targets.  The upstream robotd
# runtime has head=0.5 / legs=0.7, while the corresponding mjlab action path
# has no filter.  Match this checkpoint until a filtered training run exists.
deployment.HEAD_TARGET_LOWPASS = None
deployment.LEGS_TARGET_LOWPASS = None
# The firmware owns torque saturation. Projecting a second PD envelope onto
# the target changes the action-to-target contract and was absent in training.
deployment.PD_GAIN_OUTPUT_SCALE = None
deployment.PD_EFFORT_LIMIT_NM = None
deployment.PD_SATURATION_EFFORT_NM = None
deployment.PD_VELOCITY_LIMIT_RAD_S = None
# Enter at the tested HOME pose.  Ramp only the requested velocity command;
# at a zero command the policy receives its full balancing action immediately,
# matching the training action path instead of scaling policy output over time.
deployment.POLICY_ENTRY_ACTION = (0.0,) * deployment.ACTION_SIZE
deployment.POLICY_RAMP_MODE = "command"


