# S288 velocity dependency subset; see THIRD_PARTY_NOTICES.md.
from pathlib import Path

import mujoco

from mjlab.entity import EntityCfg, EntityArticulationInfoCfg

from mjlab.utils.spec_config import CollisionCfg

from mjlab_microduck.actuator.s288_bam_m4 import S288BamM4ActuatorCfg

_MODEL = Path(__file__).parent / 'microduck/robot_walk_s288_duck.xml'

_PARAMS = Path(__file__).parents[1] / 'actuator/params/s288_hybrid_m4.json'

def get_s288_spec():
    return mujoco.MjSpec.from_file(str(_MODEL))

BASE_HOME_FRAME = EntityCfg.InitialStateCfg(
    joint_pos={
        r".*hip_yaw.*": 0.0,
        r".*left_hip_roll.*": -0.0907571211,
        r".*right_hip_roll.*": 0.0907571211,
        r".*left_hip_pitch.*": -0.3896435001,
        r".*right_hip_pitch.*": 0.3896435001,
        r".*left_knee.*": 0.0418953063,
        r".*right_knee.*": -0.0418953063,
        r".*left_ankle.*": 0.4343992564,
        r".*right_ankle.*": -0.4343992564,
        r".*neck_pitch.*": 0.1715470293,
        r".*head_pitch.*": 0.2636946273,
        r".*head_yaw.*": 0.0,
        r".*head_roll.*": 0.0,
    },
    joint_vel={".*": 0.0},
)

S288_HOME_FRAME = EntityCfg.InitialStateCfg(
    joint_pos={
        **BASE_HOME_FRAME.joint_pos,
        r".*left_hip_pitch.*": -0.3023770375,
        r".*right_hip_pitch.*": 0.3023770375,
        r".*left_ankle.*": 0.3122262088,
        r".*right_ankle.*": -0.3122262088,
    },
    joint_vel={".*": 0.0},
)

FULL_COLLISION = CollisionCfg(
    geom_names_expr=[".*_collision"],
    condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot_collision$": 1},
    friction={r"^(left|right)_foot_collision$": (1.0,)},
)

MICRODUCK_WALK_ROBOT_CFG = EntityCfg(
    spec_fn=get_s288_spec, init_state=S288_HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(S288BamM4ActuatorCfg(
            params_path=str(_PARAMS), gear_ratio=288.35,
            kp_sdk=15/1280000, kd_sdk=0.,
            output_torque_limit_nm_12v=0.6, vin=12.,
            target_names_expr=(r'^(?!passive_).*',),
            delay_min_lag=3, delay_max_lag=6,
        ),), soft_joint_pos_limit_factor=0.9,
    ),
)
