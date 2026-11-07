"""Published S288 walking baseline, frozen from the validated model_4999 run.

Only names/packaging differ from the original training configuration.
The upright Gaussian uses sqrt(0.5); 0.5 is its squared width.
"""
import math
from dataclasses import replace
from mjlab.managers import RewardTermCfg, CurriculumTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from . import mdp
from .microduck_velocity_env_cfg import make_microduck_velocity_env_cfg, MicroduckRlCfg, NUM_STEPS_PER_ENV

TASK_ID = 'Mjlab-Velocity-Flat-S288-MicroDuck'
ENABLE_SERVO_OVERSPEED_PENALTY = True
SERVO_OVERSPEED_THRESHOLD_RAD_S = 8.0
SERVO_OVERSPEED_WEIGHT_STAGES = ((0, -0.05), (500, -0.1), (1000, -0.2))
UPRIGHT_STD = math.sqrt(0.5)
LEG_LIMIT_MARGIN_RAD = 0.1
LEG_LIMIT_JOINTS = (r'^(left|right)_(hip_yaw|hip_roll|hip_pitch|knee|ankle)$',)


def make_microduck_s288_velocity_env_cfg(play=False, rough=False):
    cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
    cfg.rewards['dof_pos_limits'].params['asset_cfg'] = SceneEntityCfg(
        'robot', joint_names=(r'^(?!passive_).*',))
    cfg.rewards['dof_pos_limits'].weight = -5.0
    cfg.rewards['leg_limit_proximity'] = RewardTermCfg(
        func=mdp.joint_pos_limit_proximity, weight=-10.0,
        params={'asset_cfg': SceneEntityCfg('robot', joint_names=LEG_LIMIT_JOINTS),
                'margin': LEG_LIMIT_MARGIN_RAD})
    if ENABLE_SERVO_OVERSPEED_PENALTY:
        cfg.rewards['servo_overspeed'] = RewardTermCfg(
            func=mdp.servo_joint_overspeed_l2,
            weight=SERVO_OVERSPEED_WEIGHT_STAGES[0][1],
            params={'threshold': SERVO_OVERSPEED_THRESHOLD_RAD_S})
        cfg.curriculum['servo_overspeed_weight'] = CurriculumTermCfg(
            func=mdp.reward_weight,
            params={'reward_name': 'servo_overspeed', 'weight_stages': [
                {'step': iteration * NUM_STEPS_PER_ENV, 'weight': weight}
                for iteration, weight in SERVO_OVERSPEED_WEIGHT_STAGES]})
    cfg.rewards['air_time'].params.update(threshold_min=0.125, threshold_max=0.3)
    cfg.rewards['air_time'].weight = 3.0
    cfg.rewards['upright'].params['std'] = UPRIGHT_STD
    cfg.rewards['upright'].weight = 2.0
    cfg.scene.num_envs = 4096
    return cfg


MicroduckS288VelocityRlCfg = replace(
    MicroduckRlCfg, experiment_name='microduck_s288_velocity',
    run_name='s288_velocity', logger='tensorboard', seed=42, max_iterations=5000)
