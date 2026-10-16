# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
"""model_6250 V0 recipe with experimental physical total play of 2--4 degrees.

Reference: velocity_s288_v0_rotor_pd_m1 /
2026-10-11_14-53-05_s288_v0_airtime3_upright005_10000 / params.
Keep rotor-only 61D, saved full gains, 0.55 Nm torque ceiling and V0 payloads.
All 14 passive widths/initial positions are DR assumptions, not measurements.
"""

from copy import deepcopy

from dataclasses import replace

from mjlab.managers import EventTermCfg

from mjlab.managers.scene_entity_config import SceneEntityCfg

from mjlab_microduck.robot.microduck_constants import get_s288_v0_backlash_dr_spec

from mjlab_microduck.tasks import mdp

from .backlash import make_backlash_variant

from .microduck_velocity_env_cfg import make_microduck_s288_v0_velocity_env_cfg, MicroduckS288V0RlCfg

HALF_WIDTH_RANGE_DEG = (1.0, 2.0)  # total physical play 2--4°, experimental

REFERENCE_KP_SDK = 1.3545e-4

REFERENCE_KD_SDK = 3.74e-6

CONTRACT_ID = "s288_v0_backlash_dr_rotor_pd_m1_61d_v1"

TASK_ID = "Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck"

METADATA = {
    "s288_observation_contract": CONTRACT_ID,
    "s288_dynamics_variant": "v0_backlash_dr_rotor_pd_m1",
    "s288_kp_sdk": str(REFERENCE_KP_SDK),
    "s288_kd_sdk": str(REFERENCE_KD_SDK),
    "s288_position_source": "rotor",
    "s288_qdot_extra_delay_steps": "0",
    "s288_backlash_half_width_deg": "1.0,2.0",
    "s288_backlash_initial_position": "uniform_within_episode_limits",
    "s288_reference_run": "2026-10-11_14-53-05_s288_v0_airtime3_upright005_10000",
}

def make_microduck_s288_v0_backlash_dr_env_cfg(play=False, rough=False):
    cfg = make_microduck_s288_v0_velocity_env_cfg(play=play, rough=rough)
    robot = deepcopy(cfg.scene.entities["robot"])
    robot.spec_fn = get_s288_v0_backlash_dr_spec
    # V0 constants now use reduced gains; pin the saved reference run's values.
    robot.articulation.actuators = tuple(replace(
        a, kp_sdk=REFERENCE_KP_SDK, kd_sdk=REFERENCE_KD_SDK,
        output_torque_limit_nm_12v=0.55,
    ) for a in robot.articulation.actuators)
    obs = {group: {term: cfg.observations[group].terms[term].func
                   for term in ("joint_pos", "joint_vel")}
           for group in ("actor", "critic")}
    cfg = make_backlash_variant(cfg, robot)
    # The generic XL330 wrapper uses output encoders; this V0 actor/critic must
    # retain their original active rotor position and velocity functions.
    for group, terms in obs.items():
        for term, func in terms.items():
            cfg.observations[group].terms[term].func = func
    # Match the saved run (these explicit values also protect against later
    # changes in the shared V0 recipe).
    cfg.rewards["air_time"].weight = 3.0
    cfg.rewards["upright"].weight = 1.0
    cfg.rewards["upright"].params["std"] = 0.22360679774997896
    # Append after all existing resets so passive state isn't overwritten.
    # Keep this DR in play too: it is the mechanics being evaluated, not a push.
    cfg.events["s288_passive_backlash_dr"] = EventTermCfg(
        func=mdp.randomize_s288_passive_backlash,
        mode="reset",
        params={"half_width_range_deg": HALF_WIDTH_RANGE_DEG,
                "asset_cfg": SceneEntityCfg("robot", joint_names=(r"^passive_.*_backlash$",))},
    )
    # The inherited armature event originally selected every joint (V0 had
    # only active hinges). Keep its original active-motor DR while leaving
    # newly added XL330 passive armatures exactly 0.001.
    if "randomize_armature" in cfg.events:
        cfg.events["randomize_armature"].params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=(r"^(?!passive_).*",)
        )
    return cfg

MicroduckS288V0BacklashDRRlCfg = replace(
    MicroduckS288V0RlCfg,
    experiment_name="velocity_s288_v0_backlash_dr_rotor_pd_m1",
    run_name="s288_v0_model6250_recipe_backlash_dr_2to4deg",
    max_iterations=10000,
    logger="tensorboard",
)
