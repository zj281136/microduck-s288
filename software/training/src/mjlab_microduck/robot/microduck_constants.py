# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
import os

from dataclasses import replace

from pathlib import Path

import mujoco

from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

from mjlab.utils.spec_config import CollisionCfg

from mjlab_microduck.actuator import BacklashEncoderBamActuatorCfg, FrictionDRBamActuatorCfg, J288PdActuatorCfg, S288BamM1ActuatorCfg

_ROBOT_DIR: Path = Path(os.path.dirname(__file__)) / "microduck"

MICRODUCK_WALK_XML: Path = _ROBOT_DIR / "robot_walk.xml"

MICRODUCK_J288_V1_WALK_XML: Path = _ROBOT_DIR / "robot_walk_j288_v1.xml"

MICRODUCK_S288_V1_WALK_XML: Path = _ROBOT_DIR / "robot_walk_s288_v1.xml"

MICRODUCK_ALLCOLLISIONS_BACKLASH_XML: Path = _ROBOT_DIR / "robot_allcollisions_backlash.xml"

def get_walk_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(MICRODUCK_WALK_XML))

def get_j288_v1_walk_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(MICRODUCK_J288_V1_WALK_XML))

def get_s288_base_walk_spec() -> mujoco.MjSpec:
    """Original 140 g battery model used by the earlier S288 M1 task."""
    return mujoco.MjSpec.from_file(str(MICRODUCK_S288_V1_WALK_XML))

def get_s288_v0_walk_spec() -> mujoco.MjSpec:
    """S288 V0 with the 155 g battery and Pi Zero 2 W at its default mesh pose.

    Only V0 gets these payloads; other S288 tasks still use the original XML.
    Extra battery mass uses a uniform 24x31x73 mm box at its existing pose.
    The 18 g Pi uses a uniform 65x30x1.6 mm box at the head mesh center.
    Values below include parallel-axis corrections to the link inertias.
    """
    spec = get_s288_base_walk_spec()
    trunk = spec.body("trunk_base")
    trunk.mass = 0.2758
    trunk.ipos = [-0.024952091616771, -5.27787934839739e-05, 0.00198023431890268]
    trunk.fullinertia = [
        0.000158478036944552, 0.000244424858741805, 0.000179885579405728,
        2.6899325589654e-07, -5.16911212822141e-05, 2.87722151626016e-07,
    ]
    head = spec.body("jaw_soft")
    head.mass = 0.0952
    head.ipos = [0.00711322752646952, 0.00297705488262605, -0.0389980864254202]
    head.fullinertia = [
        0.000126164288877226, 9.13927114961115e-05, 6.74478588347955e-05,
        1.04322076982274e-06, 9.46669333537e-06, 9.01596420205608e-07,
    ]
    return spec

def get_s288_v0_backlash_dr_spec() -> mujoco.MjSpec:
    """V0 payload/geometry plus 14 serial passive hinges; nominal half-play 2°.

    Match model_6250's V0 link inertias (155 g battery, 18 g head Pi), rather
    than the older 140 g Backlash61D model. The reset event samples 1--2°.
    Passive engineering parameters match the original XL330 backlash recipe.
    """
    spec = get_s288_v0_walk_spec()
    active = [j for j in spec.joints if j.name and not j.name.startswith("passive_")
              and j.type == mujoco.mjtJoint.mjJNT_HINGE]
    if len(active) != 14:
        raise ValueError(f"expected 14 V0 active hinges, got {len(active)}")
    for joint in active:
        joint.parent.add_joint(
            name=f"passive_{joint.name}_backlash",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=joint.axis.copy(), pos=joint.pos.copy(),
            limited=True, range=[-0.03490658503988659, 0.03490658503988659],
            damping=0.01, armature=0.001, frictionloss=0.0,
            solref_limit=[0.01, 1.0], solimp_limit=[0.95, 0.999, 0.0001, 0.5, 2.0],
        )
    return spec

def get_backlash_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(MICRODUCK_ALLCOLLISIONS_BACKLASH_XML))

HOME_FRAME = EntityCfg.InitialStateCfg(
    joint_pos={
        # Lower body — STAND2 pose: trunk shifted ~5mm forward over the feet so
        # the CoM sits over the ankle axis (was ~5mm behind it at the old HOME,
        # which biased the robot backward and made the standup policy droop its
        # head forward as a counterweight). Leg pitch chain leaned forward:
        # hip_pitch 30°→26.24°, ankle 30°→25.95°, knee 0°→0.28°. Matches the
        # STAND keyframe in scene.xml / scene_walk.xml.
        r".*hip_yaw.*": 0.0,
        r".*left_hip_roll.*": -0.0873,
        r".*right_hip_roll.*": 0.0873,
        r".*left_hip_pitch.*": -0.4579,
        r".*right_hip_pitch.*": 0.4579,
        r".*left_knee.*": -0.0049,
        r".*right_knee.*": 0.0049,
        r".*left_ankle.*": 0.4530,
        r".*right_ankle.*": -0.4530,
        # Head
        r".*neck_pitch.*": 0.3491,
        r".*head_pitch.*": 0.3491,
        r".*head_yaw.*": 0.0,
        r".*head_roll.*": 0.0,
    },
    joint_vel={".*": 0.0},
)

J288_HOME_FRAME = EntityCfg.InitialStateCfg(
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

S288_V0_HOME_FRAME = EntityCfg.InitialStateCfg(
    joint_pos={
        **J288_HOME_FRAME.joint_pos,
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

_BAM_ACTUATOR_KWARGS = dict(
    motor_name="xl330",
    model="m6",
    target_names_expr=(r"^(?!passive_).*",),
    kp_fw=200.0,  # microduck's preserved firmware stiffness (microban uses 125)
    # vin_range=(6.9, 7.9),
    vin_range=(6.5, 8.2),
    vin_drop_gain_range=(0.0, 0.2),
    vin_min=6.0,
    # max_current=1.75,
    delay_min_lag=3,
    delay_max_lag=6,
)

actuators = FrictionDRBamActuatorCfg(**_BAM_ACTUATOR_KWARGS)

backlash_actuators = BacklashEncoderBamActuatorCfg(**_BAM_ACTUATOR_KWARGS)

J288_GEAR_RATIO = 288.35

J288_V1_STIFFNESS = 5.0e-5 * J288_GEAR_RATIO**2

J288_V1_DAMPING_NORMAL = 4.0e-6 * J288_GEAR_RATIO**2

J288_V1_TRAINING_KP_SCALE = 1.3

J288_JOINT_NAMES = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)

J288_V1_KP_SDK_BY_NAME = {
    "left_hip_yaw": 5.0e-5,
    "left_hip_roll": 7.0e-5,
    "left_hip_pitch": 6.5e-5,
    "left_knee": 7.0e-5,
    "left_ankle": 6.5e-5,
    "neck_pitch": 5.0e-5,
    "head_pitch": 5.0e-5,
    "head_yaw": 5.0e-5,
    "head_roll": 5.0e-5,
    "right_hip_yaw": 5.0e-5,
    "right_hip_roll": 7.0e-5,
    "right_hip_pitch": 6.5e-5,
    "right_knee": 7.0e-5,
    "right_ankle": 6.5e-5,
}

J288_V1_KD_SDK_BY_NAME = {
    "left_hip_yaw": 4.0e-6,
    "left_hip_roll": 3.5e-6,
    "left_hip_pitch": 4.0e-6,
    "left_knee": 4.0e-6,
    "left_ankle": 4.0e-6,
    "neck_pitch": 3.0e-6,
    "head_pitch": 3.0e-6,
    "head_yaw": 4.0e-6,
    "head_roll": 4.0e-6,
    "right_hip_yaw": 4.0e-6,
    "right_hip_roll": 3.5e-6,
    "right_hip_pitch": 4.0e-6,
    "right_knee": 4.0e-6,
    "right_ankle": 4.0e-6,
}

J288_V1_STIFFNESS_BY_NAME = {
    name: gain * J288_V1_TRAINING_KP_SCALE * J288_GEAR_RATIO**2
    for name, gain in J288_V1_KP_SDK_BY_NAME.items()
}

J288_V1_DAMPING_BY_NAME = {
    name: gain * J288_GEAR_RATIO**2
    for name, gain in J288_V1_KD_SDK_BY_NAME.items()
}

j288_v1_actuators = J288PdActuatorCfg(
    target_names_expr=tuple(rf"^{name}$" for name in J288_JOINT_NAMES),
    # Scalar values satisfy the base cfg; initialize() replaces them by name.
    stiffness=J288_V1_STIFFNESS,
    damping=J288_V1_DAMPING_NORMAL,
    stiffness_by_name=J288_V1_STIFFNESS_BY_NAME,
    damping_by_name=J288_V1_DAMPING_BY_NAME,
    effort_limit=0.65,
    saturation_effort=0.65,
    velocity_limit=7.0,
    # Real-hardware phase measurements show roughly 20--40 ms of actuator-side
    # lag after removing the one-row logging offset.  At the 50 Hz policy rate,
    # randomize command delay over 1--2 policy steps for sim-to-real robustness.
    delay_min_lag=1,
    delay_max_lag=2,
    delay_hold_prob=0.9,
    delay_update_period=4,
    armature=0.0018,  # v1 placeholder inherited from the current MJCF
    # Single-motor output-side identification: ~0.04 N.m to break away and
    # ~0.03 N.m to sustain rotation.  MuJoCo frictionloss is Coulomb friction,
    # so use the measured moving value; training randomizes around it.
    frictionloss=0.03,
    viscous_damping=0.0,  # controlled damping is already represented by Kd
)

S288_GEAR_RATIO = 288.35

S288_M1_BAM_OUTPUT_TORQUE_LIMIT_NM_12V = 0.8

S288_IDENTIFIED_KP_SDK = 9.03e-5

S288_V0_GAIN_REDUCTION = 2.0 / 3.0

S288_V0_KP_SCALE = 1.5 * S288_V0_GAIN_REDUCTION

S288_V0_KP_SDK = S288_IDENTIFIED_KP_SDK * S288_V0_KP_SCALE

S288_M1_BAM_KD_SDK = 3.74e-6

S288_V0_KD_SDK = S288_M1_BAM_KD_SDK * S288_V0_GAIN_REDUCTION

S288_M1_BAM_KP_SCALE = 1.5

S288_M1_BAM_KP_SDK = S288_IDENTIFIED_KP_SDK * S288_M1_BAM_KP_SCALE

S288_M1_BAM_PARAMS = Path(__file__).parents[1] / "actuator/params/s288_m1.json"

s288_m1_bam_actuators = S288BamM1ActuatorCfg(
    params_path=str(S288_M1_BAM_PARAMS),
    target_names_expr=(r"^(?!passive_).*",),
    gear_ratio=S288_GEAR_RATIO,
    kp_sdk=S288_M1_BAM_KP_SDK,
    kd_sdk=S288_M1_BAM_KD_SDK,
    output_torque_limit_nm_12v=S288_M1_BAM_OUTPUT_TORQUE_LIMIT_NM_12V,
    vin=11.0,
    # S288 voltage range used by this configuration: 11--12 V. Feedback values below
    # 11 V in the old logs are erroneous and must not enter voltage DR.
    # No load-sag coefficient identified; sag remains disabled.
    vin_range=(11.0, 12.0),
    # Inference-end to per-motor TX: median 5.4 ms, p99 10.8 ms, rare 22.6 ms.
    # 0--5 physics steps cover the observed dispatch tail at 5 ms resolution;
    # this is transport/scheduling lag, not the motor's full response phase.
    delay_min_lag=0,
    delay_max_lag=5,
    delay_hold_prob=0.9,
    delay_update_period=4,
)

S288_V0_OUTPUT_TORQUE_LIMIT_NM_12V = 0.55

s288_v0_actuators = replace(
    s288_m1_bam_actuators,
    kp_sdk=S288_V0_KP_SDK,
    kd_sdk=S288_V0_KD_SDK,
    output_torque_limit_nm_12v=S288_V0_OUTPUT_TORQUE_LIMIT_NM_12V,
)

BACKLASH_HOME_FRAME = EntityCfg.InitialStateCfg(
    joint_pos={r".*_backlash$": 0.0, **HOME_FRAME.joint_pos},
    joint_vel={".*": 0.0},
)

MICRODUCK_WALK_ROBOT_CFG = EntityCfg(
    spec_fn=get_walk_spec,
    init_state=HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(actuators,),
        soft_joint_pos_limit_factor=0.9,
    ),
)

MICRODUCK_J288_V1_WALK_ROBOT_CFG = EntityCfg(
    spec_fn=get_j288_v1_walk_spec,
    init_state=J288_HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(j288_v1_actuators,),
        soft_joint_pos_limit_factor=0.9,
    ),
)

MICRODUCK_S288_V0_WALK_ROBOT_CFG = EntityCfg(
    spec_fn=get_s288_v0_walk_spec,
    init_state=S288_V0_HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(s288_v0_actuators,),
        soft_joint_pos_limit_factor=0.9,
    ),
)

MICRODUCK_S288_M1_BAM_WALK_ROBOT_CFG = EntityCfg(
    spec_fn=get_s288_base_walk_spec,
    init_state=S288_V0_HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(s288_m1_bam_actuators,),
        soft_joint_pos_limit_factor=0.9,
    ),
)

MICRODUCK_BACKLASH_ROBOT_CFG = EntityCfg(
    spec_fn=get_backlash_spec,
    init_state=BACKLASH_HOME_FRAME,
    collisions=(FULL_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(backlash_actuators,),
        soft_joint_pos_limit_factor=0.9,
    ),
)
