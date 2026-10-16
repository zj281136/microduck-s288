# Curated S288 rotor-only dependency subset; see THIRD_PARTY_NOTICES.md.
"""Bilateral (left-right) symmetry augmentation for the microduck 61-D envs.

Migrated 2026-08-13 from the old 51-D layout to the current 61-D family
(velocity/velstand/standup/roulade — twist + head_command + body_command obs
slots), and the augmented-obs output key fixed "policy" → "actor" (mjlab
1.3.0 group naming; the old key would KeyError in rsl_rl 5.0.1's mirror-loss
path — dead code until now since no env had symmetry enabled).

Actor observation layout (61-dim flat tensor, concatenated in term insertion order):
    [0:3]   base_ang_vel      (roll, pitch, yaw  — body-frame IMU)
    [3:6]   projected_gravity (gx, gy, gz         — body-frame)
    [6:20]  joint_pos_rel     (14 joints, relative to default pose)
    [20:34] joint_vel_rel     (14 joints)
    [34:48] last_action       (14 joints)
    [48:51] twist command     (lin_vel_x, lin_vel_y, ang_vel_z)
    [51:55] head command      (neck_pitch, head_pitch, head_yaw, head_roll deltas)
    [55:61] body command      (x, y, z, roll, pitch, yaw deltas)

Joint ordering within each 14-dim block (from robot_walk.xml body tree):
    0: left_hip_yaw    5: neck_pitch    9:  right_hip_yaw
    1: left_hip_roll   6: head_pitch    10: right_hip_roll
    2: left_hip_pitch  7: head_yaw      11: right_hip_pitch
    3: left_knee       8: head_roll     12: right_knee
    4: left_ankle                       13: right_ankle

Mirroring rules (left-right reflection about the sagittal plane):
- Swap left legs (0-4) with right legs (9-13); midline joints (5-8) stay.
- Negate after swap:
    - hip_yaw, hip_roll: yaw/roll axes reverse under L-R reflection
    - hip_pitch, knee, ankle: home frame uses opposite-sign conventions for
      left vs right (e.g., left_hip_pitch = +0.6, right_hip_pitch = -0.6),
      so relative deviations also negate
    - head_yaw, head_roll: same yaw/roll reasoning
    - neck_pitch, head_pitch: sagittal-plane joints, no sign change
- base_ang_vel: negate roll ([0]) and yaw ([2]); pitch stays
- projected_gravity: negate gy ([4]); gx and gz stay
- twist command: negate lin_vel_y ([49]) and ang_vel_z ([50]); lin_vel_x stays
- head command: negate head_yaw ([53]) and head_roll ([54]); pitches stay
- body command: negate y ([56]), roll ([58]), yaw ([60]); x, z, pitch stay
"""

from dataclasses import dataclass

from mjlab.rl import RslRlPpoAlgorithmCfg

@dataclass
class PpoWithSymmetryCfg(RslRlPpoAlgorithmCfg):
    """PPO algorithm config extended with an optional symmetry_cfg field."""

    symmetry_cfg: dict | None = None

SYMMETRY_CFG = {
    "use_data_augmentation": False,
    "use_mirror_loss": True,
    "mirror_loss_coeff": 0.5,
    "data_augmentation_func": "mjlab_microduck.tasks.symmetry.microduck_vel_symmetry",
}
