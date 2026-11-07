# 成功行走版本的奖励与课程

对应 `Mjlab-Velocity-Flat-S288-MicroDuck`；完整冻结配置在 `software/training/tests/fixtures/velocity_baseline.json`。这是实际成功版本的配方，包含用户要求的调整，并非未经修改的上游默认奖励。

| reward | 初始 weight | function |
| --- | ---: | --- |
| action_rate_l2 | -0.1 | `action_rate_l2` |
| air_time | 3.0 | `feet_air_time` |
| angular_momentum | -0.02 | `angular_momentum_penalty` |
| body_ang_vel | -0.05 | `body_angular_velocity_penalty` |
| body_pose_tracking | 0.0 | `body_pose_tracking_6d` |
| dof_pos_limits | -5.0 | `joint_pos_limits` |
| foot_clearance | -2.0 | `feet_clearance` |
| foot_slip | -0.1 | `feet_slip` |
| foot_swing_height | -0.25 | `feet_swing_height` |
| head_pose_bias | 0.0 | `head_pose_bias_penalty` |
| head_pose_tracking | 2.0 | `head_pose_tracking` |
| leg_limit_proximity | -10.0 | `joint_pos_limit_proximity` |
| pose | 1.0 | `variable_posture` |
| self_collisions | -1.0 | `self_collision_cost` |
| servo_overspeed | -0.05 | `servo_joint_overspeed_l2` |
| track_angular_velocity | 2.0 | `track_angular_velocity` |
| track_linear_velocity | 2.0 | `track_linear_velocity` |
| upright | 2.0 | `upright` |

upright std=`sqrt(0.5)`，weight=2；airtime 0.125–0.300s，weight=3；dof_pos_limits=-5；腿部限位邻近 margin=0.1rad、weight=-10；关节速度超过8rad/s开始惩罚。

## 课程（单位：训练迭代）

- action_rate_weight: 0 → -0.1, 500 → -0.2, 750 → -0.4, 1000 → -0.6, 1250 → -0.8, 1500 → -1.0
- head_pose_bias_weight: 0 → 0.0, 600 → 1.0, 1000 → 2.0, 1500 → 3.0
- servo_overspeed_weight: 0 → -0.05, 500 → -0.1, 1000 → -0.2

头部指令范围逐步放宽至 ±[1.1,1.1,1.4,0.31]rad；精确零头部指令采样概率为0。静止 command 桶由2%增至25%，turn-in-place桶保留15%。

Termination：time_out、70°倾倒、越出地形、NaN；其余命令、DR和课程与冻结配置一致。PPO seed42，24 steps/env，5000 iterations。
