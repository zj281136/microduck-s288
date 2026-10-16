# 训练配方索引与参数来源

以下权重来自保存的同task配置静态快照；curriculum会修改部分权重，不是全程常数。当前源码仍为最终依据。

| reward | 初始weight | function |
| --- | ---: | --- |
| action_rate_l2 | -0.1 | `action_rate_l2` |
| air_time | 3.0 | `feet_air_time` |
| angular_momentum | -0.02 | `angular_momentum_penalty` |
| body_ang_vel | -0.05 | `body_angular_velocity_penalty` |
| body_pose_tracking | 0.0 | `body_pose_tracking_6d` |
| dof_pos_limits | -1.0 | `joint_pos_limits` |
| foot_clearance | -2.0 | `feet_clearance` |
| foot_slip | -0.1 | `feet_slip` |
| foot_swing_height | -0.25 | `feet_swing_height` |
| head_pose_bias | 0.0 | `head_pose_bias_penalty` |
| head_pose_tracking | 2.0 | `head_pose_tracking` |
| pose | 1.0 | `variable_posture` |
| self_collisions | -1.0 | `self_collision_cost` |
| servo_overspeed | -0.05 | `servo_joint_overspeed_l2` |
| track_angular_velocity | 2.0 | `track_angular_velocity` |
| track_linear_velocity | 2.0 | `track_linear_velocity` |
| upright | 1.0 | `upright` |

## Termination

- time_out；fell_over角阈值约70°；terrain边界；NaN state。
- 完整参数见同task配置；安全终止不等同于真机max-speed。

## Command与DR

- twist范围：vx±0.4、vy±0.3m/s、wz±1rad/s；重采样3–8s；静止/前进/转向分桶比例依配置/课程。
- actor q/qdot延迟、IMU/反馈噪声、质量惯量/CoM/摩擦/armature、推力等由原配方保留。
- 新增的被动间隙事件只作用于passive，主动motor参数和观测不改。
- head/body slots保留；头部command课程与tracking仍存在，真机基线入口将head command锁零。

## 配方定位

`microduck_s288_v0_backlash_dr_env_cfg.py`固定引用run增益及air_time/upright等；`microduck_velocity_env_cfg.py`为共享构造链；`mdp.py`为自定义函数；`microduck_constants.py`提供模型/执行器/HOME。

PPO/MLP、num_steps_per_env=24和10000iterations见runner配置；完整checkpoint配置下载位置TODO。不能将reward名称/权重表视为重训性能保证。
