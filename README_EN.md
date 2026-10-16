# Microduck-S288

**An Off-the-Shelf, Solder-Free, Open-Source Bipedal Robot**

[简体中文](README.md) · [Build guide](docs/BUILD_GUIDE.md) · [Training guide](docs/TRAIN_GUIDE.md)

Microduck-S288 is a S288 version of Pollen Robotics’ little walking duck. It brings together the motors, printed parts, Raspberry Pi and a learned policy—and the practical work needed to make them cooperate.

If you would like to build one too, start with the parts and assembly guide. If you already have a robot, the calibration and deployment chapters take you toward a first protected stand. The training path is there when you want to explore a gait of your own.

<p align="center">
  <img src="assets/images/robot/microduck-s288-assembled.jpg" alt="Microduck-S288 assembled robot" width="360">
</p>
<!-- TODO: Add a matching-version demonstration video with power/support conditions -->

## Features

- Off-the-shelf electronics: S288 motors, Raspberry Pi, USB motor controller and I2C IMU.
- A solder-free, no-custom-PCB hardware route; exact boards, ready-made harnesses and adapters must be documented in the BOM. Unknown connector/crimping requirements remain TODO.
- A build-to-deployment guide with separate pretrained and self-training paths.
- A matched policy interface: rotor-only61D,14 actions, M1 BAM and physical backlash DR.

Off-the-shelf does not mean no mechanical work, tools, printed parts, power supply or cables are required. Missing specifications are not presented as independently validated assembly steps.

## Published configuration

Original duck head with 15 S288 servos in the complete robot. The V0 M1 BAM / BacklashDR policy trains and controls 14 joints; the additional servo is outside the RL interface.

| Setting | Value |
| --- | --- |
| Actor/action dimensions | 61/14 |
| Feedback | rotor-equivalent position and protocol speed |
| Configured policy/physics rate | 50Hz/5ms |
| SDK Kp/Kd | 1.3545e-4/3.74e-6 |
| Transmission DR | total play2–4°, a training assumption |
| Included policy | model9999 ONNX with observation normalization |

Simulation limits are not manufacturer ratings. Each robot requires its own rotor zero/direction calibration.

## Where would you like to start?

**Build without training:** design → BOM → printing → assembly → wiring → Pi software → motor/IMU/communication checks → calibration → pretrained policy → protected stand and low-speed operation. This path does not require a training GPU.

**Train a policy:** matched model and actuator parameters → training environment → PPO checkpoint → ONNX export → numerical/closed-loop validation → matched hardware deployment.

## Documentation

- [Architecture](docs/00-overview.md) and [design decisions](docs/01_design/README.md)
- [BOM](BOM.md) and [SolidWorks/STEP/STL/MJCF](docs/02_hardware/model_files.md)
- [Printing](docs/01-mechanics-printing.md), [assembly](docs/02-assembly.md), [wiring](docs/03-electronics.md)
- [Environment setup](docs/03_software_setup/environment_setup.md)
- [Rotor calibration](docs/04-motor-calibration.md), [joint/IMU/communication tests](docs/04_calibration/testing.md)
- [61D contract](docs/05-control-contract.md), [training](docs/06-training.md)
- [ONNX export/validation](docs/05_reinforcement_learning/onnx_export.md), [deployment](docs/07-deployment.md)
- [PD/BAM and Sim2Real notes](docs/07_engineering_notes/README.md), [troubleshooting](docs/08-troubleshooting.md)

Detailed chapters prioritize Chinese. CAD, print-ready files, exact fasteners, pinouts, purchasing links and test figures still have TODO entries. Simulation meshes are not automatically validated print files.

## Safety and attribution

Software/model checks do not verify wiring, calibration or physical safety. Use external support and an independent power cut-off; zero-gain shutdown does not hold the robot upright.

Based on [Microduck](https://github.com/pollen-robotics/microduck), [Microduck RL](https://github.com/pollen-robotics/microduck_rl), [mjlab](https://github.com/mujocolab/mjlab), [BAM](https://github.com/Rhoban/bam) and [UniArmL1](https://github.com/unitreerobotics/UniArmL1). Documentation organization references [microduck-replica](https://github.com/fanhao375/microduck-replica). See [NOTICE](NOTICE.md) and [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md) for resource provenance and license scope.
