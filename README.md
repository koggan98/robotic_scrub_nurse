# Robotic Scrub Nurse (Work in Progress)

> **⚠️ Work in Progress**  
> This repository is under active development as part of an ongoing master thesis.  
> Structure, packages, and interfaces may change significantly.

---

## Overview

This repository contains the current development workspace for the **Robotic Scrub Nurse** research platform.

The project investigates collaborative robotic instrument handover using a **UR3e manipulator**, combining:

- motion planning and manipulation
- human pose and hand tracking
- force-guided interaction
- ergonomic tool transfer strategies

The goal is to create an extensible research platform for evaluating intelligent robotic assistance in surgical environments.

---

## Table of Contents

- [Main Components](#main-components)
- [Project Status](#project-status)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Deployment](#deployment)

---

## Main Components

### tracking_pkg

Core experimental package containing:

- motion execution logic (MoveIt 2)
- hand tracking integration (MediaPipe)
- event-based handover audio feedback (`/handover_event` -> system speakers via `paplay`/`pw-play`/`aplay`)
- tool selection workflows
- scene and visualization helpers
- RealSense camera integration

### Universal_Robots_ROS2_Gazebo_Simulation

Official UR simulation environment included as a git submodule for testing and development without hardware.

---

## Project Status

### Current Focus

- Migration and cleanup from earlier proof-of-concept systems
- Establishing a stable baseline for thesis development
- Modularization of motion and interaction logic
- Preparation for advanced planning and semantic handling

### Planned Additions

- Advanced motion planning pipelines
- Semantic tool handling
- Perception-driven grasp reasoning

---

## Requirements

### System Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble

### Hardware Requirements (for physical deployment)

- Universal Robots UR3e manipulator
- Intel RealSense D455 camera
- Robotiq 2F gripper

### Software Dependencies

- **ROS 2 Humble**: Core robotics framework
- **MoveIt 2**: Motion planning
- **ur_rtde**: Direct UR RTDE motion interface for the socket runtime path
- **Intel RealSense SDK**: Camera integration
- **MediaPipe**: Hand tracking and pose estimation
- **ALSA utils (`aplay`)**: runtime speaker playback for handover events

### Python Packages

```bash
pip install mediapipe pyrealsense2 tabulate ur_rtde
```

---

## Quick Start

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/koggan98/robotic_scrub_nurse.git
cd robotic_scrub_nurse_ws
```

### 2. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Test with Simulation

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

### 4. Deploy on Hardware

For detailed step-by-step instructions on physical deployment, see **[Deployment Guide](deployment_guide.md)**.

---

## Combined MoveIt + Tracking Launch

If you want Adam-style startup (MoveIt RViz + tracking topics in one command), use:

```bash
ros2 launch tracking_pkg loop_with_moveit_launch.py ur_type:=ur3e
```

This launch starts `ur_moveit_config` without its default RViz and opens RViz with a preloaded config that already includes:
- `/annotated_hand_image`
- `/gesture_pose_marker`
- `/hand_pose_marker`
- `/hand_pose`
- TF display (including frames such as `camera_frame` and `aruco_board_frame`)

During handover, the robot now holds briefly at the target pose before force-guided release sensing is enabled. Audio events are queued sequentially so the initial `gesture_detected` tone is not overwritten by a following `unreachable` tone.

## Loop Mover Profiles

`loop_mover` now loads its tool pick poses, handover orientations, offsets, and joint presets from:

- `src/tracking_pkg/config/loop_mover_profiles.yaml`

The `/tool_selection` interface is unchanged:

- `"0"` resets the system
- `"1"` to `"6"` select the configured tool profiles
- `"9"` starts the reclaim workflow from the last successful handover pose

The same YAML also contains reclaim settings for the dropoff pose, force threshold, reclaim timing (`zero_delay_seconds`, `post_close_wait_seconds`, `post_open_pause_seconds`), and reclaim gripper close parameters. The dropoff sequence now uses a two-step motion: approach the configured pose, lower by `0.05 m`, then open the gripper.

For hammer pickup in the MoveIt path, `loop_mover` now approaches the hammer with a pure `z` lift, descends straight down to grasp, and only applies the extra Cartesian offset on the return lift: `lift_height` in `z` plus `0.05 m` in `x`.

After changing the YAML values, restart the launch so `loop_mover`, `gripper_opener_with_zeroer`, and `reclaim_controller` reload the updated parameters.

## Socket RTDE Launch (MoveIt-free)

The new socket runtime path uses `ur_rtde` (controller-side IK) and keeps the same tool-selection and handover topic logic as `loop_mover`.

```bash
ros2 launch tracking_pkg web_socket_launch.py
```

This path reads tool/reclaim/orientation parameters from `src/tracking_pkg/config/loop_mover_profiles.yaml` under `socket_mover.ros__parameters`.

Hammer pickup tuning in the socket path uses the same `tool_profiles.<id>.post_lift_joint5_offset` parameter. For the current hammer profile (`tool_profiles.2`), the socket runtime also applies `0.2 rad` of positive `joint_5` rotation after the lift to reduce post-grasp collision risk.

By default, the socket path rotates incoming Cartesian pose targets by `pi` around Z before sending them to the UR controller. This matches the UR `base_link -> base` frame difference used by the controller and keeps the Cartesian targets aligned with the existing MoveIt-tuned coordinates.

The socket launch does not start the MiR RViz collision publisher. Instead, it adds the missing static `world -> base` transform for tracking, opens a local lightweight viewer node on `/annotated_hand_image` by default, and uses RTDE TCP force data directly for handover/reclaim sensing. No parallel `ur_robot_driver` runtime is required for this path. Disable the image window with `show_annotated_feed:=false`.
