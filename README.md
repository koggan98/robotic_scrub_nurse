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
- **Intel RealSense SDK**: Camera integration
- **MediaPipe**: Hand tracking and pose estimation

### Python Packages

```bash
pip install mediapipe pyrealsense2 tabulate
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