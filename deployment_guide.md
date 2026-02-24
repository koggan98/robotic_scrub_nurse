# Deployment Guide

## Robotic Scrub Nurse (UR3e)

This document describes how to deploy and run the Robotic Scrub Nurse system on a local workstation connected to a physical UR3e robot.

---

## 1. System Overview

The system consists of three main layers:

1. **Hardware drivers**
   - UR robot driver
   - Intel RealSense camera

2. **Motion planning backend**
   - MoveIt

3. **Application logic**
   - Hand tracking
   - Tool selection
   - Motion execution
   - Force-guided release

Each component runs in a separate terminal.

---

## 2. Prerequisites

### Software

Required:

- Ubuntu 22.04 + ROS2 Humble
- colcon
- MoveIt 2
- Universal Robots ROS 2 Driver
- Intel RealSense SDK

### Python Dependencies

```bash
pip install mediapipe pyrealsense2 tabulate
```

---

## 3. Network Setup

The robot must be configured with a static IP address.

### Robot Example Configuration

- Robot IP: 192.168.12.10
- Netmask: 255.255.255.0
- Gateway: 0.0.0.0

Update the IP address in the following files:
- `gripper_mover.py`
- `gripper_opener_with_zeroer.py`

Use the following command to find your workstation's IP address:

```bash
ifconfig
```

Then update the robot IP in the launch command accordingly:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.12.10 launch_rviz:=false
```

---

## 4. File Permissions

If scripts are not executable, grant permissions:

```bash
chmod +x src/publisher/camera_publisher.py
chmod +x src/publisher/frame_publisher.py
chmod +x src/publisher/box_publisher.py
chmod +x src/hand_tracker/hand_tracker.py
chmod +x src/socket_mover/socket_mover.py
chmod +x src/moveit_mover/gripper_mover.py
```

---

## 5. Physical Setup

Set up the hardware according to the system setup:

![System Setup](images/system_setup.jpeg)

---

## 6. Startup Procedure

Start each component in a separate terminal, in the following order:

### Terminal 1: Hardware Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.12.10 \
  launch_rviz:=false
```

### Terminal 2: MoveIt Motion Planning

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true
```

### Terminal 3: Tool Selector

```bash
ros2 run tracking_pkg tool_selection.py 
```

### Terminal 4: Main Logic

In terminal B
```bash
ros2 launch tracking_pkg loop_launch.py
```

---