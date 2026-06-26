# Deployment Guide

## Robotic Scrub Nurse (UR3e)

This document describes how to deploy and run the Robotic Scrub Nurse system on an NVIDIA Spark (ARM64 / Grace-Blackwell, CUDA) workstation connected to a physical UR3e robot.

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

Runtime is started across dedicated terminals; multiple components can be grouped in one launch file.

---

## 2. Prerequisites

### Software

Required:

- Ubuntu 24.04 + ROS 2 Jazzy
- NVIDIA Spark (ARM64 / Grace-Blackwell) with CUDA drivers (`nvidia-smi` working)
- colcon
- MoveIt 2
- Universal Robots ROS 2 Driver
- Intel RealSense SDK
- ALSA utils (`aplay`) for handover sound playback

### Python Dependencies

Most pip dependencies are pinned in `requirements-spark.txt`. `torch` and
`pyrealsense2` are installed manually first (see "Spark / Jazzy Setup" below),
because on ARM64 + CUDA they are not plain PyPI wheels.

```bash
pip install -r requirements-spark.txt
```

---

## 2b. Spark / Jazzy Setup (one-time)

The Spark is **ARM64 + CUDA** running **Ubuntu 24.04 / ROS 2 Jazzy**. Set up the
machine once in this order:

1. **ROS 2 Jazzy** (Ubuntu 24.04 base):

   ```bash
   sudo apt install ros-jazzy-desktop
   ```

2. **ROS apt dependencies** (Jazzy package names):

   ```bash
   sudo apt install ros-jazzy-ur ros-jazzy-moveit ros-jazzy-cv-bridge \
     ros-jazzy-realsense2-camera
   # then resolve the rest from package.xml:
   cd ~/robotic_scrub_nurse_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

   `pyrealsense2` is installed separately via pip (see step 3); a source build
   is not needed — an aarch64 wheel is available on PyPI.

3. **GPU stack (Blackwell, CUDA 13):** The plain PyPI `torch` wheel for aarch64
   is CPU-only. Install the CUDA build from PyTorch's `cu128` index, which has
   aarch64 CUDA wheels (CUDA 12.8 libs run on the CUDA 13 driver via backward
   compatibility). The `+cu128` local version tag is required — without it pip
   matches the already-installed CPU wheel and skips the download.

   ```bash
   # pyrealsense2: ARM64 wheel available on PyPI (no source build needed)
   pip install --break-system-packages pyrealsense2

   # torch with CUDA — must use +cu128 tag to get the GPU wheel
   pip install --break-system-packages \
     "torch==2.10.0+cu128" "torchvision==0.25.0+cu128" \
     --index-url https://download.pytorch.org/whl/cu128/

   # remaining ML deps
   pip install --break-system-packages -r requirements-spark.txt
   ```

   Note: `torch==2.10.0+cu128` supports up to sm_120; the GB10 is sm_121
   (Blackwell). PyTorch falls back to PTX JIT for sm_121 kernels — adds a
   one-time compile delay on first inference, subsequent runs are normal.
   Upgrade to `torch==2.11.0+cu128` for native sm_121 codegen if needed.

   Note: the `ctranslate2` aarch64 wheel from PyPI is CPU-only. `faster-whisper`
   / ASR will auto-fall back to `device='cpu', compute_type='int8'` at runtime.

4. **PEP 668 (Ubuntu 24.04):** system pip is "externally managed". Install with
   `--break-system-packages` (chosen strategy for this machine).

5. Verify CUDA before running the stack:

   ```bash
   python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
   # Expected: True  NVIDIA GB10
   ```

6. Smoke-test all ML imports:

   ```bash
   python3 -c "
   import torch; print('torch:', torch.__version__, 'CUDA:', torch.cuda.is_available())
   import ultralytics; print('ultralytics:', ultralytics.__version__)
   import faster_whisper; print('faster_whisper: ok')
   import mediapipe; print('mediapipe:', mediapipe.__version__)
   import pyrealsense2; print('pyrealsense2:', pyrealsense2.__version__)
   import rclpy; print('rclpy: ok')
   "
   ```

---

## 3. Required UR Description Overrides

This project depends on two non-default `ur_description` overrides. Apply them before the first runtime start so the deployed robot model matches the tested thesis setup.

The canonical replacement files are stored in this repository:

- `~/robotic_scrub_nurse_ws/files/ur.urdf.xacro`
- `~/robotic_scrub_nurse_ws/files/joint_limits.yaml`

Recommended: back up the currently installed vendor files first.

```bash
sudo cp /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro \
  /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro.bak

sudo cp /opt/ros/jazzy/share/ur_description/config/ur3e/joint_limits.yaml \
  /opt/ros/jazzy/share/ur_description/config/ur3e/joint_limits.yaml.bak
```

Then copy the project-specific overrides into the installed `ur_description` package:

```bash
sudo cp ~/robotic_scrub_nurse_ws/files/ur.urdf.xacro \
  /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro

sudo cp ~/robotic_scrub_nurse_ws/files/joint_limits.yaml \
  /opt/ros/jazzy/share/ur_description/config/ur3e/joint_limits.yaml
```

The two overrides are required for the active setup:

- `ur.urdf.xacro` adds the project-specific tool cylinder to the UR description.
- `joint_limits.yaml` applies the tested UR3e joint constraints used by this project.

Optional verification:

```bash
ls -l /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro
ls -l /opt/ros/jazzy/share/ur_description/config/ur3e/joint_limits.yaml
```

Warning: these overrides modify installed `ur_description` files under `/opt/ros/jazzy`. Reapply them after ROS, `ur_description`, or UR driver updates, as package updates can overwrite them.

---

## 4. Network Setup

The robot must be configured with a static IP address within the network settings.

### Robot Example Configuration

- Robot IP: 192.168.12.10
- Netmask: 255.255.255.0
- Gateway: 0.0.0.0

Update the IP address in the following files:
- `gripper_mover.py`
- `gripper_opener_with_zeroer.py`

Then update the robot IP in the launch command accordingly:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.12.10 launch_rviz:=false
```

Use the following command to find your workstation's IP address:

```bash
ifconfig
```
Then, under `Installation` in the UR control panel, change the host IP according to the terminal output.

---

## 5. File Permissions

If scripts are not executable, grant permissions:

```bash
chmod +x src/tracking_pkg/src/publisher/camera_publisher.py
chmod +x src/tracking_pkg/src/publisher/frame_publisher.py
chmod +x src/tracking_pkg/src/publisher/gesture_pose_publisher.py
chmod +x src/tracking_pkg/src/hand_tracker/hand_tracker.py
chmod +x src/tracking_pkg/src/moveit_mover/gripper_mover.py
chmod +x src/tracking_pkg/src/moveit_mover/gripper_opener_with_zeroer.py
chmod +x src/tracking_pkg/src/moveit_mover/reclaim_controller.py
chmod +x src/tracking_pkg/src/publisher/tool_selection.py
chmod +x src/tracking_pkg/src/publisher/handover_sound_publisher.py
```

---

## 6. Physical Setup

Set up the hardware according to the system setup:

![System Setup](images/system_setup.jpeg)

---

## 7. Startup Procedure

Start each component in a separate terminal, in the following order:

### Terminal 0 (once): Build + source workspace

```bash
cd ~/robotic_scrub_nurse_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Terminal 1: UR Hardware Driver

```bash
cd ~/robotic_scrub_nurse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.12.10 \
  launch_rviz:=false
```

Add `External Control` on the UR control panel and start the program.

Activate the robotiq gripper.

### Terminal 2: Tool Selector

```bash
cd ~/robotic_scrub_nurse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tracking_pkg tool_selection.py
```

### Terminal 3 (recommended): MoveIt + RViz + tracking loop

This starts:
- `ur_moveit_config` (without its default RViz)
- RViz with preloaded tracking displays
- TF visualization with directly visible frames (`camera_frame`, `aruco_board_frame`, and robot TCP frame)
- `tracking_pkg/loop_launch.py` (camera, frames, hand tracker, loop mover, sound node, ...)

```bash
cd ~/robotic_scrub_nurse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tracking_pkg loop_with_moveit_launch.py \
  ur_type:=ur3e \
  tracking_rviz:=true
```

Optional: run the same command without RViz:

```bash
ros2 launch tracking_pkg loop_with_moveit_launch.py \
  ur_type:=ur3e \
  tracking_rviz:=false
```

### Terminal 3 (alternative): Socket + RTDE runtime (MoveIt-free)

This launch replaces `loop_mover` with `socket_mover` and uses `ur_rtde` for motion execution with controller-side IK.

```bash
cd ~/robotic_scrub_nurse_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tracking_pkg web_socket_launch.py
```

Optional: disable the annotated hand image window if you do not want the lightweight annotated image viewer:

```bash
ros2 launch tracking_pkg web_socket_launch.py show_annotated_feed:=false
```

`socket_mover` loads tool, orientation, handover, and reclaim parameters from:

- `src/tracking_pkg/config/loop_mover_profiles.yaml` (`socket_mover.ros__parameters`)

The socket runtime also applies `rtde.input_pose_frame_rotation_rpy` before Cartesian RTDE motions. The default is `[0.0, 0.0, pi]`, which compensates for the UR `base_link` to controller `base` rotation and keeps the existing MoveIt-calibrated tool coordinates consistent.
The launch also injects a static `world -> base` TF so hand tracking keeps working after MoveIt is removed from the runtime path.
Handover release and reclaim sensing are read directly from RTDE TCP force data, so this launch does not require a parallel `ur_robot_driver` process.

### Optional split mode

Use this only if you intentionally want separate launches for MoveIt and tracking.

Terminal A:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

Terminal B:

```bash
ros2 launch tracking_pkg loop_launch.py
```

---
