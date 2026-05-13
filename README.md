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
pip install mediapipe pyrealsense2 tabulate ur_rtde ultralytics transformers torch pillow
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

## Standalone YOLO Test

To quickly test whether a stock YOLO11 model detects anything plausible around the robot end effector on the connected RealSense camera, run:

```bash
python3 ros_unrelated_scripts/yolo11_realsense_test.py
```

The script opens the RealSense RGB stream, runs `yolo11n.pt`, draws detections in a local OpenCV window, and exits on `q`. You can also override the model path:

```bash
python3 ros_unrelated_scripts/yolo11_realsense_test.py --model /path/to/custom.pt
```

## Standalone Grounding DINO Test

To test whether a text-guided detector can localize the Robotiq 2-finger gripper on the connected RealSense camera, run:

```bash
python3 ros_unrelated_scripts/grounding_dino_realsense_test.py
```

The script opens the RealSense RGB stream, runs Grounding DINO Tiny through Hugging Face `transformers`, uses a fixed prompt for `Robotiq gripper`, `robotic gripper`, and `two-finger robotic gripper`, and draws live detections in an OpenCV window. It exits on `q`.

## Standalone Robotiq Detector Test

To test whether the custom YOLO model reliably detects the Robotiq gripper on the connected RealSense camera, run:

```bash
python3 ros_unrelated_scripts/robotiq_detector_realsense_test.py
```

The script opens the RealSense RGB stream, loads `files/robotiq_detector.pt` by default, and draws live `gripper` detections in an OpenCV window without covering the image with statistics. It exits on `q` and prints a final detection summary with reliability stats to the terminal. You can also override the model path:

```bash
python3 ros_unrelated_scripts/robotiq_detector_realsense_test.py --model /path/to/custom.pt
```

## Standalone ArUco Frame Test

To detect a single original ArUco marker and draw its frame directly into the RealSense image, run:

```bash
python3 ros_unrelated_scripts/aruco_marker_frame_test.py
```

The script uses OpenCV's standard ArUco detection flow on the RealSense image, overlays each detected original marker ID, and draws the marker coordinate axes using the configured physical marker size.

## Standalone RealSense Manual Capture

To capture single RealSense RGB frames directly from an SSH terminal without ROS, run:

```bash
python3 ros_unrelated_scripts/realsense_manual_capture.py
```

The script reads the RealSense color stream directly through `pyrealsense2` and is built for a lightweight dataset collection workflow for later annotation in tools such as Roboflow. By default it stores lossless PNG images outside the repository in:

```bash
~/datasets/tool_detector/<YYYY-MM-DD>/<session_timestamp>
```

Controls:
- `1` saves the current RGB frame
- `h` prints the help text again
- `q` quits the script

Optional examples:

```bash
python3 ros_unrelated_scripts/realsense_manual_capture.py --preview
python3 ros_unrelated_scripts/realsense_manual_capture.py --output-dir ~/datasets/tool_detector/custom_session
```

`--preview` opens a local OpenCV window when you have a GUI available, but the default mode stays terminal-only so it works well over SSH. The saved PNG files can be uploaded to Roboflow directly, and for larger dataset sessions it is usually more practical to move them to a Mac via SSD, `scp`, or Finder-based file sharing than to commit them into Git.

## Tool Frame Quaternion Helper

To convert tracked frame axes into a gripper quaternion for a top-down approach, run:

```bash
ros2 run tracking_pkg tool_frame_quaternion_helper.py --frame-x 0 1 0 --frame-y 0 0 1 --frame-z 1 0 0
```

The helper assumes the usual RViz axis colors `x=red`, `y=green`, `z=blue` and uses the current grasp convention `TCP z = -world z`, so the gripper always approaches from global above. `TCP x` follows the negated tool-frame `z` direction, after projecting that axis into the world-horizontal plane. The remaining TCP `y` axis is reconstructed as a right-handed cross product, then the helper prints only the quaternion in `xyzw` format so it can be copied into later grasp logic. It validates that the input axes are orthogonal and warns if the projected `TCP x` differs noticeably from the expected `-frame_z` direction.

## Grasp Approach Pose Service

To request a top-down grasp pose in `world` for a tracked TF frame such as `tool_holder_frame`, use:

```bash
ros2 service call /get_grasp_approach_pose tracking_pkg/srv/GetGraspApproachPose "{target_frame: tool_holder_frame}"
```

`loop_launch.py` starts `grasp_approach_pose_service.py` by default. The service looks up the requested TF frame, keeps the frame position, and recomputes the orientation so that `TCP z = -world z` while `TCP x` follows the world-horizontal projection of `-frame z`. The result is returned as a `geometry_msgs/PoseStamped` in `world`. If `target_frame` is empty, the node falls back to its default parameter `tool_holder_frame`.

## ROS Frame Capture for YOLO Training

To save RGB frames from the existing ROS camera stream without changing the current camera publisher, run:

```bash
ros2 run tracking_pkg frame_capture_node.py
```

The node subscribes to `/color_image`, opens a local OpenCV preview window, and writes PNG frames to `~/frame_captures/<session_timestamp>` by default. To avoid redundant training data, it stores every 30th received frame by default (about once per second with the current 30 Hz camera publisher).

Controls:
- `r` starts or stops continuous recording
- `q` closes the node

You can override the source topic, output directory, or save interval with ROS parameters, for example:

```bash
ros2 run tracking_pkg frame_capture_node.py --ros-args -p topic_name:=/color_image -p output_dir:=/tmp/yolo_frames -p save_every_n_frames:=30
```

---

## Instrument Camera World-Model Test

For the current pick-test baseline, the instrument camera is fixed in `world`
with a static `world -> tray_camera_color_optical_frame` TF:

- translation: `[-0.075, 0.349, 0.4325] m`
- quaternion `xyzw`: `[0.0, 1.0, 0.0, 0.0]`
- axis mapping: camera `x -> -world_x`, camera `y -> +world_y`, camera `z -> -world_z`

Run the isolated world-model test with:

```bash
ros2 launch tracking_pkg test_world_model_launch.py
```

The scene camera is started through the official `realsense2_camera`
`rs_launch.py` launch file and defaults to serial `239222300719`. It publishes
RGB, aligned depth, and camera info under `/scene_camera/...`. Override it with
`SCENE_CAM_SERIAL` if the camera changes:

```bash
SCENE_CAM_SERIAL=239222300719 ros2 launch tracking_pkg test_world_model_launch.py
```

The direct tray camera path defaults to serial `239222302690`, so the scene
camera and tray camera stay pinned to different devices.

Then trigger one on-demand tray capture and OBB inference:

```bash
ros2 service call /build_world_model tracking_pkg/srv/BuildWorldModel
```

The launch starts the static tray-camera TF directly, so marker 120 is no
longer part of this pick-test path.
The ArUco manager publishes the configured marker-105 frame as
`world -> aruco_marker_105_frame` in general launches. The pick-test launches
publish that static frame directly so it is always visible in RViz; once marker
ID 105 is visible in the scene camera image, the ArUco manager locks
`aruco_marker_105_frame ->
scene_camera_color_optical_frame` for hand tracking in `world`.
It also publishes the MiR base collision object and a
`tray_camera_volume` collision object on `/collision_object`. The camera volume
contains a 50 x 50 mm box that starts 100 mm down the tray camera optical axis
from `tray_camera_color_optical_frame` and then extends another 600 mm, plus a
140 x 50 x 50 mm box centered exactly at the tray camera frame.
You can verify the transform with:

```bash
ros2 run tf2_ros tf2_echo world tray_camera_color_optical_frame
```

---

## Lightweight MoveIt Pick Test

To test whether the robot can pick one detected instrument without starting the
full handover loop, launch the pick-test infrastructure:

```bash
ros2 launch tracking_pkg tool_pick_test_launch.py ur_type:=ur3e
```

This starts MoveIt, RViz, the fixed tray-camera TF, `world_model_builder.py`,
the official scene-camera RealSense node, marker-105 localization, hand
tracking, the gripper bridge, the MiR collision object, and the
`tray_camera_volume` collision object. It intentionally does not start
`loop_mover`, so there is no competing `/tool_selection` runtime.

In a second terminal with stdin attached, run:

```bash
ros2 run tracking_pkg tool_pick_test_node
```

The node calls `/build_world_model`, prints detected grasp candidates, asks for
a terminal index, then executes one conservative sequence:

1. move 5 cm above the selected tool,
2. open the gripper,
3. descend to fixed robot/world `z = 0.05 m`,
4. close the gripper,
5. lift back 5 cm.

The node commands fixed robot/world tool height `z=0.05 m`, and the pick-test
launch projects detected grasp pixels onto the same fixed world-z plane. The
default gripper yaw is rotated by `pi/2` relative to the detected tool axis.
Useful parameter overrides:

```bash
ros2 run tracking_pkg tool_pick_test_node --ros-args \
  -p tool_z_m:=0.05 \
  -p approach_height_m:=0.05 \
  -p tool_yaw_offset_rad:=1.57079632679 \
  -p velocity_scale:=0.2 \
  -p acceleration_scale:=0.2
```

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
- TF display (including frames such as `world`, `base`, `tray_camera_color_optical_frame`, `aruco_marker_105_frame`, `scene_camera_color_optical_frame`, and `tool_holder_frame`)

The active MoveIt tracking path now uses `world` as the canonical tracking frame. RViz is configured with `world` as its fixed frame, `/hand_pose` positions are interpreted in `world`, and the expected TF chain is `world -> base -> aruco_board_frame -> camera_frame`.

`world -> base` and `base -> aruco_board_frame` are now started as their own static TFs in the MoveIt launch path, so the upstream frames no longer depend on `frame_publisher.py` starting cleanly. `frame_publisher.py` is now responsible only for `aruco_board_frame -> camera_frame` and keeps retrying until it sees the first valid ArUco board pose. If the board is not visible yet, you should now see repeated warning logs instead of a silent partial initialization.

The instrument-camera/world-model path now starts `world -> tray_camera_color_optical_frame` as a fixed static TF. ArUco marker 120 has been removed from the active configuration.

The same launch now also starts `grasp_approach_pose_service.py`, which exposes `/get_grasp_approach_pose` for on-demand conversion of a TF target frame such as `tool_holder_frame` into a top-down robot grasp pose in `world`.

`loop_launch.py` now starts the camera publisher and static board TF first, then starts tracking consumers slightly later. `loop_with_moveit_launch.py` also delays RViz briefly so the TF tree is usually already populated when RViz opens, which reduces startup-time warning noise on slower hosts such as the NUC.

During handover, the robot now holds briefly at the target pose before force-guided release sensing is enabled. Audio events are queued sequentially so the initial `gesture_detected` tone is not overwritten by a following `unreachable` tone.

## Loop Mover Profiles

`loop_mover` now loads its tool pick poses, handover orientations, offsets, and joint presets from:

- `src/tracking_pkg/config/loop_mover_profiles.yaml`

The `/tool_selection` interface is unchanged:

- `"0"` resets the system
- `"1"` to `"6"` select the configured tool profiles
- `"8"` starts holder reclaim via `tool_holder_frame`, closes without force-trigger, waits `reclaim.holder_close_settle_seconds` at the lower holder pose, then finishes with the same dropoff/home sequence as reclaim
- `"9"` starts the reclaim workflow from the last successful handover pose

The same YAML also contains reclaim settings for the dropoff pose, force threshold, reclaim timing (`zero_delay_seconds`, `post_close_wait_seconds`, `post_open_pause_seconds`), reclaim gripper close parameters, and holder reclaim settings including the target frame, top-down approach distance, and lower-pose settle wait (`holder_close_settle_seconds`). The dropoff sequence uses a three-phase motion: approach the configured pose, lower by `0.05 m`, open the gripper, then lift back to the same upper dropoff pose before returning home.

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
