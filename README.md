# Robotic Scrub Nurse (Work in Progress)

> **⚠️ Work in Progress**  
> This repository is under active development as part of an ongoing master thesis.  
> Structure, packages, and interfaces may change significantly.

---

## Overview

This repository contains the current development workspace for the **Robotic Scrub Nurse** research platform.

The project investigates collaborative robotic instrument handover using a **UR3e manipulator**,
driven by a **speech → LLM → skill-action → motion** pipeline and combining:

- spoken-command understanding (local speech-to-text + an LLM orchestrator)
- perception-driven grasp reasoning (YOLOv8-OBB instrument detection)
- motion planning and manipulation (MoveIt 2)
- human hand tracking and gesture-triggered handover
- force-guided interaction and robust grasp/loss recovery
- ergonomic tool transfer strategies

The runtime is **distributed across two machines**: an **NVIDIA Jetson Orin Nano** (perception + AI)
and an **Intel NUC** (robot control + MoveIt). See [ARCHITECTURE.md](ARCHITECTURE.md) for the current
system topology and interface contract.

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

Core experimental package, organized by responsibility under `src/tracking_pkg/src/`:

- `perception/` — RealSense integration, YOLOv8-OBB instrument detection, MediaPipe hand
  tracking + gesture detection, ArUco camera localization
- `reasoning/` — grasp geometry, tool semantics (knowledge base), and the persistent world model
- `llm/` — the OpenAI function-calling orchestrator (`llm_orchestrator_node`)
- `execution/` — the C++ MoveIt skill executor (pick / handover / release / return), gripper
  control, and force-guided release
- `interfaces/` — speech-to-text (`asr_node`), handover audio cues, and the operator CLI
- `publisher/` — MoveIt collision objects (instrument tray, reclaim tray, MiR base)

Two per-machine launch files bring the system up: `jetson_launch.py` (perception + AI) and
`nuc_launch.py` (robot control). The older `llm_launch.py` is an obsolete single-host snapshot and
is not kept in sync with this active distributed runtime.

### tracking_msgs

The custom interface package (`msg/ srv/ action/`) that defines the contract between subsystems —
`PickTool`/`HandoverTool`/… actions, `GetWorldModel`/`GetWorldState`/… services, and the
`ToolDetection`/`GraspCandidate`/`HandState`/`SystemState` messages.

### Universal_Robots_ROS2_Gazebo_Simulation

Official UR simulation environment included as a git submodule for testing and development without hardware.

---

## Project Status

### Current Focus

- Distributed NUC/Jetson runtime (robot control vs. perception/AI) and Orin CPU/GPU tuning
- Speech + LLM orchestration of robot skills via function-calling
- Perception-driven grasp reasoning and a persistent world model
- Robust grasp verification, loss recovery, and force-guided handover

### Planned Additions

- Wiring the reclaim-tray perception chain into the world model and execution
- Context/affordance-aware pickup and handover orientation
- Target-hand selection robustness and evaluation tooling

---

## Requirements

### System Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble

### Hardware Requirements (for physical deployment)

- Universal Robots UR3e manipulator
- Robotiq 2F gripper
- 2× Intel RealSense D455 cameras (reclaim tray + instrument tray)
- Compute: NVIDIA Jetson Orin Nano (perception/AI) + Intel NUC (robot control)
- USB microphone connected to the Jetson for spoken commands: Samson Q2U or the Jieli-based
  `USB Composite Device` receiver

### Software Dependencies

- **ROS 2 Humble**: Core robotics framework
- **MoveIt 2**: Motion planning (installed on the robot-control machine)
- **Intel RealSense SDK** + `realsense2_camera`: Camera integration
- **Ultralytics (YOLOv8-OBB)** + **PyTorch**: instrument detection
- **MediaPipe**: Hand tracking and gesture detection
- **faster-whisper** + **sounddevice**: local speech-to-text and microphone capture
- **OpenAI Python SDK**: LLM orchestrator using `gpt-5-mini` (requires
  `OPENAI_API_KEY`)
- **ur_rtde**: Direct UR RTDE motion interface for the legacy socket runtime path
- **ALSA utils (`aplay`)**: runtime speaker playback for handover events

### Python Packages

```bash
pip install mediapipe pyrealsense2 ultralytics torch pillow \
            faster-whisper sounddevice openai tabulate ur_rtde
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

The runtime is distributed across two machines that share a `ROS_DOMAIN_ID` over the wired
`192.168.12.0/24` link (per-machine CycloneDDS configs `cyclone_dds_nuc.xml` / `cyclone_dds_jetson.xml`):

```bash
# On the Jetson Orin Nano (perception + AI)
# Export OPENAI_API_KEY first when the LLM fallback should be available.
ros2 launch tracking_pkg jetson_launch.py

# On the Intel NUC (robot control); start the UR driver separately first
ros2 launch tracking_pkg nuc_launch.py ur_type:=ur3e
```

Connect either the Samson Q2U or the Jieli-based `USB Composite Device` receiver to the Jetson;
`asr_node` detects the single connected supported microphone automatically. Samson is captured at
16 kHz. Jieli is captured at its native 48 kHz and faster-whisper decodes/resamples the WAV buffer
to 16 kHz. If neither microphone is present, ASR waits and rescans every five seconds. If both are
connected, ASR rejects the ambiguous setup until one is disconnected. Numeric ALSA device indices
are rediscovered automatically after reconnecting USB hardware.

`llm_launch.py` is retained only as an obsolete historical snapshot; do not use it for deployment.

The NUC-side Robotiq grasp check treats `gOBJ=2` as direct object contact. For thin tools that do
not trigger that contact state reliably, it additionally uses the exclusive gPO rescue window
`[180, 228)`: `gPO=227` is accepted, while `gPO=228` and the approximate empty-close position
`gPO=230` are not. The upper limit is fixed in `nuc_launch.py` and leaves only two gPO counts to
the observed empty-close value. If an empty gripper is ever logged as a thin-tool rescue, lower
`grasp_check.rescue_max_pos` in `nuc_launch.py` and restart the NUC launch; a false positive can
cause the holding guard to block subsequent picks.

The surgeon-facing HRI display changes to green `TAKE` as soon as the executor reaches the hand.
Non-red display transitions use a short `0.1 s` debounce; red motion/error states and alerts remain
immediate. The executor still observes `pre_release_dwell_seconds` before enabling force-guided
physical release, so the faster visual transition does not shorten the release safety dwell.

When `ReturnToolHome` carries a used tool from the reclaim tray to a right-side instrument home
slot (`world-x > 0`), it exits through the raised reclaim stage, transits through
`instrument_left_stage`, and then moves the TCP in one complete collision-checked Cartesian leg to
the Home position while preserving the held-tool orientation. There is no isolated Home
shoulder-pan rotation or RRT fallback on that left-stage-to-Home leg. If the complete straight path
is unavailable, the existing safety fallback returns the still-held tool to the reclaim tray.
Left-side home slots and normal `return_tool` routes retain their existing behavior.

The perception pipelines use separate YOLO-OBB weights by default:
`ros_unrelated_scripts/instrument_tray_detector.pt` for the instrument tray and
`ros_unrelated_scripts/reclaim_tray_detector.pt` for the reclaim tray. Override them when needed
with `INSTRUMENT_TRAY_MODEL_PATH` and `RECLAIM_TRAY_MODEL_PATH`; `OBB_DEVICE` selects the inference
device (default: `cuda:0`). The launch files detect both the `~/robotic_scrub_nurse` deployment
directory and the `~/robotic_scrub_nurse_ws` development workspace.

RViz shows three processed camera views: the instrument-tray tool detections, reclaim-tray hand
annotations, and reclaim-tray tool detections. Raw camera feeds are intentionally omitted. In the
distributed Jetson launch, both tool-detection feeds run at 4 Hz. The lower-resolution reclaim
tool feed uses 1 px detection outlines; the instrument feed retains the 2 px default.

For detailed step-by-step instructions (network, UR description overrides, permissions, startup
order), see **[Deployment Guide](deployment_guide.md)**.

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

`nuc_launch.py` starts `grasp_approach_pose_service.py` by default. The service looks up the requested TF frame, keeps the frame position, and recomputes the orientation so that `TCP z = -world z` while `TCP x` follows the world-horizontal projection of `-frame z`. The result is returned as a `geometry_msgs/PoseStamped` in `world`. If `target_frame` is empty, the node falls back to its default parameter `tool_holder_frame`.

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

The reclaim tray camera is started through the official `realsense2_camera`
`rs_launch.py` launch file and defaults to serial `239222300719`. It publishes
RGB, aligned depth, and camera info under `/reclaim_tray_camera/...`. Override it with
`RECLAIM_TRAY_CAM_SERIAL` if the camera changes:

```bash
RECLAIM_TRAY_CAM_SERIAL=239222300719 ros2 launch tracking_pkg test_world_model_launch.py
```

The direct tray camera path defaults to serial `239222302690`, so the reclaim tray
camera and tray camera stay pinned to different devices.

Then trigger one on-demand tray capture and OBB inference:

```bash
ros2 service call /build_world_model tracking_pkg/srv/BuildWorldModel
```

The launch starts the static tray-camera TF directly, so marker 120 is no
longer part of this pick-test path.
The world-model builder supports aligned tray-camera depth for grasp height.
The current pick-test calibration projects grasp pixels onto
`fixed_tool_plane_z_m = 0.05` while the depth thresholds remain available for
depth-mode tuning.
Grasp pixels are class-specific: hammers grip 40 mm past the handle edge toward
the tool center, scissors and needle holders grip 20 mm past that edge toward
the center, and forceps keep the configured fallback offset.
The ArUco manager publishes the configured marker-105 frame as
`world -> aruco_marker_105_frame` in general launches. The pick-test launches
publish that static frame directly so it is always visible in RViz; once marker
ID 105 is visible in the reclaim tray camera image, the ArUco manager locks
`aruco_marker_105_frame ->
reclaim_tray_camera_color_optical_frame` for hand tracking in `world`.
It also publishes the MiR base collision object and a
`tray_camera_volume` collision object on `/collision_object`. The camera volume
contains a 140 x 40 x 40 mm frame box centered at
`tray_camera_color_optical_frame`, plus a world-frame camera stand. The stand
uses a 50 x 50 x 520 mm vertical post whose top stays aligned with the camera
height; its bottom-mounted horizontal arm and cross block therefore sit 10 mm
lower with the longer post.
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
the official reclaim_tray_camera RealSense node, marker-105 localization, hand
tracking, the gripper bridge, the MiR collision object, and the
`tray_camera_volume` collision object. It intentionally does not start
`loop_mover`, so there is no competing `/tool_selection` runtime.

In a second terminal with stdin attached, run:

```bash
ros2 run tracking_pkg tool_pick_test_node --ros-args \
  --params-file install/tracking_pkg/share/tracking_pkg/config/tool_pick_joint_states.yaml
```

The node calls `/build_world_model`, prints detected grasp candidates, asks for
a terminal index, then executes a camera-pick and force-guided handover
sequence:

1. choose `tray_left`, `tray_center`, or `tray_right` from the selected tool's OBB `center_x`,
2. move to the corresponding joint-state waypoint,
3. move 5 cm above the selected tool,
4. open the gripper,
5. descend to the detected tool surface height plus `z_offset`,
6. close the gripper,
7. lift back 5 cm,
8. move back to `tray_left`,
9. wait for `/hand_pose` at `tray_left`,
10. move to the hand pose plus `hand_offset`,
11. activate `/gripper_zeroer`,
12. wait for `/gripper_done` from the force-guided release node,
13. optionally return home, which defaults to `tray_left`.

The joint-state waypoints live in
`config/tool_pick_joint_states.yaml`. All arrays use the canonical UR order:
`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`,
`wrist_2_joint`, `wrist_3_joint`.

The pick-test launch currently uses the calibrated world-z plane from
`fixed_tool_plane_z_m`. `z_offset` is added to the detected/projected surface
height. The default is `z_offset = 0.003 m`, so the robot grasps 3 mm above the
projected fixed plane; with the current `0.05 m` plane this is `0.053 m`.
The default pre-pick approach height is `0.04 m`. The default gripper yaw is
rotated by `pi/2` relative to the detected tool axis.
The handover orientation uses the same configurable quaternion style as the
main MoveIt handover path.
Useful parameter overrides:

```bash
ros2 run tracking_pkg tool_pick_test_node --ros-args \
  --params-file install/tracking_pkg/share/tracking_pkg/config/tool_pick_joint_states.yaml \
  -p z_offset:=0.003 \
  -p approach_height_m:=0.04 \
  -p tool_yaw_offset_rad:=1.57079632679 \
  -p hand_offset:="[-0.08, 0.0, 0.05]" \
  -p handover_orientation:="[-0.63, 0.63, -0.321, 0.321]" \
  -p velocity_scale:=0.6 \
  -p acceleration_scale:=0.6
```

### Live Force-Z Plot

To observe the live TCP force in z direction while picking or handing over
tools, run the standalone helper script:

```bash
python3 ros_unrelated_scripts/plot_force_z_live.py
```

It subscribes to `/force_torque_sensor_broadcaster/wrench` and plots
`wrench.force.z` in Newton. Add `--zero-start` to subtract the first received
sample as a baseline, or override the source with `--topic <wrench_topic>`.

### Joint-State Keyboard Jogger

To manually move between the saved tray/handover joint waypoints, start the
MoveIt/RViz jogger infrastructure first:

```bash
ros2 launch tracking_pkg joint_state_jogger_launch.py ur_type:=ur3e
```

Then run the interactive jogger in a second terminal:

```bash
ros2 run tracking_pkg joint_state_jogger_node
```

Keys:
- `1`: `tray_left`
- `2`: `tray_center`
- `3`: `tray_right`
- `4`: `reclaim_holder`
- `q`: quit

When moving between `tray_left` and `tray_right`, the jogger automatically
routes via `tray_center` to avoid the direct long cross-tray motion.

The jogger has built-in defaults for these waypoints. To override them from the
shared YAML file, add:

```bash
ros2 run tracking_pkg joint_state_jogger_node --ros-args \
  --params-file install/tracking_pkg/share/tracking_pkg/config/tool_pick_joint_states.yaml
```

---

---

> **⚠️ Legacy / alternative runtime paths below.**
> The sections that follow document the older MoveIt loop and the MoveIt-free RTDE path driven by
> the numeric `/tool_selection` interface. The `loop_mover` executable is retained only for direct
> manual bench testing and is not started by `nuc_launch.py`. The **active** runtime is the LLM +
> skill-action pipeline via `jetson_launch.py` + `nuc_launch.py`; `llm_launch.py` is an obsolete
> single-host snapshot — see [ARCHITECTURE.md](ARCHITECTURE.md).

## Combined MoveIt + Tracking Launch (retired)

`loop_with_moveit_launch.py` is retained as historical source but is not a supported launch path: it
references the removed `loop_launch.py`. It is intentionally not repaired as part of the active LLM
runtime. Use the active distributed launch described above.

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

After changing the YAML values, restart whichever consumers are in use. The active runtime reloads
the file through `gripper_opener_with_zeroer` and `reclaim_controller`; a manually started
`loop_mover` must be restarted separately.

## Socket RTDE Launch (MoveIt-free)

The new socket runtime path uses `ur_rtde` (controller-side IK) and keeps the same tool-selection and handover topic logic as `loop_mover`.

```bash
ros2 launch tracking_pkg web_socket_launch.py
```

This path reads tool/reclaim/orientation parameters from `src/tracking_pkg/config/loop_mover_profiles.yaml` under `socket_mover.ros__parameters`.

Hammer pickup tuning in the socket path uses the same `tool_profiles.<id>.post_lift_joint5_offset` parameter. For the current hammer profile (`tool_profiles.2`), the socket runtime also applies `0.2 rad` of positive `joint_5` rotation after the lift to reduce post-grasp collision risk.

By default, the socket path rotates incoming Cartesian pose targets by `pi` around Z before sending them to the UR controller. This matches the UR `base_link -> base` frame difference used by the controller and keeps the Cartesian targets aligned with the existing MoveIt-tuned coordinates.

The socket launch does not start the MiR RViz collision publisher. Instead, it adds the missing static `world -> base` transform for tracking, opens a local lightweight viewer node on `/annotated_hand_image` by default, and uses RTDE TCP force data directly for handover/reclaim sensing. No parallel `ur_robot_driver` runtime is required for this path. Disable the image window with `show_annotated_feed:=false`.
