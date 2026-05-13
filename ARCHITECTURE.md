# Architecture: Robotic Scrub Nurse (Current Baseline)

## Document Map
- This document's role: current system truth (topology, execution paths, flow, constraints, interfaces).
- See also: `ARCHITECTURE.md` (this file), `AGENTS.md`, `PLAN.md`.
- Contributor rules and repository boundaries: `AGENTS.md`.
- Implementation roadmap and milestones: `PLAN.md`.

## System Context
The active baseline is a ROS 2 Humble workspace centered on `src/tracking_pkg`, integrating:
- RealSense perception,
- OBB-based instrument tray detection and on-demand world-model building,
- hand tracking and gesture-triggered hand pose publication,
- MoveIt-based tool pickup and handover with UR3e,
- force-triggered gripper release and return-to-home flow.

## Deployment Topology
```text
Mode A (local runtime)
Ubuntu Host (ROS 2 + MoveIt + Drivers) --> UR3e + Robotiq + RealSense

Mode B (remote control)
Mac Client --SSH--> Ubuntu Host (ROS 2 runtime) --> UR3e + Robotiq + RealSense
```

## Host Roles
- Development host: machine used for editing and command entry (Ubuntu local or Mac over SSH).
- Runtime host: Ubuntu machine running ROS nodes, drivers, and launches.
- Runtime execution for ROS is Ubuntu-based.

## Package Boundaries in the Runtime System
- Active thesis package: `src/tracking_pkg`.
- Included upstream simulation package: `src/Universal_Robots_ROS2_Gazebo_Simulation`.
- External utility area (outside ROS node graph): `ros_unrelated_scripts`.

## Execution Paths

### Active Path (MoveIt-Centric)
- Launch entry: `src/tracking_pkg/launch/loop_launch.py`.
- Combined launch option: `src/tracking_pkg/launch/loop_with_moveit_launch.py` (includes `ur_moveit_config` RViz bringup + `loop_launch.py`).
- Motion/handover core: `src/tracking_pkg/src/moveit_mover/loop_mover.cpp`.
- Tool command flow uses `/tool_selection` and MoveIt planning.
- Instrument-camera/world-model pick-test path: `src/tracking_pkg/launch/llm_launch.py` starts a fixed `world -> tray_camera_color_optical_frame` TF and the on-demand `/build_world_model` service.
- Lightweight robot pickup/handover test: `src/tracking_pkg/launch/tool_pick_test_launch.py` starts MoveIt, the official scene-camera RealSense node, marker-105 localization, hand tracking, the fixed tray-camera TF, the world-model builder, gripper bridge, MiR collision object, and tray-camera volume collision object without `loop_mover`; `tool_pick_test_node` is run separately for terminal-based tool selection, one camera-based pick, hand-pose handover, and force-guided release.

### Alternative Path (Socket + RTDE, MoveIt-free)
- Launch entry: `src/tracking_pkg/launch/web_socket_launch.py`.
- Motion/handover core: `src/tracking_pkg/src/socket_mover/socket_mover.py`.
- Socket gripper I/O: `src/tracking_pkg/src/socket_mover/socket_gripper_controller.py`.
- IK is computed by the robot controller through `ur_rtde` (`moveJ_IK`), with reachability prechecks in the node.
- Handover/reclaim force sensing is read directly from `RTDEReceive.getActualTCPForce()` and does not depend on `ur_robot_driver`.
- Tool command flow remains `/tool_selection` compatible with the MoveIt path.
- The socket launch injects a static `world -> base` TF so tracking stays compatible without MoveIt, and it can open `/annotated_hand_image` directly through a local lightweight viewer node.

## High-Level Active ROS Flow
1. Launch starts independent static `world -> base` and `world -> tray_camera_color_optical_frame` transform publishers.
2. Scene camera serial `239222300719` is started through the official `realsense2_camera` package and publishes RGB/depth/camera parameters under `/scene_camera/...` for marker localization and hand tracking.
3. The instrument tray camera is opened directly by `world_model_builder.py` in direct RealSense mode.
4. Pick-test launches publish the MiR base and a two-primitive tray-camera volume as MoveIt collision objects on `/collision_object`.
5. `/build_world_model` captures one tray frame, runs OBB inference, pairs body/handle detections, estimates grasp height from filtered tray-camera depth over the full tool OBB, and projects grasp candidates into `world` through `tray_camera_color_optical_frame`.
6. ArUco marker 105 localizes the scene camera for hand tracking; marker 120 has been removed from the active configuration.
7. Grasp approach pose service can resolve a tracked TF frame such as `tool_holder_frame` into a `PoseStamped` in `world` on request.
8. Hand tracker detects gesture and publishes `hand_pose` in `world`.
9. Loop mover handles tool pickup, then waits for `hand_pose` as gesture trigger.
10. In valid waiting state, loop mover publishes `/handover_event = gesture_detected`.
11. Loop mover attempts MoveIt handover planning:
   - on plan failure: publishes `/handover_event = reachability:unreachable_plan_failed`,
   - on plan success: executes handover without additional audio event.
12. In the MoveIt path, `/tool_selection = "8"` triggers a holder reclaim: query `tool_holder_frame` through the grasp pose service, approach from above, close without force-trigger, wait briefly at the lower holder pose for gripper settling, then finish through the same dropoff/home reclaim tail as the normal reclaim.
13. After reaching the handover pose, loop mover holds briefly before enabling force-guided release sensing.
14. Sound publisher node subscribes to `/handover_event`, queues audio events sequentially, and plays them through the first available system backend (`paplay`, `pw-play`, then `aplay`).
15. Gripper feedback resets the system for the next cycle.

## Current Constraints
- MoveIt path remains the primary thesis runtime path.
- Socket path is implemented as an alternative runtime path and requires `ur_rtde` plus direct robot RTDE access.
- Reachability publish interface is intentionally minimal and only emits unreachable planning failures.
- Audio feedback is intentionally minimal (gesture-detected + unreachable only) but now uses queued playback so short back-to-back events are both audible.
- No audio is emitted for hand poses received outside valid waiting state.
- Target-hand selection is not yet generalized beyond current detection behavior.

## Runtime Interfaces
- `target_hand_pose`
- `/hand_pose` (`geometry_msgs/msg/Pose`) in `world`
- `/handover_event` (`std_msgs/msg/String`)
  - `gesture_detected`
  - `reachability:unreachable_plan_failed`
- `target_roi`
- `target_roi_marker`
- `/get_grasp_approach_pose` (`tracking_pkg/srv/GetGraspApproachPose`)
- `/build_world_model` (`tracking_pkg/srv/BuildWorldModel`)
- `/world_model/json` (`std_msgs/msg/String`)
- `/world_model/grasp_candidates` (`tracking_pkg/msg/GraspCandidateArray`)
- `/gripper_mover` (`std_msgs/msg/Bool`, `true=open`, `false=close`)
- `/tool_selection = "8"` holder reclaim via `tool_holder_frame`
- `/collision_object` (`moveit_msgs/msg/CollisionObject`)
  - `mir`
  - `tray_camera_volume`

## Tracking Frame Contract
- Active MoveIt tracking path canonical frame: `world`.
- Expected TF chain for perception-to-handover: `world -> base -> aruco_board_frame -> camera_frame`.
- Current instrument-camera pick-test TF: `world -> tray_camera_color_optical_frame`.
- Pick-test launches publish `world -> aruco_marker_105_frame` directly for RViz visibility and run `aruco_marker_manager.py` with marker static TF publishing disabled; after marker ID 105 is detected, the manager publishes `aruco_marker_105_frame -> scene_camera_color_optical_frame`.
- Instrument camera static translation in `world`: `[-0.075, 0.349, 0.4325] m`.
- Instrument camera static orientation maps camera axes as `x -> -world_x`, `y -> +world_y`, `z -> -world_z`; quaternion `xyzw = [0.0, 1.0, 0.0, 0.0]`.
- `tray_camera_volume` is expressed in `tray_camera_color_optical_frame` and contains two boxes: a `[0.05, 0.05, 0.60] m` volume starting at camera-frame `z = 0.10 m` and centered at `z = 0.40 m`, plus a `[0.14, 0.05, 0.05] m` box centered at the camera frame.
- Marker 120 is no longer part of the active TF configuration.
- `world -> base` and `base -> aruco_board_frame` are published independently of `frame_publisher.py` so startup races in the camera/ArUco node do not remove the upstream tracking frames.
- World-model grasp coordinates use the fixed tray-camera TF, not marker 120.
- Pick-test world-model height supports median valid depth over the full detected tool OBB, but the current calibrated `tool_pick_test_launch.py` path uses `fixed_tool_plane_z_m = 0.04` for robust candidate publication.
- `tool_pick_test_node` uses the selected tool detection's OBB `center_x` to choose `tray_left`, `tray_center`, or `tray_right` from `config/tool_pick_joint_states.yaml`, performs the camera pick, returns to `tray_left`, waits there for `/hand_pose`, moves to `hand_pose + hand_offset`, activates `/gripper_zeroer`, waits for `/gripper_done`, and optionally returns home; `tray_left` is the default home/hold position.
- Tray-region waypoint moves avoid direct `tray_left <-> tray_right` transitions by routing through `tray_center`; the same rule is used by the interactive joint-state jogger.
- `grasp_approach_pose_service.py` resolves any requested target frame from the TF tree into a top-down grasp pose in `world`; the runtime default target is `tool_holder_frame`, with grasp orientation mapped as `TCP z = -world z` and `TCP x` following the world-horizontal projection of `-frame z`.
- RViz tracking configuration uses `world` as fixed frame.
- The socket path still injects `world -> base` for its own runtime compatibility; that is not the primary MoveIt tracking contract.
