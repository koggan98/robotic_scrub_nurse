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
2. Scene camera publishes RGB/depth/camera parameters for hand tracking.
3. The instrument tray camera is opened directly by `world_model_builder.py` in direct RealSense mode.
4. `/build_world_model` captures one tray frame, runs OBB inference, pairs body/handle detections, and projects grasp candidates into `world` through `tray_camera_color_optical_frame`.
5. ArUco marker tracking remains optional for validation/provisional frames, but fixed ArUco marker 120 is not required for the current instrument-camera pick-test path.
6. Grasp approach pose service can resolve a tracked TF frame such as `tool_holder_frame` into a `PoseStamped` in `world` on request.
7. Hand tracker detects gesture and publishes `hand_pose` in `world`.
8. Loop mover handles tool pickup, then waits for `hand_pose` as gesture trigger.
9. In valid waiting state, loop mover publishes `/handover_event = gesture_detected`.
10. Loop mover attempts MoveIt handover planning:
   - on plan failure: publishes `/handover_event = reachability:unreachable_plan_failed`,
   - on plan success: executes handover without additional audio event.
11. In the MoveIt path, `/tool_selection = "8"` triggers a holder reclaim: query `tool_holder_frame` through the grasp pose service, approach from above, close without force-trigger, wait briefly at the lower holder pose for gripper settling, then finish through the same dropoff/home reclaim tail as the normal reclaim.
12. After reaching the handover pose, loop mover holds briefly before enabling force-guided release sensing.
13. Sound publisher node subscribes to `/handover_event`, queues audio events sequentially, and plays them through the first available system backend (`paplay`, `pw-play`, then `aplay`).
14. Gripper feedback resets the system for the next cycle.

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
- `/tool_selection = "8"` holder reclaim via `tool_holder_frame`

## Tracking Frame Contract
- Active MoveIt tracking path canonical frame: `world`.
- Expected TF chain for perception-to-handover: `world -> base -> aruco_board_frame -> camera_frame`.
- Current instrument-camera pick-test TF: `world -> tray_camera_color_optical_frame`.
- Instrument camera static translation in `world`: `[-0.075, 0.349, 0.4325] m`.
- Instrument camera static orientation maps camera axes as `x -> -world_x`, `y -> +world_y`, `z -> -world_z`; quaternion `xyzw = [0.0, 1.0, 0.0, 0.0]`.
- Optional marker-specific static branch after first detection: `camera_frame -> aruco_marker_120_frame -> tool_holder_frame`.
- `world -> base` and `base -> aruco_board_frame` are published independently of `frame_publisher.py` so startup races in the camera/ArUco node do not remove the upstream tracking frames.
- Fixed ArUco marker 120 is not required for world-model grasp coordinates in the current pick-test path.
- `grasp_approach_pose_service.py` resolves any requested target frame from the TF tree into a top-down grasp pose in `world`; the runtime default target is `tool_holder_frame`, with grasp orientation mapped as `TCP z = -world z` and `TCP x` following the world-horizontal projection of `-frame z`.
- RViz tracking configuration uses `world` as fixed frame.
- The socket path still injects `world -> base` for its own runtime compatibility; that is not the primary MoveIt tracking contract.
