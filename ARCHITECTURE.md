# Architecture: Robotic Scrub Nurse (Current Baseline)

## Document Map
- This document's role: current system truth (topology, execution paths, flow, constraints, interfaces).
- See also: `ARCHITECTURE.md` (this file), `AGENTS.md`, `PLAN.md`.
- Contributor rules and repository boundaries: `AGENTS.md`.
- Implementation roadmap and milestones: `PLAN.md`.

## System Context
The active baseline is a ROS 2 Humble workspace centered on `src/tracking_pkg`, integrating:
- RealSense perception,
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

### Alternative Path (Socket + RTDE, MoveIt-free)
- Launch entry: `src/tracking_pkg/launch/web_socket_launch.py`.
- Motion/handover core: `src/tracking_pkg/src/socket_mover/socket_mover.py`.
- Socket gripper I/O: `src/tracking_pkg/src/socket_mover/socket_gripper_controller.py`.
- IK is computed by the robot controller through `ur_rtde` (`moveJ_IK`), with reachability prechecks in the node.
- Handover/reclaim force sensing is read directly from `RTDEReceive.getActualTCPForce()` and does not depend on `ur_robot_driver`.
- Tool command flow remains `/tool_selection` compatible with the MoveIt path.
- The socket launch injects a static `world -> base` TF so tracking stays compatible without MoveIt, and it can open `/annotated_hand_image` directly through a local lightweight viewer node.

## High-Level Active ROS Flow
1. Camera node publishes RGB/depth/camera parameters.
2. Launch starts independent static `world -> base` and `base -> aruco_board_frame` transform publishers.
3. Frame publisher initializes `aruco_board_frame -> camera_frame` once the first valid ArUco board pose is detected.
4. Hand tracker detects gesture and publishes `hand_pose` in `world`.
5. Loop mover handles tool pickup, then waits for `hand_pose` as gesture trigger.
6. In valid waiting state, loop mover publishes `/handover_event = gesture_detected`.
7. Loop mover attempts MoveIt handover planning:
   - on plan failure: publishes `/handover_event = reachability:unreachable_plan_failed`,
   - on plan success: executes handover without additional audio event.
8. After reaching the handover pose, loop mover holds briefly before enabling force-guided release sensing.
9. Sound publisher node subscribes to `/handover_event`, queues audio events sequentially, and plays them through the first available system backend (`paplay`, `pw-play`, then `aplay`).
10. Gripper feedback resets the system for the next cycle.

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

## Tracking Frame Contract
- Active MoveIt tracking path canonical frame: `world`.
- Expected TF chain for perception-to-handover: `world -> base -> aruco_board_frame -> camera_frame`.
- `world -> base` and `base -> aruco_board_frame` are published independently of `frame_publisher.py` so startup races in the camera/ArUco node do not remove the upstream tracking frames.
- RViz tracking configuration uses `world` as fixed frame.
- The socket path still injects `world -> base` for its own runtime compatibility; that is not the primary MoveIt tracking contract.
