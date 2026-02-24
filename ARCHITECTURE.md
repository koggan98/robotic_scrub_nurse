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

### Dormant Path (Direct URScript Option)
- `src/tracking_pkg/src/socket_mover/socket_mover.py` and related `socket_mover` components.
- Current state: dormant/deferred (not active runtime path).
- Potential future use: only if IK/planning is externalized beyond MoveIt.

## High-Level Active ROS Flow
1. Camera node publishes RGB/depth/camera parameters.
2. Frame publisher provides board/camera transforms.
3. Hand tracker detects gesture and publishes `hand_pose`.
4. Loop mover handles tool pickup, then waits for `hand_pose` as gesture trigger.
5. In valid waiting state, loop mover publishes `/handover_event = gesture_detected`.
6. Loop mover attempts MoveIt handover planning:
   - on plan failure: publishes `/handover_event = reachability:unreachable_plan_failed`,
   - on plan success: executes handover without additional audio event.
7. Sound publisher node subscribes to `/handover_event` and plays system audio (`aplay`) only for those two events.
8. Gripper feedback resets the system for the next cycle.

## Current Constraints
- MoveIt path is the only supported runtime path.
- Reachability publish interface is intentionally minimal and only emits unreachable planning failures.
- Audio feedback is intentionally minimal (gesture-detected + unreachable only).
- No audio is emitted for hand poses received outside valid waiting state.
- Target-hand selection is not yet generalized beyond current detection behavior.

## Runtime Interfaces
- `target_hand_pose`
- `/handover_event` (`std_msgs/msg/String`)
  - `gesture_detected`
  - `reachability:unreachable_plan_failed`
- `target_roi`
- `target_roi_marker`
