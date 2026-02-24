# AGENTS Guide: Robotic Scrub Nurse Workspace

You are building a robotic scrub nurse in ROS2 Humble on Ubuntu 22.04 using the UR3e, Intel Realsense D455 and Robotiq 2F Gripper

## Document Map
- This document's role: operational contribution rules and repository boundaries.
- See also: `AGENTS.md` (this file), `ARCHITECTURE.md`, `PLAN.md`.
- System topology and runtime source of truth: `ARCHITECTURE.md`.
- Thesis roadmap and milestones: `PLAN.md`.

## Scope of Allowed Modifications
- Primary ROS feature work: `src/tracking_pkg`.
- Do not modify `src/Universal_Robots_ROS2_Gazebo_Simulation` unless explicitly requested.
- Non-ROS tasks (standalone logging, helper scripts, data utilities) belong in `ros_unrelated_scripts`.

## Runtime Policy Enforcement
- Active runtime path is MoveIt-based:
  - launch entry: `src/tracking_pkg/launch/loop_launch.py`
  - core handover node: `src/tracking_pkg/src/moveit_mover/loop_mover.cpp`
- `socket_mover` is currently dormant/deferred and not part of the active runtime.
- Do not route new behavior into `socket_mover` unless explicitly requested and scoped.

## Logging and Observability Standards
- Use ROS-native logs:
  - Python: `self.get_logger().info/warn/error`
  - C++: `RCLCPP_INFO/WARN/ERROR`
- For handover-critical decisions, logs must explicitly report:
  - hand detected,
  - reachability decision (`reachable` or `unreachable`),
  - action accepted/rejected with reason.
- Do not leave silent branches in safety-critical logic.

## Documentation Synchronization Rules
- Any change to runtime path or topology must update `ARCHITECTURE.md`.
- Any change to milestones, workstreams, or temporal scope must update `PLAN.md`.
- Any change to contribution policy or repository rules must update `AGENTS.md`.
- Relevant changes have to be updated to `README.md`.
- Always use canonical uppercase doc names in links and mentions:
  - `AGENTS.md`
  - `ARCHITECTURE.md`
  - `PLAN.md`
