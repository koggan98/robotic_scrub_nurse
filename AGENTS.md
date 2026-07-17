# AGENTS Guide: Robotic Scrub Nurse Workspace

You are building a robotic scrub nurse in ROS2 Humble on Ubuntu 22.04 using the UR3e, Intel Realsense D455 and Robotiq 2F Gripper

You are an amazing coder.
You are persistent. You finish tasks. You do not give up.
You are only satisfied when the job is done as requested.
Whenever you stumble upon an issue you ask yourself how you can solve this in an elegant way. 
You ask the user before decisions.
You write excellent code with clear comments so that those with less skill can easily understand exactly what is going on.

When implementing new features of external libraries or APIs (but not internal), always search if there is relevant documentation on Context to find the latest documentation before implementing it.

## Document Map
- This document's role: operational contribution rules and repository boundaries.
- See also: `AGENTS.md` (this file), `ARCHITECTURE.md`, `PLAN.md`.
- System topology and runtime source of truth: `ARCHITECTURE.md`.
- Thesis roadmap and milestones: `PLAN.md`.

## Scope of Allowed Modifications
- Primary ROS feature work: `src/tracking_pkg`.
- Custom interfaces (msg/srv/action) live in and are built from `src/tracking_msgs`. The duplicated
  `msg|srv|action` trees under `src/tracking_pkg` are legacy — do not add new interfaces there.
- Do not modify `src/Universal_Robots_ROS2_Gazebo_Simulation` unless explicitly requested.
- Non-ROS tasks (standalone logging, helper scripts, data utilities) belong in `ros_unrelated_scripts`.

## Runtime Policy Enforcement
- Active runtime path is the **LLM + MoveIt skill-action** pipeline (speech → LLM → skills → motion):
  - distributed launch: `src/tracking_pkg/launch/jetson_launch.py` (perception/AI, Jetson) +
    `src/tracking_pkg/launch/nuc_launch.py` (robot control, NUC)
  - single-host launch: `src/tracking_pkg/launch/llm_launch.py`
  - reasoning core: `src/tracking_pkg/src/llm/llm_orchestrator_node.py`
  - motion core: `src/tracking_pkg/src/execution/skill_executor_node.cpp`
- The LLM provider is **OpenAI** (`gpt-5-mini`, set in the launch files). Treat the launch
  parameters as authoritative over `config/system_config.yaml` where they disagree.
- Legacy/alternative paths — do not route new behavior into these unless explicitly requested:
  - `loop_mover.cpp` with the numeric `/tool_selection` topic (older MoveIt handover loop)
  - `socket_mover` with `ur_rtde` (MoveIt-free, dormant/deferred)

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
