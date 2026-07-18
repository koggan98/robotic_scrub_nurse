# Architecture: Robotic Scrub Nurse (Current Baseline)

## Document Map
- This document's role: current system truth (topology, execution paths, flow, constraints, interfaces).
- See also: `ARCHITECTURE.md` (this file), `AGENTS.md`, `PLAN.md`.
- Contributor rules and repository boundaries: `AGENTS.md`.
- Implementation roadmap and milestones: `PLAN.md`.

## System Context
The active baseline is a ROS 2 Humble workspace centered on `src/tracking_pkg`, implementing a
**speech → LLM → skill-action → motion** pipeline for surgical instrument handover with a UR3e:
- RealSense perception (reclaim-tray + instrument-tray cameras),
- YOLOv8-OBB instrument detection with body/handle pairing and grasp reasoning,
- MediaPipe hand tracking and gesture detection (`double_open_close`),
- a persistent, continuously-updated world model serialized as JSON for the LLM,
- an **OpenAI (function-calling) orchestrator** that plans and dispatches robot skills,
- a **C++ MoveIt skill executor** (pick / handover / release / return_home / return_tool) with
  force-triggered release and robust grasp/loss recovery.

## Deployment Topology
The system is **physically distributed across two machines** on a dedicated wired link
(`192.168.12.0/24`, CycloneDDS, unicast-only so the UR robot network stays quiet). Both machines
share the same `ROS_DOMAIN_ID`.

```text
NVIDIA Jetson Orin Nano (192.168.12.6)        Intel NUC (192.168.12.5)
Perception + AI (headless, ARM64/CUDA)        Robot control (x86_64)
  cameras, YOLO-OBB, hand tracking,             UR driver, MoveIt move_group,
  ArUco, grasp reasoning, world model,   <-->   skill_executor, gripper, collision
  ASR (Whisper), LLM orchestrator          DDS  publishers, RViz
                                                     |
                                                     v
                                          UR3e + Robotiq 2F + (MiR base as collision object)
```

- Jetson bring-up: `src/tracking_pkg/launch/jetson_launch.py`, DDS `cyclone_dds_jetson.xml`.
- NUC bring-up: `src/tracking_pkg/launch/nuc_launch.py`, DDS `cyclone_dds_nuc.xml`.
- `src/tracking_pkg/launch/llm_launch.py` is an obsolete single-host snapshot and is not an active
  deployment path.

The Jetson is deliberately **headless** — `jetson_launch.py` does not build a `robot_description`
(that would pull version-fragile UR xacros and abort the launch on failure); the robot model and
RViz live only on the NUC.

## Host Roles
- Jetson Orin Nano: perception and AI runtime.
- Intel NUC: robot control, MoveIt, and visualization runtime.
- The UR driver (`ur_control.launch.py`, `robot_ip:=…`) is started separately by the operator on the NUC.

## Package Boundaries in the Runtime System
- Active thesis package: `src/tracking_pkg`.
- Custom interface package (the one actually built/imported): `src/tracking_msgs`
  (`msg/ srv/ action/`). The duplicated `msg|srv|action` trees under `src/tracking_pkg` are legacy.
- Included upstream simulation package: `src/Universal_Robots_ROS2_Gazebo_Simulation`.
- External utility area (outside ROS node graph): `ros_unrelated_scripts`.

## Execution Paths

### Active Path (LLM + MoveIt skill actions)
- Distributed launch: `jetson_launch.py` (perception/AI) + `nuc_launch.py` (robot control).
- Reasoning core: `src/tracking_pkg/src/llm/llm_orchestrator_node.py`
  (OpenAI `gpt-5-mini`, native tool/function-calling, up to 8 tool turns).
- Motion core: `src/tracking_pkg/src/execution/skill_executor_node.cpp`
  (MoveIt `MoveGroupInterface`, group `ur_manipulator`, planning frame `world`).
- Command flow is speech-driven via `/user_speech`; the LLM dispatches ROS **actions**
  (`pick_tool`, `handover_tool`, `release_tool`, `return_home`, `return_tool`) and the
  `/get_world_model` service.

### Legacy / Alternative Paths (not the active runtime)
- **Obsolete single-host snapshot:** `llm_launch.py` is retained for reference but is not kept in
  sync with the distributed runtime. New runtime behavior belongs in the per-machine launches.
- **Loop path (MoveIt-centric, numeric tool selection):** `loop_mover.cpp` driven by the
  `/tool_selection` topic. The executable is retained for direct manual bench testing, but neither
  `nuc_launch.py` nor `llm_launch.py` starts it. This is the older generation.
- **Socket + RTDE (MoveIt-free):** `src/tracking_pkg/src/socket_mover/` with `ur_rtde`
  (controller-side IK, RTDE TCP force). Dormant/deferred; not part of the active runtime.
- **Manual pick test:** `tool_pick_test_launch.py` + `tool_pick_test_node` and the on-demand
  `world_model_builder.py` (`/build_world_model`) — bench testing only; `world_model_builder`
  is intentionally excluded from the production Jetson launch (it converts the tray image at
  camera rate, ~63% CPU + ~1.6 GB, for no production consumer).

## High-Level Active ROS Flow
1. `asr_node` (faster-whisper `tiny.en`, local, energy-based VAD) runs on the Jetson, automatically
   selects the single connected supported USB microphone, transcribes a spoken command, and
   publishes it on `/user_speech`. Samson Q2U is captured directly at 16 kHz; the Jieli receiver is
   captured at its native 48 kHz and decoded/resampled to Whisper's 16 kHz. With no supported input
   ASR waits and rescans; two simultaneous supported inputs are rejected as ambiguous. After the
   wake-word gate, the exact one-token `Oh.` substitution is corrected to `awl`; `oh` inside longer
   phrases, unaddressed speech, and direct topic injection remain untouched.
2. `llm_orchestrator_node` consumes `/user_speech`, calls `/get_world_model` (JSON scene snapshot),
   matches the request to a tracked `tool_id` via `config/tool_knowledge_base.yaml` synonyms
   (multilingual, incl. German), and runs an OpenAI tool-calling loop.
3. The LLM dispatches skills as tools: `get_world_model`, `pick_and_handover(tool_id)`,
   `return_tool`, `release_tool`, `return_home`, `abort`. Each returns a JSON result string the
   model reasons about (retry / disambiguate / stop). Terse status is published on `/system_response`.
4. `skill_executor_node` (NUC) executes motion. `pick_tool` pre-plans approach→descend→lift for each
   candidate (confidence order) and only commits to a fully-plannable one. Instrument-tray grasp
   pre-flights constrain `elbow_joint >= 0` and independently validate every trajectory point, so an
   Elbow-down IK branch is rejected before any gripper command; the constraint does not leak into
   later motion. Reclaim picks run up to
   two complete pre-flight rounds from the unchanged lower staging pose; a failed first round causes
   no motion, perception refresh, or Home retreat. Physical grasp/execution/loss failures are not
   retried locally. Generic picks attach a `held_tool` collision box after clearing their tray.
   `ReturnToolHome` instead extends each pre-flight through approach→descent→4 cm lift→the fixed
   `instrument_stage_joints` pose→the complete taught `instrument_left_stage` TCP pose. For a
   right-side slot, the pre-flight continues through a complete Cartesian interpolation to the full
   taught Home TCP pose, including its orientation; it does not freeze the Left-Stage orientation.
   All legs use chained future start states and execute from cached plans after grasping, so a
   right-corridor planning failure leaves the tool untouched on reclaim. This hardware-validated
   fixed corridor intentionally has no hypothetical tool box; the real box is attached at
   Left-Stage for left slots or Home for right slots, before the collision-checked local slot plan.
   Empty returns from Reclaim also use instrument-stage→Left-Stage→Home; other reclaim routes retain
   `exitReclaimToUpper` while carrying a tool.
5. `handover_tool` sets `/handover_waiting=true`, waits for the surgeon's `double_open_close` gesture
   on `/hand_gesture`, and plans to `hand_pose + hand_offset`. On arrival it immediately publishes
   `PRESENTING`, which becomes green `TAKE` on the HRI display after its `0.1 s` non-red debounce;
   red/alert states remain immediate. The unchanged pre-release dwell completes before the executor
   enables force-guided physical release. After release, the empty arm retraces to the exact joint
   pose from which it departed for the hand, then returns through Left-Stage→Home rather than
   free-planning directly from the hand pose.
6. `gripper_opener_with_zeroer.py` drives the Robotiq (URCap socket, port 63352) and opens on a
   force tug read from `/force_torque_sensor_broadcaster/wrench`; it publishes `/tool_grasped`
   as both grasp verification and continuous loss monitor. Robotiq `gOBJ=2` is direct grasp
   evidence; thin tools that do not set it are rescued only in the exclusive gPO interval
   `[180, 228)`. Thus `gPO=227` is accepted but `gPO=228` is not. The fixed NUC launch setting
   leaves two counts below the approximate empty-close value `230`; lower the maximum and restart
   the NUC launch if empty-gripper false positives occur, because they activate the holding guard.
   The loss monitor applies that same evaluation, so `gOBJ=3/gPO=225` remains held; `OBJ=None` and
   moving `OBJ=0` are inconclusive. It confirms a negative indication with a second sample after
   `grasp_check.loss_confirm_delay_sec=0.1` and reports loss only if both are negative.
7. The executor broadcasts `STATE:tool_id:tool_class` on `/system_state_update`; `world_model_node`
   folds this back into the world model (`gripper_holds_tool`, active tool, state).
8. On a post-grasp `ReturnToolHome` execution/place failure, the executor stops and confirms the
   Robotiq state. If the tool is still held, it waits 0.25 s, keeps the gripper closed and the
   collision box attached, publishes `RECOVERY_ERROR` with the held tool identity, and rejects new
   pick-like goals. `return_home` can reposition the arm but cannot publish `IDLE` while this holding
   lock is active; `return_tool` or an explicit `release_tool` clears it. Only a fresh negative grasp
   check permits the existing `DROPPED`/open recovery; a timeout remains inconclusive and keeps the
   gripper closed. On other dropped/lost tools the executor
   surfaces `tool_lost`/`grasp_failed`; the router can start an **autonomous** LLM turn (no human
   command) to re-pick by class, capped at 2 retries.
9. `handover_sound_publisher` plays canned WAV cues for `gesture_detected` and unreachable events
   via the first available backend (`paplay`, `pw-play`, then `aplay`).

## Perception → Reasoning → World Model Chain (Jetson)
```
tray_camera  → tool_detection_node (YOLOv8-OBB, GPU, 4 Hz) → /detected_tools_obb
                → grasp_geometry_node → /tool_grasp_candidates
                → tool_semantics_node (+ tool_knowledge_base.yaml) → /enriched_tool_grasp_candidates
reclaim_tray_camera → hand_tracker (MediaPipe, CPU) → /hand_state, /hand_gesture
reclaim_tray_camera → aruco_marker_manager (marker 105) → reclaim tray camera TF (lock once)
tray_camera  → aruco_marker_manager (marker 110) → tray camera TF (lock once)
   → world_model_node (persistent tool IDs) → /get_world_model (JSON) + /get_world_state (typed)
```
A parallel `reclaim_*` detection/grasp/semantics chain runs on the reclaim tray camera at 4 Hz for the
intermediate reclaim tray; it is **not yet wired into the world model or execution**.

## Current Constraints
- LLM path (`skill_executor` + `llm_orchestrator`) is the primary thesis runtime path.
- `loop_mover` (numeric `/tool_selection`) and `socket_mover` (RTDE) are legacy/alternative and not
  the active flow.
- Perception rates are tuned to the Orin's CPU/GPU ceiling; heavy model loads are staggered at boot.
- ArUco localization is "lock once": camera-in-marker pose is fixed after a smoothed multi-frame lock,
  after which camera subscriptions are torn down to free Jetson CPU.
- Reclaim-tray perception exists but is not yet a runtime input.
- The LLM provider is **OpenAI** (`gpt-5-mini`, set in the launch files). `config/system_config.yaml`
  still lists `gpt-4o` and `whisper base`/`de`, but the launch parameters override those; treat the
  launch files as authoritative.

## Runtime Interfaces

### Actions (`tracking_msgs/action`, LLM ↔ skill_executor)
- `pick_tool` (`PickTool`) — goal `tool_id`; result adds `picked_tool_id`/`picked_tool_class`.
- `handover_tool` (`HandoverTool`) — goal `PoseStamped hand_pose` (empty ⇒ wait for gesture);
  feedback phases `AWAITING_GESTURE` / `MOVING_TO_HAND` / `RELEASED`.
- `release_tool` (`ReleaseTool`), `return_home` (`ReturnHome`), `return_tool` (`ReturnTool`).

### Services (`tracking_msgs/srv`)
- `/get_world_model` (`GetWorldModel`) → JSON scene snapshot for the LLM.
- `/get_world_state` (`GetWorldState`) → typed `SystemState` for the executor.
- `/get_tool_candidates` (`GetToolCandidates`), `/get_grasp_approach_pose` (`GetGraspApproachPose`),
  `/build_world_model` (`BuildWorldModel`, test path only).

### Topics
- `/user_speech` (`std_msgs/String`) — ASR transcript in.
- `/system_response` (`std_msgs/String`) — terse LLM status out.
- `/hand_state` (`tracking_msgs/HandState`), `/hand_pose` (`geometry_msgs/Pose`), `/hand_gesture`
  (`std_msgs/String`), `/handover_waiting` (`std_msgs/Bool`, latched).
- `/detected_tools_obb`, `/tool_grasp_candidates`, `/enriched_tool_grasp_candidates`
  (`tracking_msgs/ToolDetectionArray` / `GraspCandidateArray`), plus `/reclaim_*` mirrors.
- `/system_state_update` (`std_msgs/String`, `STATE:tool_id:tool_class`).
- `/gripper_mover`, `/gripper_zeroer`, `/verify_grasp`, `/tool_grasped`, `/gripper_done`
  (gripper control + force-guided release).
- `/handover_event` (`std_msgs/String`: `gesture_detected`, `reachability:unreachable_*`).
- `/collision_object` (`moveit_msgs/CollisionObject`: `mir`, `instrument_tray`, reclaim tray) —
  published on the NUC with latched TRANSIENT_LOCAL QoS at 0.2 Hz.

### Custom Messages (`tracking_msgs/msg`)
`ToolDetection`, `ToolDetectionArray`, `OrientedBoundingBox2D`, `GraspCandidate`,
`GraspCandidateArray`, `HandState`, `SystemState`, `ExecutionPlan` (legacy).

## Tracking Frame Contract
- Canonical planning/tracking frame: `world`.
- `world → base` is a static TF published on the NUC (`nuc_launch.py`, yaw π).
- Camera localization is provided by ArUco on the Jetson: marker **105** localizes the reclaim tray camera
  (`aruco_marker_105_frame → reclaim_tray_camera_color_optical_frame`), marker **110** localizes the tray
  camera (`aruco_marker_110_frame → tray_camera_color_optical_frame`). Both lock once from a smoothed
  multi-frame estimate, then re-broadcast at 1 Hz for late TF subscribers. Marker 120 is retired.
- Static `world → aruco_marker_105_frame` and `world → aruco_marker_110_frame` are published by the
  launch for RViz visibility; the ArUco manager runs with `publish_marker_static_tfs=False` so the
  launch's static TFs win.
- Tray-camera depth is disabled; instrument grasp pixels are projected onto a fixed tool plane
  (`fixed_tool_plane_z_m = 0.04 m`) because the tray camera is mounted closer than its minimum
  reliable sensor depth.
- Collision geometry: `instrument_tray_collision_publisher` needs `world → tray_camera_color_optical_frame`
  (arrives from the Jetson over DDS); `mir_publisher` and `reclaim_tray_collision_publisher` are pure
  world-frame. RViz fixed frame is `world`.
