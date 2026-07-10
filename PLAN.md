# Thesis Development Plan (Workstreams and Milestones)

## Document Map
- This document's role: temporal roadmap (workstreams, milestones, sequencing, deferred gates).
- See also: `PLAN.md` (this file), `ARCHITECTURE.md`, `AGENTS.md`.
- Current baseline system truth: `ARCHITECTURE.md`.
- Contribution and boundary policy: `AGENTS.md`.

## Summary
The system has moved from a single-host, numeric `/tool_selection` MoveIt loop to a
**distributed speech → LLM → skill-action → motion** architecture:
- perception + AI on a Jetson Orin Nano, robot control on an Intel NUC, over a wired CycloneDDS link,
- an OpenAI function-calling orchestrator driving MoveIt skill actions,
- a persistent perception-driven world model and force-guided handover,
- policy-aligned execution and logging standards defined in `AGENTS.md`.

## Delivered Workstreams

### WS-1: Distributed runtime (NUC / Jetson)
- System split across the Intel NUC (robot control) and Jetson Orin Nano (perception/AI).
- Per-machine launches (`nuc_launch.py`, `jetson_launch.py`) and CycloneDDS configs.
- Orin CPU/GPU tuning: ArUco lock-once + subscription teardown, collision publishers moved to the
  NUC, deferred image conversion, staggered model loads, detection rates fitted to the Orin.

### WS-2: Speech + LLM orchestration
- Local speech-to-text (`asr_node`, faster-whisper, energy VAD, `/user_speech`).
- OpenAI function-calling orchestrator (`llm_orchestrator_node`) exposing robot skills as tools
  (`get_world_model`, `pick_and_handover`, `return_tool`, `release_tool`, `return_home`, `abort`).
- Terse status feedback on `/system_response`; audio cues via `handover_sound_publisher`.

### WS-3: Perception + world model
- YOLOv8-OBB instrument detection with body/handle pairing (`tool_detection_node`).
- Grasp geometry + semantic enrichment from `tool_knowledge_base.yaml`.
- Persistent world model with stable tool IDs, serialized as JSON for the LLM (`world_model_node`).

### WS-4: Robust skill execution
- C++ MoveIt skill executor with pick / handover / release / return_home / return_tool actions.
- Pre-flight full-sequence planning, grasp verification via Robotiq object-detect, escalating local
  re-grasp, mid-transport loss abort, holding guard, and force-guided release.
- Autonomous LLM retries on dropped/lost tools (capped).

## Open Workstreams

### WS-5: Reclaim-tray integration
- A `reclaim_*` perception/grasp/semantics chain already runs on the reclaim tray camera (0.5 Hz) but is
  **not yet wired** into the world model or execution. Objective: fold it into `world_model_node`
  (with an appropriate tracker max-age) and support reclaim/return-to-holder flows.

### WS-6: Context-aware planning
- Replace remaining hardcoded pickup/handover orientation behavior with tool/affordance-aware
  strategy driven by the knowledge base (`grip_strategy`, `handover_rule`, `functional_end`).

### WS-7: Target-hand robustness and evaluation
- Robust target-hand selection (intended receiver vs. first detection).
- Consolidated experiment/benchmark tooling and thesis-ready artifact set.

## Cross-Cutting Items
- **Execution context hardening:** keep runbooks valid for the distributed NUC/Jetson runtime and
  SSH-friendly; keep the single-host `llm_launch.py` path working as a fallback.
- **Observability:** ROS-native logging of hand detection, reachability, and accepted/rejected
  actions with reasons (per `AGENTS.md`).
- **Legacy paths:** `loop_mover` (numeric `/tool_selection`) and `socket_mover` (RTDE) remain in the
  tree as alternative/dormant paths; keep safety/logging behavior aligned if they are revived.

## Milestones

- **M0 — Baseline documentation alignment:** docs (`ARCHITECTURE.md`, `README.md`, `AGENTS.md`,
  `deployment_guide.md`, `PLAN.md`) reflect the distributed LLM/skill architecture. *(current)*
- **M1 — Distributed runtime stable:** NUC/Jetson bring-up reliable within the Orin's resource budget.
- **M2 — Speech-to-handover loop:** spoken command → pick → gesture-gated, force-released handover.
- **M3 — Reclaim-tray integration:** reclaim perception wired into the world model and execution.
- **M4 — Context-aware planning prototype:** affordance-aware pickup/handover orientation.
- **M5 — Evaluation and thesis packaging:** consolidated benchmarks and thesis-ready artifacts.

## Interfaces and Types
- Actions: `PickTool`, `HandoverTool`, `ReleaseTool`, `ReturnHome`, `ReturnTool` (`tracking_msgs/action`).
- Services: `GetWorldModel`, `GetWorldState`, `GetToolCandidates`, `GetGraspApproachPose`,
  `BuildWorldModel` (`tracking_msgs/srv`).
- Topics: `/user_speech`, `/system_response`, `/hand_state`, `/hand_gesture`, `/system_state_update`,
  `/tool_grasped`, `/handover_event`, `/collision_object`.
