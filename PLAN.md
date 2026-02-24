# Thesis Development Plan (Workstreams and Milestones)

## Document Map
- This document's role: temporal roadmap (workstreams, milestones, sequencing, deferred gates).
- See also: `PLAN.md` (this file), `ARCHITECTURE.md`, `AGENTS.md`.
- Current baseline system truth: `ARCHITECTURE.md`.
- Contribution and boundary policy: `AGENTS.md`.

## Summary
This roadmap keeps the core thesis streams intact while enforcing:
- hybrid Ubuntu runtime with optional Mac-over-SSH operation,
- deferred direct-control path (`socket_mover`),
- policy-aligned execution and logging standards defined in `AGENTS.md`.

## Workstreams

## Workstream A: Observability and Logging
- Objective: improve traceability of perception and handover decisions.
- Outputs:
  - explicit runtime decision logging for hand detection and reachability,
  - reproducible terminal-level diagnostics.
- Dependency: logging standards in `AGENTS.md`.

## Workstream B: Reachability Classification and Audio Feedback
- Objective: classify target hand poses via MoveIt handover planning and provide selective audio feedback.
- Outputs:
  - runtime event interface `/handover_event` (`std_msgs/msg/String`) with:
    - `gesture_detected` (only in valid waiting state after tool pickup),
    - `reachability:unreachable_plan_failed` (when MoveIt handover planning fails),
  - audio behavior restricted to gesture-detected and unreachable outcomes,
  - explicit no-audio behavior for non-waiting or pre-pickup hand pose messages.
- Dependency: baseline data flow in `ARCHITECTURE.md`.

## Workstream C: Target-Hand Selection and ROI Visualization
- Objective: track intended target hand instead of defaulting to first detection.
- Outputs:
  - target-hand selection strategy,
  - ROI definition and frame visualization.

## Workstream D: Context-Aware Planning
- Objective: replace hardcoded pickup/handover orientation behavior with context-aware strategy.
- Outputs:
  - tool/affordance-aware orientation selection,
  - non-hardcoded pickup decision logic.

## Cross-Cutting Item 1: Execution Context Hardening
- Verify commands and runbooks are valid for Ubuntu runtime execution.
- Provide SSH-friendly operator command variants where relevant.
- Keep development host vs runtime host separation explicit.

## Cross-Cutting Item 2: Direct Control Path (Deferred)
- `socket_mover` remains dormant/deferred.
- Decision gate: activate only after external IK/planner scope is explicitly approved.
- Until gate approval, runtime behavior remains MoveIt-centric (see `ARCHITECTURE.md`).

## Milestones

## M0: Baseline Documentation and Alignment
- Deliverables:
  - synchronized doc ownership split,
  - canonical references across `AGENTS.md`, `ARCHITECTURE.md`, `PLAN.md`.

## M1: Reachability Observability
- Deliverables:
  - reliable reachable/unreachable traceability in runtime logs,
  - acceptance checks for decision-path visibility.

## M2: Audio Feedback Integration
- Deliverables:
  - integrated event-to-audio signaling via `/handover_event`,
  - speaker output on Ubuntu runtime via `aplay`,
  - acceptance checks:
    - one tone on `gesture_detected`,
    - one negative tone on `reachability:unreachable_plan_failed`,
    - no sound for hand pose messages while not waiting for gesture.

## M3: Target-Hand Robustness
- Deliverables:
  - target-hand selection implementation concept,
  - ROI visualization and validation criteria.

## M4: Context-Aware Planning Prototype
- Deliverables:
  - context/affordance-aware orientation and pickup strategy,
  - initial performance/behavior validation protocol.

## M5: Evaluation and Thesis Packaging
- Deliverables:
  - consolidated experiment and benchmark outputs,
  - thesis-ready artifact set (architecture, policy alignment, roadmap completion).

## Deferred Decision Gate: `socket_mover`
- Gate question: is IK/planning intentionally moved outside MoveIt scope?
- If `No`: keep `socket_mover` dormant.
- If `Yes`: require explicit design/safety/logging review before activation.

## Interfaces and Types
- Implemented runtime interface:
  - `/handover_event` (`std_msgs/msg/String`) with events:
    - `gesture_detected`
    - `reachability:unreachable_plan_failed`
