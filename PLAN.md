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
- Objective: classify target hand poses as reachable/unreachable and provide audio feedback for both outcomes.
- Outputs:
  - reachability decision path,
  - audio event behavior for reachable/unreachable states.
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
  - reachable/unreachable audio behavior specification,
  - integrated event-to-audio signaling concept.

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
- No immediate code API/interface/type changes in this documentation refactor.
- Planned interfaces remain architecture-owned and non-implemented (see `ARCHITECTURE.md`).
