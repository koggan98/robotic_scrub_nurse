# Handover / Reclaim Evaluation Tooling

Quantitative evaluation for the robotic scrub nurse (master thesis). Runs a
**passive** ROS2 node that listens to the topics the system already publishes,
segments them into *cycles*, and measures how long each cycle takes, how often it
fails and why. A separate offline script turns the CSV into plots and tables.

The evaluator **only observes** — it never commands the robot. You run the
experiment as usual (voice / CLI trigger, take the tools by hand); it records
everything and prints a report at the end.

## Two scripts

| Script | Where to run | Needs |
|---|---|---|
| `eval_handover_reclaim.py` | on the robot, in a sourced ROS2 workspace | `rclpy`, `tracking_msgs` (stdlib otherwise) |
| `analyze_eval.py` | anywhere (e.g. dev laptop) | `pandas`, `matplotlib` |

## Running an experiment

1. Launch the system as normal (`jetson_launch.py` + `nuc_launch.py`).
2. In another terminal (ROS env sourced), start the evaluator:

   ```bash
   python3 ros_unrelated_scripts/eval_handover_reclaim.py --session-name pilot
   # or auto-finish after N completed cycles:
   python3 ros_unrelated_scripts/eval_handover_reclaim.py --target 100 --session-name run1
   ```

3. Run your handovers and reclaims (each "pick + hand over" and each "put back").
   The node prints a line per completed cycle.
4. Stop with **Ctrl-C** (or let `--target` finish it). It writes the CSVs +
   summary to `~/scrub_nurse_eval/<date>_<session-name>/` and prints a summary.

Optional: run `scrub_nurse_logger.py` (the keypress logger) at the same time — its
human-tagged outcome is folded into the matching cycle (`human_outcome`,
`human_failure_label`). Disable with `--no-merge-logger`.

### Useful flags

| Flag | Default | Meaning |
|---|---|---|
| `--target N` | 0 (Ctrl-C) | Auto-finish after N completed cycles |
| `--session-name` | `eval` | Label in the output folder name |
| `--data-dir` | `~/scrub_nurse_eval` | Output root |
| `--retry-window` | 12 s | A re-pick this soon after a DROPPED counts as the same cycle |
| `--cycle-timeout` | 180 s | Fail an open cycle after this long (0 = never; kept high because a handover waits for the surgeon) |
| `--merge-window` | 15 s | Window to fold a keypress-logger event into a cycle |
| `--no-merge-logger` | off | Ignore `/scrub_nurse_logger/event` |

## Analysing

```bash
pip install pandas matplotlib          # once, outside the ROS env
python3 ros_unrelated_scripts/analyze_eval.py ~/scrub_nurse_eval/20260724_pilot
```

Writes into `<session>/plots/`: `duration_hist.png`, `subphase_box.png`,
`success_by_tool.png`, `timeline.png`, and `summary_table.md` / `.tex`.

## What a "cycle" is

Anchored on `/tool_event` (emitted by the executor — authoritative, not inferred):

- **Handover** = `PICKED` (from `instrument_tray`) → `HANDED_OVER`
- **Reclaim**  = `PICKED` (from `reclaim_tray`)  → `PLACED_HOME`
- **Failure**  = `DROPPED` with no re-pick in the retry window, a `RECOVERY_ERROR`
  state, an alerting `/system_response` (`cannot`, `keeps dropping`, `failed`, …),
  or a cycle that never completes (`--cycle-timeout`). A pick that never grasps
  (alert with no `PICKED`) is still counted as a failed cycle.

A `DROPPED` followed by a fresh `PICKED` of the same tool within `--retry-window`
is treated as a **retry of the same cycle** (`attempts` > 1); the cycle still
counts as success if it eventually completes.

## Output files

### `events.csv` — raw, every message with a timestamp

`t_iso, t_rel, ros_stamp, topic, kind, value, tool_class, from_location, track_id, cycle_index`

One row per received message across all subscribed topics. `t_rel` is seconds
since the node started (monotonic, stamped on receipt); `ros_stamp` is the
message header stamp where present (`/tool_event`). This is the full state trace.

### `cycles.csv` — one row per cycle

| Column | Meaning |
|---|---|
| `cycle_index`, `type`, `tool_class`, `track_id` | identity |
| `outcome` | `success` / `failure` |
| `failure_reason` | `dropped`, `recovery_error`, `aborted`, `timeout`, `superseded`, `response:<text>` |
| `attempts` | 1 + number of in-cycle retries |
| `n_dropped`, `n_unreachable` | drops / "hand out of reach" events during the cycle |
| `t_start_iso`, `t_start_rel`, `t_end_rel` | start (UTC + relative) and end |
| `d_total` | grasp → completion (`PICKED`→`HANDED_OVER`/`PLACED_HOME`) |
| `d_command_to_pick` | voice command → grasp (blank if no `/user_speech`) |
| `d_pick` | approach → grasp (`PICKING` state → `PICKED`) |
| `d_gesture_wait` | wait for surgeon's gesture (`AWAIT_GESTURE` → gesture) — handover |
| `d_move_to_hand` | move to the hand (`HANDOVER` → `PRESENTING`) — handover |
| `d_present_to_taken` | offered → surgeon took it (`PRESENTING` → `HANDED_OVER`) — handover |
| `d_transport` | carry to home slot (`TRANSPORTING` → `RETURNING`) — reclaim |
| `d_place` | place into slot (`RETURNING` → `PLACED_HOME`) — reclaim |
| `human_outcome`, `human_failure_label` | from the keypress logger, if running |

Blank duration columns mean the relevant anchor was not observed for that cycle
(e.g. handover columns are blank for reclaim cycles and vice-versa).

### `summary.json` / `summary.txt`

Per-type counts, success rate, per-sub-phase duration stats
(n / mean / median / std / min / max / p95, over successful cycles), failure
breakdown by reason, and a per-tool-class table. `summary.txt` is what is printed
on exit.

## Notes

- Durations are measured from message **receipt time** (one monotonic clock) so
  they are consistent across the headerless status topics. `/tool_event` also
  carries a header stamp, logged in `events.csv` as `ros_stamp`.
- Because commands may be injected on `/user_speech` or via the CLI action
  client (voice ASR is gated on a wake-word model), the evaluator is
  trigger-agnostic: it anchors on `/tool_event`. `d_command_to_pick` is simply
  blank when no command was seen on `/user_speech`.
