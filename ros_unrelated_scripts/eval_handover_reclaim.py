#!/usr/bin/env python3
"""Passive ROS2 evaluator for handover + reclaim experiments (master thesis).

Subscribes to the topics the scrub-nurse system already publishes, segments the
event stream into *cycles* (handover = instrument tray -> surgeon's hand;
reclaim = reclaim tray -> instrument tray), and measures per-cycle timing,
success/failure and failure reason. Writes a raw per-event log and a per-cycle
table to CSV, and prints a summary on exit.

This node only *observes* -- it never commands the robot. Run the experiment as
usual (voice / CLI trigger, take the tools by hand); this records everything.

    # in a sourced ROS2 workspace:
    python3 ros_unrelated_scripts/eval_handover_reclaim.py
    python3 ros_unrelated_scripts/eval_handover_reclaim.py --target 100 --session-name pilot

Stop with Ctrl-C (or let it auto-finish after --target completed cycles).
Then analyse with ros_unrelated_scripts/analyze_eval.py <session_dir>.

Cycle anchors come from /tool_event (authoritative, emitted by the executor);
/system_state_update adds sub-phases; /system_response and DROPPED give failures.
See README_eval.md for column definitions.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
import unicodedata
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from tracking_msgs.msg import ToolEvent


# --- Domain constants (mirrored from the runtime, see hri_display_logic.py) ---

# /system_state_update states that mark motion sub-phases we time.
_MOVE_STATES = {
    "PICKING", "TRANSPORTING", "AWAIT_GESTURE", "HANDOVER", "PRESENTING",
    "RETURNING", "RELEASING", "HOLDING",
}
# /system_response substrings that mean a failure/alert (from hri_display_logic).
_ALERT_SUBSTRINGS = (
    "cannot", "keeps dropping", "dropped", "missed", "failed", "error",
    "unavailable", "timed out", "out of reach",
)

INSTRUMENT_TRAY = "instrument_tray"
RECLAIM_TRAY = "reclaim_tray"


# --- Small helpers (pattern borrowed from scrub_nurse_logger.py) ---

def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def slugify(value: str) -> str:
    normalized = unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii")
    cleaned = "".join(ch.lower() if ch.isalnum() else "_" for ch in normalized).strip("_")
    return cleaned or "session"


def build_unique_dir(root: Path, base: str) -> Path:
    candidate = root / base
    if not candidate.exists():
        return candidate
    version = 2
    while True:
        candidate = root / f"{base}_v{version}"
        if not candidate.exists():
            return candidate
        version += 1


def is_alert_response(text: str) -> bool:
    low = (text or "").lower()
    return any(s in low for s in _ALERT_SUBSTRINGS)


def parse_state(data: str):
    """'STATE:tool_id:tool_class' -> (state, tool_id, tool_class)."""
    parts = (data or "").split(":")
    state = parts[0].strip().upper() if parts else ""
    tool_id = parts[1].strip() if len(parts) > 1 else ""
    tool_class = parts[2].strip() if len(parts) > 2 else ""
    return state, tool_id, tool_class


# --- Cycle model ---

@dataclass
class Cycle:
    index: int
    cycle_type: str            # 'handover' | 'reclaim' | 'unknown'
    tool_class: str = ""
    track_id: str = ""
    opened_at: float = 0.0     # t_rel when the cycle first opened
    provisional: bool = True   # opened on PICKING, not yet grasp-confirmed
    awaiting_retry: bool = False
    retry_deadline: float = 0.0
    attempts: int = 1
    n_dropped: int = 0
    n_unreachable: int = 0
    outcome: str = ""          # 'success' | 'failure'
    failure_reason: str = ""
    t_start_iso: str = ""
    human_outcome: str = ""
    human_failure_label: str = ""
    # named timestamp anchors (t_rel seconds), each recorded once
    anchors: dict = field(default_factory=dict)

    def mark(self, key: str, t: float) -> None:
        self.anchors.setdefault(key, t)


class EvalNode(Node):
    def __init__(self, args) -> None:
        super().__init__("handover_reclaim_evaluator")
        self.args = args
        self.t0 = time.monotonic()
        self.done = False

        # session dir
        root = Path(args.data_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        base = f"{datetime.now().strftime('%Y%m%d')}_{slugify(args.session_name)}"
        self.session_dir = build_unique_dir(root, base)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.session_id = self.session_dir.name

        # CSV writers (flush per row -> survives a crash mid-run)
        self._events_fh = (self.session_dir / "events.csv").open("w", newline="", encoding="utf-8")
        self._events_w = csv.DictWriter(self._events_fh, fieldnames=[
            "t_iso", "t_rel", "ros_stamp", "topic", "kind", "value",
            "tool_class", "from_location", "track_id", "cycle_index",
        ])
        self._events_w.writeheader()
        self._events_fh.flush()

        self._cycle_fields = [
            "cycle_index", "type", "tool_class", "track_id", "outcome",
            "failure_reason", "attempts", "n_dropped", "n_unreachable",
            "t_start_iso", "t_start_rel", "t_end_rel", "d_total",
            "d_command_to_pick", "d_pick", "d_gesture_wait", "d_move_to_hand",
            "d_present_to_taken", "d_transport", "d_place",
            "human_outcome", "human_failure_label",
        ]
        self._cycles_fh = (self.session_dir / "cycles.csv").open("w", newline="", encoding="utf-8")
        self._cycles_w = csv.DictWriter(self._cycles_fh, fieldnames=self._cycle_fields)
        self._cycles_w.writeheader()
        self._cycles_fh.flush()

        # runtime state
        self.active: Cycle | None = None
        self.cycle_rows: list[dict] = []
        self.completed = 0
        self._next_index = 1
        self._last_user_speech_t: float | None = None
        self._last_picking_t: float | None = None
        self._active_tool_class = ""
        self._pending_human: list[dict] = []   # unconsumed /scrub_nurse_logger rows

        self._subscribe()
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(f"Evaluator recording to: {self.session_dir}")
        if args.target:
            self.get_logger().info(f"Will auto-finish after {args.target} completed cycles.")

    # --- subscriptions ---

    def _subscribe(self) -> None:
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(ToolEvent, "/tool_event", self._on_tool_event, 20)
        self.create_subscription(String, "/system_state_update", self._on_state, 20)
        self.create_subscription(String, "/system_response", self._on_response, 20)
        self.create_subscription(String, "/handover_event", self._on_handover_event, 20)
        self.create_subscription(Bool, "/handover_waiting", self._on_handover_waiting, latched)
        self.create_subscription(Bool, "/tool_grasped", self._on_tool_grasped, 20)
        self.create_subscription(Bool, "/gripper_done", self._on_gripper_done, 20)
        self.create_subscription(String, "/user_speech", self._on_user_speech, 20)
        if not self.args.no_merge_logger:
            self.create_subscription(
                String, "/scrub_nurse_logger/event", self._on_logger_event, 20)

    # --- utilities ---

    def _rel(self) -> float:
        return time.monotonic() - self.t0

    def _log_event(self, topic, kind, value, tool_class="", from_location="",
                   track_id="", ros_stamp="") -> None:
        self._events_w.writerow({
            "t_iso": now_iso(), "t_rel": round(self._rel(), 3), "ros_stamp": ros_stamp,
            "topic": topic, "kind": kind, "value": value, "tool_class": tool_class,
            "from_location": from_location, "track_id": track_id,
            "cycle_index": self.active.index if self.active else "",
        })
        self._events_fh.flush()

    # --- topic callbacks ---

    def _on_user_speech(self, msg: String) -> None:
        self._last_user_speech_t = self._rel()
        self._log_event("/user_speech", "user_speech", msg.data)

    def _on_tool_grasped(self, msg: Bool) -> None:
        self._log_event("/tool_grasped", "tool_grasped", str(bool(msg.data)))

    def _on_gripper_done(self, msg: Bool) -> None:
        self._log_event("/gripper_done", "gripper_done", str(bool(msg.data)))

    def _on_handover_waiting(self, msg: Bool) -> None:
        self._log_event("/handover_waiting", "handover_waiting", str(bool(msg.data)))

    def _on_handover_event(self, msg: String) -> None:
        val = (msg.data or "").strip()
        self._log_event("/handover_event", "handover_event", val)
        if self.active is None:
            return
        if val == "gesture_detected":
            self.active.mark("gesture_detected", self._rel())
        elif val.startswith("reachability:unreachable"):
            self.active.n_unreachable += 1

    def _on_response(self, msg: String) -> None:
        text = (msg.data or "").strip()
        self._log_event("/system_response", "response", text)
        if not is_alert_response(text):
            return
        short = text if len(text) <= 60 else text[:57] + "..."
        if self.active is not None:
            self._close(self.active, "failure", f"response:{short}")
        else:
            # pick-stage failure with no grasp -> synthesise a failed cycle so
            # totally-failed attempts are still counted.
            ctype = "reclaim" if ("put back" in text.lower() or "reclaim" in text.lower()) else "handover"
            c = self._open(ctype, self._active_tool_class, "", provisional=False)
            self._close(c, "failure", f"response:{short}")

    def _on_state(self, msg: String) -> None:
        state, _tool_id, tool_class = parse_state(msg.data)
        self._log_event("/system_state_update", "state", state, tool_class=tool_class)
        if tool_class:
            self._active_tool_class = tool_class
        if state == "PICKING":
            self._last_picking_t = self._rel()
            if self.active is None:
                self._open("handover", tool_class, "", provisional=True, mark="picking")
        if self.active is None:
            return
        if state == "RECOVERY_ERROR":
            self.active.n_dropped = max(self.active.n_dropped, self.active.n_dropped)
            self._close(self.active, "failure", "recovery_error")
            return
        if state == "IDLE":
            # arm at rest with a cycle still open => the attempt aborted without a
            # terminal tool_event. Grace guards against races right after opening.
            if self._rel() - self.active.opened_at > 1.5:
                reason = "dropped" if self.active.awaiting_retry else "aborted"
                self._close(self.active, "failure", reason)
            return
        if state in _MOVE_STATES and tool_class and not self.active.tool_class:
            self.active.tool_class = tool_class
        if state in _MOVE_STATES:
            self.active.mark(state.lower(), self._rel())

    def _on_tool_event(self, msg: ToolEvent) -> None:
        event = (msg.event or "").strip().upper()
        stamp = ""
        try:
            stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        except Exception:
            pass
        self._log_event("/tool_event", "tool_event", event, tool_class=msg.tool_class,
                        from_location=msg.from_location, track_id=msg.track_id, ros_stamp=stamp)
        if event == "PICKED":
            self._on_picked(msg)
        elif event == "HANDED_OVER":
            if self.active is not None:
                self.active.mark("handed_over", self._rel())
                self._close(self.active, "success", "")
        elif event == "PLACED_HOME":
            if self.active is not None:
                self.active.mark("placed_home", self._rel())
                self._close(self.active, "success", "")
        elif event == "DROPPED":
            if self.active is not None:
                self.active.n_dropped += 1
                self.active.mark("dropped", self._rel())
                self.active.awaiting_retry = True
                self.active.retry_deadline = self._rel() + self.args.retry_window

    def _on_picked(self, msg: ToolEvent) -> None:
        t = self._rel()
        ctype = "handover" if msg.from_location == INSTRUMENT_TRAY else \
                "reclaim" if msg.from_location == RECLAIM_TRAY else "unknown"
        a = self.active
        if a is not None and a.awaiting_retry:
            # a re-pick within the retry window continues the SAME logical cycle
            a.attempts += 1
            a.awaiting_retry = False
            a.provisional = False
            if ctype != "unknown":
                a.cycle_type = ctype
            if msg.tool_class:
                a.tool_class = msg.tool_class
            if msg.track_id:
                a.track_id = msg.track_id
            a.mark("picked_retry", t)
            return
        if a is not None and a.provisional:
            # provisional cycle opened on PICKING is now grasp-confirmed
            a.provisional = False
            a.cycle_type = ctype if ctype != "unknown" else a.cycle_type
            a.tool_class = msg.tool_class or a.tool_class
            a.track_id = msg.track_id or a.track_id
            a.mark("picked", t)
            self._backfill_pick_context(a, t)
            return
        if a is not None:
            # a fresh grasp while another cycle is open => close the old as failure
            self._close(a, "failure", "superseded")
        c = self._open(ctype, msg.tool_class, msg.track_id, provisional=False)
        c.mark("picked", t)
        self._backfill_pick_context(c, t)

    def _backfill_pick_context(self, c: Cycle, t: float) -> None:
        if self._last_picking_t is not None and 0 <= t - self._last_picking_t <= 30.0:
            c.mark("picking", self._last_picking_t)
        if self._last_user_speech_t is not None and 0 <= t - self._last_user_speech_t <= 30.0:
            c.mark("user_speech", self._last_user_speech_t)

    # --- cycle open/close ---

    def _open(self, ctype, tool_class, track_id, provisional, mark=None) -> Cycle:
        c = Cycle(index=self._next_index, cycle_type=ctype, tool_class=tool_class or "",
                  track_id=track_id or "", opened_at=self._rel(), provisional=provisional,
                  t_start_iso=now_iso())
        self._next_index += 1
        self.active = c
        if mark:
            c.mark(mark, self._rel())
        return c

    def _close(self, c: Cycle, outcome: str, reason: str) -> None:
        if c.outcome:            # already closed
            return
        c.outcome = outcome
        c.failure_reason = reason
        # consume the "last seen" command/pick markers so they cannot leak into a
        # later cycle that had no command or PICKING of its own.
        self._last_user_speech_t = None
        self._last_picking_t = None
        self._merge_human(c)
        row = self._cycle_row(c)
        self.cycle_rows.append(row)
        self._cycles_w.writerow(row)
        self._cycles_fh.flush()
        self.completed += 1
        if self.active is c:
            self.active = None
        icon = "OK " if outcome == "success" else "FAIL"
        self.get_logger().info(
            f"[{icon}] cycle {c.index} {c.cycle_type} {c.tool_class or '?'} "
            f"total={row['d_total'] or '?'}s "
            f"{'' if outcome == 'success' else '(' + reason + ')'} "
            f"| done={self.completed}")
        if self.args.target and self.completed >= self.args.target:
            self.get_logger().info(f"Reached target of {self.args.target} cycles. Finishing.")
            self.done = True

    # --- derived per-cycle row ---

    @staticmethod
    def _dur(anchors, a, b):
        if a in anchors and b in anchors:
            return round(anchors[b] - anchors[a], 3)
        return ""

    def _cycle_row(self, c: Cycle) -> dict:
        an = c.anchors
        start = an.get("picked", an.get("picking", c.opened_at))
        if c.cycle_type == "reclaim":
            terminal = an.get("placed_home")
        elif c.cycle_type == "handover":
            terminal = an.get("handed_over")
        else:
            terminal = None
        t_end = terminal if terminal is not None else self._rel()
        d_total = round(t_end - an["picked"], 3) if "picked" in an else ""
        return {
            "cycle_index": c.index, "type": c.cycle_type, "tool_class": c.tool_class,
            "track_id": c.track_id, "outcome": c.outcome,
            "failure_reason": c.failure_reason, "attempts": c.attempts,
            "n_dropped": c.n_dropped, "n_unreachable": c.n_unreachable,
            "t_start_iso": c.t_start_iso, "t_start_rel": round(start, 3),
            "t_end_rel": round(t_end, 3), "d_total": d_total,
            "d_command_to_pick": self._dur(an, "user_speech", "picked"),
            "d_pick": self._dur(an, "picking", "picked"),
            "d_gesture_wait": self._dur(an, "await_gesture", "gesture_detected"),
            "d_move_to_hand": self._dur(an, "handover", "presenting"),
            "d_present_to_taken": self._dur(an, "presenting", "handed_over"),
            "d_transport": self._dur(an, "transporting", "returning"),
            "d_place": self._dur(an, "returning", "placed_home"),
            "human_outcome": c.human_outcome,
            "human_failure_label": c.human_failure_label,
        }

    # --- human keypress-logger merge ---

    def _on_logger_event(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        row["_t_rel"] = self._rel()
        self._pending_human.append(row)
        self._log_event("/scrub_nurse_logger/event", "logger_event",
                        row.get("outcome", ""))

    def _merge_human(self, c: Cycle) -> None:
        if not self._pending_human:
            return
        t_end = self._rel()
        window = self.args.merge_window
        best, best_dt = None, window + 1
        for row in self._pending_human:
            dt = abs(row.get("_t_rel", 0.0) - t_end)
            if dt <= window and dt < best_dt:
                best, best_dt = row, dt
        if best is not None:
            c.human_outcome = best.get("outcome", "")
            c.human_failure_label = best.get("failure_label", "")
            self._pending_human.remove(best)

    # --- periodic ---

    def _on_timer(self) -> None:
        if self.active is not None and self.active.awaiting_retry \
                and self._rel() > self.active.retry_deadline:
            self._close(self.active, "failure", "dropped")
        if self.active is not None and self.args.cycle_timeout > 0 \
                and self._rel() - self.active.opened_at > self.args.cycle_timeout:
            self._close(self.active, "failure", "timeout")

    # --- shutdown / summary ---

    def finish(self) -> None:
        for fh in (self._events_fh, self._cycles_fh):
            try:
                if not fh.closed:
                    fh.flush()
                    fh.close()
            except Exception:
                pass
        summary = self._build_summary()
        (self.session_dir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        text = self._format_summary(summary)
        (self.session_dir / "summary.txt").write_text(text, encoding="utf-8")
        print("\n" + text)
        print(f"\nSaved: {self.session_dir}")
        print(f"Analyse with: python3 ros_unrelated_scripts/analyze_eval.py {self.session_dir}")

    def _build_summary(self) -> dict:
        out = {"session_id": self.session_id, "generated": now_iso(),
               "total_cycles": len(self.cycle_rows), "by_type": {}}
        duration_keys = ["d_total", "d_command_to_pick", "d_pick", "d_gesture_wait",
                         "d_move_to_hand", "d_present_to_taken", "d_transport", "d_place"]
        for ctype in ("handover", "reclaim", "unknown"):
            rows = [r for r in self.cycle_rows if r["type"] == ctype]
            if not rows:
                continue
            succ = [r for r in rows if r["outcome"] == "success"]
            fails = [r for r in rows if r["outcome"] == "failure"]
            entry = {
                "n": len(rows), "success": len(succ), "failure": len(fails),
                "success_rate": round(len(succ) / len(rows), 4),
                "failure_reasons": dict(Counter(r["failure_reason"] for r in fails)),
                "by_tool_class": {}, "durations": {},
                "total_retries": sum(int(r["attempts"]) - 1 for r in rows),
                "total_drops": sum(int(r["n_dropped"]) for r in rows),
            }
            for key in duration_keys:
                vals = [float(r[key]) for r in succ if r[key] not in ("", None)]
                if vals:
                    entry["durations"][key] = _stats(vals)
            for tool in sorted({r["tool_class"] for r in rows if r["tool_class"]}):
                trows = [r for r in rows if r["tool_class"] == tool]
                ts = [r for r in trows if r["outcome"] == "success"]
                entry["by_tool_class"][tool] = {
                    "n": len(trows), "success": len(ts),
                    "success_rate": round(len(ts) / len(trows), 4),
                }
            out["by_type"][ctype] = entry
        return out

    @staticmethod
    def _format_summary(s: dict) -> str:
        L = ["=" * 64, f"EVALUATION SUMMARY  ({s['session_id']})", "=" * 64,
             f"Total cycles: {s['total_cycles']}"]
        for ctype, e in s["by_type"].items():
            L += ["", f"--- {ctype.upper()} ---",
                  f"  cycles: {e['n']}   success: {e['success']}   "
                  f"failure: {e['failure']}   success rate: {e['success_rate'] * 100:.1f}%",
                  f"  retries: {e['total_retries']}   drops: {e['total_drops']}"]
            if e["failure_reasons"]:
                L.append("  failure reasons: " + ", ".join(
                    f"{k}={v}" for k, v in e["failure_reasons"].items()))
            if e["durations"]:
                L.append("  durations [s] (successful cycles): "
                         "n / mean / median / std / min / max / p95")
                for key, d in e["durations"].items():
                    L.append(f"    {key:<20} {d['n']:>3} / {d['mean']:.2f} / "
                             f"{d['median']:.2f} / {d['std']:.2f} / {d['min']:.2f} / "
                             f"{d['max']:.2f} / {d['p95']:.2f}")
            if e["by_tool_class"]:
                L.append("  per tool class:")
                for tool, t in e["by_tool_class"].items():
                    L.append(f"    {tool:<18} n={t['n']:>3}  "
                             f"success={t['success']:>3}  rate={t['success_rate'] * 100:.1f}%")
        L.append("=" * 64)
        return "\n".join(L)


def _percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = p * (len(sorted_vals) - 1)
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return sorted_vals[int(idx)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (idx - lo)


def _stats(vals):
    sv = sorted(vals)
    return {
        "n": len(sv), "mean": round(statistics.fmean(sv), 3),
        "median": round(statistics.median(sv), 3),
        "std": round(statistics.pstdev(sv), 3) if len(sv) > 1 else 0.0,
        "min": round(sv[0], 3), "max": round(sv[-1], 3),
        "p95": round(_percentile(sv, 0.95), 3),
    }


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", type=int, default=0,
                   help="Auto-finish after this many completed cycles (0 = run until Ctrl-C).")
    p.add_argument("--session-name", default="eval",
                   help="Label folded into the session folder name (default: eval).")
    p.add_argument("--data-dir", default="~/scrub_nurse_eval",
                   help="Root output directory (default: ~/scrub_nurse_eval).")
    p.add_argument("--retry-window", type=float, default=12.0,
                   help="Seconds after a DROPPED to still count a re-pick as the "
                        "same cycle (default: 12).")
    p.add_argument("--cycle-timeout", type=float, default=180.0,
                   help="Fail an open cycle after this many seconds (0 = never). "
                        "Kept high because a handover waits for the surgeon (default: 180).")
    p.add_argument("--merge-window", type=float, default=15.0,
                   help="Seconds window to fold a /scrub_nurse_logger keypress into "
                        "a cycle (default: 15).")
    p.add_argument("--no-merge-logger", action="store_true",
                   help="Do not subscribe to /scrub_nurse_logger/event.")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = EvalNode(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
