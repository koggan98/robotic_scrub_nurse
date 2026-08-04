#!/usr/bin/env python3
"""Standalone transcription-latency logger.

Subscribes to `/asr_timing` (published by `asr_node` for every accepted command)
and records the PURE transcription latency per utterance:

    transcription_latency = t_publish − t_speech_end

where `t_speech_end` is the moment the VAD closed the segment (utterance end) and
`t_publish` is when the transcript was put on `/user_speech`. Both timestamps come
from `asr_node`'s own monotonic clock, so the latency is exact regardless of where
this logger runs or of DDS transport time.

This is the pre-/user_speech gap that `eval_handover_reclaim.py` does not cover.

The terminal shows a running COUNT of logged commands plus a running mean, so you
can decide when you have enough and stop with Ctrl-C.

Run it (sourced ROS 2 env, system running):

    python3 ros_unrelated_scripts/asr_latency_logger.py --session pilot

Then say "Alexa, scissors", "Alexa, return the hammer please", … repeatedly.
Each accepted command appends one CSV row. Ctrl-C prints a summary.

    (transcription only — not planning/motion; combine with eval_handover_reclaim
     for the full spoken-command → handover picture.)
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FIELDS = ["index", "timestamp_iso", "transcript", "audio_s",
          "transcription_latency_s", "utterance_to_publish_s"]


def _percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q
    lo = int(pos)
    frac = pos - lo
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * frac


class AsrLatencyLogger(Node):
    def __init__(self, csv_path: Path):
        super().__init__("asr_latency_logger")
        self._csv_path = csv_path
        self._index = 0
        self._lat = []            # transcription_latency_s values
        self._got_any = False
        self._warned = False
        self._start = time.monotonic()

        self._file = csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._file.flush()

        self.create_subscription(String, "/asr_timing", self._on_timing, 10)
        self.create_timer(4.0, self._heartbeat)

        print(f"Transcription-latency logger — writing {csv_path}")
        print("Subscribed: /asr_timing")
        print('Say "Alexa, <command>" repeatedly. Ctrl-C to finish.\n')

    def _heartbeat(self, *_):
        if not self._got_any and not self._warned:
            self._warned = True
            print("… waiting for /asr_timing — is the UPDATED asr_node running? "
                  "(try: ros2 topic echo /asr_timing)")

    def _on_timing(self, msg):
        try:
            d = json.loads(msg.data)
            t_end = float(d["t_speech_end"])
            t_pub = float(d["t_publish"])
            t_start = float(d["t_speech_start"])
        except Exception:
            self.get_logger().warn(f"ignored bad /asr_timing payload: {msg.data!r}")
            return

        self._got_any = True
        self._index += 1
        text = str(d.get("text", ""))
        audio_s = float(d.get("audio_s", 0.0))
        lat = round(t_pub - t_end, 3)          # pure transcription latency
        utt = round(t_pub - t_start, 3)        # whole utterance-end-to-publish incl. speaking
        self._lat.append(lat)

        self._writer.writerow({
            "index": self._index,
            "timestamp_iso": datetime.now().isoformat(timespec="seconds"),
            "transcript": text,
            "audio_s": round(audio_s, 3),
            "transcription_latency_s": lat,
            "utterance_to_publish_s": utt,
        })
        self._file.flush()

        mean = statistics.mean(self._lat)
        med = statistics.median(self._lat)
        print(f"#{self._index:<3d} latency={lat:6.3f} s  audio={audio_s:4.1f} s  "
              f'"{text}"   [ {self._index} logged | mean {mean:.3f} | '
              f"median {med:.3f} s ]")

    def finish(self):
        if not self._file.closed:
            self._file.flush()
            self._file.close()
        d = self._lat
        print("\n" + "-" * 56)
        print(f"Commands logged           : {self._index}")
        if d:
            print(f"transcription_latency [s] : mean={statistics.mean(d):.3f}  "
                  f"median={statistics.median(d):.3f}  "
                  f"std={statistics.pstdev(d):.3f}")
            print(f"                            min={min(d):.3f}  max={max(d):.3f}  "
                  f"p95={_percentile(d, 0.95):.3f}")
        print(f"CSV: {self._csv_path}")


def main(args=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", default="asr", help="label in the output filename")
    ap.add_argument("--out", default=None, help="explicit CSV path (overrides default)")
    parsed = ap.parse_args()

    if parsed.out:
        csv_path = Path(parsed.out).expanduser()
    else:
        data_dir = Path.home() / "asr_latency_logs"
        data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = data_dir / f"{stamp}_{parsed.session}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=args)
    node = AsrLatencyLogger(csv_path)
    try:
        rclpy.spin(node)
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
