#!/usr/bin/env python3
"""Terminal-based ROS2 logger for scrub nurse handover experiments on Ubuntu."""

from __future__ import annotations

import csv
import json
import os
import select
import sys
import termios
import tty
import unicodedata
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FAILURE_MAP = {
    "1": "Gesture detection failure",
    "2": "Premature release",
    "3": "Non-release",
    "4": "Collision",
    "5": "Other anomaly",
    "6": "Timeout/No response",
}

KEYMAP = {
    "0": "Success",
    "1": FAILURE_MAP["1"],
    "2": FAILURE_MAP["2"],
    "3": FAILURE_MAP["3"],
    "4": FAILURE_MAP["4"],
    "5": FAILURE_MAP["5"],
    "6": FAILURE_MAP["6"],
    "q": "Quit + Save",
}

USE_COLOR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None
RESET = "\033[0m"
COLORS = {
    "title": "\033[1;36m",
    "success": "\033[1;32m",
    "failure_1": "\033[1;31m",
    "failure_2": "\033[1;34m",
    "failure_3": "\033[1;35m",
    "failure_4": "\033[1;33m",
    "failure_5": "\033[1;96m",
    "failure_6": "\033[1;90m",
    "neutral": "\033[1;37m",
    "note": "\033[1;37m",
}


def color(text: str, name: str) -> str:
    if not USE_COLOR:
        return text
    return f"{COLORS[name]}{text}{RESET}"


def failure_color(code: str) -> str:
    return f"failure_{code}"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def slugify(value: str) -> str:
    normalized = unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii")
    cleaned = "".join(ch.lower() if ch.isalnum() else "_" for ch in normalized).strip("_")
    return cleaned or "participant"


def build_unique_file_path(data_dir: Path, base_filename: str) -> Path:
    candidate = data_dir / f"{base_filename}.csv"
    if not candidate.exists():
        return candidate

    version = 2
    while True:
        candidate = data_dir / f"{base_filename}_v{version}.csv"
        if not candidate.exists():
            return candidate
        version += 1


def prompt_participant_name() -> str:
    while True:
        name = input("Participant name: ").strip()
        if name:
            return name
        print("Participant name is required.")


def prompt_session_number() -> int:
    while True:
        raw_value = input("Session number: ").strip()
        if not raw_value.isdigit():
            print("Session number must be a positive integer.")
            continue
        value = int(raw_value)
        if value <= 0:
            print("Session number must be greater than 0.")
            continue
        return value


def clear_screen() -> None:
    print("\033[2J\033[H", end="")


@dataclass
class SessionContext:
    participant_name: str
    session_number: int
    session_id: str
    file_path: Path


class RawKeyReader:
    """Context manager for reading single keypresses without Enter."""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old_settings = None

    def __enter__(self) -> "RawKeyReader":
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, exc_tb) -> None:
        if self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_key(self) -> str:
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if readable:
                return sys.stdin.read(1)

    def prompt_line(self, prompt: str) -> str:
        if self._old_settings is None:
            return input(prompt)
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        try:
            return input(prompt)
        finally:
            tty.setcbreak(self._fd)


class ScrubNurseLogger(Node):
    def __init__(self) -> None:
        super().__init__("scrub_nurse_logger")

        default_data_dir = Path.home() / "scrub_nurse_logs"
        self.data_dir = Path(
            str(self.declare_parameter("data_dir", str(default_data_dir)).value)
        ).expanduser()

        self.csv_row_publisher = self.create_publisher(String, "/scrub_nurse_logger/event", 10)

        self.fieldnames = [
            "timestamp_iso",
            "session_id",
            "participant_name",
            "session_number",
            "event_index",
            "outcome",
            "failure_code",
            "failure_label",
            "failure_comment",
        ]

        self.session: SessionContext | None = None
        self.csv_file = None
        self.writer: csv.DictWriter | None = None
        self.counters: Counter = Counter()
        self.event_index = 0
        self.last_message = ""
        self.pending_failure_code = ""
        self.pending_success = False

    def start_session(self) -> None:
        self.data_dir.mkdir(parents=True, exist_ok=True)

        participant_name = prompt_participant_name()
        session_number = prompt_session_number()
        self.session = self._build_session(participant_name, session_number)

        self.csv_file = self.session.file_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.get_logger().info(
            f"Session started: {self.session.session_id} ({self.session.file_path})"
        )

    def run(self) -> None:
        self.start_session()
        self._render_help_and_status()

        try:
            with RawKeyReader() as reader:
                while rclpy.ok():
                    key = reader.read_key()
                    if not key:
                        continue

                    self._handle_key(reader, key.lower())
                    if key.lower() == "q":
                        break
        except KeyboardInterrupt:
            self.last_message = "Interrupted by user. Session saved."
            self.get_logger().info(self.last_message)
        finally:
            self.finish_session()

    def finish_session(self) -> None:
        if self.session is None:
            return

        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

        self._print_summary()
        print(f"\nSaved CSV: {self.session.file_path}")
        self.get_logger().info(f"Saved CSV: {self.session.file_path}")

    def _build_session(self, participant_name: str, session_number: int) -> SessionContext:
        date_stamp = datetime.now().strftime("%Y%m%d")
        participant_slug = slugify(participant_name)
        base_filename = f"{date_stamp}_{participant_slug}_s{session_number}"
        file_path = build_unique_file_path(self.data_dir, base_filename)
        return SessionContext(
            participant_name=participant_name,
            session_number=session_number,
            session_id=file_path.stem,
            file_path=file_path.resolve(),
        )

    def _handle_key(self, reader: RawKeyReader, key: str) -> None:
        if key == "q":
            self.last_message = "Quit requested. Session saved."
            self._render_help_and_status()
            return

        if self.pending_failure_code or self.pending_success:
            self._handle_pending_confirmation(reader, key)
            return

        if key == "0":
            self.pending_success = True
            self.last_message = "Success selected. Press Enter to confirm."
        elif key in FAILURE_MAP:
            self.pending_failure_code = key
            self.last_message = (
                f"Failure selected: {key} ({FAILURE_MAP[key]}). Press Enter to confirm."
            )
        else:
            known_keys = ", ".join(KEYMAP.keys())
            self.last_message = f"Invalid key '{key}'. Valid keys: {known_keys}"

        self._render_help_and_status()

    def _handle_pending_confirmation(self, reader: RawKeyReader, key: str) -> None:
        if key in ("\n", "\r"):
            if self.pending_success:
                self._record_success()
            else:
                failure_code = self.pending_failure_code
                failure_comment = reader.prompt_line("Failure comment (optional): ").strip()
                self._record_failure(failure_code, failure_comment)
            self.pending_failure_code = ""
            self.pending_success = False
        elif key == "c":
            self.pending_failure_code = ""
            self.pending_success = False
            self.last_message = "Selection canceled."
        else:
            self.last_message = "Press Enter to confirm selection or c to cancel."

        self._render_help_and_status()

    def _record_success(self) -> None:
        self.event_index += 1
        self.counters["success"] += 1

        row = {
            "timestamp_iso": now_iso(),
            "session_id": self.session.session_id,
            "participant_name": self.session.participant_name,
            "session_number": self.session.session_number,
            "event_index": self.event_index,
            "outcome": "success",
            "failure_code": "",
            "failure_label": "",
            "failure_comment": "",
        }

        self._write_row(row)
        self.last_message = "Logged success"
        self.get_logger().info(f"Logged success event {self.event_index}.")

    def _record_failure(self, failure_code: str, failure_comment: str) -> None:
        self.event_index += 1
        self.counters["failure_total"] += 1
        self.counters[failure_code] += 1

        row = {
            "timestamp_iso": now_iso(),
            "session_id": self.session.session_id,
            "participant_name": self.session.participant_name,
            "session_number": self.session.session_number,
            "event_index": self.event_index,
            "outcome": "failure",
            "failure_code": failure_code,
            "failure_label": FAILURE_MAP[failure_code],
            "failure_comment": failure_comment,
        }

        self._write_row(row)

        if failure_comment:
            self.last_message = (
                f"Logged failure: {failure_code} ({FAILURE_MAP[failure_code]}) | comment saved"
            )
        else:
            self.last_message = f"Logged failure: {failure_code} ({FAILURE_MAP[failure_code]})"

        self.get_logger().info(
            f"Logged failure event {self.event_index}: "
            f"{failure_code} ({FAILURE_MAP[failure_code]})."
        )

    def _write_row(self, row: dict[str, object]) -> None:
        if self.writer is None or self.csv_file is None:
            raise RuntimeError("Session CSV writer is not initialized.")

        self.writer.writerow(row)
        self.csv_file.flush()
        self._publish_csv_row(row)

    def _publish_csv_row(self, row: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(row, ensure_ascii=False)
        self.csv_row_publisher.publish(msg)

    def _render_help_and_status(self) -> None:
        clear_screen()
        print(color("Scrub Nurse Handover Logger (ROS2)", "title"))
        print(f"Session ID: {self.session.session_id}")
        print(f"Participant: {self.session.participant_name}")
        print(f"Session number: {self.session.session_number}")
        print(f"CSV: {self.session.file_path}")
        print("-" * 70)
        print("Keys:")
        print(f"  {color('0', 'success')} = {color('Success', 'success')}")
        print(
            f"  {color('1', failure_color('1'))} = "
            f"{color('Gesture detection failure', failure_color('1'))}"
        )
        print(
            f"  {color('2', failure_color('2'))} = "
            f"{color('Premature release', failure_color('2'))}"
        )
        print(
            f"  {color('3', failure_color('3'))} = "
            f"{color('Non-release', failure_color('3'))}"
        )
        print(f"  {color('4', failure_color('4'))} = {color('Collision', failure_color('4'))}")
        print(
            f"  {color('5', failure_color('5'))} = "
            f"{color('Other anomaly', failure_color('5'))}"
        )
        print(
            f"  {color('6', failure_color('6'))} = "
            f"{color('Timeout/No response', failure_color('6'))}"
        )
        print(f"  {color('q', 'neutral')} = Quit + Save")
        if self.pending_failure_code or self.pending_success:
            pending_label = (
                color("success (0)", "success")
                if self.pending_success
                else color(
                    f"failure {self.pending_failure_code}",
                    failure_color(self.pending_failure_code),
                )
            )
            print(
                f"  {color('enter', 'neutral')} = Confirm {pending_label} "
                f"| {color('c', 'neutral')} = Cancel"
            )
        print("-" * 70)
        print(
            "Counts: "
            f"{color('success', 'success')}={color(str(self.counters['success']), 'success')}, "
            f"{color('gesture_fail', failure_color('1'))}="
            f"{color(str(self.counters['1']), failure_color('1'))}, "
            f"{color('premature_release', failure_color('2'))}="
            f"{color(str(self.counters['2']), failure_color('2'))}, "
            f"{color('non_release', failure_color('3'))}="
            f"{color(str(self.counters['3']), failure_color('3'))}, "
            f"{color('collision', failure_color('4'))}="
            f"{color(str(self.counters['4']), failure_color('4'))}, "
            f"{color('other', failure_color('5'))}="
            f"{color(str(self.counters['5']), failure_color('5'))}, "
            f"{color('timeout_no_response', failure_color('6'))}="
            f"{color(str(self.counters['6']), failure_color('6'))}, "
            f"{color('failures_total', 'neutral')}="
            f"{color(str(self.counters['failure_total']), 'neutral')}"
        )
        if self.last_message:
            print(f"Note: {color(self.last_message, 'note')}")
        print("\nPublishing topic:")
        print("  /scrub_nurse_logger/event")
        print("\nPress keys to log events...")

    def _print_summary(self) -> None:
        print("\nSession summary")
        print("-" * 30)
        print(f"Total events: {self.event_index}")
        print(f"Success: {self.counters['success']}")
        print(f"Failures total: {self.counters['failure_total']}")
        print(f"  1 Gesture detection failure: {self.counters['1']}")
        print(f"  2 Premature release: {self.counters['2']}")
        print(f"  3 Non-release: {self.counters['3']}")
        print(f"  4 Collision: {self.counters['4']}")
        print(f"  5 Other anomaly: {self.counters['5']}")
        print(f"  6 Timeout/No response: {self.counters['6']}")


def ensure_tty() -> None:
    if sys.stdin.isatty():
        return

    print("Error: stdin is not a TTY. Run this node in a terminal.", file=sys.stderr)
    raise SystemExit(1)


def main(args=None) -> int:
    ensure_tty()
    rclpy.init(args=args)
    node = ScrubNurseLogger()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
