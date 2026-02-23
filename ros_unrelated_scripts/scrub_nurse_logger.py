#!/usr/bin/env python3
"""Keyboard-based event logger for robotic scrub nurse handover tests."""

from __future__ import annotations

import argparse
import csv
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


def parse_args() -> argparse.Namespace:
    default_data_dir = Path(__file__).resolve().parent / "data"
    parser = argparse.ArgumentParser(
        description="Log scrub nurse handover outcomes via keyboard."
    )
    parser.add_argument(
        "--data-dir",
        default=str(default_data_dir),
        help="Directory where session CSV files are stored (default: script_dir/data)",
    )
    return parser.parse_args()


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


def build_session(participant_name: str, session_number: int, data_dir: Path) -> SessionContext:
    date_stamp = datetime.now().strftime("%Y%m%d")
    participant_slug = slugify(participant_name)
    base_filename = f"{date_stamp}_{participant_slug}_s{session_number}"
    file_path = build_unique_file_path(data_dir, base_filename)
    session_id = file_path.stem
    return SessionContext(
        participant_name=participant_name,
        session_number=session_number,
        session_id=session_id,
        file_path=file_path,
    )


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


def render_help_and_status(
    counters: Counter,
    session: SessionContext,
    message: str = "",
    pending_failure_code: str = "",
    pending_success: bool = False,
) -> None:
    clear_screen()
    print(color("Scrub Nurse Handover Logger", "title"))
    print(f"Session ID: {session.session_id}")
    print(f"Participant: {session.participant_name}")
    print(f"Session number: {session.session_number}")
    print(f"CSV: {session.file_path}")
    print("-" * 70)
    print("Keys:")
    print(f"  {color('0', 'success')} = {color('Success', 'success')}")
    print(f"  {color('1', failure_color('1'))} = {color('Gesture detection failure', failure_color('1'))}")
    print(f"  {color('2', failure_color('2'))} = {color('Premature release', failure_color('2'))}")
    print(f"  {color('3', failure_color('3'))} = {color('Non-release', failure_color('3'))}")
    print(f"  {color('4', failure_color('4'))} = {color('Collision', failure_color('4'))}")
    print(f"  {color('5', failure_color('5'))} = {color('Other anomaly', failure_color('5'))}")
    print(f"  {color('6', failure_color('6'))} = {color('Timeout/No response', failure_color('6'))}")
    print(f"  {color('q', 'neutral')} = Quit + Save")
    if pending_failure_code or pending_success:
        pending_label = (
            color("success (0)", "success")
            if pending_success
            else color(f"failure {pending_failure_code}", failure_color(pending_failure_code))
        )
        print(
            f"  {color('enter', 'neutral')} = Confirm {pending_label}"
            f" | {color('c', 'neutral')} = Cancel"
        )
    print("-" * 70)
    print(
        "Counts: "
        f"{color('success', 'success')}={color(str(counters['success']), 'success')}, "
        f"{color('gesture_fail', failure_color('1'))}={color(str(counters['1']), failure_color('1'))}, "
        f"{color('premature_release', failure_color('2'))}={color(str(counters['2']), failure_color('2'))}, "
        f"{color('non_release', failure_color('3'))}={color(str(counters['3']), failure_color('3'))}, "
        f"{color('collision', failure_color('4'))}={color(str(counters['4']), failure_color('4'))}, "
        f"{color('other', failure_color('5'))}={color(str(counters['5']), failure_color('5'))}, "
        f"{color('timeout_no_response', failure_color('6'))}={color(str(counters['6']), failure_color('6'))}, "
        f"{color('failures_total', 'neutral')}={color(str(counters['failure_total']), 'neutral')}"
    )
    if message:
        print(f"Note: {color(message, 'note')}")
    print("\nPress keys to log events...")


def log_event(
    writer: csv.DictWriter,
    event_index: int,
    session: SessionContext,
    outcome: str,
    failure_code: str = "",
    failure_label: str = "",
    failure_comment: str = "",
) -> None:
    writer.writerow(
        {
            "timestamp_iso": datetime.now(timezone.utc).isoformat(),
            "session_id": session.session_id,
            "participant_name": session.participant_name,
            "session_number": session.session_number,
            "event_index": event_index,
            "outcome": outcome,
            "failure_code": failure_code,
            "failure_label": failure_label,
            "failure_comment": failure_comment,
        }
    )


def print_summary(counters: Counter, total_events: int) -> None:
    print("\nSession summary")
    print("-" * 30)
    print(f"Total events: {total_events}")
    print(f"Success: {counters['success']}")
    print(f"Failures total: {counters['failure_total']}")
    print(f"  1 Gesture detection failure: {counters['1']}")
    print(f"  2 Premature release: {counters['2']}")
    print(f"  3 Non-release: {counters['3']}")
    print(f"  4 Collision: {counters['4']}")
    print(f"  5 Other anomaly: {counters['5']}")
    print(f"  6 Timeout/No response: {counters['6']}")


def ensure_tty() -> None:
    if not sys.stdin.isatty():
        print("Error: stdin is not a TTY. Run this script in a terminal.", file=sys.stderr)
        raise SystemExit(1)


def main() -> int:
    args = parse_args()
    ensure_tty()

    data_dir = Path(args.data_dir).expanduser().resolve()
    data_dir.mkdir(parents=True, exist_ok=True)

    participant_name = prompt_participant_name()
    session_number = prompt_session_number()
    session = build_session(participant_name, session_number, data_dir)

    fieldnames = [
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

    counters: Counter = Counter()
    event_index = 0
    last_message = ""
    pending_failure_code = ""
    pending_success = False

    with session.file_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        render_help_and_status(counters, session)

        try:
            with RawKeyReader() as reader:
                while True:
                    key = reader.read_key()
                    if not key:
                        continue

                    key = key.lower()

                    if key == "q":
                        last_message = "Quit requested. Session saved."
                        break

                    if pending_failure_code or pending_success:
                        if key in ("\n", "\r"):
                            if pending_success:
                                event_index += 1
                                counters["success"] += 1
                                log_event(writer, event_index, session, outcome="success")
                                csv_file.flush()
                                last_message = "Logged success"
                            else:
                                failure_comment = reader.prompt_line("Failure comment (optional): ").strip()
                                event_index += 1
                                counters["failure_total"] += 1
                                counters[pending_failure_code] += 1
                                log_event(
                                    writer,
                                    event_index,
                                    session,
                                    outcome="failure",
                                    failure_code=pending_failure_code,
                                    failure_label=FAILURE_MAP[pending_failure_code],
                                    failure_comment=failure_comment,
                                )
                                csv_file.flush()
                                if failure_comment:
                                    last_message = (
                                        f"Logged failure: {pending_failure_code} "
                                        f"({FAILURE_MAP[pending_failure_code]}) | comment saved"
                                    )
                                else:
                                    last_message = (
                                        f"Logged failure: {pending_failure_code} "
                                        f"({FAILURE_MAP[pending_failure_code]})"
                                    )
                            pending_failure_code = ""
                            pending_success = False
                        elif key == "c":
                            pending_failure_code = ""
                            pending_success = False
                            last_message = "Selection canceled."
                        else:
                            last_message = "Press Enter to confirm selection or c to cancel."
                        render_help_and_status(
                            counters,
                            session,
                            last_message,
                            pending_failure_code,
                            pending_success,
                        )
                        continue

                    if key == "0":
                        pending_success = True
                        last_message = "Success selected. Press Enter to confirm."
                    elif key in FAILURE_MAP:
                        pending_failure_code = key
                        pending_success = False
                        last_message = (
                            f"Failure selected: {key} ({FAILURE_MAP[key]}). "
                            "Press Enter to confirm."
                        )
                    else:
                        known_keys = ", ".join(KEYMAP.keys())
                        last_message = f"Invalid key '{key}'. Valid keys: {known_keys}"

                    render_help_and_status(
                        counters,
                        session,
                        last_message,
                        pending_failure_code,
                        pending_success,
                    )
        except KeyboardInterrupt:
            print("\nInterrupted by user. Saving and exiting...")

    print_summary(counters, event_index)
    print(f"\nSaved CSV: {session.file_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
