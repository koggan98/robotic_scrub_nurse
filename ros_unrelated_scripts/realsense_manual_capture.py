#!/usr/bin/env python3
"""Lightweight manual RealSense frame capture for SSH sessions.

Usage:
    python3 ros_unrelated_scripts/realsense_manual_capture.py
    python3 ros_unrelated_scripts/realsense_manual_capture.py --preview
    python3 ros_unrelated_scripts/realsense_manual_capture.py --output-dir ~/datasets/tool_detector/custom_session
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    from PIL import Image
except ImportError:
    Image = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


WINDOW_NAME = "RealSense Manual Capture"


class TerminalKeyReader:
    """Read single-key input from an interactive POSIX terminal."""

    def __init__(self) -> None:
        self.fd: Optional[int] = None
        self.original_settings = None

    def __enter__(self) -> "TerminalKeyReader":
        if not sys.stdin.isatty():
            raise RuntimeError(
                "This script requires an interactive terminal (TTY) so it can read single key presses."
            )

        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # Disable local echo while keeping signal handling such as Ctrl+C intact.
        updated_settings = termios.tcgetattr(self.fd)
        updated_settings[3] &= ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSADRAIN, updated_settings)
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.fd is not None and self.original_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def read_key(self) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        return sys.stdin.read(1)


def build_default_output_dir() -> Path:
    now = datetime.now()
    day_folder = now.strftime("%Y-%m-%d")
    session_folder = now.strftime("%Y%m%d_%H%M%S")
    return Path.home() / "datasets" / "tool_detector" / day_folder / session_folder


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture single RealSense RGB frames from the terminal without ROS."
    )
    parser.add_argument(
        "--output-dir",
        default=str(build_default_output_dir()),
        help="Directory that will receive the captured PNG frames.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Color stream width in pixels (default: 640).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Color stream height in pixels (default: 480).",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Color stream frame rate (default: 30).",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a local OpenCV preview window in addition to terminal controls.",
    )
    args = parser.parse_args()

    if args.width <= 0:
        parser.error("--width must be greater than 0")
    if args.height <= 0:
        parser.error("--height must be greater than 0")
    if args.fps <= 0:
        parser.error("--fps must be greater than 0")

    args.output_dir = Path(args.output_dir).expanduser()
    return args


def check_dependencies(preview_enabled: bool) -> int:
    missing = []
    if np is None:
        missing.append("numpy")
    if rs is None:
        missing.append("pyrealsense2")
    if cv2 is None and Image is None:
        missing.append("opencv-python or pillow")
    if preview_enabled and cv2 is None:
        missing.append("opencv-python (required for --preview)")

    if missing:
        print("ERROR: Missing required Python package(s): " + ", ".join(missing))
        print(
            "Install example: python3 -m pip install --user pyrealsense2 numpy opencv-python pillow"
        )
        return 2
    return 0


def create_pipeline(
    width: int,
    height: int,
    fps: int,
) -> tuple[Optional["rs.pipeline"], Optional["rs.pipeline_profile"], int]:
    try:
        context = rs.context()
        device_count = len(context.devices)
    except Exception as exc:
        print(f"ERROR: Failed to query RealSense devices: {exc}")
        return None, None, 1

    if device_count == 0:
        print("ERROR: No Intel RealSense camera detected.")
        return None, None, 1

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    try:
        profile = pipeline.start(config)
    except Exception as exc:
        print(f"ERROR: Failed to start RealSense pipeline: {exc}")
        return None, None, 1

    return pipeline, profile, 0


def get_device_name(profile: Optional["rs.pipeline_profile"]) -> str:
    if profile is None:
        return "Unknown RealSense"
    try:
        return str(profile.get_device().get_info(rs.camera_info.name))
    except Exception:
        return "Unknown RealSense"


def print_controls() -> None:
    print("Controls:")
    print("  1 -> save the current RGB frame")
    print("  h -> print this help again")
    print("  q -> quit")


def save_png(file_path: Path, image_bgr: "np.ndarray") -> bool:
    if cv2 is not None:
        return bool(cv2.imwrite(str(file_path), image_bgr))

    if Image is not None:
        try:
            rgb_image = image_bgr[:, :, ::-1]
            Image.fromarray(rgb_image).save(file_path)
            return True
        except Exception:
            return False

    return False


def build_preview_frame(image_bgr: "np.ndarray", saved_count: int) -> "np.ndarray":
    preview = image_bgr.copy()
    if cv2 is None:
        return preview

    cv2.putText(
        preview,
        "1: save  h: help  q: quit",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        preview,
        f"Saved: {saved_count}",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return preview


def main() -> int:
    args = parse_args()

    dependency_status = check_dependencies(args.preview)
    if dependency_status != 0:
        return dependency_status

    pipeline, profile, pipeline_status = create_pipeline(args.width, args.height, args.fps)
    if pipeline_status != 0:
        return pipeline_status

    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.preview:
        try:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        except Exception as exc:
            print(f"ERROR: Failed to open preview window: {exc}")
            print("Tip: run without --preview when you are connected over plain SSH.")
            try:
                pipeline.stop()
            except Exception:
                pass
            return 1

    saved_count = 0
    latest_frame = None
    device_name = get_device_name(profile)

    print(f"Using camera: {device_name}")
    print(f"Output directory: {args.output_dir}")
    print(f"Color stream: {args.width}x{args.height} @ {args.fps} FPS")
    if args.preview:
        print(f"Preview window: {WINDOW_NAME}")
    else:
        print("Preview window: disabled")
    print_controls()

    try:
        with TerminalKeyReader() as active_key_reader:
            while True:
                try:
                    frames = pipeline.wait_for_frames(1000)
                except Exception as exc:
                    print(f"ERROR: Failed while reading RealSense frames: {exc}")
                    return 1

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                latest_frame = np.asanyarray(color_frame.get_data()).copy()

                key = active_key_reader.read_key()
                if key == "1":
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    file_path = args.output_dir / f"frame_{timestamp}_{saved_count + 1:06d}.png"
                    if not save_png(file_path, latest_frame):
                        print(f"ERROR: Failed to write frame to {file_path}")
                        return 1
                    saved_count += 1
                    print(f"Saved frame {saved_count}: {file_path}")
                elif key == "h":
                    print_controls()
                elif key == "q":
                    print("Quit requested.")
                    break

                if args.preview and cv2 is not None:
                    preview_frame = build_preview_frame(latest_frame, saved_count)
                    cv2.imshow(WINDOW_NAME, preview_frame)
                    cv2.waitKey(1)
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if cv2 is not None:
            cv2.destroyAllWindows()

    print(f"Session finished. Saved {saved_count} frame(s) to {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
