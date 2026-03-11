#!/usr/bin/env python3
"""Minimal RealSense + YOLO11 live detection test.

Usage:
    python3 ros_unrelated_scripts/yolo11_realsense_test.py
    python3 ros_unrelated_scripts/yolo11_realsense_test.py --model /path/to/custom.pt
"""

from __future__ import annotations

import argparse
import sys
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
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


WINDOW_NAME = "YOLO11 RealSense Test"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a YOLO11 detection model on the connected RealSense color stream."
    )
    parser.add_argument(
        "--model",
        default="yolo11n.pt",
        help="Ultralytics model path or model name (default: yolo11n.pt)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Detection confidence threshold (default: 0.25)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size (default: 640)",
    )
    return parser.parse_args()


def check_dependencies() -> int:
    missing = []
    if cv2 is None:
        missing.append("opencv-python")
    if np is None:
        missing.append("numpy")
    if rs is None:
        missing.append("pyrealsense2")
    if YOLO is None:
        missing.append("ultralytics")

    if missing:
        print("ERROR: Missing required Python package(s): " + ", ".join(missing))
        print(
            "Install example: python3 -m pip install --user ultralytics pyrealsense2 opencv-python"
        )
        return 2
    return 0


def create_pipeline() -> tuple[Optional["rs.pipeline"], Optional["rs.pipeline_profile"], int]:
    context = rs.context()
    if len(context.devices) == 0:
        print("ERROR: No Intel RealSense camera detected.")
        return None, None, 1

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        profile = pipeline.start(config)
    except Exception as exc:
        print(f"ERROR: Failed to start RealSense pipeline: {exc}")
        return None, None, 1

    return pipeline, profile, 0


def load_model(model_name: str) -> Optional["YOLO"]:
    try:
        return YOLO(model_name)
    except Exception as exc:
        print(f"ERROR: Failed to load YOLO model '{model_name}': {exc}")
        return None


def main() -> int:
    args = parse_args()

    dep_status = check_dependencies()
    if dep_status != 0:
        return dep_status

    model = load_model(args.model)
    if model is None:
        return 1

    pipeline, profile, pipeline_status = create_pipeline()
    if pipeline_status != 0:
        return pipeline_status

    device_name = "Unknown RealSense"
    if profile is not None:
        try:
            device_name = profile.get_device().get_info(rs.camera_info.name)
        except Exception:
            pass

    print(f"Using camera: {device_name}")
    print(f"Using model: {args.model}")
    print("Press 'q' in the image window to quit.")

    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        while True:
            try:
                frames = pipeline.wait_for_frames()
            except Exception as exc:
                print(f"ERROR: Failed while reading RealSense frames: {exc}")
                return 1

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            try:
                results = model.predict(color_image, conf=args.conf, imgsz=args.imgsz, verbose=False)
            except Exception as exc:
                print(f"ERROR: YOLO inference failed: {exc}")
                return 1

            annotated = results[0].plot() if results else color_image
            cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if cv2 is not None:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
