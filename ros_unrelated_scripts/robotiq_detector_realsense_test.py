#!/usr/bin/env python3
"""Standalone RealSense test for the custom Robotiq gripper YOLO detector.

Usage:
    python3 ros_unrelated_scripts/robotiq_detector_realsense_test.py
    python3 ros_unrelated_scripts/robotiq_detector_realsense_test.py --model /path/to/custom.pt
"""

from __future__ import annotations

import argparse
import sys
from collections import deque
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
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


WINDOW_NAME = "Robotiq Detector RealSense Test"
DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "files" / "robotiq_detector.pt"
TARGET_CLASS_NAME = "gripper"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Robotiq YOLO detector on the connected RealSense color stream."
    )
    parser.add_argument(
        "--model",
        default=str(DEFAULT_MODEL_PATH),
        help=f"Ultralytics model path or model name (default: {DEFAULT_MODEL_PATH})",
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
    parser.add_argument(
        "--history-size",
        type=int,
        default=60,
        help="Rolling history size in frames for the live detection rate (default: 60)",
    )
    args = parser.parse_args()
    if args.history_size <= 0:
        parser.error("--history-size must be greater than 0")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be between 0.0 and 1.0")
    if args.imgsz <= 0:
        parser.error("--imgsz must be greater than 0")
    return args


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


def filter_gripper_detections(result) -> list[dict[str, float]]:
    detections: list[dict[str, float]] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None or boxes.cls is None or boxes.conf is None or boxes.xyxy is None:
        return detections

    names = getattr(result, "names", {}) or {}
    class_ids = boxes.cls.tolist()
    confidences = boxes.conf.tolist()
    coordinates = boxes.xyxy.tolist()

    for class_id, confidence, xyxy in zip(class_ids, confidences, coordinates):
        label = str(names.get(int(class_id), int(class_id)))
        if label != TARGET_CLASS_NAME:
            continue
        detections.append(
            {
                "confidence": float(confidence),
                "x1": float(xyxy[0]),
                "y1": float(xyxy[1]),
                "x2": float(xyxy[2]),
                "y2": float(xyxy[3]),
            }
        )
    return detections


def draw_gripper_detections(
    image_bgr: "np.ndarray",
    detections: list[dict[str, float]],
) -> "np.ndarray":
    annotated = image_bgr.copy()
    for detection in detections:
        x1 = int(round(detection["x1"]))
        y1 = int(round(detection["y1"]))
        x2 = int(round(detection["x2"]))
        y2 = int(round(detection["y2"]))
        confidence = detection["confidence"]

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(
            annotated,
            f"{TARGET_CLASS_NAME}: {confidence:.2f}",
            (x1, max(24, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 220, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated


def print_summary(
    total_frames: int,
    detected_frames: int,
    history: deque[int],
    last_detected_conf: float,
    best_conf_seen: float,
) -> None:
    detection_rate = (detected_frames / total_frames * 100.0) if total_frames else 0.0
    rolling_rate = (sum(history) / len(history) * 100.0) if history else 0.0
    print("\nDetection summary:")
    print(f"  Total frames: {total_frames}")
    print(f"  Frames with gripper detection: {detected_frames}")
    print(f"  Overall detection rate: {detection_rate:.1f}%")
    print(f"  Final rolling detection rate: {rolling_rate:.1f}%")
    print(
        "  Last detected confidence: "
        + (f"{last_detected_conf:.2f}" if last_detected_conf >= 0.0 else "no detections")
    )
    print(
        "  Best confidence seen: "
        + (f"{best_conf_seen:.2f}" if best_conf_seen >= 0.0 else "no detections")
    )


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

    total_frames = 0
    detected_frames = 0
    detection_history: deque[int] = deque(maxlen=args.history_size)
    last_detected_conf = -1.0
    best_conf_seen = -1.0

    print(f"Using camera: {device_name}")
    print(f"Using model: {args.model}")
    print(f"Confidence threshold: {args.conf:.2f}")
    print(f"Rolling history size: {args.history_size} frames")
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

            result = results[0] if results else None
            gripper_detections = filter_gripper_detections(result) if result is not None else []
            current_best_conf = max(
                (detection["confidence"] for detection in gripper_detections),
                default=-1.0,
            )

            total_frames += 1
            frame_detected = int(bool(gripper_detections))
            detected_frames += frame_detected
            detection_history.append(frame_detected)

            if current_best_conf >= 0.0:
                last_detected_conf = current_best_conf
                best_conf_seen = max(best_conf_seen, current_best_conf)

            annotated = draw_gripper_detections(color_image, gripper_detections)
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

    print_summary(
        total_frames=total_frames,
        detected_frames=detected_frames,
        history=detection_history,
        last_detected_conf=last_detected_conf,
        best_conf_seen=best_conf_seen,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
