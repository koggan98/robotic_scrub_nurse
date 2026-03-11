#!/usr/bin/env python3
"""Minimal RealSense + Grounding DINO live detection test for a Robotiq gripper.

Usage:
    python3 ros_unrelated_scripts/grounding_dino_realsense_test.py
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
    from PIL import Image
except ImportError:
    Image = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    import torch
except ImportError:
    torch = None

try:
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
except ImportError:
    AutoModelForZeroShotObjectDetection = None
    AutoProcessor = None


WINDOW_NAME = "Grounding DINO RealSense Test"
DEFAULT_MODEL_ID = "IDEA-Research/grounding-dino-tiny"
DEFAULT_TEXT_LABELS = [["Robotiq gripper", "two-finger robotic gripper", "black gripper", "black"]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Grounding DINO on the connected RealSense color stream."
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_ID,
        help=f"Model id or local path (default: {DEFAULT_MODEL_ID})",
    )
    parser.add_argument(
        "--box-threshold",
        type=float,
        default=0.35,
        help="Box confidence threshold (default: 0.35)",
    )
    parser.add_argument(
        "--text-threshold",
        type=float,
        default=0.25,
        help="Text confidence threshold (default: 0.25)",
    )
    return parser.parse_args()


def check_dependencies() -> int:
    missing = []
    if cv2 is None:
        missing.append("opencv-python")
    if np is None:
        missing.append("numpy")
    if Image is None:
        missing.append("pillow")
    if rs is None:
        missing.append("pyrealsense2")
    if torch is None:
        missing.append("torch")
    if AutoModelForZeroShotObjectDetection is None or AutoProcessor is None:
        missing.append("transformers")

    if missing:
        print("ERROR: Missing required Python package(s): " + ", ".join(missing))
        print(
            "Install example: python3 -m pip install --user transformers torch pillow pyrealsense2 opencv-python"
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


def choose_device() -> str:
    return "cuda" if torch.cuda.is_available() else "cpu"


def load_model(model_name: str, device: str):
    try:
        processor = AutoProcessor.from_pretrained(model_name)
        model = AutoModelForZeroShotObjectDetection.from_pretrained(model_name).to(device)
        model.eval()
        return processor, model
    except Exception as exc:
        print(f"ERROR: Failed to load Grounding DINO model '{model_name}': {exc}")
        return None, None


def draw_detections(
    image_bgr: "np.ndarray",
    boxes,
    scores,
    labels,
) -> "np.ndarray":
    annotated = image_bgr.copy()
    for box, score, label in zip(boxes, scores, labels):
        x1, y1, x2, y2 = [int(round(v)) for v in box.tolist()]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 0), 2)
        text = f"{label}: {float(score):.2f}"
        cv2.putText(
            annotated,
            text,
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated


def main() -> int:
    args = parse_args()

    dep_status = check_dependencies()
    if dep_status != 0:
        return dep_status

    device = choose_device()
    processor, model = load_model(args.model, device)
    if processor is None or model is None:
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
    print(f"Using device: {device}")
    print(f"Using text labels: {DEFAULT_TEXT_LABELS[0]}")
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

            color_image_bgr = np.asanyarray(color_frame.get_data())
            color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(color_image_rgb)

            try:
                inputs = processor(
                    images=pil_image,
                    text=DEFAULT_TEXT_LABELS,
                    return_tensors="pt",
                ).to(device)
                with torch.no_grad():
                    outputs = model(**inputs)
                results = processor.post_process_grounded_object_detection(
                    outputs,
                    inputs.input_ids,
                    threshold=args.box_threshold,
                    text_threshold=args.text_threshold,
                    target_sizes=[pil_image.size[::-1]],
                )
            except Exception as exc:
                print(f"ERROR: Grounding DINO inference failed: {exc}")
                return 1

            result = results[0] if results else {"boxes": [], "scores": [], "text_labels": []}
            annotated = draw_detections(
                color_image_bgr,
                result["boxes"],
                result["scores"],
                result.get("text_labels", result.get("labels", [])),
            )
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
