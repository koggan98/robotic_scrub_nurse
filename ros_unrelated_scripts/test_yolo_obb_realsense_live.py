#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


def draw_obb(frame, xyxyxyxy, label, conf, color=(0, 255, 0)):
    pts = np.array(xyxyxyxy, dtype=np.int32).reshape(4, 2)

    # Thin OBB outline
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=1, lineType=cv2.LINE_AA)

    # Thin center point
    cx = int(np.mean(pts[:, 0]))
    cy = int(np.mean(pts[:, 1]))
    cv2.circle(frame, (cx, cy), 2, color, -1, lineType=cv2.LINE_AA)

    # Thin label text
    text = f"{label} {conf:.2f}"
    x_text = int(np.min(pts[:, 0]))
    y_text = int(np.min(pts[:, 1])) - 5
    y_text = max(y_text, 12)

    cv2.putText(
        frame,
        text,
        (x_text, y_text),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        color,
        1,
        cv2.LINE_AA,
    )


def main():
    print("TEST STARTED")
    parser = argparse.ArgumentParser(description="Live YOLO OBB inference with RealSense D455.")
    parser.add_argument(
        "--model",
        type=str,
        required=True,
        help="Path to trained YOLO OBB model, e.g. runs/.../weights/best.pt",
    )
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=1024)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--show-fps", action="store_true")
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = YOLO(str(model_path))

    pipeline = rs.pipeline()
    config = rs.config()

    # Your RealSense color stream resolution
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )

    print(f"Starting RealSense color stream: {args.width}x{args.height}@{args.fps}")
    print(f"Model: {model_path}")
    print("Press q or ESC to quit.")

    pipeline.start(config)

    prev_time = time.time()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            results = model.predict(
                source=frame,
                task="obb",
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                verbose=False,
            )

            annotated = frame.copy()

            result = results[0]
            names = result.names

            if result.obb is not None and result.obb.xyxyxyxy is not None:
                obb_points = result.obb.xyxyxyxy.cpu().numpy()
                classes = result.obb.cls.cpu().numpy().astype(int)
                confs = result.obb.conf.cpu().numpy()

                for pts, cls_id, conf in zip(obb_points, classes, confs):
                    label = names.get(cls_id, str(cls_id))

                    # handle in light blue, tools in green
                    if label == "handle":
                        color = (255, 180, 0)
                    else:
                        color = (0, 255, 0)

                    draw_obb(
                        annotated,
                        pts,
                        label,
                        float(conf),
                        color=color,
                    )

            if args.show_fps:
                now = time.time()
                fps = 1.0 / max(now - prev_time, 1e-6)
                prev_time = now
                cv2.putText(
                    annotated,
                    f"FPS: {fps:.1f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            cv2.imshow("YOLO OBB RealSense Live", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()