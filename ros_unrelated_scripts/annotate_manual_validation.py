#!/usr/bin/env python3
"""Run both tray OBB detectors over the three manual-validation image sets.

The script maps each image folder to the detector used by the live ROS system,
saves a plotted copy of every image in an ``annotated`` subfolder, and creates
two CSV files that make the manual thesis review traceable.

Typical usage from the robotic_scrub_nurse repository:

    python3 ros_unrelated_scripts/annotate_manual_validation.py

If ``manual_val`` is stored elsewhere:

    python3 ros_unrelated_scripts/annotate_manual_validation.py \
        --manual-val-dir /path/to/data/manual_val
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_DIR = SCRIPT_DIR.parent

IMAGE_SUFFIXES = {
    ".bmp",
    ".jpeg",
    ".jpg",
    ".png",
    ".tif",
    ".tiff",
    ".webp",
}

REVIEW_COLUMNS = (
    "dataset",
    "image",
    "annotated_image",
    "model",
    "num_tool_detections",
    "predicted_tools",
    "num_handle_detections",
    "expected_tool_count",
    "correct_detection_count",
    "false_positive_count",
    "missed_tool_count",
    "wrong_class_count",
    "reviewed",
    "review_notes",
)

DETECTION_COLUMNS = (
    "dataset",
    "image",
    "annotated_image",
    "model",
    "detection_index",
    "class_id",
    "class_name",
    "confidence",
    "center_x_px",
    "center_y_px",
    "width_px",
    "height_px",
    "angle_rad",
    "corners_xy_px",
)


@dataclass(frozen=True)
class DatasetJob:
    """Configuration for one input folder and its matching detector."""

    name: str
    model_kind: str
    image_size: int


DATASET_NAMES = (
    "instrument_tray",
    "reclaim_tray",
    "reclaim_tray_cam_a_bit_crooked",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Annotate the three manual_val image folders with the matching "
            "Ultralytics YOLO-OBB tray detectors."
        )
    )
    parser.add_argument(
        "--manual-val-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing instrument_tray, reclaim_tray, and "
            "reclaim_tray_cam_a_bit_crooked. If omitted, common repository "
            "locations are detected automatically."
        ),
    )
    parser.add_argument(
        "--instrument-model",
        type=Path,
        default=SCRIPT_DIR / "instrument_tray_detector.pt",
        help="Instrument-tray YOLO-OBB weights.",
    )
    parser.add_argument(
        "--reclaim-model",
        type=Path,
        default=SCRIPT_DIR / "reclaim_tray_detector.pt",
        help="Reclaim-tray YOLO-OBB weights (also used for the crooked camera set).",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="Confidence threshold used by both models (default: 0.35).",
    )
    parser.add_argument(
        "--instrument-imgsz",
        type=int,
        default=1024,
        help="Inference image size for instrument_tray (default: 1024).",
    )
    parser.add_argument(
        "--reclaim-imgsz",
        type=int,
        default=640,
        help="Inference image size for both reclaim datasets (default: 640).",
    )
    parser.add_argument(
        "--device",
        default="auto",
        help=(
            "Inference device, for example cpu, mps, 0, or cuda:0. With "
            "'auto' (default), Ultralytics selects the available device."
        ),
    )
    parser.add_argument(
        "--output-subdir",
        default="annotated",
        help="Subfolder created inside every image folder (default: annotated).",
    )
    parser.add_argument(
        "--line-width",
        type=int,
        default=2,
        help="Line width of plotted oriented bounding boxes (default: 2).",
    )
    return parser.parse_args()


def _contains_all_datasets(path: Path) -> bool:
    return path.is_dir() and all((path / name).is_dir() for name in DATASET_NAMES)


def find_manual_val_dir(requested_path: Path | None) -> Path:
    """Resolve manual_val while supporting both repositories used in this workspace."""

    if requested_path is not None:
        return requested_path.expanduser().resolve()

    candidates = (
        Path.cwd(),
        Path.cwd() / "manual_val",
        Path.cwd() / "data" / "manual_val",
        REPOSITORY_DIR / "data" / "manual_val",
        REPOSITORY_DIR.parent / "instrument_detection_model" / "data" / "manual_val",
    )
    for candidate in candidates:
        resolved = candidate.resolve()
        if _contains_all_datasets(resolved):
            return resolved

    # Returning the expected sibling path produces a useful validation error below.
    return candidates[-1].resolve()


def validate_args(args: argparse.Namespace, manual_val_dir: Path) -> None:
    if not 0.0 <= args.conf <= 1.0:
        raise ValueError(f"--conf must be between 0 and 1, got {args.conf}")
    if args.instrument_imgsz <= 0 or args.reclaim_imgsz <= 0:
        raise ValueError("--instrument-imgsz and --reclaim-imgsz must be positive")
    if args.line_width <= 0:
        raise ValueError("--line-width must be positive")

    output_path = Path(args.output_subdir)
    if (
        not args.output_subdir
        or output_path.is_absolute()
        or len(output_path.parts) != 1
        or args.output_subdir in {".", ".."}
    ):
        raise ValueError("--output-subdir must be one relative folder name")

    if not _contains_all_datasets(manual_val_dir):
        expected = ", ".join(DATASET_NAMES)
        raise FileNotFoundError(
            f"Invalid manual validation directory: {manual_val_dir}\n"
            f"Expected these three subfolders: {expected}\n"
            "Pass the correct location with --manual-val-dir."
        )

    for model_path in (args.instrument_model, args.reclaim_model):
        if not model_path.expanduser().is_file():
            raise FileNotFoundError(f"Model weights not found: {model_path}")


def image_files(folder: Path) -> list[Path]:
    """Return only source images directly inside a dataset folder."""

    return sorted(
        path
        for path in folder.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def class_name(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    try:
        return str(names[class_id])
    except (IndexError, KeyError, TypeError):
        return str(class_id)


def as_numpy(tensor: Any) -> Any:
    """Move an Ultralytics result tensor to CPU and convert it to NumPy."""

    return tensor.detach().cpu().numpy()


def relative_text(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def detection_rows(
    result: Any,
    job: DatasetJob,
    image_path: Path,
    annotated_path: Path,
    model_path: Path,
    manual_val_dir: Path,
) -> tuple[list[dict[str, Any]], list[str]]:
    """Extract detailed OBB rows and return their class names."""

    obb = result.obb
    if obb is None:
        raise RuntimeError(
            f"Model returned no OBB result container for {image_path.name}. "
            "Check that OBB weights are being used."
        )
    if len(obb) == 0:
        return [], []

    corners = as_numpy(obb.xyxyxyxy)
    rotated_boxes = as_numpy(obb.xywhr)
    class_ids = as_numpy(obb.cls).astype(int)
    confidences = as_numpy(obb.conf)

    rows: list[dict[str, Any]] = []
    detected_classes: list[str] = []
    for index, (points, xywhr, class_id, confidence) in enumerate(
        zip(corners, rotated_boxes, class_ids, confidences), start=1
    ):
        detected_class = class_name(result.names, int(class_id))
        detected_classes.append(detected_class)
        center_x, center_y, width, height, angle = xywhr
        rounded_corners = [
            [round(float(x), 3), round(float(y), 3)] for x, y in points
        ]
        rows.append(
            {
                "dataset": job.name,
                "image": relative_text(image_path, manual_val_dir),
                "annotated_image": relative_text(annotated_path, manual_val_dir),
                "model": str(model_path),
                "detection_index": index,
                "class_id": int(class_id),
                "class_name": detected_class,
                "confidence": round(float(confidence), 6),
                "center_x_px": round(float(center_x), 3),
                "center_y_px": round(float(center_y), 3),
                "width_px": round(float(width), 3),
                "height_px": round(float(height), 3),
                "angle_rad": round(float(angle), 6),
                "corners_xy_px": json.dumps(rounded_corners, separators=(",", ":")),
            }
        )
    return rows, detected_classes


def summarized_classes(classes: Iterable[str]) -> str:
    counts = Counter(classes)
    return "; ".join(
        f"{name}={count}" for name, count in sorted(counts.items())
    )


def save_csv(path: Path, columns: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def predict_one(
    model: Any,
    image_path: Path,
    image_size: int,
    confidence: float,
    device: str,
) -> Any:
    prediction_args: dict[str, Any] = {
        "source": str(image_path),
        "task": "obb",
        "imgsz": image_size,
        "conf": confidence,
        "verbose": False,
    }
    if device.lower() != "auto":
        prediction_args["device"] = device

    results = model.predict(**prediction_args)
    if len(results) != 1:
        raise RuntimeError(
            f"Expected one result for {image_path.name}, received {len(results)}"
        )
    return results[0]


def main() -> int:
    args = parse_args()
    manual_val_dir = find_manual_val_dir(args.manual_val_dir)

    try:
        validate_args(args, manual_val_dir)
        from ultralytics import YOLO
    except ImportError:
        print(
            "ERROR: ultralytics is not installed. Install it with:\n"
            "  python3 -m pip install ultralytics",
            file=sys.stderr,
        )
        return 2
    except (FileNotFoundError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    instrument_model_path = args.instrument_model.expanduser().resolve()
    reclaim_model_path = args.reclaim_model.expanduser().resolve()

    print(f"Manual validation directory: {manual_val_dir}")
    print(f"Confidence threshold: {args.conf}")
    print(f"Device: {args.device}")

    try:
        print(f"Loading instrument model: {instrument_model_path}")
        instrument_model = YOLO(str(instrument_model_path))
        print(f"Loading reclaim model:    {reclaim_model_path}")
        reclaim_model = YOLO(str(reclaim_model_path))
    except Exception as error:
        print(f"ERROR: Failed to load model weights: {error}", file=sys.stderr)
        return 1

    for name, model in (
        (instrument_model_path, instrument_model),
        (reclaim_model_path, reclaim_model),
    ):
        if getattr(model, "task", None) != "obb":
            print(f"ERROR: Expected an OBB model, but {name} has task={model.task!r}")
            return 1

    jobs = (
        DatasetJob("instrument_tray", "instrument", args.instrument_imgsz),
        DatasetJob("reclaim_tray", "reclaim", args.reclaim_imgsz),
        DatasetJob(
            "reclaim_tray_cam_a_bit_crooked", "reclaim", args.reclaim_imgsz
        ),
    )
    models = {"instrument": instrument_model, "reclaim": reclaim_model}
    model_paths = {
        "instrument": instrument_model_path,
        "reclaim": reclaim_model_path,
    }

    review_rows: list[dict[str, Any]] = []
    all_detection_rows: list[dict[str, Any]] = []
    failures: list[str] = []

    for job in jobs:
        input_dir = manual_val_dir / job.name
        output_dir = input_dir / args.output_subdir
        output_dir.mkdir(parents=True, exist_ok=True)
        images = image_files(input_dir)
        print(
            f"\n[{job.name}] {len(images)} images, model={job.model_kind}, "
            f"imgsz={job.image_size}"
        )

        if not images:
            failures.append(f"{job.name}: no supported images found")
            print("  ERROR: No supported images found", file=sys.stderr)
            continue

        for number, image_path in enumerate(images, start=1):
            annotated_path = output_dir / image_path.name
            try:
                result = predict_one(
                    models[job.model_kind],
                    image_path,
                    job.image_size,
                    args.conf,
                    args.device,
                )
                result.save(
                    filename=str(annotated_path),
                    line_width=args.line_width,
                    labels=True,
                    conf=True,
                    boxes=True,
                )
                rows, detected_classes = detection_rows(
                    result,
                    job,
                    image_path,
                    annotated_path,
                    model_paths[job.model_kind],
                    manual_val_dir,
                )
                all_detection_rows.extend(rows)

                tool_classes = [name for name in detected_classes if name != "handle"]
                handle_count = sum(name == "handle" for name in detected_classes)
                review_rows.append(
                    {
                        "dataset": job.name,
                        "image": relative_text(image_path, manual_val_dir),
                        "annotated_image": relative_text(
                            annotated_path, manual_val_dir
                        ),
                        "model": str(model_paths[job.model_kind]),
                        "num_tool_detections": len(tool_classes),
                        "predicted_tools": summarized_classes(tool_classes),
                        "num_handle_detections": handle_count,
                        "expected_tool_count": "",
                        "correct_detection_count": "",
                        "false_positive_count": "",
                        "missed_tool_count": "",
                        "wrong_class_count": "",
                        "reviewed": "",
                        "review_notes": "",
                    }
                )
                print(
                    f"  {number:>3}/{len(images)}  {image_path.name}: "
                    f"{len(tool_classes)} tools, {handle_count} handles"
                )
            except Exception as error:
                failure = f"{job.name}/{image_path.name}: {error}"
                failures.append(failure)
                print(f"  ERROR: {failure}", file=sys.stderr)

    review_csv = manual_val_dir / "manual_review.csv"
    detections_csv = manual_val_dir / "detections.csv"
    save_csv(review_csv, REVIEW_COLUMNS, review_rows)
    save_csv(detections_csv, DETECTION_COLUMNS, all_detection_rows)

    print("\nFinished.")
    print(f"Annotated images: each dataset's '{args.output_subdir}' subfolder")
    print(f"Manual review CSV: {review_csv}")
    print(f"Detailed detections CSV: {detections_csv}")
    print(f"Processed images: {len(review_rows)}")
    print(f"Total detections: {len(all_detection_rows)}")

    if failures:
        print(f"Failures: {len(failures)}", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
