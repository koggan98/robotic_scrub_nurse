#!/usr/bin/env python3
"""Validate tangible outputs before marking a training stage complete."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


FEATURE_NAMES = (
    "positive_features_train.npy",
    "negative_features_train.npy",
    "positive_features_test.npy",
    "negative_features_test.npy",
)


def _check_wav_directory(
    directory: Path,
    expected: int,
) -> dict[str, object]:
    entries = list(directory.iterdir())
    invalid = [
        path.name
        for path in entries
        if not path.is_file() or path.suffix.lower() != ".wav"
    ]
    minimum = math.ceil(expected * 0.95)
    if invalid:
        raise RuntimeError(
            f"{directory} contains non-WAV entries: {invalid[:5]}"
        )
    if len(entries) < minimum:
        raise RuntimeError(
            f"{directory} contains {len(entries)} clips; at least {minimum} "
            f"are required for configured target {expected}"
        )
    return {
        "directory": str(directory),
        "wav_count": len(entries),
        "configured_target": expected,
        "minimum_accepted": minimum,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--stage",
        choices=("generate", "augment"),
        required=True,
    )
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    kit_root = Path(__file__).resolve().parent.parent
    config = yaml.safe_load(
        (kit_root / args.config).read_text(encoding="utf-8")
    )
    model_dir = (
        kit_root / config["output_dir"] / config["model_name"]
    ).resolve()
    report: dict[str, object] = {
        "checked_utc": datetime.now(timezone.utc).isoformat(),
        "stage": args.stage,
        "config": str(args.config),
    }

    if args.stage == "generate":
        report["clip_directories"] = [
            _check_wav_directory(
                model_dir / "positive_train",
                int(config["n_samples"]),
            ),
            _check_wav_directory(
                model_dir / "negative_train",
                int(config["n_samples"]),
            ),
            _check_wav_directory(
                model_dir / "positive_test",
                int(config["n_samples_val"]),
            ),
            _check_wav_directory(
                model_dir / "negative_test",
                int(config["n_samples_val"]),
            ),
        ]
    else:
        arrays: dict[str, object] = {}
        expected_rows = {
            "positive_features_train.npy": int(config["n_samples"]),
            "negative_features_train.npy": int(config["n_samples"]),
            "positive_features_test.npy": int(config["n_samples_val"]),
            "negative_features_test.npy": int(config["n_samples_val"]),
        }
        feature_frames: set[int] = set()
        for name in FEATURE_NAMES:
            path = model_dir / name
            array = np.load(path, mmap_mode="r")
            minimum = math.ceil(expected_rows[name] * 0.95)
            if (
                array.ndim != 3
                or array.shape[0] < minimum
                or array.shape[2] != 96
                or array.dtype != np.float32
            ):
                raise RuntimeError(
                    f"Unexpected feature array {path}: "
                    f"shape={array.shape}, dtype={array.dtype}, "
                    f"minimum rows={minimum}"
                )
            sample = np.asarray(array[: min(16, array.shape[0])])
            if not np.all(np.isfinite(sample)):
                raise RuntimeError(f"{path} contains non-finite sample data")
            feature_frames.add(int(array.shape[1]))
            arrays[name] = {
                "shape": list(array.shape),
                "dtype": str(array.dtype),
                "bytes": path.stat().st_size,
            }
        if len(feature_frames) != 1:
            raise RuntimeError(
                f"Feature arrays disagree on frame count: {feature_frames}"
            )
        report["feature_arrays"] = arrays

    report_path = (kit_root / args.report).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    print(f"{args.stage} postcondition check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
