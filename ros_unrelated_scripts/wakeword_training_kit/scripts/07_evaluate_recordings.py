#!/usr/bin/env python3
"""Score independent background and wake-trial WAVs across thresholds."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from openwakeword.model import Model
from scipy.io import wavfile


FRAME_SAMPLES = 1280
WARMUP_FRAMES = 19
MINIMUM_BACKGROUND_HOURS = 1.0
MINIMUM_WAKE_TRIALS = 20


def _wav_files(directory: Path | None) -> list[Path]:
    if directory is None:
        return []
    paths = sorted(directory.resolve().iterdir())
    invalid = [
        path
        for path in paths
        if not path.is_file() or path.suffix.lower() != ".wav"
    ]
    if invalid:
        raise RuntimeError(
            f"{directory} must contain WAV files only: "
            + ", ".join(path.name for path in invalid[:5])
        )
    return paths


def _read_pcm16(path: Path) -> np.ndarray:
    rate, audio = wavfile.read(path)
    if rate != 16000:
        raise RuntimeError(f"{path}: expected 16 kHz")
    if audio.dtype != np.int16:
        raise RuntimeError(f"{path}: expected PCM16, got {audio.dtype}")
    if audio.ndim == 2 and audio.shape[1] == 1:
        audio = audio[:, 0]
    if audio.ndim != 1:
        raise RuntimeError(f"{path}: expected mono")
    return audio


def _scores(model: Model, audio: np.ndarray) -> list[float]:
    values: list[float] = []
    complete = (len(audio) // FRAME_SAMPLES) * FRAME_SAMPLES
    for offset in range(0, complete, FRAME_SAMPLES):
        predictions = model.predict(audio[offset : offset + FRAME_SAMPLES])
        score = max(float(value) for value in predictions.values())
        if not math.isfinite(score) or score < 0.0 or score > 1.0:
            raise RuntimeError(f"Invalid openWakeWord score: {score}")
        values.append(score)
    return values


def _count_events(scores: list[float], threshold: float) -> int:
    events = 0
    above = False
    for score in scores:
        now_above = score >= threshold
        if now_above and not above:
            events += 1
        above = now_above
    return events


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--background-dir", type=Path)
    parser.add_argument("--wake-dir", type=Path)
    parser.add_argument(
        "--thresholds",
        default="0.50,0.55,0.60,0.65,0.70,0.75,0.80,0.85,0.90",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("logs/offline_evaluation.json"),
    )
    args = parser.parse_args()
    if args.background_dir is None and args.wake_dir is None:
        parser.error("provide --background-dir and/or --wake-dir")

    thresholds = [float(value) for value in args.thresholds.split(",")]
    if any(value <= 0.0 or value > 1.0 for value in thresholds):
        parser.error("thresholds must be in (0, 1]")
    thresholds = sorted(set(thresholds))

    model_path = args.model.resolve()
    model = Model(
        wakeword_models=[str(model_path)],
        inference_framework="onnx",
    )

    background_files = _wav_files(args.background_dir)
    wake_files = _wav_files(args.wake_dir)
    background_scores: list[float] = []
    background_samples = 0
    model.reset()
    _scores(
        model,
        np.zeros(WARMUP_FRAMES * FRAME_SAMPLES, dtype=np.int16),
    )
    for path in background_files:
        audio = _read_pcm16(path)
        background_samples += len(audio)
        background_scores.extend(_scores(model, audio))

    wake_max_scores: dict[str, float] = {}
    for path in wake_files:
        model.reset()
        _scores(
            model,
            np.zeros(WARMUP_FRAMES * FRAME_SAMPLES, dtype=np.int16),
        )
        scores = _scores(model, _read_pcm16(path))
        wake_max_scores[path.name] = max(scores, default=0.0)

    background_hours = background_samples / 16000.0 / 3600.0
    insufficient_reasons = []
    if background_hours < MINIMUM_BACKGROUND_HOURS:
        insufficient_reasons.append(
            f"need at least {MINIMUM_BACKGROUND_HOURS:.1f} h background, "
            f"found {background_hours:.3f} h"
        )
    if len(wake_files) < MINIMUM_WAKE_TRIALS:
        insufficient_reasons.append(
            f"need at least {MINIMUM_WAKE_TRIALS} wake trials, "
            f"found {len(wake_files)}"
        )
    data_sufficient = not insufficient_reasons
    results = []
    required_wake_hits = math.ceil(0.95 * len(wake_files))
    for threshold in thresholds:
        false_events = _count_events(background_scores, threshold)
        wake_hits = sum(
            score >= threshold for score in wake_max_scores.values()
        )
        results.append(
            {
                "threshold": threshold,
                "background_false_events": false_events,
                "background_false_events_per_hour": (
                    false_events / background_hours
                    if background_hours > 0
                    else None
                ),
                "wake_hits": wake_hits,
                "wake_trials": len(wake_files),
                "wake_detection_rate": (
                    wake_hits / len(wake_files) if wake_files else None
                ),
            }
        )

    acceptable = [
        item
        for item in results
        if data_sufficient
        and item["background_false_events"] == 0
        and (
            not wake_files
            or int(item["wake_hits"]) >= required_wake_hits
        )
    ]
    recommended = acceptable[0]["threshold"] if acceptable and wake_files else None
    report = {
        "state": "offline_calibration_only",
        "evaluated_utc": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path),
        "background_directory": (
            str(args.background_dir.resolve()) if args.background_dir else None
        ),
        "background_files": len(background_files),
        "background_hours": background_hours,
        "wake_directory": str(args.wake_dir.resolve()) if args.wake_dir else None,
        "wake_files": len(wake_files),
        "wake_max_scores": wake_max_scores,
        "stream_warmup_seconds": (
            WARMUP_FRAMES * FRAME_SAMPLES / 16000.0
        ),
        "data_sufficient_for_recommendation": data_sufficient,
        "insufficient_data_reasons": insufficient_reasons,
        "results": results,
        "lowest_offline_acceptable_threshold": recommended,
        "note": (
            "Final acceptance must still run live on both microphones through "
            "the ROS ASR state machine."
        ),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
