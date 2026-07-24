#!/usr/bin/env python3
"""Structurally and functionally validate a trained Robot ONNX candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from openwakeword.model import Model


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("output/robot_training/robot.onnx"),
    )
    parser.add_argument(
        "--allow-smoke",
        action="store_true",
        help="Permit validation of a DO_NOT_DEPLOY smoke model.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("logs/onnx_validation.json"),
    )
    args = parser.parse_args()
    model_path = args.model.resolve()
    if not model_path.is_file() or model_path.stat().st_size == 0:
        raise FileNotFoundError(model_path)
    if "smoke" in model_path.name.casefold() and not args.allow_smoke:
        raise RuntimeError("Refusing smoke model without --allow-smoke")

    graph = onnx.load(str(model_path))
    onnx.checker.check_model(graph)
    session = ort.InferenceSession(
        str(model_path),
        providers=["CPUExecutionProvider"],
    )
    model_input = session.get_inputs()[0]
    model_output = session.get_outputs()[0]
    shape = list(model_input.shape)
    if len(shape) != 3 or shape[-2:] != [16, 96]:
        raise RuntimeError(f"Unexpected model input shape: {shape}")

    zero_input = np.zeros((1, 16, 96), dtype=np.float32)
    random_input = np.random.default_rng(0).normal(
        size=(1, 16, 96)
    ).astype(np.float32)
    zero_output = np.asarray(
        session.run(None, {model_input.name: zero_input})[0]
    )
    random_output = np.asarray(
        session.run(None, {model_input.name: random_input})[0]
    )
    for name, output in (
        ("zero", zero_output),
        ("random", random_output),
    ):
        if output.size != 1 or not np.all(np.isfinite(output)):
            raise RuntimeError(f"{name} inference returned invalid output")
        if float(output.flat[0]) < 0.0 or float(output.flat[0]) > 1.0:
            raise RuntimeError(f"{name} inference is outside [0, 1]")

    wake_model = Model(
        wakeword_models=[str(model_path)],
        inference_framework="onnx",
    )
    silence_scores = []
    for _ in range(50):
        predictions = wake_model.predict(np.zeros(1280, dtype=np.int16))
        silence_scores.extend(float(value) for value in predictions.values())
    if not silence_scores or not all(math.isfinite(x) for x in silence_scores):
        raise RuntimeError("openWakeWord integration returned invalid scores")

    report = {
        "state": "candidate_unvalidated",
        "validated_utc": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path),
        "bytes": model_path.stat().st_size,
        "sha256": _sha256(model_path),
        "onnx_ir_version": graph.ir_version,
        "input_name": model_input.name,
        "input_shape": shape,
        "output_name": model_output.name,
        "output_shape": list(model_output.shape),
        "zero_feature_score": float(zero_output.flat[0]),
        "random_feature_score": float(random_output.flat[0]),
        "max_silence_stream_score": max(silence_scores),
        "runtime_provider": session.get_providers()[0],
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    print("ONNX candidate validation passed; hardware validation is still required.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

