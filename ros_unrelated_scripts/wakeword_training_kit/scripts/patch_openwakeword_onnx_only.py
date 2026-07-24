#!/usr/bin/env python3
"""Make openWakeWord v0.6.0's ONNX training path PyTorch-2.10-safe."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


UPSTREAM_SHA256 = (
    "a9a994dd10203ef290a251f902e4181d832263876065a6b7c5dfc25f61ca293e"
)
PREVIOUS_KIT_PATCH_SHA256 = (
    "7ced1f7423824b91f03be2214425ccba19eb47f5cb6893b08ab40a783855908c"
)
PATCHED_SHA256 = (
    "48be9c8f640ad13a8816988a6efeb3b26d1d3ef6d2d8c54b948691b55ba1eda5"
)
MARKER = b"This deployment consumes ONNX directly."
CPU_MARKER = b"os.sched_getaffinity"
WORKER_MARKER = b"max(1, n_cpus//2)"
DYNAMO_MARKER = b"dynamo=False"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def patch_train_file(path: Path) -> None:
    """Patch the pinned upstream file exactly once, preserving line endings."""
    original = path.read_bytes()
    if (
        MARKER in original
        and CPU_MARKER in original
        and original.count(WORKER_MARKER) == 2
        and original.count(DYNAMO_MARKER) == 3
    ):
        if _sha256(original) != PATCHED_SHA256:
            raise RuntimeError(
                "Refusing an altered previously patched openWakeWord "
                f"train.py: {_sha256(original)}"
            )
        print(f"PyTorch-2.10 ONNX patch already present: {path}")
        return

    actual_sha256 = _sha256(original)
    if actual_sha256 not in {
        UPSTREAM_SHA256,
        PREVIOUS_KIT_PATCH_SHA256,
    }:
        raise RuntimeError(
            "Refusing to patch an unexpected openWakeWord train.py: "
            f"expected upstream SHA-256 {UPSTREAM_SHA256} or previous-kit "
            f"SHA-256 {PREVIOUS_KIT_PATCH_SHA256}, got {actual_sha256}"
        )

    patched = original
    if actual_sha256 == UPSTREAM_SHA256:
        newline = b"\r\n" if b"\r\n" in original else b"\n"
        old_lines = [
            b"        # Convert the model from onnx to tflite format",
            (
                b'        convert_onnx_to_tflite(os.path.join(config["output_dir"], '
                b'config["model_name"] + ".onnx"),'
            ),
            (
                b'                               os.path.join(config["output_dir"], '
                b'config["model_name"] + ".tflite"))'
            ),
        ]
        new_lines = [
            (
                b"        # This deployment consumes ONNX directly. Skipping the "
                b"optional TFLite"
            ),
            (
                b"        # conversion removes the obsolete TensorFlow 2.8.1 "
                b"dependency without"
            ),
            (
                b"        # changing clip generation, augmentation, training, or "
                b"ONNX export."
            ),
        ]
        old_block = newline.join(old_lines)
        new_block = newline.join(new_lines)
        if patched.count(old_block) != 1:
            raise RuntimeError(
                "Expected exactly one upstream TFLite conversion block"
            )
        patched = patched.replace(old_block, new_block, 1)

        old_cpu_count = b"n_cpus = os.cpu_count()"
        new_cpu_count = (
            b'n_cpus = (len(os.sched_getaffinity(0)) if '
            b'hasattr(os, "sched_getaffinity") else os.cpu_count())'
        )
        if patched.count(old_cpu_count) != 2:
            raise RuntimeError("Expected two upstream os.cpu_count() assignments")
        patched = patched.replace(old_cpu_count, new_cpu_count)
        old_worker_count = b"n_cpus = n_cpus//2"
        new_worker_count = b"n_cpus = max(1, n_cpus//2)"
        if patched.count(old_worker_count) != 2:
            raise RuntimeError("Expected two upstream worker-halving assignments")
        patched = patched.replace(old_worker_count, new_worker_count)

    utility_export = b"output_names=[class_mapping])"
    utility_export_210 = (
        b"output_names=[class_mapping], opset_version=13, dynamo=False)"
    )
    if patched.count(utility_export) != 2:
        raise RuntimeError("Expected two utility ONNX export calls")
    patched = patched.replace(utility_export, utility_export_210)

    production_export = (
        b'model_name + ".onnx"), opset_version=13)'
    )
    production_export_210 = (
        b'model_name + ".onnx"), opset_version=13, dynamo=False)'
    )
    if patched.count(production_export) != 1:
        raise RuntimeError("Expected one production ONNX export call")
    patched = patched.replace(
        production_export,
        production_export_210,
        1,
    )
    path.write_bytes(patched)
    if (
        MARKER not in patched
        or CPU_MARKER not in patched
        or patched.count(WORKER_MARKER) != 2
        or patched.count(DYNAMO_MARKER) != 3
        or _sha256(patched) != PATCHED_SHA256
    ):
        raise RuntimeError("PyTorch-2.10 ONNX patch verification failed")
    print(f"Applied PyTorch-2.10 ONNX patch: {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "train_file",
        type=Path,
        help="Path to the pinned openWakeWord v0.6.0 openwakeword/train.py",
    )
    args = parser.parse_args()
    patch_train_file(args.train_file.resolve())


if __name__ == "__main__":
    main()
