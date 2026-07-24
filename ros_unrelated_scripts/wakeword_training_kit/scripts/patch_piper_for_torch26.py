#!/usr/bin/env python3
"""Apply the trusted-model PyTorch 2.6+ load compatibility change to Piper."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import torch


PIPER_SOURCE_SHA256 = (
    "6e7b334e6de85fe2f4ad0a70d9ef79e28b6cc6d8a9a715377882df5c4264ef60"
)
PIPER_PATCHED_SHA256 = (
    "a6107e0240d66d97a3c3c7dd2e6194be0a177b2cbf18fb88e56b6d67ae48edfb"
)
PIPER_MODEL_SHA256 = (
    "e95ee53770bf598c354a6e6dbfc95ccb259aeeb501d35a86be8a767429ab0ff6"
)
OLD_CALL = b"model = torch.load(model_path)"
NEW_CALL = b"model = torch.load(model_path, weights_only=False)"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _torch_version_pair() -> tuple[int, int]:
    major, minor = torch.__version__.split("+", 1)[0].split(".", 2)[:2]
    return int(major), int(minor)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("generate_samples", type=Path)
    parser.add_argument("trusted_model", type=Path)
    args = parser.parse_args()

    if _torch_version_pair() < (2, 6):
        print(
            f"PyTorch {torch.__version__} needs no Piper compatibility patch."
        )
        return

    model_sha256 = _sha256(args.trusted_model)
    if model_sha256 != PIPER_MODEL_SHA256:
        raise RuntimeError(
            "Refusing weights_only=False for an unverified model: "
            f"expected {PIPER_MODEL_SHA256}, got {model_sha256}"
        )

    source = args.generate_samples.read_bytes()
    if NEW_CALL in source:
        if hashlib.sha256(source).hexdigest() != PIPER_PATCHED_SHA256:
            raise RuntimeError(
                "Refusing an altered previously patched Piper source: "
                f"{hashlib.sha256(source).hexdigest()}"
            )
        print("Piper PyTorch 2.6+ compatibility patch already present.")
        return
    source_sha256 = hashlib.sha256(source).hexdigest()
    if source_sha256 != PIPER_SOURCE_SHA256:
        raise RuntimeError(
            "Refusing to patch unexpected Piper source: "
            f"expected {PIPER_SOURCE_SHA256}, got {source_sha256}"
        )
    if source.count(OLD_CALL) != 1:
        raise RuntimeError("Expected exactly one torch.load(model_path) call")

    patched = source.replace(OLD_CALL, NEW_CALL, 1)
    if hashlib.sha256(patched).hexdigest() != PIPER_PATCHED_SHA256:
        raise RuntimeError("Piper compatibility patch verification failed")
    args.generate_samples.write_bytes(patched)
    print(
        "Applied Piper PyTorch 2.6+ compatibility patch after verifying the "
        "official model SHA-256."
    )


if __name__ == "__main__":
    main()
