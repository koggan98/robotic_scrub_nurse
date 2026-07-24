#!/usr/bin/env python3
"""Validate every source, asset, directory, WAV, and GPU training provider."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import wave
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
import torchaudio
import yaml

# Import torch before ONNX Runtime so CUDA/cuDNN libraries from the Hub image
# are loaded first.
import onnxruntime as ort


OPENWAKEWORD_COMMIT = "c8ef6912c5feccf1037b852d9bc6c7ed644135ba"
PIPER_COMMIT = "195e3bd967d54589c2137c9de2b22ad526ba6b6f"
PATCHED_TRAIN_SHA256 = (
    "48be9c8f640ad13a8816988a6efeb3b26d1d3ef6d2d8c54b948691b55ba1eda5"
)
PATCHED_DATA_SHA256 = (
    "bb81e61216dd55f6b595822c879ca81538294d8bd1f0b02fcbbaa2ba5c1432e8"
)
PATCHED_PIPER_SHA256 = (
    "a6107e0240d66d97a3c3c7dd2e6194be0a177b2cbf18fb88e56b6d67ae48edfb"
)
EXPECTED_RIR_COUNT = 271
ASSETS = {
    "work/piper-sample-generator/models/en_US-libritts_r-medium.pt": (
        204089915,
        "e95ee53770bf598c354a6e6dbfc95ccb259aeeb501d35a86be8a767429ab0ff6",
    ),
    "work/openWakeWord/openwakeword/resources/models/embedding_model.onnx": (
        1326578,
        "70d164290c1d095d1d4ee149bc5e00543250a7316b59f31d056cff7bd3075c1f",
    ),
    "work/openWakeWord/openwakeword/resources/models/melspectrogram.onnx": (
        1087958,
        "ba2b0e0f8b7b875369a2c89cb13360ff53bac436f2895cced9f479fa65eb176f",
    ),
    "data/openwakeword_features_ACAV100M_2000_hrs_16bit.npy": (
        17280000128,
        "721a66d0682c65a1b5c1da0aa109409cede1d20e28b15235c344b000cbb7654f",
    ),
    "data/validation_set_features.npy": (
        184836608,
        "a56a8a0f8e0efb91900acc6de4c0cdf4c564842e8475a7d49b36c039e17a690f",
    ),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _version_pair(version: str) -> tuple[int, int]:
    major, minor = version.split("+", 1)[0].split(".", 2)[:2]
    return int(major), int(minor)


def _git_head(path: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        text=True,
    ).strip()


def _validate_flat_wavs(
    directory: Path,
    minimum_files: int,
    minimum_seconds: float,
) -> tuple[int, float, dict[str, float]]:
    entries = list(directory.iterdir())
    invalid = [
        path
        for path in entries
        if not path.is_file() or path.suffix.lower() != ".wav"
    ]
    if invalid:
        raise RuntimeError(
            f"{directory} must be flat and WAV-only; invalid entries: "
            + ", ".join(path.name for path in invalid[:10])
        )

    total_seconds = 0.0
    durations: dict[str, float] = {}
    for path in entries:
        with wave.open(str(path), "rb") as wav_file:
            if wav_file.getnchannels() != 1:
                raise RuntimeError(f"{path}: expected mono")
            if wav_file.getframerate() != 16000:
                raise RuntimeError(f"{path}: expected 16 kHz")
            if wav_file.getsampwidth() != 2:
                raise RuntimeError(f"{path}: expected PCM16")
            if wav_file.getcomptype() != "NONE":
                raise RuntimeError(f"{path}: expected uncompressed PCM")
            duration = (
                wav_file.getnframes() / float(wav_file.getframerate())
            )
            total_seconds += duration
            durations[path.name] = duration

    if len(entries) < minimum_files:
        raise RuntimeError(
            f"{directory}: {len(entries)} WAVs, at least {minimum_files} required"
        )
    if total_seconds < minimum_seconds:
        raise RuntimeError(
            f"{directory}: {total_seconds / 60:.1f} minutes, at least "
            f"{minimum_seconds / 60:.1f} required"
        )
    return len(entries), total_seconds, durations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("robot.training.yaml"),
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Check byte counts but skip the expensive 17-GB SHA pass.",
    )
    args = parser.parse_args()

    kit_root = Path(__file__).resolve().parent.parent
    os.chdir(kit_root)
    report: dict[str, object] = {
        "checked_utc": datetime.now(timezone.utc).isoformat(),
        "config": str(args.config),
        "fast": args.fast,
        "assets": {},
    }

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    production = int(config["n_samples"]) >= 20_000

    if _version_pair(torch.__version__) != (2, 10):
        raise RuntimeError(f"Expected PyTorch 2.10.x, got {torch.__version__}")
    if _version_pair(torchaudio.__version__) != (2, 10):
        raise RuntimeError(
            f"Expected TorchAudio 2.10.x, got {torchaudio.__version__}"
        )
    torch_cuda = str(torch.version.cuda or "")
    if torch_cuda not in {"12.6", "12.8"}:
        raise RuntimeError(
            f"Expected a CUDA 12.6/12.8 PyTorch build, got {torch_cuda!r}"
        )
    expected_audio_suffix = f"+cu{torch_cuda.replace('.', '')}"
    if expected_audio_suffix not in torchaudio.__version__:
        raise RuntimeError(
            "TorchAudio CUDA build does not match PyTorch: "
            f"{torchaudio.__version__} versus CUDA {torch_cuda}"
        )
    if ort.__version__ != "1.20.1":
        raise RuntimeError(
            f"Expected ONNX Runtime GPU 1.20.1, got {ort.__version__}"
        )

    if _git_head(Path("work/openWakeWord")) != OPENWAKEWORD_COMMIT:
        raise RuntimeError("Unexpected openWakeWord source commit")
    if _git_head(Path("work/piper-sample-generator")) != PIPER_COMMIT:
        raise RuntimeError("Unexpected Piper source commit")
    effective_sources = {
        "openwakeword_train": Path(
            "work/openWakeWord/openwakeword/train.py"
        ),
        "openwakeword_data": Path(
            "work/openWakeWord/openwakeword/data.py"
        ),
        "piper_generate_samples": Path(
            "work/piper-sample-generator/generate_samples.py"
        ),
    }
    source_hashes = {
        name: _sha256(path) for name, path in effective_sources.items()
    }
    if source_hashes["openwakeword_train"] != PATCHED_TRAIN_SHA256:
        raise RuntimeError(
            "Effective openWakeWord train.py has an unexpected patch state: "
            f"{source_hashes['openwakeword_train']}"
        )
    if source_hashes["openwakeword_data"] != PATCHED_DATA_SHA256:
        raise RuntimeError(
            "Effective openWakeWord data.py has an unexpected patch state: "
            f"{source_hashes['openwakeword_data']}"
        )
    if source_hashes["piper_generate_samples"] != PATCHED_PIPER_SHA256:
        raise RuntimeError(
            "Effective Piper generate_samples.py is not the audited "
            "PyTorch-2.10 patch: "
            f"{source_hashes['piper_generate_samples']}"
        )
    report["effective_source_sha256"] = source_hashes

    for relative_path, (expected_bytes, expected_sha256) in ASSETS.items():
        path = Path(relative_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        actual_bytes = path.stat().st_size
        if actual_bytes != expected_bytes:
            raise RuntimeError(
                f"{path}: expected {expected_bytes} bytes, got {actual_bytes}"
            )
        should_hash = not args.fast or expected_bytes < 1024**3
        actual_sha256 = _sha256(path) if should_hash else "skipped-fast-mode"
        if should_hash and actual_sha256 != expected_sha256:
            raise RuntimeError(
                f"{path}: expected SHA-256 {expected_sha256}, got "
                f"{actual_sha256}"
            )
        report["assets"][relative_path] = {
            "bytes": actual_bytes,
            "sha256": actual_sha256,
        }

    acav = np.load(
        "data/openwakeword_features_ACAV100M_2000_hrs_16bit.npy",
        mmap_mode="r",
    )
    validation = np.load(
        "data/validation_set_features.npy",
        mmap_mode="r",
    )
    if acav.shape != (5_625_000, 16, 96) or acav.dtype != np.float16:
        raise RuntimeError(f"Unexpected ACAV array: {acav.shape} {acav.dtype}")
    if validation.shape != (481_345, 96) or validation.dtype != np.float32:
        raise RuntimeError(
            f"Unexpected validation array: {validation.shape} "
            f"{validation.dtype}"
        )
    report["arrays"] = {
        "acav_shape": list(acav.shape),
        "acav_dtype": str(acav.dtype),
        "validation_shape": list(validation.shape),
        "validation_dtype": str(validation.dtype),
    }

    background_count, background_seconds, background_durations = (
        _validate_flat_wavs(
        Path("data/background_clips"),
        minimum_files=max(16, int(config["augmentation_batch_size"])),
        minimum_seconds=1800.0 if production else 60.0,
        )
    )
    rir_count, rir_seconds, _ = _validate_flat_wavs(
        Path("data/mit_rirs"),
        minimum_files=EXPECTED_RIR_COUNT,
        minimum_seconds=1.0,
    )
    if rir_count != EXPECTED_RIR_COUNT:
        raise RuntimeError(
            f"Expected exactly {EXPECTED_RIR_COUNT} MIT RIR WAVs, got "
            f"{rir_count}"
        )
    per_microphone_minutes = {
        prefix: sum(
            duration
            for name, duration in background_durations.items()
            if name.startswith(prefix)
        )
        / 60.0
        for prefix in ("bgtrain_samson_", "bgtrain_jieli_")
    }
    if production:
        for prefix, minutes in per_microphone_minutes.items():
            if minutes < 60.0:
                raise RuntimeError(
                    f"Production needs 60 minutes with prefix {prefix!r}; "
                    f"found {minutes:.1f}. Use the documented recorder "
                    "commands and audit every clip before import."
                )
    report["audio"] = {
        "background_wavs": background_count,
        "background_minutes": background_seconds / 60.0,
        "background_minutes_by_required_prefix": per_microphone_minutes,
        "rir_wavs": rir_count,
        "rir_seconds": rir_seconds,
    }

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch CUDA is unavailable")
    if "CUDAExecutionProvider" not in ort.get_available_providers():
        raise RuntimeError(
            "ONNX Runtime CUDAExecutionProvider is unavailable: "
            f"{ort.get_available_providers()}"
        )
    from openwakeword.utils import AudioFeatures

    features = AudioFeatures(device="gpu", inference_framework="onnx")
    feature_result = features(
        np.zeros(1280, dtype=np.int16)
    )
    melspec_providers = features.melspec_model.get_providers()
    embedding_providers = features.embedding_model.get_providers()
    if (
        melspec_providers[0] != "CUDAExecutionProvider"
        or embedding_providers[0] != "CUDAExecutionProvider"
    ):
        raise RuntimeError(
            "Feature models did not select CUDA: "
            f"mel={melspec_providers}, embedding={embedding_providers}"
        )
    if not isinstance(feature_result, (int, np.integer)):
        raise RuntimeError(
            f"Unexpected streaming feature result: {feature_result!r}"
        )
    report["gpu"] = {
        "torch": torch.__version__,
        "torchaudio": torchaudio.__version__,
        "torch_cuda": torch.version.cuda,
        "device": torch.cuda.get_device_name(0),
        "onnxruntime": ort.__version__,
        "providers": ort.get_available_providers(),
        "melspectrogram_providers": melspec_providers,
        "embedding_providers": embedding_providers,
        "feature_inference": "passed",
    }

    usage = shutil.disk_usage(kit_root)
    stat = os.statvfs(kit_root)
    report["storage"] = {
        "free_gib": usage.free / 1024**3,
        "free_inodes": stat.f_favail,
    }
    if production and usage.free < 25 * 1024**3:
        raise RuntimeError(
            "Less than 25 GiB remains after asset download; use larger scratch."
        )
    if production and stat.f_favail < 250_000:
        raise RuntimeError(
            "Fewer than 250,000 inodes remain for generated WAV files."
        )

    Path("logs").mkdir(exist_ok=True)
    report_path = Path(
        "logs/input_validation_fast.json"
        if args.fast
        else "logs/input_validation_full.json"
    )
    report_path.write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    print("Input validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
