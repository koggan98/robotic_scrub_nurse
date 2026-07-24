#!/usr/bin/env python3
"""Fingerprint the effective inputs of one openWakeWord training stage."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


GENERATE_KEYS = (
    "model_name",
    "target_phrase",
    "custom_negative_phrases",
    "n_samples",
    "n_samples_val",
    "piper_sample_generator_path",
    "output_dir",
)
AUGMENT_KEYS = GENERATE_KEYS + (
    "rir_paths",
    "background_paths",
    "background_paths_duplication_rate",
    "augmentation_rounds",
)
IGNORED_BATCH_KEYS = {"tts_batch_size", "augmentation_batch_size"}
FEATURE_NAMES = (
    "positive_features_train.npy",
    "negative_features_train.npy",
    "positive_features_test.npy",
    "negative_features_test.npy",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_fingerprint(directory: Path, suffix: str) -> dict[str, object]:
    paths = sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() == suffix
    )
    digest = hashlib.sha256()
    total_bytes = 0
    for path in paths:
        size = path.stat().st_size
        total_bytes += size
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return {
        "directory": str(directory),
        "files": len(paths),
        "bytes": total_bytes,
        "sha256": digest.hexdigest(),
    }


def _marker_fingerprint(path: Path) -> str:
    if not path.is_file():
        raise FileNotFoundError(
            f"Required completed-stage marker is missing: {path}"
        )
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = value
    fingerprint = values.get("fingerprint")
    if not fingerprint:
        raise RuntimeError(f"Marker has no fingerprint: {path}")
    return fingerprint


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mode", choices=("smoke", "production"), required=True)
    parser.add_argument(
        "--stage",
        choices=("generate", "augment", "train"),
        required=True,
    )
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    kit_root = Path(__file__).resolve().parent.parent
    config_path = (kit_root / args.config).resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if args.stage == "generate":
        effective_config = {
            key: config[key] for key in GENERATE_KEYS
        }
    elif args.stage == "augment":
        effective_config = {
            key: config[key] for key in AUGMENT_KEYS
        }
    else:
        effective_config = {
            key: value
            for key, value in config.items()
            if key not in IGNORED_BATCH_KEYS
        }

    payload: dict[str, object] = {
        "schema_version": 1,
        "mode": args.mode,
        "stage": args.stage,
        "effective_config": effective_config,
        "effective_sources": {
            "openwakeword_train.py": _sha256(
                kit_root / "work/openWakeWord/openwakeword/train.py"
            ),
            "openwakeword_data.py": _sha256(
                kit_root / "work/openWakeWord/openwakeword/data.py"
            ),
        },
        "kit_inputs": {
            "assets.json": _sha256(kit_root / "manifests/assets.json"),
            "source-provenance.json": _sha256(
                kit_root / "manifests/source-provenance.json"
            ),
            "requirements-onnx-training.txt": _sha256(
                kit_root / "requirements-onnx-training.txt"
            ),
        },
    }
    full_validation = kit_root / "logs/input_validation_full.json"
    if full_validation.is_file():
        payload["kit_inputs"]["input_validation_full.json"] = _sha256(
            full_validation
        )

    if args.stage == "generate":
        payload["effective_sources"]["piper_generate_samples.py"] = _sha256(
            kit_root / "work/piper-sample-generator/generate_samples.py"
        )
        payload["piper_model"] = _sha256(
            kit_root
            / "work/piper-sample-generator/models/en_US-libritts_r-medium.pt"
        )
    elif args.stage == "augment":
        payload["dependency_fingerprint"] = _marker_fingerprint(
            kit_root / f"logs/state/{args.mode}.generate.done"
        )
        payload["audio_inputs"] = [
            _tree_fingerprint((kit_root / path).resolve(), ".wav")
            for path in (
                list(config["background_paths"]) + list(config["rir_paths"])
            )
        ]
        payload["feature_models"] = {
            name: _sha256(
                kit_root / "work/openWakeWord/openwakeword/resources/models" / name
            )
            for name in ("melspectrogram.onnx", "embedding_model.onnx")
        }
    else:
        payload["dependency_fingerprint"] = _marker_fingerprint(
            kit_root / f"logs/state/{args.mode}.augment.done"
        )
        feature_directory = (
            kit_root / config["output_dir"] / config["model_name"]
        ).resolve()
        payload["generated_features"] = {
            name: _sha256(feature_directory / name)
            for name in FEATURE_NAMES
        }

    canonical = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    fingerprint = hashlib.sha256(canonical).hexdigest()
    report = {
        "fingerprint": fingerprint,
        **payload,
    }
    if args.report:
        report_path = (kit_root / args.report).resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(fingerprint)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
