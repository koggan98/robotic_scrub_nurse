#!/usr/bin/env python3
"""Download the official notebook's MIT room impulse responses as PCM WAV."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import scipy.io.wavfile
from datasets import load_dataset
from tqdm import tqdm


DATASET_NAME = "davidscripka/MIT_environmental_impulse_responses"
DATASET_REVISION = "c4e8a0ebe36ab727bf57ddbb6215afdf3af0a0ec"
EXPECTED_WAV_COUNT = 271


def _wav_inventory(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    entries = sorted(directory.iterdir())
    invalid = [
        path
        for path in entries
        if not path.is_file() or path.suffix.lower() != ".wav"
    ]
    if invalid:
        raise RuntimeError(
            f"{directory} must be flat and WAV-only: "
            + ", ".join(path.name for path in invalid[:5])
        )
    return entries


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/mit_rirs"),
    )
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    existing = _wav_inventory(output_dir)
    if len(existing) == EXPECTED_WAV_COUNT:
        print(
            f"Using complete {len(existing)}-file RIR set in {output_dir}"
        )
        return 0

    staging_dir = output_dir.parent / f".{output_dir.name}_staging"
    staging_dir.mkdir(exist_ok=True)
    _wav_inventory(staging_dir)

    dataset = load_dataset(
        DATASET_NAME,
        revision=DATASET_REVISION,
        split="train",
        streaming=True,
    )
    count = 0
    for row in tqdm(dataset, desc="MIT RIR"):
        audio = np.asarray(row["audio"]["array"], dtype=np.float64)
        if audio.ndim == 2:
            audio = audio.mean(axis=1)
        if audio.ndim != 1 or audio.size == 0:
            raise RuntimeError("Unexpected RIR audio shape")
        audio = np.clip(audio, -1.0, 1.0)
        pcm16 = np.round(audio * 32767.0).astype(np.int16)
        filename = Path(row["audio"]["path"]).name
        if not filename.lower().endswith(".wav"):
            filename += ".wav"
        scipy.io.wavfile.write(staging_dir / filename, 16000, pcm16)
        count += 1

    staged = _wav_inventory(staging_dir)
    if count != EXPECTED_WAV_COUNT or len(staged) != EXPECTED_WAV_COUNT:
        raise RuntimeError(
            "Incomplete MIT RIR dataset: "
            f"stream yielded {count}, staging contains {len(staged)}, "
            f"expected {EXPECTED_WAV_COUNT}. Re-run the command."
        )

    if output_dir.exists():
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        quarantine = (
            output_dir.parent / f"{output_dir.name}_incomplete_{timestamp}"
        )
        output_dir.rename(quarantine)
        print(f"Preserved incomplete prior RIR directory as {quarantine}")
    staging_dir.rename(output_dir)

    metadata_dir = output_dir.parent / "recording_metadata"
    metadata_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "dataset": DATASET_NAME,
        "revision": DATASET_REVISION,
        "prepared_utc": datetime.now(timezone.utc).isoformat(),
        "wav_count": len(staged),
        "sample_rate_hz": 16000,
        "format": "mono PCM16 WAV",
    }
    (metadata_dir / "mit_rirs.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Prepared {len(staged)} RIR files in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
