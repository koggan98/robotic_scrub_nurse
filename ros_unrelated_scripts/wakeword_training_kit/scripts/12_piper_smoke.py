#!/usr/bin/env python3
"""Generate and validate one real Piper "robot" clip."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import wave
from array import array
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "piper_directory",
        type=Path,
        help="Pinned piper-sample-generator v2.0.0 directory",
    )
    args = parser.parse_args()
    piper_directory = args.piper_directory.resolve()
    generator = piper_directory / "generate_samples.py"
    model = (
        piper_directory
        / "models"
        / "en_US-libritts_r-medium.pt"
    )
    if not generator.is_file() or not model.is_file():
        raise FileNotFoundError(
            f"Piper generator/model missing below {piper_directory}"
        )

    with tempfile.TemporaryDirectory(prefix="robot-piper-smoke-") as temp:
        output_directory = Path(temp)
        subprocess.run(
            [
                sys.executable,
                str(generator),
                "robot",
                "--max-samples",
                "1",
                "--batch-size",
                "1",
                "--max-speakers",
                "1",
                "--output-dir",
                str(output_directory),
            ],
            cwd=piper_directory,
            check=True,
        )
        output = output_directory / "0.wav"
        with wave.open(str(output), "rb") as wav_file:
            channels = wav_file.getnchannels()
            sample_rate = wav_file.getframerate()
            sample_width = wav_file.getsampwidth()
            frame_count = wav_file.getnframes()
            samples = array("h", wav_file.readframes(frame_count))
        duration = frame_count / float(sample_rate)
        if (
            channels != 1
            or sample_rate != 16_000
            or sample_width != 2
            or not 0.1 <= duration <= 5.0
            or not samples
            or max(abs(value) for value in samples) == 0
        ):
            raise RuntimeError(
                "Invalid Piper smoke WAV: "
                f"channels={channels}, rate={sample_rate}, "
                f"width={sample_width}, duration={duration:.3f}s"
            )

    print(
        "Piper smoke passed: generated a non-silent mono PCM16/16-kHz "
        f"'robot' clip ({duration:.3f} s)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
