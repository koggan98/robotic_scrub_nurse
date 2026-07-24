#!/usr/bin/env python3
"""Record privacy-conscious 16-kHz PCM16 background or wake-trial WAVs."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


OUTPUT_RATE = 16000
PROFILE_RATES = {
    "samson": 16000,
    "jieli": 48000,
}


def _dependencies():
    try:
        import sounddevice as sd
        from scipy.io import wavfile
        from scipy.signal import resample_poly
    except ImportError as exc:
        raise SystemExit(
            "Install recording dependencies first: "
            "python3 -m pip install sounddevice scipy"
        ) from exc
    return sd, wavfile, resample_poly


def _resolve_device(
    sd,
    selector: str | None,
) -> tuple[int | None, str, float]:
    devices = sd.query_devices()

    def result(index: int) -> tuple[int, str, float]:
        device = devices[index]
        return (
            index,
            str(device["name"]),
            float(device["default_samplerate"]),
        )

    if selector is None:
        default_index = sd.default.device[0]
        if default_index is None or int(default_index) < 0:
            raise RuntimeError("No default input device; pass --device")
        index = int(default_index)
        return result(index)

    if selector.isdecimal():
        index = int(selector)
        if index < 0 or index >= len(devices):
            raise ValueError(f"Invalid device index: {index}")
        if int(devices[index]["max_input_channels"]) < 1:
            raise ValueError(f"Device {index} has no input channels")
        return result(index)

    matches = [
        (index, str(device["name"]))
        for index, device in enumerate(devices)
        if int(device["max_input_channels"]) >= 1
        and selector.casefold() in str(device["name"]).casefold()
    ]
    if len(matches) != 1:
        names = ", ".join(f"{index}:{name}" for index, name in matches)
        raise ValueError(
            f"Device selector {selector!r} matched {len(matches)} inputs: "
            f"{names or 'none'}"
        )
    index, _ = matches[0]
    return result(index)


def _validate_profile(
    profile: str,
    input_rate: int,
    device_name: str,
    default_rate: float,
) -> None:
    expected_rate = PROFILE_RATES[profile]
    if input_rate != expected_rate:
        raise ValueError(
            f"Profile {profile!r} requires --input-rate {expected_rate}, "
            f"not {input_rate}"
        )
    print(
        f"Input device: {device_name!r}; profile={profile}; "
        f"capture={input_rate} Hz; device default={default_rate:g} Hz"
    )


def _slug(value: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "_", value.casefold()).strip("_")
    return normalized[:48] or "microphone"


def _capture(sd, device, input_rate: int, seconds: float) -> np.ndarray:
    frames = int(round(input_rate * seconds))
    audio = sd.rec(
        frames,
        samplerate=input_rate,
        channels=1,
        dtype="float32",
        device=device,
        blocking=True,
    )
    return np.asarray(audio[:, 0], dtype=np.float64)


def _capture_after_cue(
    sd,
    device,
    input_rate: int,
    seconds: float,
    pre_roll_seconds: float = 0.75,
) -> np.ndarray:
    """Start the stream before the cue so the beginning cannot be clipped."""
    total_frames = int(round(input_rate * seconds))
    pre_roll_frames = min(
        total_frames - 1,
        int(round(input_rate * pre_roll_seconds)),
    )
    with sd.InputStream(
        samplerate=input_rate,
        channels=1,
        dtype="float32",
        device=device,
    ) as stream:
        pre_roll, pre_overflow = stream.read(pre_roll_frames)
        print("JETZT: nur „Robot“ sagen.", flush=True)
        remainder, post_overflow = stream.read(total_frames - pre_roll_frames)
    if pre_overflow or post_overflow:
        print("WARNING: audio input overflow; repeat this trial.")
    audio = np.concatenate((pre_roll[:, 0], remainder[:, 0]))
    return np.asarray(audio, dtype=np.float64)


def _to_pcm16(samples: np.ndarray, input_rate: int, resample_poly) -> np.ndarray:
    if input_rate != OUTPUT_RATE:
        divisor = math.gcd(input_rate, OUTPUT_RATE)
        samples = resample_poly(
            samples,
            OUTPUT_RATE // divisor,
            input_rate // divisor,
        )
    samples = np.clip(samples, -1.0, 1.0)
    scaled = np.where(samples >= 0.0, samples * 32767.0, samples * 32768.0)
    return np.round(scaled).astype(np.int16)


def _statistics(samples: np.ndarray) -> dict[str, float]:
    absolute = np.abs(samples.astype(np.float64))
    rms = float(np.sqrt(np.mean(np.square(samples)))) if samples.size else 0.0
    peak = float(absolute.max()) if samples.size else 0.0

    def dbfs(value: float) -> float:
        return float(20.0 * math.log10(max(value, 1e-12)))

    return {
        "rms": rms,
        "rms_dbfs": dbfs(rms),
        "peak": peak,
        "peak_dbfs": dbfs(peak),
        "dc_offset": float(np.mean(samples)) if samples.size else 0.0,
        "clipped_fraction": (
            float(np.mean(absolute >= 0.99)) if samples.size else 0.0
        ),
    }


def _quality_warnings(stats: dict[str, float], filename: str) -> None:
    if stats["clipped_fraction"] > 0 or stats["peak_dbfs"] > -1.0:
        print(
            f"WARNING: {filename} is clipping or too close to full scale "
            f"(peak {stats['peak_dbfs']:.1f} dBFS)."
        )
    if stats["peak_dbfs"] < -30.0 or stats["rms_dbfs"] < -50.0:
        print(
            f"WARNING: {filename} is very quiet "
            f"(peak {stats['peak_dbfs']:.1f}, RMS "
            f"{stats['rms_dbfs']:.1f} dBFS)."
        )
    if abs(stats["dc_offset"]) > 0.02:
        print(
            f"WARNING: {filename} has a large DC offset "
            f"({stats['dc_offset']:.4f})."
        )


def _write_metadata(
    output_dir: Path,
    session_id: str,
    metadata: dict,
) -> Path:
    metadata_path = (
        output_dir.parent
        / f"{output_dir.name}_{session_id}_recording_metadata.json"
    )
    temporary_path = metadata_path.with_suffix(".json.tmp")
    temporary_path.write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary_path.chmod(0o600)
    temporary_path.replace(metadata_path)
    metadata_path.chmod(0o600)
    return metadata_path


def _record_background(args) -> int:
    if not args.consent_confirmed:
        raise SystemExit(
            "Background recording requires --consent-confirmed. Never record "
            "patients or unconsenting/private conversations."
        )
    sd, wavfile, resample_poly = _dependencies()
    device_index, device_name, default_rate = _resolve_device(sd, args.device)
    _validate_profile(
        args.microphone_profile,
        args.input_rate,
        device_name,
        default_rate,
    )
    sd.check_input_settings(
        device=device_index,
        channels=1,
        samplerate=args.input_rate,
        dtype="float32",
    )

    output_dir = args.output_dir.resolve()
    if "background_clips" in output_dir.parts:
        raise ValueError(
            "Record into a review folder first; direct recording below "
            "data/background_clips is forbidden."
        )
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    output_dir.chmod(0o700)
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    chunk_count = int(math.ceil(args.minutes * 60.0 / args.chunk_seconds))
    remaining = args.minutes * 60.0
    records = []
    metadata = {
        "mode": "background",
        "state": "recording",
        "session_id": session_id,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "device_index": device_index,
        "device_name": device_name,
        "microphone_profile": args.microphone_profile,
        "capture_input_rate_hz": args.input_rate,
        "device_reported_default_rate_hz": default_rate,
        "file_prefix": _slug(args.prefix),
        "output_rate_hz": OUTPUT_RATE,
        "output_format": "mono PCM16 WAV",
        "consent_confirmed": True,
        "files": records,
    }
    metadata_path = _write_metadata(output_dir, session_id, metadata)
    print(
        "Recording starts now. Do not say 'Robot' in any context or form "
        "(including 'the robot', 'robots', 'robotic' or commands) during "
        "training-background sessions."
    )
    try:
        for index in range(chunk_count):
            duration = min(args.chunk_seconds, remaining)
            if duration <= 0:
                break
            print(f"Chunk {index + 1}/{chunk_count}: {duration:.1f} s")
            samples = _capture(sd, device_index, args.input_rate, duration)
            pcm16 = _to_pcm16(samples, args.input_rate, resample_poly)
            output_samples = pcm16.astype(np.float64) / 32768.0
            stats = _statistics(output_samples)
            filename = (
                f"{_slug(args.prefix)}_{_slug(device_name)}_"
                f"{session_id}_{index:04d}.wav"
            )
            output_path = output_dir / filename
            wavfile.write(output_path, OUTPUT_RATE, pcm16)
            output_path.chmod(0o600)
            records.append(
                {
                    "file": filename,
                    "duration_seconds": len(pcm16) / OUTPUT_RATE,
                    **stats,
                }
            )
            _quality_warnings(stats, filename)
            remaining -= duration
            metadata["updated_utc"] = datetime.now(timezone.utc).isoformat()
            _write_metadata(output_dir, session_id, metadata)
    except BaseException:
        metadata["state"] = "interrupted"
        metadata["updated_utc"] = datetime.now(timezone.utc).isoformat()
        _write_metadata(output_dir, session_id, metadata)
        raise
    metadata["state"] = "complete"
    metadata["completed_utc"] = datetime.now(timezone.utc).isoformat()
    _write_metadata(output_dir, session_id, metadata)
    print(f"Wrote {len(records)} WAV files to {output_dir}")
    print(f"Metadata (kept outside WAV-only folder): {metadata_path}")
    return 0


def _record_wake_trials(args) -> int:
    if not args.consent_confirmed:
        raise SystemExit(
            "Wake-trial recording requires --consent-confirmed. Obtain "
            "consent from every recorded speaker."
        )
    sd, wavfile, resample_poly = _dependencies()
    device_index, device_name, default_rate = _resolve_device(sd, args.device)
    _validate_profile(
        args.microphone_profile,
        args.input_rate,
        device_name,
        default_rate,
    )
    sd.check_input_settings(
        device=device_index,
        channels=1,
        samplerate=args.input_rate,
        dtype="float32",
    )
    output_dir = args.output_dir.resolve()
    if "background_clips" in output_dir.parts:
        raise ValueError(
            "Wake trials must never be written below data/background_clips"
        )
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    output_dir.chmod(0o700)
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    records = []
    metadata = {
        "mode": "wake_trials",
        "state": "recording",
        "session_id": session_id,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "device_index": device_index,
        "device_name": device_name,
        "microphone_profile": args.microphone_profile,
        "capture_input_rate_hz": args.input_rate,
        "device_reported_default_rate_hz": default_rate,
        "output_rate_hz": OUTPUT_RATE,
        "output_format": "mono PCM16 WAV",
        "consent_confirmed": True,
        "files": records,
    }
    metadata_path = _write_metadata(output_dir, session_id, metadata)

    print(
        "These clips are for calibration/acceptance only. Do not copy them "
        "into data/background_clips."
    )
    try:
        for index in range(args.count):
            input(
                f"Trial {index + 1}/{args.count}: Enter drücken, dann bis "
                "zum Hinweis „JETZT“ warten. "
            )
            for remaining in (3, 2, 1):
                print(remaining, flush=True)
                time.sleep(1.0)
            samples = _capture_after_cue(
                sd,
                device_index,
                args.input_rate,
                args.seconds,
            )
            pcm16 = _to_pcm16(samples, args.input_rate, resample_poly)
            output_samples = pcm16.astype(np.float64) / 32768.0
            stats = _statistics(output_samples)
            filename = (
                f"wake_{_slug(device_name)}_{session_id}_{index:03d}.wav"
            )
            output_path = output_dir / filename
            wavfile.write(output_path, OUTPUT_RATE, pcm16)
            output_path.chmod(0o600)
            records.append(
                {
                    "file": filename,
                    "duration_seconds": len(pcm16) / OUTPUT_RATE,
                    **stats,
                }
            )
            _quality_warnings(stats, filename)
            print(
                f"Saved {filename}; peak={stats['peak_dbfs']:.1f} dBFS"
            )
            metadata["updated_utc"] = datetime.now(timezone.utc).isoformat()
            _write_metadata(output_dir, session_id, metadata)
    except BaseException:
        metadata["state"] = "interrupted"
        metadata["updated_utc"] = datetime.now(timezone.utc).isoformat()
        _write_metadata(output_dir, session_id, metadata)
        raise
    metadata["state"] = "complete"
    metadata["completed_utc"] = datetime.now(timezone.utc).isoformat()
    _write_metadata(output_dir, session_id, metadata)
    print(f"Wrote {len(records)} wake trials to {output_dir}")
    print(f"Metadata: {metadata_path}")
    return 0


def _level_check(args) -> int:
    sd, _, resample_poly = _dependencies()
    device_index, device_name, default_rate = _resolve_device(sd, args.device)
    _validate_profile(
        args.microphone_profile,
        args.input_rate,
        device_name,
        default_rate,
    )
    sd.check_input_settings(
        device=device_index,
        channels=1,
        samplerate=args.input_rate,
        dtype="float32",
    )
    print(
        f"Speak at the intended distance and volume for {args.seconds:g} s."
    )
    samples = _capture(sd, device_index, args.input_rate, args.seconds)
    pcm16 = _to_pcm16(samples, args.input_rate, resample_poly)
    stats = _statistics(pcm16.astype(np.float64) / 32768.0)
    print(json.dumps(stats, indent=2))
    _quality_warnings(stats, "level check")
    if -12.0 <= stats["peak_dbfs"] <= -6.0:
        print("Level looks good: peak is in the recommended -12 to -6 dBFS range.")
        return 0
    print(
        "Adjust microphone gain/distance and repeat; target peak is "
        "-12 to -6 dBFS without clipping."
    )
    return 2


def main() -> int:
    os.umask(0o077)
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List audio devices")

    level = subparsers.add_parser(
        "level-check",
        help="Run a private 10-second level check without saving audio",
    )
    level.add_argument("--device")
    level.add_argument(
        "--microphone-profile",
        choices=tuple(PROFILE_RATES),
        required=True,
    )
    level.add_argument(
        "--input-rate",
        type=int,
        choices=(16000, 48000),
        required=True,
    )
    level.add_argument("--seconds", type=float, default=10.0)

    background = subparsers.add_parser(
        "background",
        help="Record chunked background audio",
    )
    background.add_argument("--device")
    background.add_argument(
        "--microphone-profile",
        choices=tuple(PROFILE_RATES),
        required=True,
    )
    background.add_argument("--input-rate", type=int, choices=(16000, 48000), required=True)
    background.add_argument("--minutes", type=float, default=60.0)
    background.add_argument("--chunk-seconds", type=float, default=20.0)
    background.add_argument("--prefix", default="bgtrain")
    background.add_argument("--output-dir", type=Path, required=True)
    background.add_argument("--consent-confirmed", action="store_true")

    wake = subparsers.add_parser(
        "wake-trials",
        help="Interactively record isolated Robot trials",
    )
    wake.add_argument("--device")
    wake.add_argument(
        "--microphone-profile",
        choices=tuple(PROFILE_RATES),
        required=True,
    )
    wake.add_argument("--input-rate", type=int, choices=(16000, 48000), required=True)
    wake.add_argument("--count", type=int, default=20)
    wake.add_argument("--seconds", type=float, default=3.0)
    wake.add_argument("--output-dir", type=Path, required=True)
    wake.add_argument("--consent-confirmed", action="store_true")

    args = parser.parse_args()
    if args.command == "list":
        sd, _, _ = _dependencies()
        print(sd.query_devices())
        return 0
    if args.command == "level-check":
        if args.seconds <= 0:
            parser.error("--seconds must be positive")
        return _level_check(args)
    if args.command == "background":
        if args.minutes <= 0 or args.chunk_seconds <= 0:
            parser.error("--minutes and --chunk-seconds must be positive")
        return _record_background(args)
    if args.count <= 0 or args.seconds < 1.5:
        parser.error("--count must be positive and --seconds at least 1.5")
    return _record_wake_trials(args)


if __name__ == "__main__":
    raise SystemExit(main())
