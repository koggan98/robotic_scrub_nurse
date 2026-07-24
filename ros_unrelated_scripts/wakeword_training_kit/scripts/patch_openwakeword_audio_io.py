#!/usr/bin/env python3
"""Patch openWakeWord v0.6.0 for TorchAudio 2.10's removed audio I/O APIs."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


UPSTREAM_SHA256 = (
    "e6df2fb9f0781ff3cc573c1aa4d3dbbff151d89e42a5a4a76ae988e6b8a5b669"
)
PATCHED_SHA256 = (
    "bb81e61216dd55f6b595822c879ca81538294d8bd1f0b02fcbbaa2ba5c1432e8"
)
MARKER = b"PyTorch 2.10 / TorchAudio 2.10 compatibility"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def patch_data_file(path: Path) -> None:
    """Install a WAV-only SoundFile adapter before legacy audio imports."""
    original = path.read_bytes()
    if MARKER in original:
        if _sha256(original) != PATCHED_SHA256:
            raise RuntimeError(
                "Refusing an altered previously patched openWakeWord "
                f"data.py: {_sha256(original)}"
            )
        print(f"TorchAudio 2.10 audio-I/O patch already present: {path}")
        return

    actual_sha256 = _sha256(original)
    if actual_sha256 != UPSTREAM_SHA256:
        raise RuntimeError(
            "Refusing to patch an unexpected openWakeWord data.py: "
            f"expected SHA-256 {UPSTREAM_SHA256}, got {actual_sha256}"
        )

    newline = b"\r\n" if b"\r\n" in original else b"\n"
    old_lines = [
        b"import torch",
        b"import audiomentations",
        b"import torch_audiomentations",
        b"from numpy.lib.format import open_memmap",
        b"from speechbrain.dataio.dataio import read_audio",
        b"from speechbrain.processing.signal_processing import reverberate",
        b"import torchaudio",
        b"import mutagen",
    ]
    new_lines = [
        b"import torch",
        b"import soundfile as sf",
        b"import torchaudio",
        b"",
        (
            b"# PyTorch 2.10 / TorchAudio 2.10 compatibility: the legacy "
            b"training stack"
        ),
        (
            b"# still calls torchaudio.info/load. All kit inputs are validated "
            b"PCM WAV,"
        ),
        (
            b"# so a small SoundFile adapter avoids TorchCodec/FFmpeg and "
            b"preserves the"
        ),
        b"# tensor layout expected by SpeechBrain and torch-audiomentations.",
        b"class _SoundFileMetadata:",
        b"    def __init__(self, info):",
        b"        self.sample_rate = int(info.samplerate)",
        b"        self.num_frames = int(info.frames)",
        b"        self.num_channels = int(info.channels)",
        (
            b"        self.bits_per_sample = {\"PCM_16\": 16, \"PCM_24\": 24, "
            b"\"PCM_32\": 32}.get(info.subtype, 0)"
        ),
        b"        self.encoding = str(info.subtype)",
        b"",
        b"",
        b"def _soundfile_info(filepath, *args, **kwargs):",
        b"    return _SoundFileMetadata(sf.info(str(filepath)))",
        b"",
        b"",
        b"def _soundfile_load(",
        b"        filepath,",
        b"        frame_offset=0,",
        b"        num_frames=-1,",
        b"        normalize=True,",
        b"        channels_first=True,",
        b"        format=None,",
        b"        buffer_size=4096,",
        b"        backend=None):",
        b"    del format, buffer_size, backend",
        b"    if frame_offset < 0:",
        b'        raise ValueError("frame_offset must be non-negative")',
        b"    dtype = \"float32\"",
        b"    if not normalize:",
        b"        subtype = sf.info(str(filepath)).subtype",
        b"        if subtype != \"PCM_16\":",
        (
            b'            raise ValueError("normalize=False is supported only '
            b'for PCM_16 WAV")'
        ),
        b"        dtype = \"int16\"",
        b"    frames = -1 if num_frames is None or num_frames < 0 else num_frames",
        b"    with sf.SoundFile(str(filepath), mode=\"r\") as stream:",
        b"        stream.seek(frame_offset)",
        b"        audio = stream.read(frames=frames, dtype=dtype, always_2d=True)",
        b"        sample_rate = int(stream.samplerate)",
        b"    if channels_first:",
        b"        audio = audio.T",
        b"    return torch.from_numpy(np.ascontiguousarray(audio)), sample_rate",
        b"",
        b"",
        b"torchaudio.info = _soundfile_info",
        b"torchaudio.load = _soundfile_load",
        b"if not hasattr(torchaudio, \"list_audio_backends\"):",
        b"    torchaudio.list_audio_backends = lambda: [\"soundfile\"]",
        b"if not hasattr(torchaudio, \"set_audio_backend\"):",
        b"    torchaudio.set_audio_backend = lambda backend: None",
        b"",
        b"import audiomentations",
        b"import torch_audiomentations",
        b"from numpy.lib.format import open_memmap",
        b"from speechbrain.dataio.dataio import read_audio",
        b"from speechbrain.processing.signal_processing import reverberate",
        b"import mutagen",
    ]
    old_block = newline.join(old_lines)
    new_block = newline.join(new_lines)
    if original.count(old_block) != 1:
        raise RuntimeError("Expected exactly one upstream audio import block")

    patched = original.replace(old_block, new_block, 1)
    if (
        patched.count(MARKER) != 1
        or _sha256(patched) != PATCHED_SHA256
    ):
        raise RuntimeError("TorchAudio 2.10 audio-I/O patch verification failed")
    path.write_bytes(patched)
    print(f"Applied TorchAudio 2.10 audio-I/O patch: {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "data_file",
        type=Path,
        help="Path to pinned openWakeWord v0.6.0 openwakeword/data.py",
    )
    args = parser.parse_args()
    patch_data_file(args.data_file.resolve())


if __name__ == "__main__":
    main()
