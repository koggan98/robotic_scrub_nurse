#!/usr/bin/env python3
"""Exercise the PyTorch/TorchAudio 2.10 compatibility boundary."""

from __future__ import annotations

import argparse
import tempfile
import warnings
from pathlib import Path

import numpy as np
import onnx
import soundfile as sf
import torch
import torchaudio

# Load PyTorch's CUDA/cuDNN libraries before ONNX Runtime resolves its CUDA
# provider dependencies.
import onnxruntime as ort


def _version_pair(version: str) -> tuple[int, int]:
    major, minor = version.split("+", 1)[0].split(".", 2)[:2]
    return int(major), int(minor)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--require-cuda",
        action="store_true",
        help="Also require a scheduler-visible CUDA device.",
    )
    args = parser.parse_args()

    if _version_pair(torch.__version__) != (2, 10):
        raise RuntimeError(f"Expected PyTorch 2.10.x, got {torch.__version__}")
    if _version_pair(torchaudio.__version__) != (2, 10):
        raise RuntimeError(
            f"Expected TorchAudio 2.10.x, got {torchaudio.__version__}"
        )
    if str(torch.version.cuda or "") not in {"12.6", "12.8"}:
        raise RuntimeError(
            f"Expected a CUDA 12.6/12.8 PyTorch build, got {torch.version.cuda}"
        )
    if args.require_cuda and not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is unavailable; run setup inside an allocated GPU job."
        )

    # Importing openwakeword.data installs the audited SoundFile adapter before
    # legacy SpeechBrain/torch-audiomentations modules inspect TorchAudio.
    from openwakeword import data as wake_data
    from openwakeword.train import Model as WakeModel
    from speechbrain.processing.signal_processing import reverberate
    import torch_audiomentations

    assert callable(reverberate)
    assert hasattr(torch_audiomentations, "Compose")

    with tempfile.TemporaryDirectory(prefix="robot-wake-compat-") as temp:
        temp_path = Path(temp)
        mono_path = temp_path / "mono.wav"
        stereo_path = temp_path / "stereo.wav"
        background_path = temp_path / "background.wav"
        rir_path = temp_path / "rir.wav"
        phase = np.linspace(0.0, 8.0 * np.pi, 1600, endpoint=False)
        mono = (0.2 * np.sin(phase)).astype(np.float32)
        stereo = np.column_stack((mono, -mono))
        sf.write(mono_path, mono, 16_000, subtype="PCM_16")
        sf.write(stereo_path, stereo, 16_000, subtype="PCM_16")
        background = np.random.default_rng(7).normal(
            0.0,
            0.02,
            48_000,
        ).astype(np.float32)
        impulse = np.zeros(1600, dtype=np.float32)
        impulse[0] = 1.0
        impulse[160] = 0.2
        sf.write(
            background_path,
            background,
            16_000,
            subtype="PCM_16",
        )
        sf.write(rir_path, impulse, 16_000, subtype="PCM_16")

        metadata = torchaudio.info(mono_path)
        if (
            metadata.sample_rate != 16_000
            or metadata.num_frames != 1600
            or metadata.num_channels != 1
            or metadata.bits_per_sample != 16
        ):
            raise RuntimeError(f"Unexpected WAV metadata: {vars(metadata)}")

        mono_tensor, mono_rate = torchaudio.load(mono_path)
        stereo_tensor, stereo_rate = torchaudio.load(
            stereo_path,
            channels_first=False,
        )
        partial_tensor, _ = torchaudio.load(
            mono_path,
            frame_offset=100,
            num_frames=250,
        )
        integer_tensor, _ = torchaudio.load(mono_path, normalize=False)
        if mono_rate != 16_000 or mono_tensor.shape != (1, 1600):
            raise RuntimeError(
                f"Unexpected mono load: {mono_tensor.shape}, {mono_rate}"
            )
        if stereo_rate != 16_000 or stereo_tensor.shape != (1600, 2):
            raise RuntimeError(
                f"Unexpected stereo load: {stereo_tensor.shape}, {stereo_rate}"
            )
        if partial_tensor.shape != (1, 250):
            raise RuntimeError(
                f"Unexpected partial load: {partial_tensor.shape}"
            )
        if integer_tensor.dtype != torch.int16:
            raise RuntimeError(
                f"normalize=False returned {integer_tensor.dtype}"
            )
        if wake_data.read_audio(str(mono_path)).numel() != 1600:
            raise RuntimeError("SpeechBrain read_audio compatibility failed")

        probabilities = {
            "SevenBandParametricEQ": 0.0,
            "TanhDistortion": 0.0,
            "PitchShift": 0.0,
            "BandStopFilter": 0.0,
            "AddColoredNoise": 0.0,
            "AddBackgroundNoise": 1.0,
            "Gain": 0.0,
            "RIR": 1.0,
        }
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", FutureWarning)
            warnings.simplefilter("ignore", UserWarning)
            augmented = next(
                wake_data.augment_clips(
                    [str(mono_path)],
                    total_length=32_000,
                    batch_size=1,
                    augmentation_probabilities=probabilities,
                    background_clip_paths=[str(background_path)],
                    RIR_paths=[str(rir_path)],
                )
            )
        if augmented.shape != (1, 32_000) or augmented.dtype != np.int16:
            raise RuntimeError(
                f"Augmentation returned {augmented.shape} {augmented.dtype}"
            )

        resampled = torchaudio.transforms.Resample(16_000, 8_000)(
            mono_tensor
        )
        if resampled.shape[-1] not in {799, 800, 801}:
            raise RuntimeError(
                f"TorchAudio resampling returned {resampled.shape}"
            )

        torch.manual_seed(7)
        model = WakeModel(
            n_classes=1,
            input_shape=(16, 96),
            layer_dim=8,
            n_blocks=1,
        )
        example = torch.rand((1, 16, 96), dtype=torch.float32)
        utility_path = temp_path / "utility.onnx"
        production_path = temp_path / "production.onnx"
        multiclass_path = temp_path / "multiclass.onnx"
        model.export_to_onnx(utility_path, class_mapping="robot")
        model.export_model(model.model, "production", temp_path)
        multiclass = WakeModel(
            n_classes=2,
            input_shape=(16, 96),
            layer_dim=8,
            n_blocks=1,
        )
        multiclass.export_to_onnx(multiclass_path, class_mapping="scores")
        for exported_path in (
            utility_path,
            production_path,
            multiclass_path,
        ):
            onnx.checker.check_model(onnx.load(exported_path))

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if args.require_cuda
            else ["CPUExecutionProvider"]
        )
        session = ort.InferenceSession(
            str(production_path),
            providers=providers,
        )
        if (
            args.require_cuda
            and session.get_providers()[0] != "CUDAExecutionProvider"
        ):
            raise RuntimeError(
                "Mini ONNX model did not activate CUDAExecutionProvider: "
                f"{session.get_providers()}"
            )
        expected = model.model(example).detach().numpy()
        actual = session.run(
            None,
            {session.get_inputs()[0].name: example.numpy()},
        )[0]
        max_error = float(np.max(np.abs(expected - actual)))
        if max_error > 1e-5:
            raise RuntimeError(
                f"ONNX Runtime differs from PyTorch by {max_error}"
            )

    print(
        "Compatibility smoke passed: "
        f"torch={torch.__version__}, torchaudio={torchaudio.__version__}, "
        f"CUDA build={torch.version.cuda}, ORT={ort.__version__}, "
        f"max ONNX error={max_error:.3g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
