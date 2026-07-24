#!/usr/bin/env python3
"""Check that the selected GPU-Hub environment can run production training."""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import sys
import sysconfig
from pathlib import Path


GIB = 1024**3
SUPPORTED_TORCH = {(2, 10)}
SUPPORTED_CUDA = {"12.6", "12.8"}


def _version_pair(version: str) -> tuple[int, int] | None:
    try:
        major, minor = version.split("+", 1)[0].split(".", 2)[:2]
        return int(major), int(minor)
    except (TypeError, ValueError):
        return None


def _cudnn_major(version: int) -> int:
    # cuDNN 9.10+ uses 9MMmm (for example 91002), while older releases use
    # Mmmpp (for example 8902 for 8.9.2).
    return version // 10_000 if version >= 90_000 else version // 1_000


def _visible_memory_bytes() -> int:
    host_bytes = (
        os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES")
    )
    candidates = [
        Path("/sys/fs/cgroup/memory.max"),
        Path("/sys/fs/cgroup/memory/memory.limit_in_bytes"),
    ]
    limits = [host_bytes]
    for path in candidates:
        try:
            value = path.read_text(encoding="ascii").strip()
            if value != "max":
                limit = int(value)
                if limit > 0:
                    limits.append(limit)
        except (FileNotFoundError, OSError, ValueError):
            continue
    return min(limits)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--require-cuda",
        action="store_true",
        help="Fail unless PyTorch can use an NVIDIA CUDA device.",
    )
    parser.add_argument(
        "--path",
        type=Path,
        default=Path.cwd(),
        help="Persistent filesystem path whose free space/inodes are checked.",
    )
    parser.add_argument(
        "--bootstrap",
        action="store_true",
        help=(
            "Allow missing/mismatched torchaudio before the setup script "
            "installs the exact CUDA-matched 2.10 wheel."
        ),
    )
    args = parser.parse_args()

    errors: list[str] = []
    warnings: list[str] = []

    print("=== Robot wake-word training preflight ===")
    print(f"Python: {sys.version.split()[0]} ({sys.executable})")
    if sys.version_info[:2] != (3, 10):
        errors.append(
            "Select a PyTorch image with Python 3.10; the pinned openWakeWord "
            "0.6.0 training stack is not supported by this kit on another "
            "Python version."
        )
    try:
        import pip
        import venv  # noqa: F401

        pip_pair = _version_pair(pip.__version__)
        print(f"bootstrap pip: {pip.__version__}")
        if pip_pair is None or pip_pair < (22, 3):
            errors.append(
                "Bootstrap pip 22.3 or newer is required to seed the private "
                "virtual environment with pip --python."
            )
    except ImportError as exc:
        errors.append(
            f"Bootstrap Python needs pip and venv modules: {exc}"
        )

    machine = platform.machine().lower()
    print(f"Platform: {platform.system()} {machine}")
    if platform.system() != "Linux" or machine not in {"x86_64", "amd64"}:
        errors.append("Training kit requires x86_64 Linux.")

    check_path = args.path.expanduser().resolve()
    usage = shutil.disk_usage(check_path)
    free_gib = usage.free / GIB
    stat = os.statvfs(check_path)
    free_inodes = stat.f_favail
    print(f"Persistent storage free: {free_gib:.1f} GiB")
    print(f"Persistent inodes free: {free_inodes:,}")
    if free_gib < 45:
        errors.append(
            "At least 45 GiB free storage is required before downloads; "
            "80 GiB or more is recommended."
        )
    elif free_gib < 80:
        warnings.append(
            "Less than 80 GiB is free. Monitor storage during 100k-sample "
            "generation."
        )
    if free_inodes < 300_000:
        errors.append(
            "At least 300,000 free inodes are required for generated clips."
        )

    ram_gib = _visible_memory_bytes() / GIB
    print(f"RAM available to this process: {ram_gib:.1f} GiB")
    if ram_gib < 32:
        errors.append(
            "At least 32 GiB job memory is required because upstream "
            "false-positive validation expands sliding windows in memory."
        )

    affinity_count = (
        len(os.sched_getaffinity(0))
        if hasattr(os, "sched_getaffinity")
        else (os.cpu_count() or 1)
    )
    print(f"CPUs available to this process: {affinity_count}")
    if affinity_count < 2:
        errors.append("At least 2 scheduler-visible CPU cores are required.")
    elif affinity_count < 8:
        warnings.append("8 or more CPU cores are recommended.")

    try:
        import torch
    except ImportError:
        errors.append(
            "PyTorch is missing. Start a PyTorch GPU environment; do not use "
            "the TensorFlow image."
        )
        torch = None

    try:
        import torchaudio
    except (ImportError, OSError) as exc:
        message = (
            "A CUDA-matched torchaudio installation is required: "
            f"{type(exc).__name__}: {exc}"
        )
        (warnings if args.bootstrap else errors).append(message)
        torchaudio = None

    if torch is not None:
        torch_pair = _version_pair(torch.__version__)
        print(
            f"PyTorch: {torch.__version__}; build CUDA: {torch.version.cuda}; "
            f"CUDA available: {torch.cuda.is_available()}"
        )
        if torch_pair not in SUPPORTED_TORCH:
            errors.append(
                "This kit supports the available PyTorch 2.10.x Hub image. "
                f"Selected image provides {torch.__version__}."
            )
        cuda_version = str(torch.version.cuda or "")
        cudnn_version = torch.backends.cudnn.version() or 0
        cudnn_major = _cudnn_major(int(cudnn_version))
        print(f"cuDNN: {cudnn_version}")
        if cuda_version not in SUPPORTED_CUDA:
            errors.append(
                "Select the PyTorch 2.10 CUDA 12.6 or CUDA 12.8 variant; "
                f"detected build CUDA {torch.version.cuda!r}. CUDA 13 is not "
                "supported by this pinned ONNX Runtime setup."
            )
        if cudnn_major != 9:
            errors.append(
                "The PyTorch 2.10 CUDA 12.6/12.8 image must provide cuDNN 9 "
                f"for ONNX Runtime 1.20.1; detected {cudnn_version}."
            )
        if torchaudio is not None:
            print(f"torchaudio: {torchaudio.__version__}")
            if torch_pair != _version_pair(
                torchaudio.__version__
            ):
                message = (
                    "PyTorch and torchaudio major/minor versions do not match."
                )
                (warnings if args.bootstrap else errors).append(message)
            expected_audio_suffix = (
                f"+cu{cuda_version.replace('.', '')}"
                if cuda_version in SUPPORTED_CUDA
                else ""
            )
            if (
                expected_audio_suffix
                and expected_audio_suffix not in torchaudio.__version__
            ):
                message = (
                    "TorchAudio does not match the PyTorch CUDA build: "
                    f"expected suffix {expected_audio_suffix}, got "
                    f"{torchaudio.__version__}."
                )
                (warnings if args.bootstrap else errors).append(message)
        if args.require_cuda and not torch.cuda.is_available():
            errors.append(
                "CUDA is not available to PyTorch. Request a GPU and start "
                "this command inside the allocated GPU job."
            )
        elif torch.cuda.is_available():
            for index in range(torch.cuda.device_count()):
                properties = torch.cuda.get_device_properties(index)
                print(
                    f"GPU {index}: {properties.name}, "
                    f"{properties.total_memory / GIB:.1f} GiB VRAM"
                )
                if properties.total_memory < 8 * GIB:
                    warnings.append(
                        f"GPU {index} has less than 8 GiB VRAM; reduce TTS/"
                        "augmentation batch sizes if out-of-memory occurs."
                    )

    for command in ("git", "curl", "sha256sum", "flock", "gcc"):
        location = shutil.which(command)
        print(f"{command}: {location or 'MISSING'}")
        if location is None:
            errors.append(f"Required command is missing: {command}")

    python_header = (
        Path(sysconfig.get_paths()["include"]) / "Python.h"
    )
    print(f"Python headers: {python_header if python_header.is_file() else 'MISSING'}")
    if not python_header.is_file():
        errors.append(
            "Python development headers are missing; the pinned webrtcvad "
            "dependency cannot be built."
        )

    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}")

    if errors:
        print(f"Preflight failed with {len(errors)} error(s).")
        return 1
    print("Preflight passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
