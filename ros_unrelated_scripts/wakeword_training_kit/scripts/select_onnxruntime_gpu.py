#!/usr/bin/env python3
"""Select an ONNX Runtime build compatible with the Hub PyTorch image."""

from __future__ import annotations

import sys

import torch


def _cudnn_major(version: int) -> int:
    """Decode both legacy (8902) and cuDNN-9.10+ (91002) integers."""
    return version // 10_000 if version >= 90_000 else version // 1_000


def main() -> int:
    cuda_version = torch.version.cuda or ""
    cudnn_version = torch.backends.cudnn.version() or 0
    cudnn_major = _cudnn_major(int(cudnn_version))

    if cuda_version not in {"12.6", "12.8"}:
        print(
            "This kit expects a PyTorch 2.10 CUDA 12.6/12.8 image. Select "
            f"such an image in the GPU Hub; detected torch CUDA "
            f"{cuda_version or 'none'}.",
            file=sys.stderr,
        )
        return 1

    # Official PyTorch 2.10 CUDA 12.6/12.8 wheels use cuDNN 9. ORT 1.20.1
    # is the pinned CUDA-12/cuDNN-9 runtime validated by this kit.
    if cudnn_major == 9:
        print("onnxruntime-gpu==1.20.1")
        return 0

    print(
        f"Unsupported cuDNN version reported by PyTorch: {cudnn_version}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
