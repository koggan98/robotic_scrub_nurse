#!/usr/bin/env bash
# Prepare a private Python environment and pinned upstream source trees.

set -Eeuo pipefail

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bootstrap="${PYTHON_BOOTSTRAP:-python3}"
venv_python="${kit_root}/.venv/bin/python"
openwakeword_dir="${kit_root}/work/openWakeWord"
piper_dir="${kit_root}/work/piper-sample-generator"
openwakeword_commit="c8ef6912c5feccf1037b852d9bc6c7ed644135ba"
piper_commit="195e3bd967d54589c2137c9de2b22ad526ba6b6f"

cd "${kit_root}"
"${python_bootstrap}" scripts/00_preflight.py \
  --require-cuda --bootstrap --path "${kit_root}"

if [[ ! -x "${venv_python}" ]]; then
  # --without-pip also works in lean Hub images that omit the distro's
  # python3-venv/ensurepip package.
  "${python_bootstrap}" -m venv --without-pip --system-site-packages \
    "${kit_root}/.venv"
fi

if ! "${venv_python}" -c \
  'from pathlib import Path; import pip, sys; assert Path(pip.__file__).resolve().is_relative_to(Path(sys.prefix).resolve())' \
  >/dev/null 2>&1; then
  "${python_bootstrap}" -m pip --python "${kit_root}/.venv" install \
    --ignore-installed pip setuptools wheel
fi
"${venv_python}" -m pip install --upgrade pip setuptools wheel
torch_cuda="$("${venv_python}" -c 'import torch; print(torch.version.cuda or "")')"
case "${torch_cuda}" in
  12.6) torchaudio_index="cu126" ;;
  12.8) torchaudio_index="cu128" ;;
  *)
    echo "Unsupported PyTorch CUDA build: ${torch_cuda}" >&2
    exit 1
    ;;
esac
"${venv_python}" -m pip install --ignore-installed --no-deps \
  --index-url "https://download.pytorch.org/whl/${torchaudio_index}" \
  "torchaudio==2.10.0"
"${venv_python}" -m pip install -r requirements-onnx-training.txt
ort_requirement="$("${venv_python}" scripts/select_onnxruntime_gpu.py)"
"${venv_python}" -m pip install "${ort_requirement}"
"${venv_python}" scripts/00_preflight.py --require-cuda --path "${kit_root}"

mkdir -p work data/background_clips data/mit_rirs \
  data/recording_metadata data/calibration recordings logs output results

if [[ ! -d "${openwakeword_dir}/.git" ]]; then
  git clone --branch v0.6.0 --depth 1 \
    https://github.com/dscripka/openWakeWord.git "${openwakeword_dir}"
fi
actual_openwakeword_commit="$(git -C "${openwakeword_dir}" rev-parse HEAD)"
if [[ "${actual_openwakeword_commit}" != "${openwakeword_commit}" ]]; then
  echo "Unexpected openWakeWord commit: ${actual_openwakeword_commit}" >&2
  exit 1
fi

if [[ ! -d "${piper_dir}/.git" ]]; then
  git clone --branch v2.0.0 --depth 1 \
    https://github.com/rhasspy/piper-sample-generator.git "${piper_dir}"
fi
actual_piper_commit="$(git -C "${piper_dir}" rev-parse HEAD)"
if [[ "${actual_piper_commit}" != "${piper_commit}" ]]; then
  echo "Unexpected piper-sample-generator commit: ${actual_piper_commit}" >&2
  exit 1
fi

"${venv_python}" scripts/patch_openwakeword_onnx_only.py \
  "${openwakeword_dir}/openwakeword/train.py"
"${venv_python}" scripts/patch_openwakeword_audio_io.py \
  "${openwakeword_dir}/openwakeword/data.py"
"${venv_python}" -m pip install --no-deps --editable "${openwakeword_dir}"

"${venv_python}" scripts/11_compatibility_smoke.py --require-cuda
"${venv_python}" -c \
  "import torch, torchaudio, onnxruntime as ort; assert torch.cuda.is_available(); assert 'CUDAExecutionProvider' in ort.get_available_providers(); print('CUDA setup OK:', torch.__version__, torchaudio.__version__, torch.cuda.get_device_name(0), ort.get_available_providers())"

echo
echo "Setup complete."
echo "Next: bash scripts/02_download_assets.sh"
