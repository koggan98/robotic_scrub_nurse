#!/usr/bin/env bash
# Download immutable training assets with resume, byte-count, and SHA checks.

set -Eeuo pipefail

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_python="${kit_root}/.venv/bin/python"
openwakeword_models="${kit_root}/work/openWakeWord/openwakeword/resources/models"
piper_models="${kit_root}/work/piper-sample-generator/models"
feature_revision="2b54ab75af92aac3a2c49f82ba58c1daae350563"

cd "${kit_root}"

if [[ ! -x "${venv_python}" ]]; then
  echo "Run scripts/01_setup.sh first." >&2
  exit 1
fi

mkdir -p "${openwakeword_models}" "${piper_models}" "${kit_root}/data"

sha256_of() {
  sha256sum "$1" | awk '{print $1}'
}

download_checked() {
  local url="$1"
  local destination="$2"
  local expected_bytes="$3"
  local expected_sha256="$4"
  local actual_bytes
  local actual_sha256

  if [[ -f "${destination}" ]]; then
    actual_bytes="$(stat --format='%s' "${destination}")"
    actual_sha256="$(sha256_of "${destination}")"
    if [[ "${actual_bytes}" == "${expected_bytes}" ]] \
      && [[ "${actual_sha256}" == "${expected_sha256}" ]]; then
      echo "Verified existing asset: ${destination}"
      return
    fi
    if (( actual_bytes >= expected_bytes )); then
      echo "Existing full-size/oversize asset has the wrong checksum." >&2
      echo "Move it aside and rerun: ${destination}" >&2
      exit 1
    fi
    echo "Existing partial asset will be resumed: ${destination}"
    curl -fL --retry 5 --retry-all-errors --continue-at - \
      "${url}" --output "${destination}"
  else
    curl -fL --retry 5 --retry-all-errors \
      "${url}" --output "${destination}"
  fi

  actual_bytes="$(stat --format='%s' "${destination}")"
  actual_sha256="$(sha256_of "${destination}")"
  if [[ "${actual_bytes}" != "${expected_bytes}" ]] \
    || [[ "${actual_sha256}" != "${expected_sha256}" ]]; then
    echo "Asset verification failed: ${destination}" >&2
    echo "Expected bytes/SHA: ${expected_bytes} ${expected_sha256}" >&2
    echo "Actual bytes/SHA:   ${actual_bytes} ${actual_sha256}" >&2
    echo "Move the bad file aside and rerun this script." >&2
    exit 1
  fi
  echo "Downloaded and verified: ${destination}"
}

download_checked \
  "https://github.com/rhasspy/piper-sample-generator/releases/download/v2.0.0/en_US-libritts_r-medium.pt" \
  "${piper_models}/en_US-libritts_r-medium.pt" \
  "204089915" \
  "e95ee53770bf598c354a6e6dbfc95ccb259aeeb501d35a86be8a767429ab0ff6"

"${venv_python}" scripts/patch_piper_for_torch26.py \
  "${kit_root}/work/piper-sample-generator/generate_samples.py" \
  "${piper_models}/en_US-libritts_r-medium.pt"
"${venv_python}" scripts/12_piper_smoke.py \
  "${kit_root}/work/piper-sample-generator"

download_checked \
  "https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/embedding_model.onnx" \
  "${openwakeword_models}/embedding_model.onnx" \
  "1326578" \
  "70d164290c1d095d1d4ee149bc5e00543250a7316b59f31d056cff7bd3075c1f"

download_checked \
  "https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/melspectrogram.onnx" \
  "${openwakeword_models}/melspectrogram.onnx" \
  "1087958" \
  "ba2b0e0f8b7b875369a2c89cb13360ff53bac436f2895cced9f479fa65eb176f"

download_checked \
  "https://huggingface.co/datasets/davidscripka/openwakeword_features/resolve/${feature_revision}/openwakeword_features_ACAV100M_2000_hrs_16bit.npy" \
  "${kit_root}/data/openwakeword_features_ACAV100M_2000_hrs_16bit.npy" \
  "17280000128" \
  "721a66d0682c65a1b5c1da0aa109409cede1d20e28b15235c344b000cbb7654f"

download_checked \
  "https://huggingface.co/datasets/davidscripka/openwakeword_features/resolve/${feature_revision}/validation_set_features.npy" \
  "${kit_root}/data/validation_set_features.npy" \
  "184836608" \
  "a56a8a0f8e0efb91900acc6de4c0cdf4c564842e8475a7d49b36c039e17a690f"

echo
echo "All immutable assets are present and verified."
echo "Next: ${venv_python} scripts/03_prepare_rirs.py"
