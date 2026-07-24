#!/usr/bin/env bash
# Package a fingerprint-bound production ONNX candidate and its evidence.

set -Eeuo pipefail

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_python="${kit_root}/.venv/bin/python"
model="${kit_root}/output/robot_training/robot.onnx"
done_marker="${kit_root}/logs/state/production.train.done"
report="${kit_root}/logs/onnx_validation.json"
fingerprint_report="${kit_root}/logs/production_train_fingerprint.json"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
candidate_dir="${kit_root}/results/robot_candidate_${timestamp}"
archive="${kit_root}/results/robot_candidate_${timestamp}.tar.gz"
archive_checksum="${archive}.sha256"

cd "${kit_root}"
if [[ ! -x "${venv_python}" ]]; then
  echo "Training environment is missing; run scripts/01_setup.sh." >&2
  exit 1
fi
if [[ ! -f "${model}" ]]; then
  echo "Production model is missing: ${model}" >&2
  exit 1
fi
if [[ ! -f "${done_marker}" ]]; then
  echo "Refusing export without a successful production train marker." >&2
  exit 1
fi
if [[ ! -f logs/input_validation_full.json ]]; then
  echo "Refusing export without the full asset/input validation report." >&2
  exit 1
fi

expected_fingerprint="$("${venv_python}" scripts/09_run_fingerprint.py \
  --config robot.training.yaml --mode production --stage train \
  --report "${fingerprint_report}")"
recorded_fingerprint="$(sed -n 's/^fingerprint=//p' "${done_marker}" | tail -n 1)"
if [[ "${recorded_fingerprint}" != "${expected_fingerprint}" ]]; then
  echo "Current inputs do not match the completed production training run." >&2
  exit 1
fi

expected_model_sha="$(sed -n 's/^artifact_sha256=//p' "${done_marker}" | tail -n 1)"
actual_model_sha="$(sha256sum "${model}" | awk '{print $1}')"
if [[ -z "${expected_model_sha}" || "${actual_model_sha}" != "${expected_model_sha}" ]]; then
  echo "Model hash does not match the completed production training run." >&2
  exit 1
fi

"${venv_python}" scripts/06_validate_onnx.py \
  --model "${model}" --report "${report}"

if [[ -e "${candidate_dir}" ]] \
  || [[ -e "${archive}" ]] \
  || [[ -e "${archive_checksum}" ]]; then
  echo "Candidate output already exists for timestamp ${timestamp}; retry." >&2
  exit 1
fi
mkdir -p "${candidate_dir}/logs"
install -m 0644 "${model}" "${candidate_dir}/robot.onnx"
install -m 0644 robot.training.yaml "${candidate_dir}/robot.training.yaml"
install -m 0644 requirements-onnx-training.txt \
  "${candidate_dir}/requirements-onnx-training.txt"
install -m 0644 VERSION "${candidate_dir}/KIT_VERSION"
install -m 0644 "${report}" "${candidate_dir}/onnx_validation.json"
install -m 0644 "${fingerprint_report}" \
  "${candidate_dir}/training_fingerprint.json"
install -m 0644 "${done_marker}" \
  "${candidate_dir}/production.train.done"
install -m 0644 manifests/assets.json "${candidate_dir}/assets.json"
install -m 0644 manifests/source-provenance.json \
  "${candidate_dir}/source-provenance.json"

shopt -s nullglob
evidence=(
  logs/*.log
  logs/input_validation_*.json
  logs/evaluation_*.json
  logs/*_postcheck.json
)
shopt -u nullglob
if (( ${#evidence[@]} > 0 )); then
  cp -a -- "${evidence[@]}" "${candidate_dir}/logs/"
fi

{
  printf 'state=candidate_unvalidated\n'
  printf 'created_utc=%s\n' "${timestamp}"
  printf 'training_fingerprint=%s\n' "${expected_fingerprint}"
  printf 'model_sha256=%s\n' "${actual_model_sha}"
  printf 'openwakeword_commit=%s\n' \
    "$(git -C work/openWakeWord rev-parse HEAD)"
  printf 'piper_commit=%s\n' \
    "$(git -C work/piper-sample-generator rev-parse HEAD)"
  printf 'python=%s\n' "$("${venv_python}" --version 2>&1)"
} > "${candidate_dir}/PROVENANCE.txt"

"${venv_python}" -m pip freeze > "${candidate_dir}/pip-freeze.txt"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi > "${candidate_dir}/nvidia-smi.txt"
fi

(
  cd "${candidate_dir}"
  find . -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum > SHA256SUMS
)
tar -czf "${archive}" -C "${kit_root}/results" \
  "$(basename "${candidate_dir}")"
(
  cd "${kit_root}/results"
  sha256sum "$(basename "${archive}")" \
    > "$(basename "${archive_checksum}")"
)

echo "Candidate archive and sidecar checksum created:"
echo "${archive}"
echo "${archive_checksum}"
echo
echo "It is still unvalidated. Bring this archive back for Samson/Jieli tests."
