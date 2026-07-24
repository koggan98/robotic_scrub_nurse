#!/usr/bin/env bash
# Run locked, fingerprinted openWakeWord stages with durable evidence.

set -Eeuo pipefail

usage() {
  echo "Usage: bash scripts/05_train.sh <smoke|production> <generate|augment|train|all> [--recover-stale]" >&2
}

if [[ "$#" -lt 2 || "$#" -gt 3 ]]; then
  usage
  exit 2
fi

mode="$1"
requested_stage="$2"
recover_stale="false"
if [[ "$#" -eq 3 ]]; then
  if [[ "$3" != "--recover-stale" ]]; then
    usage
    exit 2
  fi
  recover_stale="true"
fi

case "${mode}" in
  smoke)
    config_name="robot.smoke.yaml"
    output_dir="output/robot_smoke"
    model_name="robot_smoke_DO_NOT_DEPLOY"
    ;;
  production)
    config_name="robot.training.yaml"
    output_dir="output/robot_training"
    model_name="robot"
    ;;
  *)
    usage
    exit 2
    ;;
esac

case "${requested_stage}" in
  generate|augment|train)
    stages=("${requested_stage}")
    ;;
  all)
    stages=(generate augment train)
    ;;
  *)
    usage
    exit 2
    ;;
esac

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_python="${kit_root}/.venv/bin/python"
trainer="${kit_root}/work/openWakeWord/openwakeword/train.py"
state_dir="${kit_root}/logs/state"
quarantine_root="${kit_root}/output/quarantine"
model_work_dir="${kit_root}/${output_dir}/${model_name}"
onnx_model="${kit_root}/${output_dir}/${model_name}.onnx"

cd "${kit_root}"
if [[ ! -x "${venv_python}" ]]; then
  echo "Run scripts/01_setup.sh first." >&2
  exit 1
fi

mkdir -p logs "${state_dir}" "${quarantine_root}" .cache/huggingface \
  .cache/torch .cache/xdg
export HF_HOME="${kit_root}/.cache/huggingface"
export TORCH_HOME="${kit_root}/.cache/torch"
export XDG_CACHE_HOME="${kit_root}/.cache/xdg"
export PYTHONUNBUFFERED=1

# One scheduler submission may operate on a mode at a time. This prevents a
# duplicate augmentation job from moving files that another job is writing.
exec 9>"${state_dir}/${mode}.lock"
if ! flock --nonblock 9; then
  echo "Another ${mode} training job currently holds the stage lock." >&2
  exit 1
fi

if [[ "${mode}" == "production" ]] \
  && [[ ! -f logs/input_validation_full.json ]]; then
  echo "Run the full input check before production:" >&2
  echo "  .venv/bin/python scripts/04_validate_inputs.py --config robot.training.yaml" >&2
  exit 1
fi

"${venv_python}" scripts/04_validate_inputs.py \
  --config "${config_name}" --fast

marker_value() {
  local marker="$1"
  local key="$2"
  sed -n "s/^${key}=//p" "${marker}" | tail -n 1
}

fingerprint_for() {
  local stage="$1"
  "${venv_python}" scripts/09_run_fingerprint.py \
    --config "${config_name}" --mode "${mode}" --stage "${stage}"
}

verify_dependency() {
  local stage="$1"
  local marker="${state_dir}/${mode}.${stage}.done"
  local expected
  local actual
  if [[ ! -f "${marker}" ]]; then
    echo "Required stage is not complete: ${mode}/${stage}" >&2
    exit 1
  fi
  expected="$(fingerprint_for "${stage}")"
  actual="$(marker_value "${marker}" fingerprint)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "Inputs changed since ${mode}/${stage} completed." >&2
    echo "Re-run that stage explicitly with --recover-stale." >&2
    exit 1
  fi
}

recover_stage() {
  local stage="$1"
  local stale_fingerprint="$2"
  local current_fingerprint="$3"
  local timestamp
  local quarantine_dir
  local marker
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  quarantine_dir="${quarantine_root}/${mode}_${stage}_${timestamp}"
  mkdir -p "${quarantine_dir}"

  for marker in \
    "${state_dir}/${mode}.${stage}.running" \
    "${state_dir}/${mode}.${stage}.done"; do
    if [[ -f "${marker}" ]]; then
      mv -- "${marker}" "${quarantine_dir}/"
    fi
  done

  case "${stage}" in
    generate)
      if [[ "${stale_fingerprint}" == "${current_fingerprint}" ]]; then
        echo "Keeping partial generated WAVs for a same-input resume."
      elif [[ -d "${model_work_dir}" ]]; then
        mv -- "${model_work_dir}" "${quarantine_dir}/"
      fi
      ;;
    augment)
      shopt -s nullglob
      partial_features=("${model_work_dir}"/*_features_*.npy)
      shopt -u nullglob
      if (( ${#partial_features[@]} > 0 )); then
        mv -- "${partial_features[@]}" "${quarantine_dir}/"
      fi
      ;;
    train)
      if [[ -f "${onnx_model}" ]]; then
        mv -- "${onnx_model}" "${quarantine_dir}/"
      fi
      ;;
  esac
  echo "Preserved stale ${stage} state in ${quarantine_dir}"
}

run_stage() {
  local stage="$1"
  local running_marker="${state_dir}/${mode}.${stage}.running"
  local done_marker="${state_dir}/${mode}.${stage}.done"
  local timestamp
  local log_file
  local flag
  local fingerprint
  local stale_fingerprint=""
  local marker_tmp
  local postcheck_report

  case "${stage}" in
    augment) verify_dependency generate ;;
    train) verify_dependency augment ;;
  esac
  fingerprint="$(fingerprint_for "${stage}")"

  if [[ -f "${done_marker}" ]]; then
    stale_fingerprint="$(marker_value "${done_marker}" fingerprint)"
    if [[ "${stale_fingerprint}" == "${fingerprint}" ]]; then
      echo "Stage already complete with matching inputs: ${mode}/${stage}"
      return
    fi
    if [[ "${recover_stale}" != "true" ]]; then
      echo "Completed ${mode}/${stage} belongs to different inputs." >&2
      echo "Inspect it, then re-run with --recover-stale." >&2
      return 1
    fi
    recover_stage "${stage}" "${stale_fingerprint}" "${fingerprint}"
  fi

  if [[ -f "${running_marker}" ]]; then
    stale_fingerprint="$(marker_value "${running_marker}" fingerprint)"
    if [[ "${recover_stale}" != "true" ]]; then
      echo "A stale/running marker exists: ${running_marker}" >&2
      echo "Confirm the old job is gone, then use --recover-stale." >&2
      return 1
    fi
    recover_stage "${stage}" "${stale_fingerprint}" "${fingerprint}"
  fi

  # An unmarked ONNX file must never survive into a new training attempt.
  if [[ "${stage}" == "train" && -f "${onnx_model}" ]]; then
    if [[ "${recover_stale}" != "true" ]]; then
      echo "Untracked existing model found: ${onnx_model}" >&2
      echo "Re-run with --recover-stale to preserve and replace it." >&2
      return 1
    fi
    recover_stage "${stage}" "" "${fingerprint}"
  fi

  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  log_file="${kit_root}/logs/${mode}_${stage}_${timestamp}.log"
  marker_tmp="${running_marker}.tmp"
  {
    printf 'state=running\n'
    printf 'stage=%s\n' "${stage}"
    printf 'fingerprint=%s\n' "${fingerprint}"
    printf 'started_utc=%s\n' "${timestamp}"
    printf 'host=%s\n' "$(hostname)"
    printf 'pid=%s\n' "$$"
    printf 'slurm_job_id=%s\n' "${SLURM_JOB_ID:-}"
  } > "${marker_tmp}"
  mv -- "${marker_tmp}" "${running_marker}"

  case "${stage}" in
    generate) flag="--generate_clips" ;;
    augment) flag="--augment_clips" ;;
    train) flag="--train_model" ;;
  esac

  echo "Running ${mode}/${stage}; log: ${log_file}"
  if ! "${venv_python}" -u "${trainer}" \
      --training_config "${config_name}" "${flag}" \
      2>&1 | tee "${log_file}"; then
    echo "Stage failed; marker retained: ${running_marker}" >&2
    return 1
  fi

  postcheck_report="logs/${mode}_${stage}_postcheck.json"
  case "${stage}" in
    generate|augment)
      "${venv_python}" scripts/10_stage_postcheck.py \
        --config "${config_name}" --stage "${stage}" \
        --report "${postcheck_report}"
      ;;
    train)
      validation_args=(
        --model "${onnx_model}"
        --report "${postcheck_report}"
      )
      if [[ "${mode}" == "smoke" ]]; then
        validation_args+=(--allow-smoke)
      fi
      "${venv_python}" scripts/06_validate_onnx.py \
        "${validation_args[@]}"
      ;;
  esac

  {
    printf 'state=complete\n'
    printf 'stage=%s\n' "${stage}"
    printf 'fingerprint=%s\n' "${fingerprint}"
    printf 'completed_utc=%s\n' "$(date -u +%Y%m%dT%H%M%SZ)"
    if [[ "${stage}" == "train" ]]; then
      printf 'artifact_sha256=%s\n' \
        "$(sha256sum "${onnx_model}" | awk '{print $1}')"
    fi
  } > "${done_marker}.tmp"
  mv -- "${done_marker}.tmp" "${done_marker}"
  rm -- "${running_marker}"
}

for stage in "${stages[@]}"; do
  run_stage "${stage}"
done

echo "Requested stage(s) complete."
