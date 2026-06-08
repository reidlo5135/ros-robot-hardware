#!/usr/bin/env bash
set -euo pipefail

CONTRACT_FILE=""
SUMMARY_PERIOD_SEC="2.0"
SUMMARY_PERIOD_SET="false"
LOG_LEVEL="info"

usage() {
  cat <<'USAGE'
Usage: robot_diag_watch.sh [--contract-file PATH] [--summary-period SEC] [--log-level LEVEL] [SUMMARY_PERIOD_SEC] [--help]

Run robot_diagnostics as a live monitor until interrupted.

Options:
  --contract-file PATH   Contract YAML path. Default: package tb3_contract.yaml.
  --summary-period SEC   Seconds between periodic summaries. Default: 2.0.
  --log-level LEVEL      ROS log level for robot_diagnostics_node. Default: info.
  -h, --help             Show this help.

Arguments:
  SUMMARY_PERIOD_SEC     Optional shorthand for --summary-period.

The node checks /tf, /tf_static, /scan, /odom, /imu, /joint_states, and /cmd_vel graph state.
Use robot_diag_snapshot.sh for one-shot snapshot/CI/report runs that exit after one summary.
USAGE
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "ERROR: ${option} requires a value." >&2
    exit 2
  fi
}

set_summary_period() {
  local value="$1"
  if [[ "${SUMMARY_PERIOD_SET}" == "true" ]]; then
    echo "ERROR: summary period was provided more than once." >&2
    exit 2
  fi
  SUMMARY_PERIOD_SEC="${value}"
  SUMMARY_PERIOD_SET="true"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --contract-file)
      require_value "$1" "${2:-}"
      CONTRACT_FILE="$2"
      shift 2
      ;;
    --summary-period)
      require_value "$1" "${2:-}"
      set_summary_period "$2"
      shift 2
      ;;
    --log-level)
      require_value "$1" "${2:-}"
      LOG_LEVEL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      set_summary_period "$1"
      shift
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source ROS 2 and the workspace before running this script." >&2
  exit 1
fi

declare -a launch_args=(
  "once:=false"
  "summary_period_sec:=${SUMMARY_PERIOD_SEC}"
  "log_level:=${LOG_LEVEL}"
)

if [[ -n "${CONTRACT_FILE}" ]]; then
  launch_args+=("contract_file:=${CONTRACT_FILE}")
fi

exec ros2 launch robot_diagnostics diagnostics.launch.py "${launch_args[@]}"