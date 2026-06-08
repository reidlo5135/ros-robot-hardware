#!/usr/bin/env bash
set -euo pipefail

CONTRACT_FILE=""
SUMMARY_PERIOD_SEC="5.0"
LOG_LEVEL="info"

usage() {
  cat <<'USAGE'
Usage: robot_diag_snapshot.sh [--contract-file PATH] [--summary-period SEC] [--log-level LEVEL] [--help]

Run robot_diagnostics once against the robot_hardware ROS contract.

Options:
  --contract-file PATH   Contract YAML path. Default: package tb3_contract.yaml.
  --summary-period SEC   Seconds to collect data before summarizing. Default: 5.0.
  --log-level LEVEL      ROS log level for robot_diagnostics_node. Default: info.
  -h, --help             Show this help.

The node checks /tf, /tf_static, /scan, /odom, /imu, /joint_states, and /cmd_vel graph state.
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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --contract-file)
      require_value "$1" "${2:-}"
      CONTRACT_FILE="$2"
      shift 2
      ;;
    --summary-period)
      require_value "$1" "${2:-}"
      SUMMARY_PERIOD_SEC="$2"
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
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source ROS 2 and the workspace before running this script." >&2
  exit 1
fi

declare -a launch_args=(
  "once:=true"
  "summary_period_sec:=${SUMMARY_PERIOD_SEC}"
  "log_level:=${LOG_LEVEL}"
)

if [[ -n "${CONTRACT_FILE}" ]]; then
  launch_args+=("contract_file:=${CONTRACT_FILE}")
fi

exec ros2 launch robot_diagnostics diagnostics.launch.py "${launch_args[@]}"