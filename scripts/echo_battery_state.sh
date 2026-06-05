#!/usr/bin/env bash
set -euo pipefail

TOPIC="/battery_state"
ONCE="false"

usage() {
  cat <<'USAGE'
Usage: echo_battery_state.sh [--topic TOPIC] [--once] [--help]

Echo sensor_msgs/msg/BatteryState from the robot_base_driver OpenCR battery topic.
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
    --topic)
      require_value "$1" "${2:-}"
      TOPIC="$2"
      shift 2
      ;;
    --once)
      ONCE="true"
      shift
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

command=(ros2 topic echo "${TOPIC}" sensor_msgs/msg/BatteryState)
if [[ "${ONCE}" == "true" ]]; then
  command+=(--once)
fi

exec "${command[@]}"