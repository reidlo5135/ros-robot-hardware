#!/usr/bin/env bash
set -euo pipefail

TOPIC="/battery_state"
ONCE="false"
HZ="false"

usage() {
  cat <<'USAGE'
Usage: echo_battery_state.sh [--topic TOPIC] [--once] [--hz] [--help]

Echo sensor_msgs/msg/BatteryState from the robot_base_driver OpenCR battery topic.

Options:
  --topic TOPIC   BatteryState topic to inspect (default: /battery_state)
  --once          Print one message and exit
  --hz            Run ros2 topic hz instead of echo
  -h, --help      Show this help

If no messages arrive:
  - Confirm robot_base_driver is running and publishes the topic.
  - Check battery.publish_battery_state=true.
  - Check battery.read_enabled, battery.register_address, and battery.mapping_state.
  - Look for ROBOT_HW_LOG event=battery_config, battery_raw, and battery_state_unavailable.
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
    --hz)
      HZ="true"
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

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source your ROS 2 and workspace setup first." >&2
  exit 1
fi

if [[ "${HZ}" == "true" ]]; then
  exec ros2 topic hz "${TOPIC}"
fi

command=(ros2 topic echo "${TOPIC}" sensor_msgs/msg/BatteryState)
if [[ "${ONCE}" == "true" ]]; then
  command+=(--once)
fi

exec "${command[@]}"
