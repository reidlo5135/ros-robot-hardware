#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: check_tf_chain.sh [--timeout SEC]

Run basic tf2 checks for robot hardware frames.

Options:
  --timeout SEC   Seconds to wait for each tf2_echo command. Default: 5.
  -h, --help      Show this help.

Checks:
  odom -> base_footprint          robot_base_driver dynamic TF
  base_footprint -> base_link     robot_state_publisher static TF
  base_link -> base_scan          robot_state_publisher static TF
  base_link -> imu_link           robot_state_publisher static TF

Note:
  map -> odom is not expected from robot_hardware. It belongs to localization/navigation.
USAGE
}

TIMEOUT_SEC=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)
      [[ $# -ge 2 ]] || { echo "--timeout requires a value" >&2; exit 2; }
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS 2 and the workspace before running this script." >&2
  exit 1
fi

check_tf() {
  local parent_frame="$1"
  local child_frame="$2"
  local owner="$3"

  echo "== Checking ${parent_frame} -> ${child_frame} (${owner}) =="
  if timeout "${TIMEOUT_SEC}s" ros2 run tf2_ros tf2_echo "$parent_frame" "$child_frame"; then
    echo "OK: ${parent_frame} -> ${child_frame}"
  else
    echo "WARN: ${parent_frame} -> ${child_frame} not available within ${TIMEOUT_SEC}s"
  fi
  echo
}

echo "robot_hardware TF expectation"
echo "- map -> odom: external localization/navigation, not robot_hardware"
echo "- odom -> base_footprint: robot_base_driver when publish_tf=true"
echo "- static frames: robot_state_publisher from robot_description"
echo

check_tf odom base_footprint robot_base_driver
check_tf base_footprint base_link robot_state_publisher
check_tf base_link base_scan robot_state_publisher
check_tf base_link imu_link robot_state_publisher
