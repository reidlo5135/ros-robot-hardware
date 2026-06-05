#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: run_robot_hw_nohup.sh [ros2 launch args...]

Run robot_bringup robot.launch.py under nohup and store logs.

Environment variables:
  ROBOT_HW_LOG_DIR       Log directory. Default: ~/ws/logs/robot_hw
  ROBOT_HW_WS            Workspace root to source. Default: ~/ws
  ROS_DOMAIN_ID          Passed through when already set.
  RMW_IMPLEMENTATION     Passed through when already set.

Examples:
  ./scripts/run_robot_hw_nohup.sh
  ./scripts/run_robot_hw_nohup.sh debug_tf:=true debug_odom:=true
  ROBOT_HW_LOG_DIR=/tmp/robot_hw_logs ./scripts/run_robot_hw_nohup.sh debug_scan_geometry:=true
USAGE
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v nohup >/dev/null 2>&1; then
  echo "nohup command not found" >&2
  exit 1
fi

ROBOT_HW_LOG_DIR="${ROBOT_HW_LOG_DIR:-${HOME}/ws/logs/robot_hw}"
ROBOT_HW_WS="${ROBOT_HW_WS:-${HOME}/ws}"
mkdir -p "$ROBOT_HW_LOG_DIR"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${ROBOT_HW_LOG_DIR}/robot_hw_${TIMESTAMP}.log"
LATEST_LINK="${ROBOT_HW_LOG_DIR}/latest.log"

COMMAND=(ros2 launch robot_bringup robot.launch.py "$@")

if [[ -f "${ROBOT_HW_WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROBOT_HW_WS}/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS 2 and the workspace before running this script." >&2
  exit 1
fi

ln -sfn "$LOG_FILE" "$LATEST_LINK"

echo "Starting robot hardware bringup under nohup"
echo "Log file: $LOG_FILE"
echo "Latest link: $LATEST_LINK"
echo "Command: ${COMMAND[*]}"

nohup "${COMMAND[@]}" > "$LOG_FILE" 2>&1 &
PID=$!

echo "$PID" > "${ROBOT_HW_LOG_DIR}/robot_hw_${TIMESTAMP}.pid"
echo "Started PID: $PID"
echo "Follow logs: ./scripts/watch_robot_hw_logs.sh --follow --file $LATEST_LINK"
