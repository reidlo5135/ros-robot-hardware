#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: record_robot_hw_bag_light.sh [--output PATH] [--duration SEC]

Record a lightweight robot hardware bag.

Options:
  --output PATH   Output bag path. Default: ~/ws/bags/robot_hw_light_YYYYmmdd_HHMMSS
  --duration SEC  Stop recording after SEC seconds using timeout.
  -h, --help      Show this help.

Recorded topics:
  /scan /odom /imu /joint_states /tf /tf_static /cmd_vel
  /cmd_vel_stamped is recorded when the topic is present.
USAGE
}

OUTPUT_PATH="${HOME}/ws/bags/robot_hw_light_$(date +%Y%m%d_%H%M%S)"
DURATION_SEC=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      [[ $# -ge 2 ]] || { echo "--output requires a path" >&2; exit 2; }
      OUTPUT_PATH="$2"
      shift 2
      ;;
    --duration)
      [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
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

mkdir -p "$(dirname "$OUTPUT_PATH")"

TOPICS=(/scan /odom /imu /joint_states /tf /tf_static /cmd_vel)
if ros2 topic list 2>/dev/null | grep -qx '/cmd_vel_stamped'; then
  TOPICS+=(/cmd_vel_stamped)
fi

echo "Recording robot hardware bag: $OUTPUT_PATH"
echo "Topics: ${TOPICS[*]}"

if [[ -n "$DURATION_SEC" ]]; then
  timeout "${DURATION_SEC}s" ros2 bag record -o "$OUTPUT_PATH" "${TOPICS[@]}"
else
  ros2 bag record -o "$OUTPUT_PATH" "${TOPICS[@]}"
fi
