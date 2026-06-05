#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: inspect_robot_hw_topics.sh

Inspect core robot hardware topics.

Options:
  -h, --help   Show this help.

Commands:
  ros2 topic list
  ros2 topic info -v /scan
  ros2 topic info -v /odom
  ros2 topic info -v /imu
  ros2 topic info -v /joint_states
  ros2 topic info -v /cmd_vel
  ros2 topic info -v /tf
  ros2 topic info -v /tf_static
USAGE
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "Unknown option: $1" >&2
  usage >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS 2 and the workspace before running this script." >&2
  exit 1
fi

echo "== ros2 topic list =="
ros2 topic list || true

declare -a TOPICS=(/scan /odom /imu /joint_states /cmd_vel /tf /tf_static)
for topic in "${TOPICS[@]}"; do
  echo
  echo "== ros2 topic info -v ${topic} =="
  ros2 topic info -v "$topic" || true
done
