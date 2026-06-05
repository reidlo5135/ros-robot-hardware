#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: check_robot_hw_tf_publishers.sh [--help]

Inspect ROS 2 TF topic publishers and print expected robot_hardware ownership.
This script does not change any robot state.
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "Unknown argument: $1" >&2
  usage >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source your ROS 2 and workspace setup first." >&2
  exit 1
fi

print_expected_ownership() {
  cat <<'EOF'
Expected TF ownership:
  odom -> base_footprint: robot_base_driver
  base_footprint -> base_link: robot_state_publisher
  base_link -> base_scan: robot_state_publisher
  base_link -> imu_link: robot_state_publisher
  map -> odom: localization/navigation, not robot_hardware
EOF
}

print_topic_info() {
  local topic="$1"
  echo
  echo "== ${topic} publishers/subscribers =="
  if ! ros2 topic info -v "${topic}"; then
    echo "WARN: unable to inspect ${topic}. Is ROS running?"
  fi
}

print_relevant_nodes() {
  echo
  echo "== Relevant running nodes =="
  local nodes
  if ! nodes="$(ros2 node list 2>/dev/null)"; then
    echo "WARN: unable to list ROS nodes."
    return
  fi

  if [[ -z "${nodes}" ]]; then
    echo "No ROS nodes reported."
    return
  fi

  echo "${nodes}" | grep -Ei 'robot_base_driver|robot_state_publisher|amcl|slam|localization|ekf|robot_localization|nav2|map' || \
    echo "No common robot_hardware/localization nodes matched. Full node list:"
  echo "${nodes}"
}

print_expected_ownership
print_topic_info /tf
print_topic_info /tf_static
print_relevant_nodes

echo
echo "Check for duplicate ownership by confirming each expected parent/child pair is produced by exactly one responsible node."
