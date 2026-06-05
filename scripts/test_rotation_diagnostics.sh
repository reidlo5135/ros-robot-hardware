#!/usr/bin/env bash
set -euo pipefail

ANGULAR_Z="0.5"
DURATION_SEC="10"
DO_BAG=false
DO_PUBLISH=false
BAG_ROOT="${HOME}/ws/bags"
BAG_PID=""
PUBLISHED_MOTION=false

usage() {
  cat <<'USAGE'
Usage: test_rotation_diagnostics.sh [options]

Run a rotation-focused diagnostic session. By default this script does not publish motion.

Options:
  --angular-z VALUE   Angular velocity for explicit publish mode, rad/s (default: 0.5)
  --duration SEC      Test duration in seconds (default: 10)
  --bag               Record /odom /imu /tf /tf_static /scan /cmd_vel during the test
  --publish           Explicitly publish a rotate-in-place /cmd_vel for the duration
  --bag-root PATH     Bag output directory root (default: ~/ws/bags)
  -h, --help          Show this help
USAGE
}

is_number() {
  [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "ERROR: ${option} requires a value." >&2
    exit 2
  fi
}

publish_stop() {
  if [[ "${PUBLISHED_MOTION}" == true ]] && command -v ros2 >/dev/null 2>&1; then
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  publish_stop
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --angular-z)
      require_value "$1" "${2:-}"
      ANGULAR_Z="${2:-}"
      shift 2
      ;;
    --duration)
      require_value "$1" "${2:-}"
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --bag)
      DO_BAG=true
      shift
      ;;
    --publish)
      DO_PUBLISH=true
      shift
      ;;
    --bag-root)
      require_value "$1" "${2:-}"
      BAG_ROOT="${2:-}"
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
  echo "ERROR: ros2 command not found. Source your ROS 2 and workspace setup first." >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "ERROR: timeout command not found." >&2
  exit 1
fi

if ! is_number "${ANGULAR_Z}"; then
  echo "ERROR: --angular-z must be numeric." >&2
  exit 2
fi

if ! is_number "${DURATION_SEC}" || ! awk -v value="${DURATION_SEC}" 'BEGIN { exit(value > 0.0 ? 0 : 1) }'; then
  echo "ERROR: --duration must be a positive number." >&2
  exit 2
fi

cat <<EOF
Rotation diagnostics checklist:
  Logs: watch for event=rotation_state and event=rotation_consistency
  Odom: ros2 topic echo /odom
  IMU:  ros2 topic echo /imu
  TF:   ros2 run tf2_ros tf2_echo odom base_footprint
  Scan: verify front obstacles appear near front_angle_rad ~= 0 in event=scan_geometry
EOF

if [[ "${DO_PUBLISH}" != true ]]; then
  cat <<EOF

Motion publish mode is OFF. The robot will not move from this script.
Use --publish only in a safe test area when you explicitly want rotate-in-place /cmd_vel.
EOF
fi

if [[ "${DO_BAG}" == true ]]; then
  mkdir -p "${BAG_ROOT}"
  BAG_PATH="${BAG_ROOT}/robot_hw_rotation_$(date +%Y%m%d_%H%M%S)"
  echo
  echo "Recording bag: ${BAG_PATH}"
  ros2 bag record -o "${BAG_PATH}" /odom /imu /tf /tf_static /scan /cmd_vel &
  BAG_PID="$!"
  sleep 1
fi

if [[ "${DO_PUBLISH}" == true ]]; then
  echo
  echo "Publishing rotate-in-place cmd_vel: angular.z=${ANGULAR_Z} rad/s duration=${DURATION_SEC}s"
  PUBLISHED_MOTION=true
  timeout "${DURATION_SEC}s" ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}" || true
  publish_stop
else
  if [[ "${DO_BAG}" == true ]]; then
    echo
    echo "Recording for ${DURATION_SEC}s. Rotate the robot manually or with your normal controller if safe."
    sleep "${DURATION_SEC}"
  fi
fi

echo
echo "Done. Review ROBOT_HW_LOG event=rotation_consistency reason/result fields and compare odom yaw, IMU yaw, scan yaw, and TF."
