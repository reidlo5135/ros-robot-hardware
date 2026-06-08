#!/usr/bin/env bash
set -euo pipefail

MODE="rotate"
DURATION_SEC="10"
LINEAR_X="0.05"
ANGULAR_Z="0.5"
DO_BAG=false
DO_PUBLISH=true
CMD_VEL_TOPIC="/cmd_vel"
ODOM_TOPIC="/odom"
IMU_TOPIC="/imu"
SCAN_TOPIC="/scan"
TF_PARENT="odom"
TF_CHILD="base_footprint"
OUTPUT_DIR="${HOME}/ws/logs/robot_hw/motion"
SUMMARY_PERIOD_SEC="2.0"
ROSBAG_DURATION_SEC=""
SAFE_STOP=true
BAG_PID=""
BAG_STOPPER_PID=""
BAG_PATH="none"
PUBLISHED_MOTION=false

usage() {
  cat <<'USAGE'
Usage: test_motion_diagnostics.sh [options]

Run motion diagnostics for cmd_vel versus odom, IMU, TF, and scan behavior.
By default this script publishes conservative TurtleBot3 Burger style motion.

Options:
  --mode rotate|linear|square     Motion test mode (default: rotate)
  --duration SEC                  Test duration in seconds (default: 10)
  --linear-x VALUE                Linear velocity, m/s (default: 0.05)
  --angular-z VALUE               Angular velocity, rad/s (default: 0.5)
  --bag                           Record a rosbag during the test
  --publish                       Publish motion commands (default)
  --no-publish                    Observe only; do not publish cmd_vel
  --topic-cmd-vel TOPIC           cmd_vel topic (default: /cmd_vel)
  --topic-odom TOPIC              odom topic (default: /odom)
  --topic-imu TOPIC               imu topic (default: /imu)
  --topic-scan TOPIC              scan topic (default: /scan)
  --tf-parent FRAME               TF parent frame (default: odom)
  --tf-child FRAME                TF child frame (default: base_footprint)
  --output-dir DIR                Output directory root (default: ~/ws/logs/robot_hw/motion)
  --summary-period SEC            Progress summary period, seconds (default: 2.0)
  --rosbag-duration SEC           Record bag for this many seconds instead of --duration
  --safe-stop                     Publish several zero cmd_vel messages at exit (default)
  --no-safe-stop                  Disable exit-time zero cmd_vel messages
  -h, --help                      Show this help

Examples:
  ./scripts/test_motion_diagnostics.sh --mode rotate --publish --angular-z 0.5 --duration 10 --bag
  ./scripts/test_motion_diagnostics.sh --mode linear --publish --linear-x 0.05 --duration 10 --bag
  ./scripts/test_motion_diagnostics.sh --mode square --publish --duration 16 --bag
  ./scripts/test_motion_diagnostics.sh --mode rotate --no-publish --duration 10
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
  if [[ "${SAFE_STOP}" == true && "${PUBLISHED_MOTION}" == true ]] && command -v ros2 >/dev/null 2>&1; then
    for _ in 1 2 3; do
      ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
      sleep 0.1
    done
  fi
}

cleanup() {
  publish_stop
  if [[ -n "${BAG_STOPPER_PID}" ]]; then
    kill "${BAG_STOPPER_PID}" >/dev/null 2>&1 || true
    wait "${BAG_STOPPER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      require_value "$1" "${2:-}"
      MODE="${2:-}"
      shift 2
      ;;
    --duration)
      require_value "$1" "${2:-}"
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --linear-x)
      require_value "$1" "${2:-}"
      LINEAR_X="${2:-}"
      shift 2
      ;;
    --angular-z)
      require_value "$1" "${2:-}"
      ANGULAR_Z="${2:-}"
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
    --no-publish)
      DO_PUBLISH=false
      shift
      ;;
    --topic-cmd-vel)
      require_value "$1" "${2:-}"
      CMD_VEL_TOPIC="${2:-}"
      shift 2
      ;;
    --topic-odom)
      require_value "$1" "${2:-}"
      ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --topic-imu)
      require_value "$1" "${2:-}"
      IMU_TOPIC="${2:-}"
      shift 2
      ;;
    --topic-scan)
      require_value "$1" "${2:-}"
      SCAN_TOPIC="${2:-}"
      shift 2
      ;;
    --tf-parent)
      require_value "$1" "${2:-}"
      TF_PARENT="${2:-}"
      shift 2
      ;;
    --tf-child)
      require_value "$1" "${2:-}"
      TF_CHILD="${2:-}"
      shift 2
      ;;
    --output-dir)
      require_value "$1" "${2:-}"
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --summary-period)
      require_value "$1" "${2:-}"
      SUMMARY_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --rosbag-duration)
      require_value "$1" "${2:-}"
      ROSBAG_DURATION_SEC="${2:-}"
      shift 2
      ;;
    --safe-stop)
      SAFE_STOP=true
      shift
      ;;
    --no-safe-stop)
      SAFE_STOP=false
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

case "${MODE}" in
  rotate|linear|square) ;;
  *)
    echo "ERROR: --mode must be rotate, linear, or square." >&2
    exit 2
    ;;
esac

for numeric_arg in DURATION_SEC LINEAR_X ANGULAR_Z SUMMARY_PERIOD_SEC; do
  if ! is_number "${!numeric_arg}"; then
    echo "ERROR: ${numeric_arg} must be numeric." >&2
    exit 2
  fi
done

if ! awk -v value="${DURATION_SEC}" 'BEGIN { exit(value > 0.0 ? 0 : 1) }'; then
  echo "ERROR: --duration must be positive." >&2
  exit 2
fi

if ! awk -v value="${SUMMARY_PERIOD_SEC}" 'BEGIN { exit(value > 0.0 ? 0 : 1) }'; then
  echo "ERROR: --summary-period must be positive." >&2
  exit 2
fi

if [[ -n "${ROSBAG_DURATION_SEC}" ]] && ! is_number "${ROSBAG_DURATION_SEC}"; then
  echo "ERROR: --rosbag-duration must be numeric." >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source your ROS 2 and workspace setup first." >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 command not found." >&2
  exit 1
fi

if [[ "${DO_BAG}" == true ]]; then
  mkdir -p "${OUTPUT_DIR}"
  stamp="$(date +%Y%m%d_%H%M%S)"
  BAG_PATH="${OUTPUT_DIR}/motion_${MODE}_${stamp}"
  record_topics=("${CMD_VEL_TOPIC}" "${ODOM_TOPIC}" "${IMU_TOPIC}" "${SCAN_TOPIC}" /tf /tf_static /joint_states)
  echo "Recording bag: ${BAG_PATH}"
  ros2 bag record -o "${BAG_PATH}" "${record_topics[@]}" &
  BAG_PID="$!"
  if [[ -n "${ROSBAG_DURATION_SEC}" ]]; then
    (
      sleep "${ROSBAG_DURATION_SEC}"
      kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    ) &
    BAG_STOPPER_PID="$!"
  fi
  sleep 1
fi

if [[ "${DO_PUBLISH}" == true ]]; then
  PUBLISHED_MOTION=true
  echo "Motion publish enabled: mode=${MODE} duration=${DURATION_SEC}s linear.x=${LINEAR_X} angular.z=${ANGULAR_Z}"
else
  echo "Motion publish disabled: observing for ${DURATION_SEC}s"
fi

if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${TMPDIR:-/tmp}/robot_hw_motion_ros_logs"
  mkdir -p "${ROS_LOG_DIR}"
fi

python3 - \
  --mode "${MODE}" \
  --duration "${DURATION_SEC}" \
  --linear-x "${LINEAR_X}" \
  --angular-z "${ANGULAR_Z}" \
  --publish "${DO_PUBLISH}" \
  --cmd-vel-topic "${CMD_VEL_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --scan-topic "${SCAN_TOPIC}" \
  --tf-parent "${TF_PARENT}" \
  --tf-child "${TF_CHILD}" \
  --summary-period "${SUMMARY_PERIOD_SEC}" \
  --safe-stop "${SAFE_STOP}" \
  --bag-path "${BAG_PATH}" <<'PY'
import argparse
import math
import sys
import time
from statistics import mean

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def shortest_angular_delta(previous_yaw, current_yaw):
    return normalize_angle(current_yaw - previous_yaw)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))
    return math.atan2(siny_cosp, cosy_cosp)


def signed(value):
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


def same_sign(command, observed, deadband):
    if abs(command) <= deadband:
        return True
    if abs(observed) <= deadband:
        return False
    return signed(command) == signed(observed)


def fmt(value, precision=6):
    if value is None:
        return "unavailable"
    if isinstance(value, bool):
        return "true" if value else "false"
    return f"{value:.{precision}f}"


def status_to_result(status):
    return {"PASS": "ok", "WARN": "warn", "FAIL": "fail"}[status]


class MotionDiagnostics(Node):
    def __init__(self, args):
        super().__init__("motion_diagnostics_observer")
        self.args = args
        self.start_time = self.get_clock().now()
        self.last_progress_time = self.start_time
        self.odom_samples = []
        self.imu_samples = []
        self.tf_samples = []
        self.scan_count = 0
        self.scan_geometry_stable = True
        self.scan_geometry_reason = "none"
        self.scan_baseline = None
        self.last_tf_error = "none"

        sensor_qos = QoSProfile(depth=50)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, args.odom_topic, self.handle_odom, 50)
        self.create_subscription(Imu, args.imu_topic, self.handle_imu, sensor_qos)
        self.create_subscription(LaserScan, args.scan_topic, self.handle_scan, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=max(args.duration + 5.0, 15.0)))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def handle_odom(self, msg):
        self.odom_samples.append({
            "t": self.elapsed(),
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
            "linear_x": msg.twist.twist.linear.x,
            "angular_z": msg.twist.twist.angular.z,
        })

    def handle_imu(self, msg):
        self.imu_samples.append({
            "t": self.elapsed(),
            "angular_z": msg.angular_velocity.z,
        })

    def handle_scan(self, msg):
        geometry = (len(msg.ranges), msg.angle_min, msg.angle_max, msg.angle_increment)
        self.scan_count += 1
        if self.scan_baseline is None:
            self.scan_baseline = geometry
            return

        size_ok = geometry[0] == self.scan_baseline[0]
        angle_min_ok = abs(geometry[1] - self.scan_baseline[1]) <= 1e-3
        angle_max_ok = abs(geometry[2] - self.scan_baseline[2]) <= 1e-3
        increment_ok = abs(geometry[3] - self.scan_baseline[3]) <= 1e-4
        if not (size_ok and angle_min_ok and angle_max_ok and increment_ok):
            self.scan_geometry_stable = False
            self.scan_geometry_reason = "scan_geometry_changed"

    def command_for_elapsed(self, elapsed):
        msg = Twist()
        if not self.args.publish:
            return msg
        if self.args.mode == "rotate":
            msg.angular.z = self.args.angular_z
        elif self.args.mode == "linear":
            msg.linear.x = self.args.linear_x
        else:
            segment = max(self.args.duration / 8.0, 0.5)
            index = int(elapsed / segment)
            if index < 8:
                if index % 2 == 0:
                    msg.linear.x = self.args.linear_x
                else:
                    msg.angular.z = self.args.angular_z
        return msg

    def publish_zero(self):
        msg = Twist()
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def tick(self):
        elapsed = self.elapsed()
        if elapsed <= self.args.duration:
            cmd = self.command_for_elapsed(elapsed)
            if self.args.publish:
                self.cmd_pub.publish(cmd)

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.tf_parent,
                    self.args.tf_child,
                    Time())
                translation = transform.transform.translation
                self.tf_samples.append({
                    "t": elapsed,
                    "x": translation.x,
                    "y": translation.y,
                    "yaw": yaw_from_quaternion(transform.transform.rotation),
                })
                self.last_tf_error = "none"
            except TransformException as exc:
                self.last_tf_error = str(exc).replace(" ", "_")

            if elapsed - ((self.last_progress_time - self.start_time).nanoseconds / 1e9) >= self.args.summary_period:
                self.last_progress_time = self.get_clock().now()
                print(
                    f"ROBOT_HW_LOG schema=v1 tag=DIAG component=motion event=motion_progress "
                    f"mode={self.args.mode} elapsed_sec={elapsed:.3f} "
                    f"odom_samples={len(self.odom_samples)} imu_samples={len(self.imu_samples)} "
                    f"scan_samples={self.scan_count} tf_samples={len(self.tf_samples)} result=ok reason=none",
                    flush=True)
        else:
            if self.args.publish and self.args.safe_stop:
                self.publish_zero()
            self.print_summary()
            rclpy.shutdown()

    def yaw_values(self, samples):
        return [sample["yaw"] for sample in samples if math.isfinite(sample["yaw"])]

    def normalized_yaw_delta(self, samples):
        yaw_values = self.yaw_values(samples)
        if len(yaw_values) < 2:
            return None
        return normalize_angle(yaw_values[-1] - yaw_values[0])

    def unwrapped_yaw_delta(self, samples):
        yaw_values = self.yaw_values(samples)
        if len(yaw_values) < 2:
            return None
        total = 0.0
        previous_yaw = yaw_values[0]
        for current_yaw in yaw_values[1:]:
            total += shortest_angular_delta(previous_yaw, current_yaw)
            previous_yaw = current_yaw
        return total

    def distance_delta(self, samples):
        if len(samples) < 2:
            return None
        dx = samples[-1]["x"] - samples[0]["x"]
        dy = samples[-1]["y"] - samples[0]["y"]
        return math.hypot(dx, dy)

    def x_delta(self, samples):
        if len(samples) < 2:
            return None
        return samples[-1]["x"] - samples[0]["x"]

    def mean_field(self, samples, field):
        values = [sample[field] for sample in samples if math.isfinite(sample[field])]
        return mean(values) if values else None

    def add_status(self, checks, status, reason):
        checks.append((status, reason))

    def aggregate(self, checks):
        if any(status == "FAIL" for status, _ in checks):
            return "FAIL"
        if any(status == "WARN" for status, _ in checks):
            return "WARN"
        return "PASS"

    def reasons(self, checks):
        reasons = [reason for status, reason in checks if status != "PASS"]
        return ",".join(reasons) if reasons else "none"

    def print_summary(self):
        odom_yaw_delta = self.unwrapped_yaw_delta(self.odom_samples)
        tf_yaw_delta = self.unwrapped_yaw_delta(self.tf_samples)
        odom_yaw_delta_normalized = self.normalized_yaw_delta(self.odom_samples)
        tf_yaw_delta_normalized = self.normalized_yaw_delta(self.tf_samples)
        odom_distance = self.distance_delta(self.odom_samples)
        tf_distance = self.distance_delta(self.tf_samples)
        odom_x_delta = self.x_delta(self.odom_samples)
        tf_x_delta = self.x_delta(self.tf_samples)
        xy_drift = odom_distance if self.args.mode == "rotate" else None
        heading_drift = abs(odom_yaw_delta) if self.args.mode == "linear" and odom_yaw_delta is not None else None
        odom_angular_z_mean = self.mean_field(self.odom_samples, "angular_z")
        odom_linear_x_mean = self.mean_field(self.odom_samples, "linear_x")
        imu_angular_z_mean = self.mean_field(self.imu_samples, "angular_z")

        checks = []
        if len(self.odom_samples) < 2:
            self.add_status(checks, "FAIL", "insufficient_odom_samples")
        if len(self.tf_samples) < 2:
            self.add_status(checks, "WARN", "insufficient_tf_samples")
        if self.scan_count == 0:
            self.add_status(checks, "WARN", "no_scan_samples")
        elif not self.scan_geometry_stable:
            self.add_status(checks, "FAIL", self.scan_geometry_reason)

        odom_tf_yaw_delta_error = None
        if odom_yaw_delta is not None and tf_yaw_delta is not None:
            odom_tf_yaw_delta_error = abs(odom_yaw_delta - tf_yaw_delta)

        if self.args.mode == "rotate":
            if self.args.publish and abs(self.args.angular_z) > 1e-6:
                if odom_angular_z_mean is None:
                    self.add_status(checks, "WARN", "no_odom_angular_velocity")
                elif not same_sign(self.args.angular_z, odom_angular_z_mean, 0.02):
                    self.add_status(checks, "FAIL", "odom_angular_z_sign_mismatch")
                if odom_yaw_delta is not None and not same_sign(self.args.angular_z, odom_yaw_delta, 0.02):
                    self.add_status(checks, "FAIL", "odom_yaw_delta_sign_mismatch")
            if imu_angular_z_mean is None:
                self.add_status(checks, "WARN", "no_imu_angular_velocity")
            elif self.args.publish:
                imu_matches_command = same_sign(self.args.angular_z, imu_angular_z_mean, 0.02)
                imu_matches_odom = (
                    odom_angular_z_mean is None
                    or abs(odom_angular_z_mean) <= 0.02
                    or same_sign(odom_angular_z_mean, imu_angular_z_mean, 0.02)
                )
                if not (imu_matches_command and imu_matches_odom):
                    self.add_status(checks, "WARN", "imu_angular_z_sign_mismatch")
            if odom_tf_yaw_delta_error is not None:
                if odom_tf_yaw_delta_error > 0.25:
                    self.add_status(checks, "FAIL", "odom_tf_yaw_delta_mismatch")
                elif odom_tf_yaw_delta_error > 0.10:
                    self.add_status(checks, "WARN", "odom_tf_yaw_delta_mismatch")
            if xy_drift is not None and xy_drift >= 0.05:
                self.add_status(checks, "WARN", "xy_drift_over_0_05m")
        elif self.args.mode == "linear":
            if self.args.publish and abs(self.args.linear_x) > 1e-6:
                forward_ok = odom_x_delta is not None and same_sign(self.args.linear_x, odom_x_delta, 0.01)
                distance_ok = odom_distance is not None and odom_distance > 0.02
                if not (forward_ok or distance_ok):
                    self.add_status(checks, "FAIL", "odom_linear_motion_not_observed")
                if odom_linear_x_mean is not None and not same_sign(self.args.linear_x, odom_linear_x_mean, 0.01):
                    self.add_status(checks, "FAIL", "odom_linear_x_sign_mismatch")
            if heading_drift is not None and heading_drift > 0.10:
                self.add_status(checks, "WARN", "heading_drift_over_0_10rad")
            if odom_distance is not None and tf_distance is not None and abs(odom_distance - tf_distance) > 0.05:
                self.add_status(checks, "WARN", "odom_tf_distance_mismatch")
        else:
            if odom_distance is not None and odom_distance <= 0.02:
                self.add_status(checks, "WARN", "square_motion_distance_small")
            if odom_yaw_delta is not None and abs(odom_yaw_delta) <= 0.02:
                self.add_status(checks, "WARN", "square_motion_yaw_delta_small")

        if not checks:
            self.add_status(checks, "PASS", "none")

        result = self.aggregate(checks)
        reason = self.reasons(checks)

        print("")
        print("Motion diagnostics summary")
        print(f"- mode: {self.args.mode}")
        print(f"- duration_sec: {self.args.duration:.3f}")
        print(f"- commanded_linear_x: {self.args.linear_x if self.args.mode != 'rotate' else 0.0:.3f}")
        print(f"- commanded_angular_z: {self.args.angular_z if self.args.mode != 'linear' else 0.0:.3f}")
        print(f"- publish_enabled: {str(self.args.publish).lower()}")
        print(f"- odom_samples: {len(self.odom_samples)}")
        print(f"- imu_samples: {len(self.imu_samples)}")
        print(f"- tf_samples: {len(self.tf_samples)}")
        print(f"- scan_samples: {self.scan_count}")
        print(f"- odom_yaw_delta_rad: {fmt(odom_yaw_delta)}")
        print(f"- tf_yaw_delta_rad: {fmt(tf_yaw_delta)}")
        print(f"- odom_yaw_delta_normalized_rad: {fmt(odom_yaw_delta_normalized)}")
        print(f"- tf_yaw_delta_normalized_rad: {fmt(tf_yaw_delta_normalized)}")
        print(f"- odom_tf_yaw_delta_error_rad: {fmt(odom_tf_yaw_delta_error)}")
        print(f"- imu_angular_z_mean: {fmt(imu_angular_z_mean)}")
        print(f"- odom_angular_z_mean: {fmt(odom_angular_z_mean)}")
        print(f"- odom_linear_x_mean: {fmt(odom_linear_x_mean)}")
        print(f"- odom_distance_m: {fmt(odom_distance)}")
        print(f"- tf_distance_m: {fmt(tf_distance)}")
        print(f"- odom_x_delta_m: {fmt(odom_x_delta)}")
        print(f"- tf_x_delta_m: {fmt(tf_x_delta)}")
        print(f"- xy_drift_m: {fmt(xy_drift)}")
        print(f"- heading_drift_rad: {fmt(heading_drift)}")
        print(f"- scan_geometry_stable: {fmt(self.scan_geometry_stable)}")
        print(f"- bag_path: {self.args.bag_path}")
        print(f"- result: {result}")
        print(f"- reason: {reason}")

        print(
            "ROBOT_HW_LOG schema=v1 tag=DIAG component=motion event=motion_summary "
            f"mode={self.args.mode} duration_sec={self.args.duration:.3f} "
            f"commanded_linear_x={self.args.linear_x if self.args.mode != 'rotate' else 0.0:.6f} "
            f"commanded_angular_z={self.args.angular_z if self.args.mode != 'linear' else 0.0:.6f} "
            f"publish_enabled={str(self.args.publish).lower()} "
            f"odom_samples={len(self.odom_samples)} imu_samples={len(self.imu_samples)} "
            f"tf_samples={len(self.tf_samples)} scan_samples={self.scan_count} "
            f"odom_yaw_delta_rad={fmt(odom_yaw_delta)} tf_yaw_delta_rad={fmt(tf_yaw_delta)} "
            f"odom_yaw_delta_normalized_rad={fmt(odom_yaw_delta_normalized)} "
            f"tf_yaw_delta_normalized_rad={fmt(tf_yaw_delta_normalized)} "
            f"odom_tf_yaw_delta_error_rad={fmt(odom_tf_yaw_delta_error)} "
            f"imu_angular_z_mean={fmt(imu_angular_z_mean)} odom_angular_z_mean={fmt(odom_angular_z_mean)} "
            f"odom_linear_x_mean={fmt(odom_linear_x_mean)} odom_distance_m={fmt(odom_distance)} "
            f"tf_distance_m={fmt(tf_distance)} odom_x_delta_m={fmt(odom_x_delta)} "
            f"tf_x_delta_m={fmt(tf_x_delta)} xy_drift_m={fmt(xy_drift)} "
            f"heading_drift_rad={fmt(heading_drift)} scan_geometry_stable={fmt(self.scan_geometry_stable)} "
            f"bag_path={self.args.bag_path} result={status_to_result(result)} reason={reason}",
            flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["rotate", "linear", "square"], required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--linear-x", type=float, required=True)
    parser.add_argument("--angular-z", type=float, required=True)
    parser.add_argument("--publish", type=lambda value: value == "true", required=True)
    parser.add_argument("--cmd-vel-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--scan-topic", required=True)
    parser.add_argument("--tf-parent", required=True)
    parser.add_argument("--tf-child", required=True)
    parser.add_argument("--summary-period", type=float, required=True)
    parser.add_argument("--safe-stop", type=lambda value: value == "true", required=True)
    parser.add_argument("--bag-path", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MotionDiagnostics(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and args.publish and args.safe_stop:
            node.publish_zero()
        node.destroy_node()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
PY
