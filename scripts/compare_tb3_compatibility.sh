#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="5.0"

usage() {
  cat <<'USAGE'
Usage: compare_tb3_compatibility.sh [--timeout SEC] [--help]

Capture a one-shot compatibility snapshot of /scan, /odom, /imu, /tf, and /tf_static.
Run once under turtlebot3_bringup and once under ros-robot-hardware, then compare outputs.
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
    --timeout)
      require_value "$1" "${2:-}"
      TIMEOUT_SEC="$2"
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

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 command not found." >&2
  exit 1
fi

python3 - "${TIMEOUT_SEC}" <<'PY'
import math
import sys
import time

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_ros import Buffer, TransformListener
except Exception as exc:
    print(f"error.import={exc}")
    sys.exit(1)


def fmt(value, digits=9):
    if value is None:
        return "none"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(numeric):
        return "nan"
    if math.isinf(numeric):
        return "inf" if numeric > 0.0 else "-inf"
    return f"{numeric:.{digits}f}"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * ((q.w * q.z) + (q.x * q.y)),
        1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))),
    )


def normalize_angle(angle):
    normalized = math.fmod(angle, 2.0 * math.pi)
    if normalized <= -math.pi:
        normalized += 2.0 * math.pi
    if normalized > math.pi:
        normalized -= 2.0 * math.pi
    return normalized


def scan_angle_at(scan, index):
    return float(scan.angle_min) + (float(index) * float(scan.angle_increment))


def scan_index_for_angle(scan, angle):
    count = len(scan.ranges)
    if count == 0 or abs(float(scan.angle_increment)) <= 1e-12:
        return -1
    angle_min = float(scan.angle_min)
    angle_max = float(scan.angle_max)
    angle_increment = float(scan.angle_increment)
    span = angle_max - angle_min
    if span >= ((2.0 * math.pi) - 1e-6):
        relative = math.fmod(normalize_angle(angle) - angle_min, 2.0 * math.pi)
        if relative < 0.0:
            relative += 2.0 * math.pi
    else:
        target = normalize_angle(angle)
        while target < angle_min:
            target += 2.0 * math.pi
        while target > angle_max:
            target -= 2.0 * math.pi
        if target < angle_min or target > angle_max:
            return -1
        relative = target - angle_min
    index = int(round(relative / angle_increment))
    return max(0, min(count - 1, index))


def valid_scan_ranges(scan):
    valid = []
    for index, value in enumerate(scan.ranges):
        value = float(value)
        if math.isfinite(value) and float(scan.range_min) <= value <= float(scan.range_max):
            valid.append((index, value))
    return valid


def print_scan(scan):
    print("scan.available=true")
    print(f"scan.header.frame_id={scan.header.frame_id}")
    print(f"scan.angle_min={fmt(scan.angle_min)}")
    print(f"scan.angle_max={fmt(scan.angle_max)}")
    print(f"scan.angle_increment={fmt(scan.angle_increment)}")
    print(f"scan.time_increment={fmt(scan.time_increment)}")
    print(f"scan.scan_time={fmt(scan.scan_time)}")
    print(f"scan.ranges_length={len(scan.ranges)}")
    valid = valid_scan_ranges(scan)
    if valid:
        min_index, min_value = min(valid, key=lambda item: item[1])
        max_index, max_value = max(valid, key=lambda item: item[1])
        print(f"scan.valid_min_range={fmt(min_value)}")
        print(f"scan.valid_max_range={fmt(max_value)}")
        print(f"scan.nearest.index={min_index}")
        print(f"scan.nearest.angle={fmt(scan_angle_at(scan, min_index))}")
        print(f"scan.nearest.range={fmt(min_value)}")
    else:
        print("scan.valid_min_range=nan")
        print("scan.valid_max_range=nan")
        print("scan.nearest.index=-1")
        print("scan.nearest.angle=nan")
        print("scan.nearest.range=nan")
    for label, angle in (
        ("front", 0.0),
        ("left", math.pi * 0.5),
        ("right", -math.pi * 0.5),
        ("rear", math.pi),
    ):
        index = scan_index_for_angle(scan, angle)
        sample_angle = scan_angle_at(scan, index) if index >= 0 else math.nan
        sample_range = float(scan.ranges[index]) if index >= 0 else math.nan
        print(f"scan.{label}.index={index}")
        print(f"scan.{label}.angle={fmt(sample_angle)}")
        print(f"scan.{label}.range={fmt(sample_range)}")


def covariance_diagonal_6(covariance):
    return [covariance[index] for index in (0, 7, 14, 21, 28, 35)]


def print_odom(odom):
    print("odom.available=true")
    print(f"odom.header.frame_id={odom.header.frame_id}")
    print(f"odom.child_frame_id={odom.child_frame_id}")
    print("odom.pose_covariance_diagonal=" + ",".join(fmt(v, 6) for v in covariance_diagonal_6(odom.pose.covariance)))
    print("odom.twist_covariance_diagonal=" + ",".join(fmt(v, 6) for v in covariance_diagonal_6(odom.twist.covariance)))
    print(f"odom.twist.linear.x={fmt(odom.twist.twist.linear.x)}")
    print(f"odom.twist.angular.z={fmt(odom.twist.twist.angular.z)}")


def print_imu(imu):
    orientation_norm = math.sqrt(
        (imu.orientation.w * imu.orientation.w)
        + (imu.orientation.x * imu.orientation.x)
        + (imu.orientation.y * imu.orientation.y)
        + (imu.orientation.z * imu.orientation.z)
    )
    has_orientation = imu.orientation_covariance[0] != -1.0 and orientation_norm > 1e-9
    print("imu.available=true")
    print(f"imu.header.frame_id={imu.header.frame_id}")
    print(f"imu.has_orientation={'true' if has_orientation else 'false'}")
    print("imu.orientation_covariance=" + ",".join(fmt(v, 6) for v in imu.orientation_covariance))
    print(f"imu.angular_velocity.z={fmt(imu.angular_velocity.z)}")
    print(f"imu.linear_acceleration.x={fmt(imu.linear_acceleration.x)}")
    print(f"imu.linear_acceleration.y={fmt(imu.linear_acceleration.y)}")
    print(f"imu.linear_acceleration.z={fmt(imu.linear_acceleration.z)}")
    print(f"imu.orientation_yaw={fmt(yaw_from_quaternion(imu.orientation) if has_orientation else math.nan)}")


def print_missing(prefix):
    print(f"{prefix}.available=false")


def print_transform(buffer, parent, child):
    key = f"tf.{parent}_to_{child}".replace("/", "_")
    try:
        transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
    except Exception as exc:
        print(f"{key}.available=false")
        print(f"{key}.error={str(exc).replace(' ', '_')}")
        return False
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    print(f"{key}.available=true")
    print(f"{key}.parent={transform.header.frame_id}")
    print(f"{key}.child={transform.child_frame_id}")
    print(f"{key}.x={fmt(translation.x)}")
    print(f"{key}.y={fmt(translation.y)}")
    print(f"{key}.z={fmt(translation.z)}")
    print(f"{key}.yaw={fmt(yaw_from_quaternion(rotation))}")
    return True


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = rclpy.create_node("tb3_compatibility_snapshot")
    messages = {"scan": None, "odom": None, "imu": None}
    scan_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(LaserScan, "/scan", lambda msg: messages.__setitem__("scan", messages["scan"] or msg), scan_qos)
    node.create_subscription(Odometry, "/odom", lambda msg: messages.__setitem__("odom", messages["odom"] or msg), 10)
    node.create_subscription(Imu, "/imu", lambda msg: messages.__setitem__("imu", messages["imu"] or msg), 10)
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and any(value is None for value in messages.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("# tb3_compatibility_snapshot schema=v1")
    print(f"timeout_sec={fmt(timeout, 3)}")
    print("section=scan")
    print_scan(messages["scan"]) if messages["scan"] is not None else print_missing("scan")
    print("section=odom")
    print_odom(messages["odom"]) if messages["odom"] is not None else print_missing("odom")
    print("section=imu")
    print_imu(messages["imu"]) if messages["imu"] is not None else print_missing("imu")

    tf_deadline = time.monotonic() + timeout
    while time.monotonic() < tf_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("section=tf")
    for parent, child in (
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "base_scan"),
        ("base_link", "imu_link"),
    ):
        print_transform(buffer, parent, child)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
