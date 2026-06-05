#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="5.0"
SAMPLES="1"

usage() {
  cat <<'USAGE'
Usage: compare_tb3_compatibility.sh [--timeout SEC] [--samples COUNT] [--help]

Capture a compatibility snapshot of /scan, /odom, /imu, /tf, and /tf_static.
Run once under turtlebot3_bringup and once under ros-robot-hardware, then compare outputs.

/scan and /imu are subscribed with SensorDataQoS-compatible best_effort settings.
If a RELIABILITY QoS warning appears for /imu, inspect the publisher with:
    ros2 topic info -v /imu
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
    --samples)
      require_value "$1" "${2:-}"
      SAMPLES="$2"
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

python3 - "${TIMEOUT_SEC}" "${SAMPLES}" <<'PY'
from collections import Counter
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


SENSOR_QOS_PROFILE_NAME = "best_effort_sensor_data"
SENSOR_QOS_RELIABILITY = "best_effort"
SENSOR_QOS_DURABILITY = "volatile"
SENSOR_QOS_HISTORY = "keep_last"
SENSOR_QOS_DEPTH = 10


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


def fmt_float_list(values, digits=9):
    return ",".join(fmt(value, digits) for value in values) if values else "none"


def qos_policy_name(policy):
    if policy is None:
        return "unknown"
    name = getattr(policy, "name", None)
    if name:
        return name.lower()
    text = str(policy).lower()
    for candidate in (
        "best_effort",
        "reliable",
        "volatile",
        "transient_local",
        "keep_last",
        "keep_all",
        "system_default",
        "unknown",
    ):
        if candidate in text:
            return candidate
    return text.replace(" ", "_")


def print_sensor_subscription_qos(prefix):
    print(f"{prefix}.subscription_qos.profile={SENSOR_QOS_PROFILE_NAME}")
    print(f"{prefix}.subscription_qos.reliability={SENSOR_QOS_RELIABILITY}")
    print(f"{prefix}.subscription_qos.durability={SENSOR_QOS_DURABILITY}")
    print(f"{prefix}.subscription_qos.history={SENSOR_QOS_HISTORY}")
    print(f"{prefix}.subscription_qos.depth={SENSOR_QOS_DEPTH}")


def print_publisher_qos(node, topic_name, prefix, subscriber_reliability):
    try:
        publisher_infos = node.get_publishers_info_by_topic(topic_name)
    except Exception as exc:
        print(f"{prefix}.publisher_qos.count=unknown")
        print(f"{prefix}.publisher_qos.compatibility=unknown")
        print(f"{prefix}.publisher_qos.error={str(exc).replace(' ', '_')}")
        return

    print(f"{prefix}.publisher_qos.count={len(publisher_infos)}")
    if not publisher_infos:
        print(f"{prefix}.publisher_qos.compatibility=unknown")
        print(f"{prefix}.publisher_qos.reason=no_publishers_discovered")
        return

    incompatible_reliability = False
    for index, publisher_info in enumerate(publisher_infos):
        qos_profile = publisher_info.qos_profile
        reliability = qos_policy_name(getattr(qos_profile, "reliability", None))
        durability = qos_policy_name(getattr(qos_profile, "durability", None))
        history = qos_policy_name(getattr(qos_profile, "history", None))
        depth = getattr(qos_profile, "depth", "unknown")
        print(f"{prefix}.publisher_qos.{index}.reliability={reliability}")
        print(f"{prefix}.publisher_qos.{index}.durability={durability}")
        print(f"{prefix}.publisher_qos.{index}.history={history}")
        print(f"{prefix}.publisher_qos.{index}.depth={depth}")
        if subscriber_reliability == "reliable" and reliability == "best_effort":
            incompatible_reliability = True

    if incompatible_reliability:
        print(f"{prefix}.publisher_qos.compatibility=warn")
        print(f"{prefix}.publisher_qos.warning=incompatible_reliability")
    else:
        print(f"{prefix}.publisher_qos.compatibility=ok")


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


def print_scan_stats(scans, requested_samples, timeout_occurred):
    print(f"scan.samples_requested={requested_samples}")
    print(f"scan.samples_received={len(scans)}")
    print(f"scan.timeout={'true' if timeout_occurred else 'false'}")
    if not scans:
        print("scan.ranges_length.unique=none")
        print("scan.ranges_length.min=none")
        print("scan.ranges_length.max=none")
        print("scan.ranges_length.most_common=none")
        print("scan.angle_increment.unique=none")
        print("scan.angle_increment.min=none")
        print("scan.angle_increment.max=none")
        print("scan.nearest.angle.min=none")
        print("scan.nearest.angle.max=none")
        print("scan.nearest.angle.mean=none")
        print("scan.nearest.range.min=none")
        print("scan.nearest.range.max=none")
        return

    lengths = [len(scan.ranges) for scan in scans]
    length_counts = Counter(lengths)
    most_common_length, most_common_count = length_counts.most_common(1)[0]
    increments = [float(scan.angle_increment) for scan in scans]
    nearest_angles = []
    nearest_ranges = []
    for scan in scans:
        valid = valid_scan_ranges(scan)
        if not valid:
            continue
        index, value = min(valid, key=lambda item: item[1])
        nearest_angles.append(scan_angle_at(scan, index))
        nearest_ranges.append(value)

    print("scan.ranges_length.unique=" + ",".join(str(value) for value in sorted(length_counts)))
    print(f"scan.ranges_length.min={min(lengths)}")
    print(f"scan.ranges_length.max={max(lengths)}")
    print(f"scan.ranges_length.most_common={most_common_length}")
    print(f"scan.ranges_length.most_common_count={most_common_count}")
    print("scan.angle_increment.unique=" + fmt_float_list(sorted(set(increments))))
    print(f"scan.angle_increment.min={fmt(min(increments))}")
    print(f"scan.angle_increment.max={fmt(max(increments))}")
    print(f"scan.nearest.angle.min={fmt(min(nearest_angles) if nearest_angles else math.nan)}")
    print(f"scan.nearest.angle.max={fmt(max(nearest_angles) if nearest_angles else math.nan)}")
    print(f"scan.nearest.angle.mean={fmt((sum(nearest_angles) / len(nearest_angles)) if nearest_angles else math.nan)}")
    print(f"scan.nearest.range.min={fmt(min(nearest_ranges) if nearest_ranges else math.nan)}")
    print(f"scan.nearest.range.max={fmt(max(nearest_ranges) if nearest_ranges else math.nan)}")


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


def print_sensor_missing(prefix, timeout_occurred, error, include_timeout=True):
    print(f"{prefix}.available=false")
    if include_timeout:
        print(f"{prefix}.timeout={'true' if timeout_occurred else 'false'}")
    print(f"{prefix}.qos_profile={SENSOR_QOS_PROFILE_NAME}")
    print(f"{prefix}.error={error}")


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
    requested_samples = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    if requested_samples <= 0:
        requested_samples = 1

    rclpy.init()
    node = rclpy.create_node("tb3_compatibility_snapshot")
    messages = {"scan": [], "odom": [], "imu": []}
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=SENSOR_QOS_DEPTH,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def capture(topic, limit):
        def callback(msg):
            if len(messages[topic]) < limit:
                messages[topic].append(msg)
        return callback

    node.create_subscription(LaserScan, "/scan", capture("scan", requested_samples), sensor_qos)
    node.create_subscription(Odometry, "/odom", capture("odom", 1), 10)
    node.create_subscription(Imu, "/imu", capture("imu", 1), sensor_qos)
    buffer = Buffer()
    TransformListener(buffer, node)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (len(messages["scan"]) < requested_samples or not messages["odom"] or not messages["imu"]):
        rclpy.spin_once(node, timeout_sec=0.1)

    scan_timeout = len(messages["scan"]) < requested_samples
    odom_timeout = not messages["odom"]
    imu_timeout = not messages["imu"]

    print("# tb3_compatibility_snapshot schema=v1")
    print(f"timeout_sec={fmt(timeout, 3)}")
    print("section=scan")
    print_sensor_subscription_qos("scan")
    print_publisher_qos(node, "/scan", "scan", SENSOR_QOS_RELIABILITY)
    print_scan_stats(messages["scan"], requested_samples, scan_timeout)
    print_scan(messages["scan"][-1]) if messages["scan"] else print_sensor_missing(
        "scan", scan_timeout, "timeout_waiting_for_scan", include_timeout=False)
    print("section=odom")
    print(f"odom.timeout={'true' if odom_timeout else 'false'}")
    print_odom(messages["odom"][-1]) if messages["odom"] else print_missing("odom")
    print("section=imu")
    print_sensor_subscription_qos("imu")
    print_publisher_qos(node, "/imu", "imu", SENSOR_QOS_RELIABILITY)
    if messages["imu"]:
        print("imu.timeout=false")
        print_imu(messages["imu"][-1])
    else:
        print_sensor_missing("imu", imu_timeout, "timeout_waiting_for_imu")

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