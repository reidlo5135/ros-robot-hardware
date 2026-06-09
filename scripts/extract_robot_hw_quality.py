#!/usr/bin/env python3
"""Summarize robot_hw structured log quality from a saved ROS log file."""

from __future__ import annotations

import argparse
import math
import re
import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Optional


KV_RE = re.compile(r"([A-Za-z0-9_.-]+)=([^ ]+)")


def parse_robot_hw_entries(lines: Iterable[str]) -> Iterable[Dict[str, str]]:
    for line in lines:
        if "ROBOT_HW_LOG" not in line:
            continue
        yield {match.group(1): match.group(2) for match in KV_RE.finditer(line)}


def as_float(entry: Dict[str, str], key: str) -> Optional[float]:
    value = entry.get(key)
    if value is None:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def mean_min_max(values: List[float]) -> str:
    if not values:
        return "n/a"
    return (
        f"avg={statistics.fmean(values):.3f} "
        f"min={min(values):.3f} max={max(values):.3f}"
    )


def count_reason(entries: List[Dict[str, str]], event: str, reason: str) -> int:
    return sum(1 for entry in entries if entry.get("event") == event and entry.get("reason") == reason)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize robot hardware quality metrics from ROBOT_HW_LOG lines."
    )
    parser.add_argument("log_file", type=Path, help="Path to a saved ROS log file")
    args = parser.parse_args()

    entries = list(parse_robot_hw_entries(args.log_file.read_text(errors="replace").splitlines()))

    scan_publish = [entry for entry in entries if entry.get("event") == "scan_publish"]
    scan_geometry = [entry for entry in entries if entry.get("event") == "scan_geometry_stability"]
    poll_cycle = [entry for entry in entries if entry.get("event") == "poll_cycle"]
    poll_timing = [entry for entry in entries if entry.get("event") == "poll_timing"]
    rotation = [entry for entry in entries if entry.get("event") == "rotation_consistency"]
    serial = [entry for entry in entries if entry.get("event") == "dxl_transaction"]
    tf_publish = [entry for entry in entries if entry.get("event") == "tf_publish"]

    lidar_publish_rates = [value for entry in scan_publish if (value := as_float(entry, "publish_rate_hz")) is not None]
    lidar_valid_ranges = [value for entry in scan_publish if (value := as_float(entry, "valid_ranges")) is not None]
    lidar_invalid_ranges = [value for entry in scan_publish if (value := as_float(entry, "invalid_ranges")) is not None]
    odom_rates = [value for entry in poll_cycle if (value := as_float(entry, "actual_odom_rate_hz")) is not None]
    poll_avg_total = [value for entry in poll_timing if (value := as_float(entry, "avg_total_ms")) is not None]
    poll_max_total = [value for entry in poll_timing if (value := as_float(entry, "max_total_ms")) is not None]
    odom_cmd_ratios = [value for entry in rotation if (value := as_float(entry, "odom_cmd_ratio")) is not None]
    yaw_deltas = [abs(value) for entry in rotation if (value := as_float(entry, "odom_imu_yaw_delta_rad")) is not None]
    tf_stamp_ages = [value for entry in tf_publish if (value := as_float(entry, "stamp_age_sec")) is not None]

    scan_mapping_states = sorted({entry.get("mapping_validation_state", "unknown") for entry in scan_geometry})
    scan_geometry_results = sorted({entry.get("result", "unknown") for entry in scan_geometry})

    print("robot_hw quality summary")
    print(f"log_file={args.log_file}")
    print(f"robot_hw_log_entries={len(entries)}")
    print(f"lidar_publish_rate_hz {mean_min_max(lidar_publish_rates)}")
    print(f"lidar_valid_ranges {mean_min_max(lidar_valid_ranges)}")
    print(f"lidar_invalid_ranges {mean_min_max(lidar_invalid_ranges)}")
    print(f"scan_geometry_results={','.join(scan_geometry_results) if scan_geometry_results else 'n/a'}")
    print(f"scan_mapping_states={','.join(scan_mapping_states) if scan_mapping_states else 'n/a'}")
    print(f"opencr_actual_odom_rate_hz {mean_min_max(odom_rates)}")
    print(f"poll_avg_total_ms {mean_min_max(poll_avg_total)}")
    print(f"poll_max_total_ms {mean_min_max(poll_max_total)}")
    print(f"odom_cmd_angular_ratio {mean_min_max(odom_cmd_ratios)}")
    print(f"odom_imu_yaw_delta_abs_rad {mean_min_max(yaw_deltas)}")
    print(f"cmd_odom_sign_mismatch_count={count_reason(rotation, 'rotation_consistency', 'cmd_odom_sign_mismatch')}")
    print(f"odom_imu_sign_mismatch_count={count_reason(rotation, 'rotation_consistency', 'odom_imu_sign_mismatch')}")
    print(f"serial_timeout_count={sum(1 for entry in serial if entry.get('reason') == 'timeout')}")
    print(f"serial_failure_count={sum(1 for entry in serial if entry.get('result') in {'failed', 'warn'})}")
    print(f"tf_stamp_age_sec {mean_min_max(tf_stamp_ages)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
