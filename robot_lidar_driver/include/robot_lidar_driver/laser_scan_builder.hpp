#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_lidar_driver/lidar_parser.hpp"

namespace robot::hw::lidar
{

class LaserScanBuilder
{
private:
	static constexpr double PI = 3.14159265358979323846;
	static constexpr double TWO_PI = 2.0 * PI;

	std::string frame_id_;
	double angle_min_;
	double angle_max_;
	double range_min_;
	double range_max_;
	double scan_angle_offset_;
	bool is_scan_direction_reversed_;
	double publish_rate_hint_hz_;
	bool fixed_scan_geometry_;
	std::size_t fixed_scan_samples_;
	double fixed_angle_min_;
	double fixed_angle_max_;
	double fixed_scan_time_;
	double fixed_time_increment_;

	static double normalizeAngle(double angle_rad);

protected:
public:
	explicit LaserScanBuilder(
		const std::string &frame_id,
		double angle_min,
		double angle_max,
		double range_min,
		double range_max,
		double scan_angle_offset,
		bool scan_direction_reversed,
		double publish_rate_hint_hz,
		bool fixed_scan_geometry,
		std::size_t fixed_scan_samples,
		double fixed_angle_min,
		double fixed_angle_max,
		double fixed_scan_time,
		double fixed_time_increment);
	virtual ~LaserScanBuilder() = default;

	sensor_msgs::msg::LaserScan buildScan(const LidarScan &completed_scan, const rclcpp::Time &stamp) const;
};

}  // namespace robot::hw::lidar
