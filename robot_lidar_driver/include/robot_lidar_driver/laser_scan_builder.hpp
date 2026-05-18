#pragma once

#include <algorithm>
#include <cmath>
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

	std::string m_frame_id;
	double m_angle_min;
	double m_angle_max;
	double m_range_min;
	double m_range_max;
	bool m_is_scan_direction_reversed;
	double m_publish_rate_hint_hz;

	static double normalizeAngle(double angle_rad);

protected:
public:
	explicit LaserScanBuilder(
		const std::string &frame_id,
		double angle_min,
		double angle_max,
		double range_min,
		double range_max,
		bool scan_direction_reversed,
		double publish_rate_hint_hz);
	virtual ~LaserScanBuilder() = default;

	sensor_msgs::msg::LaserScan buildScan(const LidarScan &completed_scan, const rclcpp::Time &stamp) const;
};

}  // namespace robot::hw::lidar
