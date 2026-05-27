#include "robot_lidar_driver/laser_scan_builder.hpp"

using namespace robot::hw::lidar;

LaserScanBuilder::LaserScanBuilder(
	const std::string &frame_id,
	double angle_min,
	double angle_max,
	double range_min,
	double range_max,
	double scan_angle_offset,
	bool scan_direction_reversed,
	double publish_rate_hint_hz)
: frame_id_(frame_id),
	angle_min_(angle_min),
	angle_max_(angle_max),
	range_min_(range_min),
	range_max_(range_max),
	scan_angle_offset_(scan_angle_offset),
	is_scan_direction_reversed_(scan_direction_reversed),
	publish_rate_hint_hz_(publish_rate_hint_hz)
{
}

sensor_msgs::msg::LaserScan LaserScanBuilder::buildScan(const LidarScan &completed_scan, const rclcpp::Time &stamp) const
{
	sensor_msgs::msg::LaserScan scan_message;
	scan_message.header.stamp = stamp;
	scan_message.header.frame_id = frame_id_;
	scan_message.angle_min = static_cast<float>(angle_min_);
	scan_message.angle_max = static_cast<float>(angle_max_);
	scan_message.range_min = static_cast<float>(range_min_);
	scan_message.range_max = static_cast<float>(range_max_);

	const std::size_t bin_count = completed_scan.points.size() >= 2U ? completed_scan.points.size() : 360U;
	scan_message.ranges.assign(bin_count, std::numeric_limits<float>::infinity());
	scan_message.intensities.assign(bin_count, 0.0F);

	const double scan_time = publish_rate_hint_hz_ > 0.0 ? 1.0 / publish_rate_hint_hz_ : 0.1;
	const double angle_span = angle_max_ - angle_min_;
	const double angle_increment = bin_count > 1U ? angle_span / static_cast<double>(bin_count - 1U) : 0.0;
	const bool is_full_circle = angle_span >= (TWO_PI - 1e-6);

	scan_message.scan_time = static_cast<float>(scan_time);
	scan_message.time_increment = static_cast<float>(bin_count > 0U ? scan_time / static_cast<double>(bin_count) : 0.0);
	scan_message.angle_increment = static_cast<float>(angle_increment);

	for (const LidarPoint &point : completed_scan.points)
	{
		double target_angle = normalizeAngle(point.angle_rad + scan_angle_offset_);
		double relative_angle = 0.0;

		if (is_full_circle)
		{
			relative_angle = std::fmod(target_angle - angle_min_, TWO_PI);
			if (relative_angle < 0.0)
			{
				relative_angle += TWO_PI;
			}
		}
		else
		{
			target_angle = normalizeAngle(target_angle);
			while (target_angle < angle_min_)
			{
				target_angle += TWO_PI;
			}
			while (target_angle > angle_max_)
			{
				target_angle -= TWO_PI;
			}

			if (target_angle < angle_min_ || target_angle > angle_max_)
			{
				continue;
			}

			relative_angle = target_angle - angle_min_;
		}

		if (bin_count == 0U || angle_increment <= 0.0)
		{
			continue;
		}

		const double unclamped_index = relative_angle / angle_increment;
		std::size_t bin_index = static_cast<std::size_t>(std::llround(unclamped_index));
		if (bin_index >= bin_count)
		{
			bin_index = bin_count - 1U;
		}

		const bool is_valid_range = point.range_m >= range_min_ && point.range_m <= range_max_;
		if (!is_valid_range)
		{
			continue;
		}

		if (!std::isfinite(scan_message.ranges[bin_index]) || point.range_m < static_cast<double>(scan_message.ranges[bin_index]))
		{
			scan_message.ranges[bin_index] = static_cast<float>(point.range_m);
			scan_message.intensities[bin_index] = static_cast<float>(point.intensity);
		}
	}

	if (is_scan_direction_reversed_)
	{
		std::reverse(scan_message.ranges.begin(), scan_message.ranges.end());
		std::reverse(scan_message.intensities.begin(), scan_message.intensities.end());
	}

	return scan_message;
}

double LaserScanBuilder::normalizeAngle(double angle_rad)
{
	double normalized = std::fmod(angle_rad, TWO_PI);
	if (normalized < -PI)
	{
		normalized += TWO_PI;
	}
	if (normalized > PI)
	{
		normalized -= TWO_PI;
	}

	return normalized;
}
