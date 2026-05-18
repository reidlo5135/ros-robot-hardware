#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_lidar_driver/ring_buffer.hpp"

namespace robot::hw::lidar
{

struct LidarPoint
{
	double angle_rad;
	double range_m;
	double intensity;
};

struct LidarScan
{
	std::vector<LidarPoint> points;
	rclcpp::Time stamp;
	double scan_frequency_hz;
};

class LidarParser
{
private:
protected:
public:
	LidarParser() = default;
	virtual ~LidarParser() = default;

	virtual bool consume(RingBuffer &buffer, std::vector<LidarScan> &completed_scans) = 0;
	virtual void reset() = 0;
};

}  // namespace robot::hw::lidar
