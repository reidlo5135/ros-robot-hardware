#pragma once

#include <cstdint>
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

struct LidarParserStats
{
	std::uint64_t packet_count;
	std::uint64_t valid_packet_count;
	std::uint64_t invalid_packet_count;
	std::uint64_t checksum_error_count;
	std::uint64_t malformed_packet_count;
	std::uint64_t dropped_bytes;
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
	virtual LidarParserStats getStats() const = 0;
};

}  // namespace robot::hw::lidar
