#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_lidar_driver/lidar_parser.hpp"

namespace robot::hw::lidar
{

class Lds03Parser : public LidarParser
{
private:
	// Verified against the TurtleBot3 Humble coin_d4_driver M1CT_TOF parser path.
	static constexpr uint8_t PACKET_SYNC_LOW = 0xAA;
	static constexpr uint8_t PACKET_SYNC_HIGH = 0x55;
	static constexpr uint16_t PACKET_HEADER_VALUE = 0x55AA;
	static constexpr std::size_t HEADER_SIZE = 10U;
	static constexpr std::size_t SAMPLE_SIZE = 3U;
	static constexpr std::size_t MAX_SAMPLES_PER_PACKET = 64U;
	static constexpr uint16_t FULL_ROTATION_Q6 = 360U * 64U;
	static constexpr uint16_t WRAP_THRESHOLD_HIGH_Q6 = 270U * 64U;
	static constexpr uint16_t WRAP_THRESHOLD_LOW_Q6 = 90U * 64U;
	static constexpr double ANGLE_CORRECTION_NUMERATOR = 19.16;
	static constexpr double ANGLE_CORRECTION_DENOMINATOR = 90.15;
	static constexpr double PI = 3.14159265358979323846;

	rclcpp::Logger logger_;
	std::function<rclcpp::Time()> now_cb_;
	bool is_raw_packet_logging_enabled_;
	bool is_packet_error_logging_enabled_;
	rclcpp::Clock throttle_clock_;
	bool has_scan_sync_;
	bool has_logged_sync_success_;
	bool has_logged_checksum_success_;
	double current_scan_frequency_hz_;
	std::vector<LidarPoint> current_points_;

	static double degreesToRadians(double degrees);
	static double normalizeRadians(double angle_rad);
	static uint16_t readLe16(const uint8_t *data);
	static std::string packetToHexString(const std::vector<uint8_t> &packet);

	bool alignToPacketStart(RingBuffer &buffer, bool &made_progress);
	bool decodePacket(const std::vector<uint8_t> &packet, std::vector<LidarPoint> &points, bool &is_ring_start, double &scan_frequency_hz);
	uint16_t computeChecksum(const std::vector<uint8_t> &packet, std::size_t sample_count) const;
	double computeAngleIncrementQ6(uint16_t first_angle_q6, uint16_t last_angle_q6, std::size_t sample_count) const;
	double computeCorrectedAngleQ6(uint16_t first_angle_q6, double interval_q6, std::size_t sample_index, uint16_t distance_q2) const;

	void logRawPacket(const std::vector<uint8_t> &packet);
	void logPacketWarning(const char *message);

protected:
public:
	explicit Lds03Parser(const rclcpp::Logger &logger, std::function<rclcpp::Time()> now_cb, bool log_raw_packet, bool log_packet_error);
	virtual ~Lds03Parser() = default;

	bool consume(RingBuffer &buffer, std::vector<LidarScan> &completed_scans) override;
	void reset() override;
};

}  // namespace robot::hw::lidar
