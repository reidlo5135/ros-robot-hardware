#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_lidar_driver/epoll_serial_reader.hpp"
#include "robot_lidar_driver/laser_scan_builder.hpp"
#include "robot_lidar_driver/lidar_parser.hpp"
#include "robot_lidar_driver/lds03_parser.hpp"
#include "robot_lidar_driver/ring_buffer.hpp"
#include "robot_lidar_driver/serial_port.hpp"

namespace robot::hw::lidar
{

class LidarDriverNode : public rclcpp::Node
{
private:
	static constexpr double PI = 3.14159265358979323846;
	static constexpr std::array<uint8_t, 4> COIN_D4_START_COMMAND = {0xAA, 0x55, 0xF0, 0x0F};
	static constexpr std::array<uint8_t, 4> COIN_D4_STOP_COMMAND = {0xAA, 0x55, 0xF5, 0x0A};

	std::string lidar_model_;
	std::string port_;
	int baudrate_;
	std::string frame_id_;
	std::string topic_name_;
	double range_min_;
	double range_max_;
	double angle_min_;
	double angle_max_;
	double scan_angle_offset_;
	bool is_scan_direction_reversed_;
	bool reverse_scan_;
	bool debug_scan_geometry_;
	double publish_rate_hint_hz_;
	int read_buffer_size_;
	int ring_buffer_size_;
	bool use_epoll_;
	bool is_reconnect_on_error_;
	int reconnect_interval_ms_;
	int serial_read_timeout_ms_;
	int startup_delay_ms_;
	bool is_set_dtr_;
	bool is_set_rts_;
	bool is_dtr_active_;
	bool is_rts_active_;
	bool is_mock_mode_;
	bool is_read_rate_logging_enabled_;
	bool is_raw_packet_logging_enabled_;
	bool is_packet_error_logging_enabled_;

	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_publisher_;
	std::shared_ptr<SerialPort> serial_port_;
	std::shared_ptr<EpollSerialReader> reader_;
	std::shared_ptr<LidarParser> parser_;
	std::shared_ptr<LaserScanBuilder> scan_builder_;
	std::shared_ptr<rclcpp::TimerBase> mock_timer_;
	std::shared_ptr<rclcpp::TimerBase> reconnect_timer_;

	RingBuffer ring_buffer_;
	std::mutex data_mutex_;
	std::atomic_bool is_shutdown_requested_;
	std::atomic_bool is_reconnecting_;
	rclcpp::Clock throttle_clock_;
	std::uint64_t read_bytes_accumulator_;
	std::chrono::steady_clock::time_point last_read_rate_log_time_;
	bool has_logged_serial_read_success_;
	bool has_logged_publish_success_;

	void declareParameters();
	void loadParameters();
	void validateParameters();
	void logParameterSummary() const;
	void setupPublisher();
	bool setupParser();
	void setupLaserScanBuilder();
	void startDriver();
	void startRealMode();
	void startMockMode();
	bool sendCoinD4StartCommand();
	void sendCoinD4StopCommand();
	void stopRealMode();
	void stopMockMode();
	void scheduleReconnect(const std::string &reason);
	void cancelReconnect();
	void attemptReconnect();
	void applySerialControlSignals();
	void waitForStartupDelayAndLog();
	void handleSerialBytes(const uint8_t *data, std::size_t size);
	void handleReaderError(const std::string &message);
	void logRawReadChunk(const uint8_t *data, std::size_t size);
	void logReadRate(std::size_t size);
	void publishCompletedScans(const std::vector<LidarScan> &completed_scans);
	void publishMockScan();
	void logScanGeometry(const LidarScan &completed_scan, const sensor_msgs::msg::LaserScan &scan_message) const;
	int computeScanIndexForAngle(const sensor_msgs::msg::LaserScan &scan_message, double angle_rad) const;
	std::string resolveTopicName() const;

protected:
public:
	explicit LidarDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	virtual ~LidarDriverNode();

	using SharedPtr = std::shared_ptr<LidarDriverNode>;
};

}  // namespace robot::hw::lidar
