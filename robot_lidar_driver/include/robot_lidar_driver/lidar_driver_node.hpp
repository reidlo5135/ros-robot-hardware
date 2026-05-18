#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
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

	std::string m_lidar_model;
	std::string m_port;
	int m_baudrate;
	std::string m_frame_id;
	std::string m_topic_name;
	double m_range_min;
	double m_range_max;
	double m_angle_min;
	double m_angle_max;
	bool m_is_scan_direction_reversed;
	double m_publish_rate_hint_hz;
	int m_read_buffer_size;
	int m_ring_buffer_size;
	bool m_use_epoll;
	bool m_is_reconnect_on_error;
	int m_reconnect_interval_ms;
	int m_serial_read_timeout_ms;
	bool m_is_mock_mode;
	bool m_is_raw_packet_logging_enabled;
	bool m_is_packet_error_logging_enabled;

	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> m_scan_publisher;
	std::shared_ptr<SerialPort> m_serial_port;
	std::shared_ptr<EpollSerialReader> m_reader;
	std::shared_ptr<LidarParser> m_parser;
	std::shared_ptr<LaserScanBuilder> m_scan_builder;
	std::shared_ptr<rclcpp::TimerBase> m_mock_timer;
	std::shared_ptr<rclcpp::TimerBase> m_reconnect_timer;

	RingBuffer m_ring_buffer;
	std::mutex m_data_mutex;
	std::atomic_bool m_is_shutdown_requested;
	std::atomic_bool m_is_reconnecting;
	rclcpp::Clock m_throttle_clock;

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
	void stopRealMode();
	void stopMockMode();
	void scheduleReconnect(const std::string &reason);
	void cancelReconnect();
	void attemptReconnect();
	void handleSerialBytes(const uint8_t *data, std::size_t size);
	void handleReaderError(const std::string &message);
	void publishCompletedScans(const std::vector<LidarScan> &completed_scans);
	void publishMockScan();
	std::string resolveTopicName() const;

protected:
public:
	explicit LidarDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	virtual ~LidarDriverNode();
};

}  // namespace robot::hw::lidar
