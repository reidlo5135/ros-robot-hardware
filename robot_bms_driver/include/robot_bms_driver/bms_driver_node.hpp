#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "robot_bms_driver/bms_parser.hpp"
#include "robot_bms_driver/serial_port.hpp"

namespace robot::hw::bms
{

class BmsDriverNode : public rclcpp::Node
{
private:
	bool enabled_;
	std::string port_;
	int baudrate_;
	std::string frame_id_;
	std::string topic_name_;
	int poll_interval_ms_;
	int read_timeout_ms_;
	int frame_timeout_ms_;
	std::string protocol_;
	bool publish_diagnostics_;
	bool log_raw_frames_;
	int warn_timeout_ms_;
	std::size_t read_buffer_size_;
	bool structured_logging_enabled_;
	double diagnostics_throttle_sec_;

	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
	std::unique_ptr<SerialPort> serial_port_;
	std::unique_ptr<BmsParser> parser_;
	rclcpp::TimerBase::SharedPtr poll_timer_;
	mutable rclcpp::Clock throttle_clock_;
	std::chrono::steady_clock::time_point start_time_;
	std::chrono::steady_clock::time_point last_rx_time_;
	std::chrono::steady_clock::time_point last_valid_frame_time_;
	bool has_rx_bytes_;
	bool has_valid_frame_;

	void declareParameters();
	void loadParameters();
	void validateParameters();
	void setup();
	void pollSerial();
	void publishBatteryState(const BatterySample &sample);
	void logConfig() const;
	void logFrameRejected(const std::string &reason, std::size_t bytes_received) const;
	void logTimeoutIfNeeded() const;
	void logRawFrame(const std::vector<std::uint8_t> &bytes) const;

	bool openSerialIfNeeded();
	std::string resolveTopicName() const;
	static float optionalToFloat(const std::optional<double> &value);
	static std::vector<float> toFloatVector(const std::vector<double> &values);

public:
	explicit BmsDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	~BmsDriverNode() override;
};

}  // namespace robot::hw::bms