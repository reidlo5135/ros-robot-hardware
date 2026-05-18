#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_base_driver/control_table.hpp"
#include "robot_base_driver/dxl_packet_codec.hpp"
#include "robot_base_driver/serial_port.hpp"

namespace robot::hw::base
{

struct VelocityCommand
{
	double m_linear_x_mps;
	double m_angular_z_rps;
};

struct OpencrState
{
	int8_t m_device_status;
	int32_t m_present_velocity_left;
	int32_t m_present_velocity_right;
	int32_t m_present_position_left;
	int32_t m_present_position_right;
	float m_imu_angular_velocity_x;
	float m_imu_angular_velocity_y;
	float m_imu_angular_velocity_z;
	float m_imu_linear_acceleration_x;
	float m_imu_linear_acceleration_y;
	float m_imu_linear_acceleration_z;
	float m_imu_orientation_w;
	float m_imu_orientation_x;
	float m_imu_orientation_y;
	float m_imu_orientation_z;
	bool m_has_imu_data;
};

struct OpencrClientConfig
{
	uint8_t m_opencr_id;
	double m_protocol_version;
	int m_response_timeout_ms;
	int m_startup_delay_ms;
	int m_poll_interval_ms;
	int m_heartbeat_interval_ms;
	bool m_is_heartbeat_enabled;
	bool m_is_imu_recalibration_on_startup;
	bool m_is_imu_recalibration_ack_required;
	bool m_is_profile_acceleration_ack_required;
	bool m_is_heartbeat_ack_required;
	bool m_is_startup_initial_state_read_required;
	int m_startup_initial_state_read_retries;
	int m_startup_initial_state_read_retry_interval_ms;
	bool m_is_serial_packet_logging_enabled;
	bool m_is_read_rate_logging_enabled;
	double m_profile_acceleration_constant;
	double m_profile_acceleration;
};

class OpencrClient
{
private:
	rclcpp::Logger m_logger;
	SerialPort *m_serial_port;
	OpencrClientConfig m_config;
	std::thread m_worker_thread;
	std::mutex m_command_mutex;
	std::condition_variable m_command_cv;
	std::atomic_bool m_is_running;
	bool m_has_pending_velocity_command;
	VelocityCommand m_pending_velocity_command;
	std::vector<uint8_t> m_rx_buffer;
	std::function<void(const OpencrState &)> m_state_callback;
	std::function<void()> m_connected_callback;
	std::function<void(const std::string &)> m_error_callback;
	uint8_t m_heartbeat_counter;
	std::uint64_t m_read_rate_accumulator;
	std::chrono::steady_clock::time_point m_last_read_rate_log_time;
	rclcpp::Clock m_throttle_clock;

	void workerLoop();
	bool performStartupSequence();
	bool pingDevice();
	bool writeImuRecalibration();
	bool writeProfileAcceleration();
	bool writeVelocityCommand(const VelocityCommand &command);
	bool writeHeartbeat();
	bool readState(OpencrState &state);
	bool readInitialState();
	bool transact(
		DxlInstruction instruction,
		const std::vector<uint8_t> &parameters,
		DxlStatusPacket &status_packet,
		bool is_failure_fatal);
	bool transactWriteOnly(
		DxlInstruction instruction,
		const std::vector<uint8_t> &parameters,
		bool is_failure_fatal);
	bool waitForStatusPacket(
		DxlStatusPacket &status_packet,
		DxlInstruction instruction,
		uint8_t target_id,
		const std::vector<uint8_t> &parameters,
		bool is_failure_fatal);
	void prepareForTransaction();
	void discardOptionalResponses(uint8_t target_id);
	void appendReadBytes(const uint8_t *data, std::size_t size);
	void logTimeoutDiagnostics(
		DxlInstruction instruction,
		uint8_t target_id,
		const std::vector<uint8_t> &parameters,
		bool header_seen) const;
	void logRawBytes(const char *direction, const uint8_t *data, std::size_t size);
	void logSerialPacket(const char *direction, const std::vector<uint8_t> &packet);
	std::string formatBytes(const uint8_t *data, std::size_t size) const;
	std::string formatBytes(const std::vector<uint8_t> &data) const;
	const char *instructionToString(DxlInstruction instruction) const;
	void logReadRate(std::size_t size);
	int32_t readInt32(const std::vector<uint8_t> &data, uint16_t address) const;
	float readFloat32(const std::vector<uint8_t> &data, uint16_t address) const;

protected:
public:
	explicit OpencrClient(
		const rclcpp::Logger &logger,
		SerialPort *serial_port,
		const OpencrClientConfig &config);
	virtual ~OpencrClient();

	bool start(
		const std::function<void(const OpencrState &)> &state_callback,
		const std::function<void()> &connected_callback,
		const std::function<void(const std::string &)> &error_callback);
	void stop();
	bool isRunning() const;
	void setVelocityCommand(const VelocityCommand &command);
};

}  // namespace robot::hw::base
