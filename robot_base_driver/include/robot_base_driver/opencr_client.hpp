#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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

enum class OpencrPollMode : uint8_t
{
	Minimal = 0,
	Full = 1,
	Odom = 2
};

enum class OpencrCommandMode : uint8_t
{
	BodyTwist = 0,
	WheelVelocity = 1
};

struct VelocityCommand
{
	double linear_x_mps;
	double angular_z_rps;
	std::string source;
};

struct OpencrState
{
	int8_t device_status;
	bool has_device_status;
	float battery_voltage;
	bool has_battery_voltage;
	bool has_motor_torque_enable;
	bool motor_torque_enabled;
	int32_t present_velocity_left;
	int32_t present_velocity_right;
	int32_t present_position_left;
	int32_t present_position_right;
	float imu_angular_velocity_x;
	float imu_angular_velocity_y;
	float imu_angular_velocity_z;
	float imu_linear_acceleration_x;
	float imu_linear_acceleration_y;
	float imu_linear_acceleration_z;
	float imu_orientation_w;
	float imu_orientation_x;
	float imu_orientation_y;
	float imu_orientation_z;
	bool has_imu_data;
};

struct OpencrClientConfig
{
	uint8_t opencr_id;
	std::string port;
	int baudrate;
	double protocol_version;
	int response_timeout_ms;
	int startup_delay_ms;
	int poll_interval_ms;
	int heartbeat_interval_ms;
	bool is_heartbeat_enabled;
	bool is_motor_torque_enable_on_startup;
	bool is_motor_torque_enable_ack_required;
	bool is_imu_recalibration_on_startup;
	bool is_imu_recalibration_ack_required;
	bool is_profile_acceleration_ack_required;
	bool is_heartbeat_ack_required;
	bool is_profile_acceleration_on_startup;
	bool is_startup_initial_state_read_required;
	int startup_initial_state_read_retries;
	int startup_initial_state_read_retry_interval_ms;
	bool is_serial_packet_logging_enabled;
	bool is_read_rate_logging_enabled;
	bool is_structured_logging_enabled;
	double serial_state_throttle_sec;
	double opencr_state_throttle_sec;
	double poll_timing_throttle_sec;
	bool debug_motor_command;
	bool debug_poll_timing;
	int transaction_gap_us;
	double wheel_separation_m;
	double wheel_radius_m;
	double profile_acceleration_constant;
	double profile_acceleration;
	double target_odom_rate_hz;
	OpencrCommandMode command_mode;
	OpencrPollMode poll_mode;
	int max_consecutive_poll_failures;
	bool poll_device_status;
	bool require_device_status;
	bool require_imu;
	bool reconnect_on_poll_failure;
	bool reopen_serial_on_poll_failure;
	bool probe_registers_on_startup;
};

struct PollCycleTiming
{
	double total_cycle_ms;
	double required_state_read_ms;
	double optional_imu_read_ms;
	double device_status_read_ms;
	double command_write_ms;
	double sleep_wait_ms;
	bool timeout_occurred;
	bool retry_occurred;
	bool poll_failed;
	bool transport_error;
	bool imu_read_failed;
	bool device_status_read_failed;
};

struct PollTimingAccumulator
{
	std::size_t cycle_count;
	double total_cycle_ms_sum;
	double required_state_read_ms_sum;
	double optional_imu_read_ms_sum;
	double device_status_read_ms_sum;
	double command_write_ms_sum;
	double sleep_wait_ms_sum;
	double max_cycle_ms;
	std::size_t timeout_count;
	std::size_t retry_count;
	std::size_t poll_failure_count;
	std::size_t transport_error_count;
	std::size_t imu_failure_count;
	std::size_t device_status_failure_count;
};

class OpencrClient
{
private:
	rclcpp::Logger logger_;
	SerialPort *serial_port_;
	OpencrClientConfig config_;
	std::thread worker_thread_;
	std::mutex command_mutex_;
	std::mutex transaction_mutex_;
	std::condition_variable command_cv_;
	std::atomic_bool is_running_;
	bool has_pending_velocity_command_;
	VelocityCommand pending_velocity_command_;
	std::vector<uint8_t> rx_buffer_;
	std::function<void(const OpencrState &)> state_callback_;
	std::function<void()> connected_callback_;
	std::function<void(const std::string &)> error_callback_;
	uint8_t heartbeat_counter_;
	std::uint64_t read_rate_accumulator_;
	std::chrono::steady_clock::time_point last_read_rate_log_time_;
	std::chrono::steady_clock::time_point last_transaction_time_;
	std::chrono::steady_clock::time_point last_parser_stats_log_time_;
	std::chrono::steady_clock::time_point last_poll_timing_log_time_;
	rclcpp::Clock throttle_clock_;
	std::vector<uint8_t> last_tx_packet_;
	std::vector<uint8_t> last_rx_packet_;
	int consecutive_poll_failures_;
	bool last_transport_error_;
	std::uint64_t crc_failures_;
	std::uint64_t sync_recoveries_;
	std::uint64_t partial_reads_;
	std::uint64_t packets_decoded_;
	std::uint64_t packets_dropped_;
	bool last_transaction_timed_out_;
	PollTimingAccumulator poll_timing_accumulator_;
	PollCycleTiming last_poll_cycle_timing_;

	void workerLoop();
	bool performStartupSequence();
	bool pingDevice();
	bool writeTorqueEnable(bool enabled);
	bool writeImuRecalibration();
	bool writeProfileAcceleration();
	bool writeVelocityCommand(const VelocityCommand &command);
	bool writeHeartbeat();
	bool readState(OpencrState &state, PollCycleTiming *timing);
	bool readInitialState();
	bool readRequiredStateGroup(OpencrState &state);
	bool readImuStateGroup(OpencrState &state);
	bool probeRegisterRead(uint16_t address, uint16_t length);
	void probeRegistersOnStartup();
	bool readRegister(
		uint16_t address,
		uint16_t length,
		DxlStatusPacket &status_packet,
		bool is_failure_fatal);
	bool transactReadRegister(
		uint16_t address,
		uint16_t length,
		DxlStatusPacket &status_packet,
		bool is_failure_fatal);
	bool readBytes(
		uint16_t address,
		uint16_t length,
		std::vector<uint8_t> &output_vector);
	bool readUint8Register(uint16_t address, uint8_t &value);
	bool readInt32Register(uint16_t address, int32_t &value);
	bool readFloat32Register(uint16_t address, float &value);
	bool writeUint8Register(
		uint16_t address,
		uint8_t value,
		bool require_ack,
		const char *context);
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
	bool waitForReadStatusPacket(
		DxlStatusPacket &status_packet,
		uint16_t address,
		uint16_t expected_length,
		bool is_failure_fatal);
	void waitTransactionGap();
	void markTransactionComplete();
	void discardOptionalResponses(uint8_t target_id);
	void appendReadBytes(const uint8_t *data, std::size_t size);
	void updateParserStats(const DxlDecodeResult &decode_result);
	void maybeLogParserStats();
	void logDecodeDiagnostics(const DxlDecodeResult &decode_result);
	void logTimeoutDiagnostics(
		DxlInstruction instruction,
		uint8_t target_id,
		const std::vector<uint8_t> &parameters,
		bool header_seen,
		std::size_t rx_buffer_size);
	void logShortReadDiagnostics(
		uint16_t address,
		uint16_t requested_length,
		const DxlStatusPacket &status_packet);
	void logIgnoredStalePacket(
		uint16_t address,
		uint16_t requested_length,
		const DxlStatusPacket &status_packet);
	void logProbeDiagnostics(
		uint16_t address,
		uint16_t requested_length,
		const DxlStatusPacket &status_packet);
	void logRawBytes(
		const char *direction,
		const uint8_t *data,
		std::size_t size);
	void logSerialPacket(
		const char *direction,
		const std::vector<uint8_t> &packet);
	std::string formatBytes(const uint8_t *data, std::size_t size) const;
	std::string formatBytes(const std::vector<uint8_t> &data) const;
	const char *instructionToString(DxlInstruction instruction) const;
	const char *commandModeToString(OpencrCommandMode command_mode) const;
	const char *pollModeToString(OpencrPollMode poll_mode) const;
	void logReadRate(std::size_t size);
	void accumulatePollTiming(const PollCycleTiming &timing);
	void maybeLogPollTimingSummary();
	void resetPollTimingAccumulator();
	uint8_t parseUint8(const std::vector<uint8_t> &data, std::size_t offset) const;
	int32_t parseInt32(const std::vector<uint8_t> &data, std::size_t offset) const;
	float parseFloat32(const std::vector<uint8_t> &data, std::size_t offset) const;

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
