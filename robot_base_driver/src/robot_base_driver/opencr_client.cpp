#include "robot_base_driver/opencr_client.hpp"

using namespace robot::hw::base;

namespace
{

const char *boolToString(bool value)
{
	if (value)
	{
		return "true";
	}

	return "false";
}

constexpr uint8_t PROBE_ERROR_UNAVAILABLE = 0xFF;
constexpr auto PARSER_STATS_LOG_INTERVAL = std::chrono::seconds(5);
constexpr double TURTLEBOT3_VELOCITY_CONSTANT_VALUE = 1263.632956882;
constexpr double TURTLEBOT3_MAX_GOAL_VELOCITY = 337.0;

}  // namespace

OpencrClient::OpencrClient(const rclcpp::Logger &logger, SerialPort *serial_port, const OpencrClientConfig &config)
: logger_(logger),
	serial_port_(serial_port),
	config_(config),
	worker_thread_(),
	command_mutex_(),
	transaction_mutex_(),
	command_cv_(),
	is_running_(false),
	has_pending_velocity_command_(false),
	pending_velocity_command_({0.0, 0.0, ""}),
	rx_buffer_(),
	state_callback_(nullptr),
	connected_callback_(nullptr),
	error_callback_(nullptr),
	heartbeat_counter_(0U),
	read_rate_accumulator_(0U),
	last_read_rate_log_time_(std::chrono::steady_clock::now()),
	last_transaction_time_(std::chrono::steady_clock::now() - std::chrono::microseconds(config.transaction_gap_us)),
	last_parser_stats_log_time_(std::chrono::steady_clock::now()),
	throttle_clock_(RCL_STEADY_TIME),
	last_tx_packet_(),
	last_rx_packet_(),
	consecutive_poll_failures_(0),
	last_transport_error_(false),
	crc_failures_(0U),
	sync_recoveries_(0U),
	partial_reads_(0U),
	packets_decoded_(0U),
	packets_dropped_(0U)
{
}

OpencrClient::~OpencrClient()
{
	stop();
}

bool OpencrClient::start(
	const std::function<void(const OpencrState &)> &state_callback,
	const std::function<void()> &connected_callback,
	const std::function<void(const std::string &)> &error_callback)
{
	if (serial_port_ == nullptr || !serial_port_->isOpen())
	{
		RCLCPP_ERROR(logger_, "Cannot start OpenCR client without an open serial port");
		return false;
	}

	if (is_running_.load())
	{
		return true;
	}

	state_callback_ = state_callback;
	connected_callback_ = connected_callback;
	error_callback_ = error_callback;
	rx_buffer_.clear();
	last_tx_packet_.clear();
	last_rx_packet_.clear();
	consecutive_poll_failures_ = 0;
	last_transport_error_ = false;
	crc_failures_ = 0U;
	sync_recoveries_ = 0U;
	partial_reads_ = 0U;
	packets_decoded_ = 0U;
	packets_dropped_ = 0U;
	last_read_rate_log_time_ = std::chrono::steady_clock::now();
	last_transaction_time_ =
		std::chrono::steady_clock::now() - std::chrono::microseconds(config_.transaction_gap_us);
	last_parser_stats_log_time_ = std::chrono::steady_clock::now();
	is_running_.store(true);
	worker_thread_ = std::thread(&OpencrClient::workerLoop, this);
	return true;
}

void OpencrClient::stop()
{
	is_running_.store(false);
	command_cv_.notify_all();

	if (worker_thread_.joinable())
	{
		worker_thread_.join();
	}
}

bool OpencrClient::isRunning() const
{
	return is_running_.load();
}

void OpencrClient::setVelocityCommand(const VelocityCommand &command)
{
	{
		std::lock_guard<std::mutex> lock(command_mutex_);
		pending_velocity_command_ = command;
		has_pending_velocity_command_ = true;
	}

	command_cv_.notify_all();
}

void OpencrClient::workerLoop()
{
	if (!performStartupSequence())
	{
		if (error_callback_)
		{
			error_callback_("startup_failure:OpenCR startup sequence failed");
		}

		is_running_.store(false);
		return;
	}

	if (connected_callback_)
	{
		connected_callback_();
	}

	std::chrono::steady_clock::time_point last_poll_time = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point last_heartbeat_time = std::chrono::steady_clock::now();

	while (is_running_.load())
	{
		VelocityCommand pending_command;
		bool has_command = false;
		{
			std::lock_guard<std::mutex> lock(command_mutex_);
			has_command = has_pending_velocity_command_;
			pending_command = pending_velocity_command_;
			has_pending_velocity_command_ = false;
		}

		if (has_command && !writeVelocityCommand(pending_command))
		{
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"OpenCR cmd_vel write failed, continuing bringup validation");
		}

		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		if (config_.is_heartbeat_enabled &&
			std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_time).count() >= config_.heartbeat_interval_ms)
		{
			if (!writeHeartbeat())
			{
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"OpenCR heartbeat write failed, continuing");
			}

			last_heartbeat_time = now;
		}

		now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_poll_time).count() >= config_.poll_interval_ms)
		{
			OpencrState state = {};
			if (!readState(state))
			{
				++consecutive_poll_failures_;

				if (last_transport_error_)
				{
					if (error_callback_)
					{
						error_callback_("hard_serial_error:OpenCR poll transport failure");
					}
					break;
				}

				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"OpenCR poll failure: consecutive_failures=%d/%d mode=%s reconnect_on_poll_failure=%s",
					consecutive_poll_failures_,
					config_.max_consecutive_poll_failures,
					pollModeToString(config_.poll_mode),
					boolToString(config_.reconnect_on_poll_failure));

				if (config_.reconnect_on_poll_failure &&
					consecutive_poll_failures_ >= config_.max_consecutive_poll_failures)
				{
					if (error_callback_)
					{
						error_callback_("poll_failure:OpenCR required poll failed repeatedly");
					}
					break;
				}
			}
			else
			{
				consecutive_poll_failures_ = 0;
				if (state_callback_)
				{
					state_callback_(state);
				}

				RCLCPP_DEBUG(logger_, "OpenCR poll cycle completed successfully");
			}

			last_poll_time = now;
		}

		maybeLogParserStats();

		std::unique_lock<std::mutex> lock(command_mutex_);
		command_cv_.wait_for(lock, std::chrono::milliseconds(5));
	}

	is_running_.store(false);
}

bool OpencrClient::performStartupSequence()
{
	if (config_.startup_delay_ms > 0)
	{
		RCLCPP_INFO(logger_, "Waiting %d ms before OpenCR startup sequence", config_.startup_delay_ms);
		std::this_thread::sleep_for(std::chrono::milliseconds(config_.startup_delay_ms));
	}

	if (!pingDevice())
	{
		RCLCPP_ERROR(logger_, "OpenCR ping failed");
		return false;
	}

	RCLCPP_INFO(
		logger_,
		"OpenCR ping succeeded: id=%u protocol=%.1f",
		static_cast<unsigned int>(config_.opencr_id),
		config_.protocol_version);

	if (config_.probe_registers_on_startup)
	{
		probeRegistersOnStartup();
	}

	if (!config_.is_heartbeat_enabled)
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR heartbeat is disabled. Official TurtleBot3 bringup keeps heartbeat active at 100 ms; motor commands may not take effect without it.");
	}

	uint8_t startup_device_status = 0U;
	if (readUint8Register(ControlTable::DEVICE_STATUS.address, startup_device_status))
	{
		RCLCPP_INFO(
			logger_,
			"OpenCR startup device_status read: raw=%u signed=%d",
			static_cast<unsigned int>(startup_device_status),
			static_cast<int>(static_cast<int8_t>(startup_device_status)));
	}
	else
	{
		RCLCPP_WARN(logger_, "OpenCR startup device_status read failed before motor enable sequence");
	}

	if (config_.is_startup_initial_state_read_required && !readInitialState())
	{
		RCLCPP_ERROR(logger_, "OpenCR initial state read failed during startup");
		return false;
	}

	if (config_.is_motor_torque_enable_on_startup)
	{
		if (!writeTorqueEnable(true))
		{
			RCLCPP_WARN(
				logger_,
				"OpenCR motor torque enable command failed during startup. cmd_vel may be ignored until motor power/torque becomes ready.");
		}
	}

	if (config_.is_imu_recalibration_on_startup)
	{
		if (!writeImuRecalibration())
		{
			RCLCPP_WARN(logger_, "OpenCR IMU recalibration command failed or timed out, continuing startup");
		}
		else
		{
			RCLCPP_INFO(logger_, "OpenCR IMU recalibration command completed");
		}
	}

	if (config_.is_profile_acceleration_on_startup && !writeProfileAcceleration())
	{
		RCLCPP_WARN(logger_, "OpenCR profile acceleration write failed or timed out, continuing startup");
	}

	return true;
}

bool OpencrClient::pingDevice()
{
	DxlStatusPacket status_packet;
	std::vector<uint8_t> parameters;
	return transact(DxlInstruction::Ping, parameters, status_packet, true);
}

bool OpencrClient::writeTorqueEnable(bool enabled)
{
	const uint8_t raw_value = enabled ? 1U : 0U;
	if (!writeUint8Register(
			ControlTable::MOTOR_TORQUE_ENABLE.address,
			raw_value,
			config_.is_motor_torque_enable_ack_required,
			"motor torque enable"))
	{
		return false;
	}

	uint8_t readback_value = 0U;
	if (readUint8Register(ControlTable::MOTOR_TORQUE_ENABLE.address, readback_value))
	{
		RCLCPP_INFO(
			logger_,
			"OpenCR motor torque state: requested=%u readback=%u",
			static_cast<unsigned int>(raw_value),
			static_cast<unsigned int>(readback_value));
	}
	else
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR motor torque enable write completed but readback failed at address=%u",
			static_cast<unsigned int>(ControlTable::MOTOR_TORQUE_ENABLE.address));
	}

	return true;
}

bool OpencrClient::writeImuRecalibration()
{
	std::vector<uint8_t> parameters;
	parameters.push_back(static_cast<uint8_t>(ControlTable::IMU_RECALIBRATION.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::IMU_RECALIBRATION.address >> 8) & 0xFF));
	parameters.push_back(1U);

	DxlStatusPacket status_packet;
	if (config_.is_imu_recalibration_ack_required)
	{
		return transact(DxlInstruction::Write, parameters, status_packet, false);
	}

	return transactWriteOnly(DxlInstruction::Write, parameters, false);
}

bool OpencrClient::writeVelocityCommand(const VelocityCommand &command)
{
	const double expected_left_wheel_linear_mps =
		command.linear_x_mps - (command.angular_z_rps * config_.wheel_separation_m * 0.5);
	const double expected_right_wheel_linear_mps =
		command.linear_x_mps + (command.angular_z_rps * config_.wheel_separation_m * 0.5);
	const int32_t expected_left_goal_velocity = static_cast<int32_t>(
		std::clamp(expected_left_wheel_linear_mps * TURTLEBOT3_VELOCITY_CONSTANT_VALUE,
			-TURTLEBOT3_MAX_GOAL_VELOCITY,
			TURTLEBOT3_MAX_GOAL_VELOCITY));
	const int32_t expected_right_goal_velocity = static_cast<int32_t>(
		std::clamp(expected_right_wheel_linear_mps * TURTLEBOT3_VELOCITY_CONSTANT_VALUE,
			-TURTLEBOT3_MAX_GOAL_VELOCITY,
			TURTLEBOT3_MAX_GOAL_VELOCITY));

	if (config_.command_mode == OpencrCommandMode::WheelVelocity)
	{
		RCLCPP_ERROR_THROTTLE(
			logger_,
			throttle_clock_,
			2000,
			"OpenCR wheel_velocity mode selected, but per-wheel register mapping is not verified in this workspace. "
			"Suppressing command write for safety: source=%s linear.x=%.3f angular.z=%.3f expected_left_goal_velocity=%d expected_right_goal_velocity=%d",
			command.source.c_str(),
			command.linear_x_mps,
			command.angular_z_rps,
			expected_left_goal_velocity,
			expected_right_goal_velocity);
		return false;
	}

	DxlStatusPacket status_packet;
	std::vector<uint8_t> parameters;
	parameters.reserve(2U + sizeof(int32_t) * 6U);

	const int32_t linear_x_raw = static_cast<int32_t>(command.linear_x_mps * 100.0);
	const int32_t angular_z_raw = static_cast<int32_t>(command.angular_z_rps * 100.0);
	int32_t body_twist_values[6] = {
		linear_x_raw,
		0,
		0,
		0,
		0,
		angular_z_raw
	};

	parameters.push_back(static_cast<uint8_t>(ControlTable::CMD_VELOCITY_LINEAR_X.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::CMD_VELOCITY_LINEAR_X.address >> 8) & 0xFF));

	for (std::size_t index = 0; index < 6U; ++index)
	{
		uint8_t *value_bytes = reinterpret_cast<uint8_t *>(&body_twist_values[index]);
		parameters.insert(parameters.end(), value_bytes, value_bytes + sizeof(int32_t));
	}

	bool success = transact(DxlInstruction::Write, parameters, status_packet, false);
	if (!success)
	{
		RCLCPP_WARN_THROTTLE(
			logger_,
			throttle_clock_,
			1000,
			"OpenCR cmd_vel command failed: source=%s command_mode=%s linear.x=%.3f angular.z=%.3f "
			"linear_x_raw=%d angular_z_raw=%d expected_left_goal_velocity=%d expected_right_goal_velocity=%d "
			"register_start=%u payload=%s",
			command.source.c_str(),
			commandModeToString(config_.command_mode),
			command.linear_x_mps,
			command.angular_z_rps,
			linear_x_raw,
			angular_z_raw,
			expected_left_goal_velocity,
			expected_right_goal_velocity,
			static_cast<unsigned int>(ControlTable::CMD_VELOCITY_LINEAR_X.address),
			formatBytes(parameters).c_str());
	}
	if (success)
	{
		if (config_.debug_motor_command)
		{
			RCLCPP_INFO_THROTTLE(
				logger_,
				throttle_clock_,
				1000,
				"OpenCR cmd_vel command acknowledged: source=%s command_mode=%s start_addr=%u register_span=%u "
				"linear.x=%.3f angular.z=%.3f linear_x_raw=%d angular_z_raw=%d expected_left_goal_velocity=%d "
				"expected_right_goal_velocity=%d payload=%s",
				command.source.c_str(),
				commandModeToString(config_.command_mode),
				static_cast<unsigned int>(ControlTable::CMD_VELOCITY_LINEAR_X.address),
				static_cast<unsigned int>((ControlTable::CMD_VELOCITY_ANGULAR_Z.address - ControlTable::CMD_VELOCITY_LINEAR_X.address) + ControlTable::CMD_VELOCITY_ANGULAR_Z.length),
				command.linear_x_mps,
				command.angular_z_rps,
				linear_x_raw,
				angular_z_raw,
				expected_left_goal_velocity,
				expected_right_goal_velocity,
				formatBytes(parameters).c_str());
		}
	}

	return success;
}

bool OpencrClient::writeProfileAcceleration()
{
	std::vector<uint8_t> parameters;
	parameters.reserve(2U + sizeof(int32_t) * 2U);

	double scaled_acceleration = 0.0;
	if (config_.profile_acceleration_constant > std::numeric_limits<double>::epsilon())
	{
		scaled_acceleration = config_.profile_acceleration / config_.profile_acceleration_constant;
	}

	int32_t profile_values[2] = {
		static_cast<int32_t>(scaled_acceleration),
		static_cast<int32_t>(scaled_acceleration)
	};

	parameters.push_back(static_cast<uint8_t>(ControlTable::PROFILE_ACCELERATION_LEFT.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::PROFILE_ACCELERATION_LEFT.address >> 8) & 0xFF));

	for (std::size_t index = 0; index < 2U; ++index)
	{
		uint8_t *value_bytes = reinterpret_cast<uint8_t *>(&profile_values[index]);
		parameters.insert(parameters.end(), value_bytes, value_bytes + sizeof(int32_t));
	}

	DxlStatusPacket status_packet;
	bool success = false;
	if (config_.is_profile_acceleration_ack_required)
	{
		success = transact(DxlInstruction::Write, parameters, status_packet, false);
	}
	else
	{
		success = transactWriteOnly(DxlInstruction::Write, parameters, false);
	}

	if (success)
	{
		RCLCPP_INFO(
			logger_,
			"OpenCR profile acceleration write succeeded: requested=%.3f raw=%.3f",
			config_.profile_acceleration,
			scaled_acceleration);
	}

	return success;
}

bool OpencrClient::writeHeartbeat()
{
	std::vector<uint8_t> parameters;
	parameters.push_back(static_cast<uint8_t>(ControlTable::HEARTBEAT.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::HEARTBEAT.address >> 8) & 0xFF));
	parameters.push_back(heartbeat_counter_);

	DxlStatusPacket status_packet;
	bool success = false;
	if (config_.is_heartbeat_ack_required)
	{
		success = transact(DxlInstruction::Write, parameters, status_packet, false);
	}
	else
	{
		success = transactWriteOnly(DxlInstruction::Write, parameters, false);
	}

	if (success)
	{
		++heartbeat_counter_;
	}

	return success;
}

bool OpencrClient::readState(OpencrState &state)
{
	state = {};
	state.device_status = -1;
	state.has_device_status = false;
	state.has_motor_torque_enable = false;
	state.motor_torque_enabled = false;
	state.has_imu_data = false;
	last_transport_error_ = false;

	if (!readRequiredStateGroup(state))
	{
		return false;
	}

	if (config_.require_device_status)
	{
		uint8_t device_status_value = 0U;
		if (!readUint8Register(ControlTable::DEVICE_STATUS.address, device_status_value))
		{
			return false;
		}

		state.device_status = static_cast<int8_t>(device_status_value);
		state.has_device_status = true;
	}
	else if (config_.poll_device_status)
	{
		uint8_t device_status_value = 0U;
		if (readUint8Register(ControlTable::DEVICE_STATUS.address, device_status_value))
		{
			state.device_status = static_cast<int8_t>(device_status_value);
			state.has_device_status = true;
		}
		else
		{
			last_transport_error_ = false;
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"Optional DEVICE_STATUS read failed, continuing without device status");
		}
	}

	uint8_t torque_enable_value = 0U;
	if (readUint8Register(ControlTable::MOTOR_TORQUE_ENABLE.address, torque_enable_value))
	{
		state.has_motor_torque_enable = true;
		state.motor_torque_enabled = torque_enable_value != 0U;
	}
	else
	{
		last_transport_error_ = false;
		RCLCPP_WARN_THROTTLE(
			logger_,
			throttle_clock_,
			2000,
			"Optional MOTOR_TORQUE_ENABLE read failed, continuing without torque state");
	}

	if (config_.poll_mode == OpencrPollMode::Full)
	{
		if (!readImuStateGroup(state))
		{
			if (config_.require_imu)
			{
				return false;
			}

			last_transport_error_ = false;
			state.has_imu_data = false;
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"Optional IMU poll failed in full mode, continuing without IMU data");
		}
	}

	return true;
}

bool OpencrClient::readInitialState()
{
	int retries = std::max(1, config_.startup_initial_state_read_retries);
	int retry_interval_ms = std::max(0, config_.startup_initial_state_read_retry_interval_ms);

	for (int attempt = 1; attempt <= retries; ++attempt)
	{
		OpencrState initial_state = {};
		if (readRequiredStateGroup(initial_state))
		{
			RCLCPP_INFO(
				logger_,
				"OpenCR initial state read succeeded on attempt %d/%d",
				attempt,
				retries);
			return true;
		}

		RCLCPP_WARN(
			logger_,
			"OpenCR initial state read failed on attempt %d/%d",
			attempt,
			retries);

		if (attempt < retries && retry_interval_ms > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
		}
	}

	return false;
}

bool OpencrClient::readRequiredStateGroup(OpencrState &state)
{
	if (!readInt32Register(ControlTable::PRESENT_VELOCITY_LEFT.address, state.present_velocity_left))
	{
		return false;
	}

	if (!readInt32Register(ControlTable::PRESENT_VELOCITY_RIGHT.address, state.present_velocity_right))
	{
		return false;
	}

	if (!readInt32Register(ControlTable::PRESENT_POSITION_LEFT.address, state.present_position_left))
	{
		return false;
	}

	if (!readInt32Register(ControlTable::PRESENT_POSITION_RIGHT.address, state.present_position_right))
	{
		return false;
	}

	return true;
}

bool OpencrClient::readImuStateGroup(OpencrState &state)
{
	if (!readFloat32Register(ControlTable::IMU_ANGULAR_VELOCITY_X.address, state.imu_angular_velocity_x))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ANGULAR_VELOCITY_Y.address, state.imu_angular_velocity_y))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ANGULAR_VELOCITY_Z.address, state.imu_angular_velocity_z))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_LINEAR_ACCELERATION_X.address, state.imu_linear_acceleration_x))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_LINEAR_ACCELERATION_Y.address, state.imu_linear_acceleration_y))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_LINEAR_ACCELERATION_Z.address, state.imu_linear_acceleration_z))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ORIENTATION_W.address, state.imu_orientation_w))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ORIENTATION_X.address, state.imu_orientation_x))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ORIENTATION_Y.address, state.imu_orientation_y))
	{
		return false;
	}

	if (!readFloat32Register(ControlTable::IMU_ORIENTATION_Z.address, state.imu_orientation_z))
	{
		return false;
	}

	state.has_imu_data = true;
	return true;
}

bool OpencrClient::probeRegisterRead(uint16_t address, uint16_t length)
{
	DxlStatusPacket status_packet = {};
	bool success = readRegister(address, length, status_packet, false);
	logProbeDiagnostics(address, length, status_packet);
	return success;
}

void OpencrClient::probeRegistersOnStartup()
{
	RCLCPP_INFO(logger_, "Probing OpenCR registers individually after ping");

	(void)probeRegisterRead(18U, 1U);
	(void)probeRegisterRead(128U, 4U);
	(void)probeRegisterRead(132U, 4U);
	(void)probeRegisterRead(136U, 4U);
	(void)probeRegisterRead(140U, 4U);
	(void)probeRegisterRead(150U, 4U);
	(void)probeRegisterRead(170U, 4U);
}

bool OpencrClient::readRegister(uint16_t address, uint16_t length, DxlStatusPacket &status_packet, bool is_failure_fatal)
{
	return transactReadRegister(address, length, status_packet, is_failure_fatal);
}

bool OpencrClient::transactReadRegister(uint16_t address, uint16_t length, DxlStatusPacket &status_packet, bool is_failure_fatal)
{
	std::vector<uint8_t> parameters;
	parameters.reserve(4U);
	parameters.push_back(static_cast<uint8_t>(address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
	parameters.push_back(static_cast<uint8_t>(length & 0xFF));
	parameters.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));

	std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
	last_transport_error_ = false;
	waitTransactionGap();
	rx_buffer_.clear();
	(void)serial_port_->flushInput();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		DxlInstruction::Read,
		parameters);
	last_tx_packet_ = packet;
	logSerialPacket("tx", packet);

	if (!serial_port_->writeAll(packet.data(), packet.size(), config_.response_timeout_ms))
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "Serial write failed during OpenCR read transaction");
		}
		else
		{
			RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Serial write failed during OpenCR read transaction");
		}
		markTransactionComplete();
		return false;
	}

	if (!serial_port_->drainOutput())
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "Serial drain failed during OpenCR read transaction");
		}
		else
		{
			RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Serial drain failed during OpenCR read transaction");
		}
		markTransactionComplete();
		return false;
	}

	bool success = waitForReadStatusPacket(status_packet, address, length, is_failure_fatal);
	markTransactionComplete();
	return success;
}

bool OpencrClient::readBytes(uint16_t address, uint16_t length, std::vector<uint8_t> &output_vector)
{
	DxlStatusPacket status_packet = {};
	if (!readRegister(address, length, status_packet, false))
	{
		return false;
	}

	if (status_packet.parameters.size() != length)
	{
		logShortReadDiagnostics(address, length, status_packet);
		last_transport_error_ = false;
		return false;
	}

	output_vector.assign(
		status_packet.parameters.begin(),
		status_packet.parameters.begin() + static_cast<std::ptrdiff_t>(length));
	return true;
}

bool OpencrClient::readUint8Register(uint16_t address, uint8_t &value)
{
	std::vector<uint8_t> bytes;
	if (!readBytes(address, 1U, bytes))
	{
		return false;
	}

	value = parseUint8(bytes, 0U);
	return true;
}

bool OpencrClient::readInt32Register(uint16_t address, int32_t &value)
{
	std::vector<uint8_t> bytes;
	if (!readBytes(address, 4U, bytes))
	{
		return false;
	}

	value = parseInt32(bytes, 0U);
	return true;
}

bool OpencrClient::readFloat32Register(uint16_t address, float &value)
{
	std::vector<uint8_t> bytes;
	if (!readBytes(address, 4U, bytes))
	{
		return false;
	}

	value = parseFloat32(bytes, 0U);
	return true;
}

bool OpencrClient::writeUint8Register(uint16_t address, uint8_t value, bool require_ack, const char *context)
{
	std::vector<uint8_t> parameters;
	parameters.reserve(3U);
	parameters.push_back(static_cast<uint8_t>(address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
	parameters.push_back(value);

	DxlStatusPacket status_packet = {};
	const bool success = require_ack
		? transact(DxlInstruction::Write, parameters, status_packet, false)
		: transactWriteOnly(DxlInstruction::Write, parameters, false);
	if (!success)
	{
		RCLCPP_WARN_THROTTLE(
			logger_,
			throttle_clock_,
			1000,
			"OpenCR %s write failed: address=%u value=%u require_ack=%s",
			context,
			static_cast<unsigned int>(address),
			static_cast<unsigned int>(value),
			boolToString(require_ack));
		return false;
	}

	RCLCPP_INFO(
		logger_,
		"OpenCR %s write sent: address=%u value=%u require_ack=%s",
		context,
		static_cast<unsigned int>(address),
		static_cast<unsigned int>(value),
		boolToString(require_ack));
	return true;
}

bool OpencrClient::transact(DxlInstruction instruction, const std::vector<uint8_t> &parameters, DxlStatusPacket &status_packet, bool is_failure_fatal)
{
	std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
	last_transport_error_ = false;
	waitTransactionGap();
	rx_buffer_.clear();
	(void)serial_port_->flushInput();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		instruction,
		parameters);
	last_tx_packet_ = packet;
	logSerialPacket("tx", packet);

	if (!serial_port_->writeAll(packet.data(), packet.size(), config_.response_timeout_ms))
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "Serial write failed during OpenCR transaction");
		}
		else
		{
			RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Serial write failed during OpenCR transaction");
		}
		markTransactionComplete();
		return false;
	}

	if (!serial_port_->drainOutput())
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "Serial drain failed during OpenCR transaction");
		}
		else
		{
			RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Serial drain failed during OpenCR transaction");
		}
		markTransactionComplete();
		return false;
	}

	if (!waitForStatusPacket(status_packet, instruction, config_.opencr_id, parameters, is_failure_fatal))
	{
		markTransactionComplete();
		return false;
	}

	last_rx_packet_ = status_packet.raw_bytes;

	if (status_packet.error != 0U)
	{
		last_transport_error_ = false;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "OpenCR status packet returned device error: 0x%02X", status_packet.error);
		}
		else
		{
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"OpenCR status packet returned device error: 0x%02X",
				status_packet.error);
		}
		markTransactionComplete();
		return false;
	}

	markTransactionComplete();
	return true;
}

bool OpencrClient::transactWriteOnly(DxlInstruction instruction, const std::vector<uint8_t> &parameters, bool is_failure_fatal)
{
	std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
	last_transport_error_ = false;
	waitTransactionGap();
	rx_buffer_.clear();
	(void)serial_port_->flushInput();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		instruction,
		parameters);
	last_tx_packet_ = packet;
	logSerialPacket("tx", packet);

	if (!serial_port_->writeAll(packet.data(), packet.size(), config_.response_timeout_ms))
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(
				logger_,
				"Serial write failed during OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(config_.opencr_id));
		}
		else
		{
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"Serial write failed during OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(config_.opencr_id));
		}
		markTransactionComplete();
		return false;
	}

	if (!serial_port_->drainOutput())
	{
		last_transport_error_ = true;
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(
				logger_,
				"Serial drain failed during OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(config_.opencr_id));
		}
		else
		{
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"Serial drain failed during OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(config_.opencr_id));
		}
		markTransactionComplete();
		return false;
	}

	discardOptionalResponses(config_.opencr_id);
	markTransactionComplete();
	return true;
}

bool OpencrClient::waitForReadStatusPacket(DxlStatusPacket &status_packet, uint16_t address, uint16_t expected_length, bool is_failure_fatal)
{
	std::chrono::steady_clock::time_point deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.response_timeout_ms);
	bool header_seen = DxlPacketCodec::containsPacketHeader(rx_buffer_);

	while (is_running_.load() || !rx_buffer_.empty())
	{
		DxlDecodeResult decode_result = DxlPacketCodec::tryDecodeStatusPacket(rx_buffer_);
		updateParserStats(decode_result);
		logDecodeDiagnostics(decode_result);
		header_seen = header_seen || decode_result.header_seen;

		if (decode_result.packet.has_value())
		{
			status_packet = decode_result.packet.value();
			last_rx_packet_ = status_packet.raw_bytes;

			if (status_packet.id != config_.opencr_id)
			{
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"Ignoring OpenCR read status packet from unexpected id=%u while waiting for address=%u length=%u",
					static_cast<unsigned int>(status_packet.id),
					static_cast<unsigned int>(address),
					static_cast<unsigned int>(expected_length));
				continue;
			}

			if (status_packet.error != 0U)
			{
				last_transport_error_ = false;
				if (is_failure_fatal)
				{
					RCLCPP_ERROR(
						logger_,
						"OpenCR read status packet returned device error: address=%u length=%u error=0x%02X",
						static_cast<unsigned int>(address),
						static_cast<unsigned int>(expected_length),
						status_packet.error);
				}
				else
				{
					RCLCPP_WARN_THROTTLE(
						logger_,
						throttle_clock_,
						2000,
						"OpenCR read status packet returned device error: address=%u length=%u error=0x%02X",
						static_cast<unsigned int>(address),
						static_cast<unsigned int>(expected_length),
						status_packet.error);
				}
				return false;
			}

			if (status_packet.parameters.size() != expected_length)
			{
				logIgnoredStalePacket(address, expected_length, status_packet);
				continue;
			}

			return true;
		}

		if (decode_result.bytes_dropped > 0U ||
			decode_result.crc_failed ||
			(decode_result.packet_found && !decode_result.is_partial_packet))
		{
			continue;
		}

		if (std::chrono::steady_clock::now() >= deadline)
		{
			last_transport_error_ = false;
			std::vector<uint8_t> parameters;
			parameters.reserve(4U);
			parameters.push_back(static_cast<uint8_t>(address & 0xFF));
			parameters.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
			parameters.push_back(static_cast<uint8_t>(expected_length & 0xFF));
			parameters.push_back(static_cast<uint8_t>((expected_length >> 8) & 0xFF));
			logTimeoutDiagnostics(
				DxlInstruction::Read,
				config_.opencr_id,
				parameters,
				header_seen,
				rx_buffer_.size());
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Timed out waiting for OpenCR read status packet");
			}
			else
			{
				RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Timed out waiting for OpenCR read status packet");
			}
			return false;
		}

		int timeout_ms = static_cast<int>(
			std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
		if (timeout_ms < 0)
		{
			timeout_ms = 0;
		}

		if (!serial_port_->waitForReadable(timeout_ms))
		{
			continue;
		}

		uint8_t read_buffer[512];
		ssize_t read_size = serial_port_->readSome(read_buffer, sizeof(read_buffer));
		if (read_size > 0)
		{
			if (static_cast<std::size_t>(read_size) < sizeof(read_buffer))
			{
				++partial_reads_;
			}

			appendReadBytes(read_buffer, static_cast<std::size_t>(read_size));
			logReadRate(static_cast<std::size_t>(read_size));
			continue;
		}

		if (read_size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			last_transport_error_ = true;
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Serial read failed: errno=%d (%s)", errno, std::strerror(errno));
			}
			else
			{
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"Serial read failed: errno=%d (%s)",
					errno,
					std::strerror(errno));
			}
			return false;
		}
	}

	last_transport_error_ = false;
	return false;
}

bool OpencrClient::waitForStatusPacket(DxlStatusPacket &status_packet, DxlInstruction instruction, uint8_t target_id, const std::vector<uint8_t> &parameters, bool is_failure_fatal)
{
	std::chrono::steady_clock::time_point deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.response_timeout_ms);
	bool header_seen = DxlPacketCodec::containsPacketHeader(rx_buffer_);

	while (is_running_.load() || !rx_buffer_.empty())
	{
		DxlDecodeResult decode_result = DxlPacketCodec::tryDecodeStatusPacket(rx_buffer_);
		updateParserStats(decode_result);
		logDecodeDiagnostics(decode_result);
		header_seen = header_seen || decode_result.header_seen;

		if (decode_result.packet.has_value())
		{
			status_packet = decode_result.packet.value();
			if (status_packet.id != target_id)
			{
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"Ignoring OpenCR status packet from unexpected id=%u while waiting for %s id=%u",
					static_cast<unsigned int>(status_packet.id),
					instructionToString(instruction),
					static_cast<unsigned int>(target_id));
				continue;
			}

			return true;
		}

		if (decode_result.bytes_dropped > 0U ||
			decode_result.crc_failed ||
			(decode_result.packet_found && !decode_result.is_partial_packet))
		{
			continue;
		}

		if (std::chrono::steady_clock::now() >= deadline)
		{
			last_transport_error_ = false;
			logTimeoutDiagnostics(instruction, target_id, parameters, header_seen, rx_buffer_.size());
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Timed out waiting for OpenCR status packet");
			}
			else
			{
				RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 2000, "Timed out waiting for OpenCR status packet");
			}
			return false;
		}

		int timeout_ms = static_cast<int>(
			std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
		if (timeout_ms < 0)
		{
			timeout_ms = 0;
		}

		if (!serial_port_->waitForReadable(timeout_ms))
		{
			continue;
		}

		uint8_t read_buffer[512];
		ssize_t read_size = serial_port_->readSome(read_buffer, sizeof(read_buffer));
		if (read_size > 0)
		{
			if (static_cast<std::size_t>(read_size) < sizeof(read_buffer))
			{
				++partial_reads_;
			}

			appendReadBytes(read_buffer, static_cast<std::size_t>(read_size));
			logReadRate(static_cast<std::size_t>(read_size));
			continue;
		}

		if (read_size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			last_transport_error_ = true;
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Serial read failed: errno=%d (%s)", errno, std::strerror(errno));
			}
			else
			{
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"Serial read failed: errno=%d (%s)",
					errno,
					std::strerror(errno));
			}
			return false;
		}
	}

	last_transport_error_ = false;
	return false;
}

void OpencrClient::waitTransactionGap()
{
	if (config_.transaction_gap_us <= 0)
	{
		return;
	}

	std::chrono::microseconds transaction_gap(config_.transaction_gap_us);
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point earliest_next_transaction = last_transaction_time_ + transaction_gap;
	if (now < earliest_next_transaction)
	{
		std::this_thread::sleep_for(earliest_next_transaction - now);
	}
}

void OpencrClient::markTransactionComplete()
{
	last_transaction_time_ = std::chrono::steady_clock::now();
}

void OpencrClient::discardOptionalResponses(uint8_t target_id)
{
	std::chrono::steady_clock::time_point deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

	while (std::chrono::steady_clock::now() < deadline)
	{
		if (!serial_port_->waitForReadable(1))
		{
			break;
		}

		uint8_t read_buffer[256];
		ssize_t read_size = serial_port_->readSome(read_buffer, sizeof(read_buffer));
		if (read_size > 0)
		{
			appendReadBytes(read_buffer, static_cast<std::size_t>(read_size));
			logReadRate(static_cast<std::size_t>(read_size));
		}
		else
		{
			break;
		}
	}

	while (true)
	{
		DxlDecodeResult decode_result = DxlPacketCodec::tryDecodeStatusPacket(rx_buffer_);
		updateParserStats(decode_result);
		logDecodeDiagnostics(decode_result);
		if (decode_result.packet.has_value())
		{
			DxlStatusPacket decoded_packet = decode_result.packet.value();
			if (decoded_packet.id == target_id)
			{
				RCLCPP_DEBUG(
					logger_,
					"Discarded optional OpenCR status packet from id=%u after write-only transaction",
					static_cast<unsigned int>(target_id));
			}
			continue;
		}

		if (decode_result.bytes_dropped > 0U ||
			decode_result.crc_failed ||
			(decode_result.packet_found && !decode_result.is_partial_packet))
		{
			continue;
		}

		if (!decode_result.packet.has_value())
		{
			break;
		}
	}
}

void OpencrClient::appendReadBytes(const uint8_t *data, std::size_t size)
{
	rx_buffer_.insert(rx_buffer_.end(), data, data + size);
	logRawBytes("rx", data, size);
}

void OpencrClient::updateParserStats(const DxlDecodeResult &decode_result)
{
	if (decode_result.crc_failed)
	{
		++crc_failures_;
		++packets_dropped_;
	}

	if (decode_result.bytes_dropped > 0U)
	{
		sync_recoveries_ += decode_result.bytes_dropped;
		++packets_dropped_;
	}

	if (decode_result.packet.has_value())
	{
		++packets_decoded_;
	}
}

void OpencrClient::maybeLogParserStats()
{
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	if (now - last_parser_stats_log_time_ < PARSER_STATS_LOG_INTERVAL)
	{
		return;
	}

	RCLCPP_INFO(
		logger_,
		"OpenCR parser stats: packets_decoded=%llu packets_dropped=%llu crc_failures=%llu sync_recoveries=%llu partial_reads=%llu rx_buffer_size=%zu",
		static_cast<unsigned long long>(packets_decoded_),
		static_cast<unsigned long long>(packets_dropped_),
		static_cast<unsigned long long>(crc_failures_),
		static_cast<unsigned long long>(sync_recoveries_),
		static_cast<unsigned long long>(partial_reads_),
		rx_buffer_.size());
	last_parser_stats_log_time_ = now;
}

void OpencrClient::logDecodeDiagnostics(const DxlDecodeResult &decode_result)
{
	if (!config_.is_serial_packet_logging_enabled)
	{
		return;
	}

	if (!(decode_result.packet_found || decode_result.bytes_dropped > 0U || decode_result.crc_failed))
	{
		return;
	}

	RCLCPP_DEBUG_THROTTLE(
		logger_,
		throttle_clock_,
		500,
		"OpenCR decode: packet_found=%s packet_decoded=%s partial=%s crc_failed=%s bytes_dropped=%zu buffer_before=%zu buffer_after=%zu",
		boolToString(decode_result.packet_found),
		boolToString(decode_result.packet.has_value()),
		boolToString(decode_result.is_partial_packet),
		boolToString(decode_result.crc_failed),
		decode_result.bytes_dropped,
		decode_result.buffer_size_before,
		decode_result.buffer_size_after);

	if (decode_result.crc_failed)
	{
		RCLCPP_WARN_THROTTLE(
			logger_,
			throttle_clock_,
			2000,
			"OpenCR packet CRC failure detected: buffer_before=%zu buffer_after=%zu dropped=%zu",
			decode_result.buffer_size_before,
			decode_result.buffer_size_after,
			decode_result.bytes_dropped);
	}

	if (decode_result.packet.has_value())
	{
		RCLCPP_DEBUG_THROTTLE(
			logger_,
			throttle_clock_,
			500,
			"OpenCR packet extracted: id=%u error=0x%02X raw=%s",
			static_cast<unsigned int>(decode_result.packet->id),
			decode_result.packet->error,
			formatBytes(decode_result.packet->raw_bytes).c_str());
	}
}

void OpencrClient::logTimeoutDiagnostics(DxlInstruction instruction, uint8_t target_id, const std::vector<uint8_t> &parameters, bool header_seen, std::size_t rx_buffer_size)
{
	RCLCPP_WARN_THROTTLE(
		logger_,
		throttle_clock_,
		2000,
		"Timed out waiting for OpenCR status packet: instruction=%s id=%u parameters=[%s] rx_buffer_size=%zu header_seen=%s",
		instructionToString(instruction),
		static_cast<unsigned int>(target_id),
		formatBytes(parameters).c_str(),
		rx_buffer_size,
		boolToString(header_seen));

	if (config_.is_serial_packet_logging_enabled)
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR timeout diagnostics: tx_packet=%s last_rx_packet=%s rx_buffer=%s",
			formatBytes(last_tx_packet_).c_str(),
			formatBytes(last_rx_packet_).c_str(),
			formatBytes(rx_buffer_).c_str());
	}
}

void OpencrClient::logShortReadDiagnostics(uint16_t address, uint16_t requested_length, const DxlStatusPacket &status_packet)
{
	RCLCPP_WARN_THROTTLE(
		logger_,
		throttle_clock_,
		2000,
		"OpenCR short read: address=%u requested_length=%u returned_length=%zu status_error=0x%02X parameter_bytes=%s",
		static_cast<unsigned int>(address),
		static_cast<unsigned int>(requested_length),
		status_packet.parameters.size(),
		status_packet.error,
		formatBytes(status_packet.parameters).c_str());

	if (config_.is_serial_packet_logging_enabled)
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR short read packets: tx_packet=%s rx_raw=%s",
			formatBytes(last_tx_packet_).c_str(),
			formatBytes(status_packet.raw_bytes).c_str());
	}
}

void OpencrClient::logIgnoredStalePacket(uint16_t address, uint16_t requested_length, const DxlStatusPacket &status_packet)
{
	RCLCPP_WARN_THROTTLE(
		logger_,
		throttle_clock_,
		2000,
		"Ignoring stale OpenCR status packet: current_address=%u current_length=%u stale_length=%zu stale_parameter_bytes=%s",
		static_cast<unsigned int>(address),
		static_cast<unsigned int>(requested_length),
		status_packet.parameters.size(),
		formatBytes(status_packet.parameters).c_str());

	if (config_.is_serial_packet_logging_enabled)
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR stale packet raw bytes: %s",
			formatBytes(status_packet.raw_bytes).c_str());
	}
}

void OpencrClient::logProbeDiagnostics(uint16_t address, uint16_t requested_length, const DxlStatusPacket &status_packet)
{
	const uint8_t status_error =
		status_packet.raw_bytes.empty() ? PROBE_ERROR_UNAVAILABLE : status_packet.error;
	RCLCPP_INFO(
		logger_,
		"OpenCR startup probe: address=%u requested_length=%u returned_length=%zu status_error=0x%02X parameter_bytes=%s",
		static_cast<unsigned int>(address),
		static_cast<unsigned int>(requested_length),
		status_packet.parameters.size(),
		status_error,
		formatBytes(status_packet.parameters).c_str());
}

void OpencrClient::logRawBytes(const char *direction, const uint8_t *data, std::size_t size)
{
	if (!config_.is_serial_packet_logging_enabled)
	{
		return;
	}

	RCLCPP_DEBUG_THROTTLE(
		logger_,
		throttle_clock_,
		500,
		"OpenCR %s raw bytes (%zu): %s",
		direction,
		size,
		formatBytes(data, size).c_str());
}

void OpencrClient::logSerialPacket(const char *direction, const std::vector<uint8_t> &packet)
{
	if (!config_.is_serial_packet_logging_enabled)
	{
		return;
	}

	RCLCPP_DEBUG_THROTTLE(
		logger_,
		throttle_clock_,
		2000,
		"OpenCR %s packet: %s",
		direction,
		formatBytes(packet).c_str());
}

std::string OpencrClient::formatBytes(const uint8_t *data, std::size_t size) const
{
	if (data == nullptr || size == 0U)
	{
		return "";
	}

	std::ostringstream stream;
	stream << std::hex << std::setfill('0');
	for (std::size_t index = 0; index < size; ++index)
	{
		stream << std::setw(2) << static_cast<int>(data[index]);
		if (index + 1U < size)
		{
			stream << ' ';
		}
	}

	return stream.str();
}

std::string OpencrClient::formatBytes(const std::vector<uint8_t> &data) const
{
	return formatBytes(data.data(), data.size());
}

const char *OpencrClient::instructionToString(DxlInstruction instruction) const
{
	switch (instruction)
	{
		case DxlInstruction::Ping:
			return "Ping";
		case DxlInstruction::Read:
			return "Read";
		case DxlInstruction::Write:
			return "Write";
		case DxlInstruction::Status:
			return "Status";
		default:
			return "Unknown";
	}
}

const char *OpencrClient::commandModeToString(OpencrCommandMode command_mode) const
{
	switch (command_mode)
	{
		case OpencrCommandMode::BodyTwist:
			return "body_twist";
		case OpencrCommandMode::WheelVelocity:
			return "wheel_velocity";
		default:
			return "unknown";
	}
}

const char *OpencrClient::pollModeToString(OpencrPollMode poll_mode) const
{
	switch (poll_mode)
	{
		case OpencrPollMode::Minimal:
			return "minimal";
		case OpencrPollMode::Full:
			return "full";
		default:
			return "unknown";
	}
}

void OpencrClient::logReadRate(std::size_t size)
{
	if (!config_.is_read_rate_logging_enabled)
	{
		return;
	}

	read_rate_accumulator_ += size;
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	long long elapsed_ms =
		std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_rate_log_time_).count();
	if (elapsed_ms >= 1000)
	{
		double bytes_per_sec =
			static_cast<double>(read_rate_accumulator_) * 1000.0 / static_cast<double>(elapsed_ms);
		RCLCPP_INFO(logger_, "OpenCR serial read throughput: %.1f B/s", bytes_per_sec);
		read_rate_accumulator_ = 0U;
		last_read_rate_log_time_ = now;
	}
}

uint8_t OpencrClient::parseUint8(const std::vector<uint8_t> &data, std::size_t offset) const
{
	return data[offset];
}

int32_t OpencrClient::parseInt32(const std::vector<uint8_t> &data, std::size_t offset) const
{
	int32_t value = 0;
	std::memcpy(&value, data.data() + static_cast<std::ptrdiff_t>(offset), sizeof(int32_t));
	return value;
}

float OpencrClient::parseFloat32(const std::vector<uint8_t> &data, std::size_t offset) const
{
	float value = 0.0F;
	std::memcpy(&value, data.data() + static_cast<std::ptrdiff_t>(offset), sizeof(float));
	return value;
}
