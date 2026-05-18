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

}  // namespace

OpencrClient::OpencrClient(const rclcpp::Logger &logger, SerialPort *serial_port, const OpencrClientConfig &config)
: logger_(logger),
	serial_port_(serial_port),
	config_(config),
	worker_thread_(),
	command_mutex_(),
	command_cv_(),
	is_running_(false),
	has_pending_velocity_command_(false),
	pending_velocity_command_({0.0, 0.0}),
	rx_buffer_(),
	state_callback_(nullptr),
	connected_callback_(nullptr),
	error_callback_(nullptr),
	heartbeat_counter_(0U),
	read_rate_accumulator_(0U),
	last_read_rate_log_time_(std::chrono::steady_clock::now()),
	throttle_clock_(RCL_STEADY_TIME),
	last_tx_packet_(),
	last_rx_packet_(),
	consecutive_poll_failures_(0)
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
			error_callback_("OpenCR startup sequence failed");
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

		if (has_command)
		{
			if (!writeVelocityCommand(pending_command))
			{
				if (error_callback_)
				{
					error_callback_("Failed to write velocity command to OpenCR");
				}
				break;
			}
		}

		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		if (config_.is_heartbeat_enabled &&
			std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_time).count() >= config_.heartbeat_interval_ms)
		{
			if (!writeHeartbeat())
			{
				if (error_callback_)
				{
					error_callback_("Failed to write OpenCR heartbeat");
				}
				break;
			}

			last_heartbeat_time = now;
		}

		now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_poll_time).count() >= config_.poll_interval_ms)
		{
			OpencrState state;
			if (!readState(state))
			{
				++consecutive_poll_failures_;
				RCLCPP_WARN_THROTTLE(
					logger_,
					throttle_clock_,
					2000,
					"OpenCR required poll failed: consecutive_failures=%d/%d mode=%s",
					consecutive_poll_failures_,
					config_.max_consecutive_poll_failures,
					pollModeToString(config_.poll_mode));

				if (consecutive_poll_failures_ >= config_.max_consecutive_poll_failures)
				{
					if (error_callback_)
					{
						error_callback_("Failed to poll required OpenCR state repeatedly");
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

	if (config_.is_startup_initial_state_read_required && !readInitialState())
	{
		RCLCPP_ERROR(logger_, "OpenCR initial state read failed during startup");
		return false;
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

	if (!writeProfileAcceleration())
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

bool OpencrClient::writeImuRecalibration()
{
	std::vector<uint8_t> parameters;
	parameters.push_back(static_cast<uint8_t>(ControlTable::IMU_RECALIBRATION.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::IMU_RECALIBRATION.address >> 8) & 0xFF));
	parameters.push_back(1U);

	if (config_.is_imu_recalibration_ack_required)
	{
		DxlStatusPacket status_packet;
		return transact(DxlInstruction::Write, parameters, status_packet, false);
	}

	return transactWriteOnly(DxlInstruction::Write, parameters, false);
}

bool OpencrClient::writeVelocityCommand(const VelocityCommand &command)
{
	DxlStatusPacket status_packet;
	std::vector<uint8_t> parameters;
	parameters.reserve(2U + sizeof(int32_t) * 6U);

	int32_t velocity_values[6] = {
		static_cast<int32_t>(command.linear_x_mps * 100.0),
		0,
		0,
		0,
		0,
		static_cast<int32_t>(command.angular_z_rps * 100.0)
	};

	parameters.push_back(static_cast<uint8_t>(ControlTable::CMD_VELOCITY_LINEAR_X.address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::CMD_VELOCITY_LINEAR_X.address >> 8) & 0xFF));

	for (std::size_t index = 0; index < 6U; ++index)
	{
		uint8_t *value_bytes = reinterpret_cast<uint8_t *>(&velocity_values[index]);
		parameters.insert(parameters.end(), value_bytes, value_bytes + sizeof(int32_t));
	}

	bool success = transact(DxlInstruction::Write, parameters, status_packet, true);
	if (success)
	{
		RCLCPP_DEBUG(
			logger_,
			"OpenCR cmd_vel write succeeded: linear.x=%.3f angular.z=%.3f",
			command.linear_x_mps,
			command.angular_z_rps);
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

	bool success = false;
	if (config_.is_profile_acceleration_ack_required)
	{
		DxlStatusPacket status_packet;
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

	bool success = false;
	if (config_.is_heartbeat_ack_required)
	{
		DxlStatusPacket status_packet;
		success = transact(DxlInstruction::Write, parameters, status_packet, true);
	}
	else
	{
		success = transactWriteOnly(DxlInstruction::Write, parameters, true);
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
	state.has_imu_data = false;

	if (!readRequiredStateGroup(state))
	{
		return false;
	}

	if (config_.poll_mode == OpencrPollMode::Full)
	{
		if (!readImuStateGroup(state))
		{
			state.has_imu_data = false;
			RCLCPP_WARN_THROTTLE(
				logger_,
				throttle_clock_,
				2000,
				"OpenCR IMU poll failed in full mode, continuing without IMU data");
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
		OpencrState initial_state;
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
	std::vector<uint8_t> device_status_bytes;
	if (!readBytes(ControlTable::DEVICE_STATUS.address, ControlTable::DEVICE_STATUS.length, device_status_bytes))
	{
		return false;
	}

	std::vector<uint8_t> wheel_block_bytes;
	static constexpr uint16_t WHEEL_BLOCK_START = ControlTable::PRESENT_VELOCITY_LEFT.address;
	static constexpr uint16_t WHEEL_BLOCK_LENGTH =
		(ControlTable::PRESENT_POSITION_RIGHT.address - ControlTable::PRESENT_VELOCITY_LEFT.address) +
		ControlTable::PRESENT_POSITION_RIGHT.length;

	if (!readBytes(WHEEL_BLOCK_START, WHEEL_BLOCK_LENGTH, wheel_block_bytes))
	{
		return false;
	}

	state.device_status = static_cast<int8_t>(parseUint8(device_status_bytes, 0U));
	state.present_velocity_left = parseInt32(wheel_block_bytes, ControlTable::PRESENT_VELOCITY_LEFT.address - WHEEL_BLOCK_START);
	state.present_velocity_right = parseInt32(wheel_block_bytes, ControlTable::PRESENT_VELOCITY_RIGHT.address - WHEEL_BLOCK_START);
	state.present_position_left = parseInt32(wheel_block_bytes, ControlTable::PRESENT_POSITION_LEFT.address - WHEEL_BLOCK_START);
	state.present_position_right = parseInt32(wheel_block_bytes, ControlTable::PRESENT_POSITION_RIGHT.address - WHEEL_BLOCK_START);
	return true;
}

bool OpencrClient::readImuStateGroup(OpencrState &state)
{
	std::vector<uint8_t> imu_motion_bytes;
	static constexpr uint16_t IMU_MOTION_START = ControlTable::IMU_ANGULAR_VELOCITY_X.address;
	static constexpr uint16_t IMU_MOTION_LENGTH =
		(ControlTable::IMU_LINEAR_ACCELERATION_Z.address - ControlTable::IMU_ANGULAR_VELOCITY_X.address) +
		ControlTable::IMU_LINEAR_ACCELERATION_Z.length;

	if (!readBytes(IMU_MOTION_START, IMU_MOTION_LENGTH, imu_motion_bytes))
	{
		return false;
	}

	std::vector<uint8_t> imu_orientation_bytes;
	static constexpr uint16_t IMU_ORIENTATION_START = ControlTable::IMU_ORIENTATION_W.address;
	static constexpr uint16_t IMU_ORIENTATION_LENGTH =
		(ControlTable::IMU_ORIENTATION_Z.address - ControlTable::IMU_ORIENTATION_W.address) +
		ControlTable::IMU_ORIENTATION_Z.length;

	if (!readBytes(IMU_ORIENTATION_START, IMU_ORIENTATION_LENGTH, imu_orientation_bytes))
	{
		return false;
	}

	state.imu_angular_velocity_x = parseFloat32(imu_motion_bytes, ControlTable::IMU_ANGULAR_VELOCITY_X.address - IMU_MOTION_START);
	state.imu_angular_velocity_y = parseFloat32(imu_motion_bytes, ControlTable::IMU_ANGULAR_VELOCITY_Y.address - IMU_MOTION_START);
	state.imu_angular_velocity_z = parseFloat32(imu_motion_bytes, ControlTable::IMU_ANGULAR_VELOCITY_Z.address - IMU_MOTION_START);
	state.imu_linear_acceleration_x = parseFloat32(imu_motion_bytes, ControlTable::IMU_LINEAR_ACCELERATION_X.address - IMU_MOTION_START);
	state.imu_linear_acceleration_y = parseFloat32(imu_motion_bytes, ControlTable::IMU_LINEAR_ACCELERATION_Y.address - IMU_MOTION_START);
	state.imu_linear_acceleration_z = parseFloat32(imu_motion_bytes, ControlTable::IMU_LINEAR_ACCELERATION_Z.address - IMU_MOTION_START);
	state.imu_orientation_w = parseFloat32(imu_orientation_bytes, ControlTable::IMU_ORIENTATION_W.address - IMU_ORIENTATION_START);
	state.imu_orientation_x = parseFloat32(imu_orientation_bytes, ControlTable::IMU_ORIENTATION_X.address - IMU_ORIENTATION_START);
	state.imu_orientation_y = parseFloat32(imu_orientation_bytes, ControlTable::IMU_ORIENTATION_Y.address - IMU_ORIENTATION_START);
	state.imu_orientation_z = parseFloat32(imu_orientation_bytes, ControlTable::IMU_ORIENTATION_Z.address - IMU_ORIENTATION_START);
	state.has_imu_data = true;
	return true;
}

bool OpencrClient::readBytes(uint16_t address, uint16_t length, std::vector<uint8_t> &output_vector)
{
	DxlStatusPacket status_packet;
	std::vector<uint8_t> parameters;
	parameters.reserve(4U);
	parameters.push_back(static_cast<uint8_t>(address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
	parameters.push_back(static_cast<uint8_t>(length & 0xFF));
	parameters.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));

	if (!transact(DxlInstruction::Read, parameters, status_packet, true))
	{
		return false;
	}

	if (status_packet.parameters.size() < length)
	{
		logShortReadDiagnostics(address, length, status_packet);
		return false;
	}

	output_vector.assign(status_packet.parameters.begin(), status_packet.parameters.begin() + static_cast<std::ptrdiff_t>(length));
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

bool OpencrClient::transact(DxlInstruction instruction, const std::vector<uint8_t> &parameters, DxlStatusPacket &status_packet, bool is_failure_fatal)
{
	prepareForTransaction();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		instruction,
		parameters);
	last_tx_packet_ = packet;
	logSerialPacket("tx", packet);

	if (!serial_port_->writeAll(packet.data(), packet.size(), config_.response_timeout_ms))
	{
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "Serial write failed during OpenCR transaction");
		}
		else
		{
			RCLCPP_WARN(logger_, "Serial write failed during optional OpenCR transaction");
		}
		return false;
	}

	if (!waitForStatusPacket(status_packet, instruction, config_.opencr_id, parameters, is_failure_fatal))
	{
		return false;
	}

	last_rx_packet_ = status_packet.raw_bytes;

	if (status_packet.error != 0U)
	{
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(logger_, "OpenCR status packet returned device error: 0x%02X", status_packet.error);
		}
		else
		{
			RCLCPP_WARN(logger_, "Optional OpenCR status packet returned device error: 0x%02X", status_packet.error);
		}
		return false;
	}

	return true;
}

bool OpencrClient::transactWriteOnly(DxlInstruction instruction, const std::vector<uint8_t> &parameters, bool is_failure_fatal)
{
	prepareForTransaction();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		instruction,
		parameters);
	last_tx_packet_ = packet;
	logSerialPacket("tx", packet);

	if (!serial_port_->writeAll(packet.data(), packet.size(), config_.response_timeout_ms))
	{
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
			RCLCPP_WARN(
				logger_,
				"Serial write failed during optional OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(config_.opencr_id));
		}
		return false;
	}

	discardOptionalResponses(config_.opencr_id);
	return true;
}

bool OpencrClient::waitForStatusPacket(DxlStatusPacket &status_packet, DxlInstruction instruction, uint8_t target_id, const std::vector<uint8_t> &parameters, bool is_failure_fatal)
{
	std::chrono::steady_clock::time_point deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.response_timeout_ms);
	bool packet_found = false;
	bool header_seen = DxlPacketCodec::containsPacketHeader(rx_buffer_);

	while (is_running_.load() || !rx_buffer_.empty())
	{
		std::optional<DxlStatusPacket> decoded_packet = DxlPacketCodec::tryDecodeStatusPacket(rx_buffer_, &packet_found);
		if (decoded_packet.has_value())
		{
			status_packet = decoded_packet.value();
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

		if (std::chrono::steady_clock::now() >= deadline)
		{
			logTimeoutDiagnostics(instruction, target_id, parameters, header_seen);
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Timed out waiting for OpenCR status packet");
			}
			else
			{
				RCLCPP_WARN(logger_, "Timed out waiting for optional OpenCR status packet");
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
			appendReadBytes(read_buffer, static_cast<std::size_t>(read_size));
			header_seen = header_seen || DxlPacketCodec::containsPacketHeader(rx_buffer_);
			logReadRate(static_cast<std::size_t>(read_size));
			continue;
		}

		if (read_size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(logger_, "Serial read failed: errno=%d (%s)", errno, std::strerror(errno));
			}
			else
			{
				RCLCPP_WARN(logger_, "Serial read failed during optional transaction: errno=%d (%s)", errno, std::strerror(errno));
			}
			return false;
		}
	}

	return false;
}

void OpencrClient::prepareForTransaction()
{
	if (rx_buffer_.empty())
	{
		return;
	}

	if (!DxlPacketCodec::containsPacketHeader(rx_buffer_))
	{
		RCLCPP_DEBUG(
			logger_,
			"Clearing stale OpenCR rx buffer without a valid packet header: %zu bytes",
			rx_buffer_.size());
		rx_buffer_.clear();
	}
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

	bool packet_found = false;
	while (true)
	{
		std::optional<DxlStatusPacket> decoded_packet = DxlPacketCodec::tryDecodeStatusPacket(rx_buffer_, &packet_found);
		if (!decoded_packet.has_value())
		{
			break;
		}

		if (decoded_packet->id == target_id)
		{
			RCLCPP_DEBUG(
				logger_,
				"Discarded optional OpenCR status packet from id=%u after write-only transaction",
				static_cast<unsigned int>(target_id));
		}
	}
}

void OpencrClient::appendReadBytes(const uint8_t *data, std::size_t size)
{
	rx_buffer_.insert(rx_buffer_.end(), data, data + size);
	logRawBytes("rx", data, size);
}

void OpencrClient::logTimeoutDiagnostics(DxlInstruction instruction, uint8_t target_id, const std::vector<uint8_t> &parameters, bool header_seen)
{
	RCLCPP_WARN_THROTTLE(
		logger_,
		throttle_clock_,
		2000,
		"Timed out waiting for OpenCR status packet: instruction=%s id=%u parameters=[%s] rx_buffer_size=%zu header_seen=%s",
		instructionToString(instruction),
		static_cast<unsigned int>(target_id),
		formatBytes(parameters).c_str(),
		rx_buffer_.size(),
		boolToString(header_seen));
}

void OpencrClient::logShortReadDiagnostics(uint16_t address, uint16_t requested_length, const DxlStatusPacket &status_packet)
{
	RCLCPP_ERROR(
		logger_,
		"OpenCR read returned fewer parameters than expected: address=%u length=%u returned=%zu",
		static_cast<unsigned int>(address),
		static_cast<unsigned int>(requested_length),
		status_packet.parameters.size());

	if (config_.is_serial_packet_logging_enabled)
	{
		RCLCPP_WARN(
			logger_,
			"OpenCR short read diagnostics: tx_packet=%s rx_packet=%s",
			formatBytes(last_tx_packet_).c_str(),
			formatBytes(status_packet.raw_bytes).c_str());
	}
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
