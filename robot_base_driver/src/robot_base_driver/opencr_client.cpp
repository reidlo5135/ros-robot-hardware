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
	throttle_clock_(RCL_STEADY_TIME)
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
				if (error_callback_)
				{
					error_callback_("Failed to poll OpenCR state block");
				}
				break;
			}

			if (state_callback_)
			{
				state_callback_(state);
			}

			RCLCPP_DEBUG(logger_, "OpenCR poll cycle completed successfully");
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
	DxlStatusPacket status_packet;
	std::vector<uint8_t> parameters;
	parameters.push_back(static_cast<uint8_t>(ControlTable::READ_START_ADDRESS & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::READ_START_ADDRESS >> 8) & 0xFF));
	parameters.push_back(static_cast<uint8_t>(ControlTable::READ_BLOCK_LENGTH & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::READ_BLOCK_LENGTH >> 8) & 0xFF));

	if (!transact(DxlInstruction::Read, parameters, status_packet, true))
	{
		return false;
	}

	if (status_packet.parameters.size() < ControlTable::READ_BLOCK_LENGTH)
	{
		RCLCPP_ERROR(
			logger_,
			"OpenCR read returned %zu bytes but %u bytes were expected",
			status_packet.parameters.size(),
			ControlTable::READ_BLOCK_LENGTH);
		return false;
	}

	state.device_status = static_cast<int8_t>(status_packet.parameters[ControlTable::DEVICE_STATUS.address - ControlTable::READ_START_ADDRESS]);
	state.present_velocity_left = readInt32(status_packet.parameters, ControlTable::PRESENT_VELOCITY_LEFT.address);
	state.present_velocity_right = readInt32(status_packet.parameters, ControlTable::PRESENT_VELOCITY_RIGHT.address);
	state.present_position_left = readInt32(status_packet.parameters, ControlTable::PRESENT_POSITION_LEFT.address);
	state.present_position_right = readInt32(status_packet.parameters, ControlTable::PRESENT_POSITION_RIGHT.address);
	state.imu_angular_velocity_x = readFloat32(status_packet.parameters, ControlTable::IMU_ANGULAR_VELOCITY_X.address);
	state.imu_angular_velocity_y = readFloat32(status_packet.parameters, ControlTable::IMU_ANGULAR_VELOCITY_Y.address);
	state.imu_angular_velocity_z = readFloat32(status_packet.parameters, ControlTable::IMU_ANGULAR_VELOCITY_Z.address);
	state.imu_linear_acceleration_x = readFloat32(status_packet.parameters, ControlTable::IMU_LINEAR_ACCELERATION_X.address);
	state.imu_linear_acceleration_y = readFloat32(status_packet.parameters, ControlTable::IMU_LINEAR_ACCELERATION_Y.address);
	state.imu_linear_acceleration_z = readFloat32(status_packet.parameters, ControlTable::IMU_LINEAR_ACCELERATION_Z.address);
	state.imu_orientation_w = readFloat32(status_packet.parameters, ControlTable::IMU_ORIENTATION_W.address);
	state.imu_orientation_x = readFloat32(status_packet.parameters, ControlTable::IMU_ORIENTATION_X.address);
	state.imu_orientation_y = readFloat32(status_packet.parameters, ControlTable::IMU_ORIENTATION_Y.address);
	state.imu_orientation_z = readFloat32(status_packet.parameters, ControlTable::IMU_ORIENTATION_Z.address);
	state.has_imu_data = true;
	return true;
}

bool OpencrClient::readInitialState()
{
	int retries = std::max(1, config_.startup_initial_state_read_retries);
	int retry_interval_ms = std::max(0, config_.startup_initial_state_read_retry_interval_ms);

	for (int attempt = 1; attempt <= retries; ++attempt)
	{
		OpencrState initial_state;
		if (readState(initial_state))
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

bool OpencrClient::transact(DxlInstruction instruction, const std::vector<uint8_t> &parameters, DxlStatusPacket &status_packet, bool is_failure_fatal)
{
	prepareForTransaction();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		config_.opencr_id,
		instruction,
		parameters);
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

int32_t OpencrClient::readInt32(const std::vector<uint8_t> &data, uint16_t address) const
{
	std::size_t offset = static_cast<std::size_t>(address - ControlTable::READ_START_ADDRESS);
	int32_t value = 0;
	std::memcpy(&value, data.data() + static_cast<std::ptrdiff_t>(offset), sizeof(int32_t));
	return value;
}

float OpencrClient::readFloat32(const std::vector<uint8_t> &data, uint16_t address) const
{
	std::size_t offset = static_cast<std::size_t>(address - ControlTable::READ_START_ADDRESS);
	float value = 0.0F;
	std::memcpy(&value, data.data() + static_cast<std::ptrdiff_t>(offset), sizeof(float));
	return value;
}
