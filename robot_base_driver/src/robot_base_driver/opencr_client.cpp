#include "robot_base_driver/opencr_client.hpp"

using namespace robot::hw::base;

OpencrClient::OpencrClient(
	const rclcpp::Logger &logger,
	SerialPort *serial_port,
	const OpencrClientConfig &config)
: m_logger(logger),
	m_serial_port(serial_port),
	m_config(config),
	m_worker_thread(),
	m_command_mutex(),
	m_command_cv(),
	m_is_running(false),
	m_has_pending_velocity_command(false),
	m_pending_velocity_command({0.0, 0.0}),
	m_rx_buffer(),
	m_state_callback(nullptr),
	m_connected_callback(nullptr),
	m_error_callback(nullptr),
	m_heartbeat_counter(0U),
	m_read_rate_accumulator(0U),
	m_last_read_rate_log_time(std::chrono::steady_clock::now()),
	m_throttle_clock(RCL_STEADY_TIME)
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
	if (m_serial_port == nullptr || !m_serial_port->isOpen())
	{
		RCLCPP_ERROR(m_logger, "Cannot start OpenCR client without an open serial port");
		return false;
	}

	if (m_is_running.load())
	{
		return true;
	}

	m_state_callback = state_callback;
	m_connected_callback = connected_callback;
	m_error_callback = error_callback;
	m_rx_buffer.clear();
	m_is_running.store(true);
	m_worker_thread = std::thread(&OpencrClient::workerLoop, this);
	return true;
}

void OpencrClient::stop()
{
	m_is_running.store(false);
	m_command_cv.notify_all();

	if (m_worker_thread.joinable())
	{
		m_worker_thread.join();
	}
}

bool OpencrClient::isRunning() const
{
	return m_is_running.load();
}

void OpencrClient::setVelocityCommand(const VelocityCommand &command)
{
	{
		std::lock_guard<std::mutex> lock(m_command_mutex);
		m_pending_velocity_command = command;
		m_has_pending_velocity_command = true;
	}

	m_command_cv.notify_all();
}

void OpencrClient::workerLoop()
{
	if (!performStartupSequence())
	{
		if (m_error_callback)
		{
			m_error_callback("OpenCR startup sequence failed");
		}

		m_is_running.store(false);
		return;
	}

	if (m_connected_callback)
	{
		m_connected_callback();
	}

	auto last_poll_time = std::chrono::steady_clock::now();
	auto last_heartbeat_time = std::chrono::steady_clock::now();

	while (m_is_running.load())
	{
		VelocityCommand pending_command;
		bool has_command = false;
		{
			std::lock_guard<std::mutex> lock(m_command_mutex);
			has_command = m_has_pending_velocity_command;
			pending_command = m_pending_velocity_command;
			m_has_pending_velocity_command = false;
		}

		if (has_command)
		{
			if (!writeVelocityCommand(pending_command))
			{
				if (m_error_callback)
				{
					m_error_callback("Failed to write velocity command to OpenCR");
				}
				break;
			}
		}

		auto now = std::chrono::steady_clock::now();
		if (m_config.m_is_heartbeat_enabled &&
			std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_time).count() >= m_config.m_heartbeat_interval_ms)
		{
			if (!writeHeartbeat())
			{
				if (m_error_callback)
				{
					m_error_callback("Failed to write OpenCR heartbeat");
				}
				break;
			}

			last_heartbeat_time = now;
		}

		now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_poll_time).count() >= m_config.m_poll_interval_ms)
		{
			OpencrState state;
			if (!readState(state))
			{
				if (m_error_callback)
				{
					m_error_callback("Failed to poll OpenCR state block");
				}
				break;
			}

			if (m_state_callback)
			{
				m_state_callback(state);
			}

			RCLCPP_DEBUG(m_logger, "OpenCR poll cycle completed successfully");
			last_poll_time = now;
		}

		std::unique_lock<std::mutex> lock(m_command_mutex);
		m_command_cv.wait_for(lock, std::chrono::milliseconds(5));
	}

	m_is_running.store(false);
}

bool OpencrClient::performStartupSequence()
{
	if (m_config.m_startup_delay_ms > 0)
	{
		RCLCPP_INFO(m_logger, "Waiting %d ms before OpenCR startup sequence", m_config.m_startup_delay_ms);
		std::this_thread::sleep_for(std::chrono::milliseconds(m_config.m_startup_delay_ms));
	}

	if (!pingDevice())
	{
		RCLCPP_ERROR(m_logger, "OpenCR ping failed");
		return false;
	}

	RCLCPP_INFO(
		m_logger,
		"OpenCR ping succeeded: id=%u protocol=%.1f",
		static_cast<unsigned int>(m_config.m_opencr_id),
		m_config.m_protocol_version);

	if (m_config.m_is_startup_initial_state_read_required && !readInitialState())
	{
		RCLCPP_ERROR(m_logger, "OpenCR initial state read failed during startup");
		return false;
	}

	if (m_config.m_is_imu_recalibration_on_startup)
	{
		if (!writeImuRecalibration())
		{
			RCLCPP_WARN(m_logger, "OpenCR IMU recalibration command failed or timed out, continuing startup");
		}
		else
		{
			RCLCPP_INFO(m_logger, "OpenCR IMU recalibration command completed");
		}
	}

	if (!writeProfileAcceleration())
	{
		RCLCPP_WARN(m_logger, "OpenCR profile acceleration write failed or timed out, continuing startup");
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
	parameters.push_back(static_cast<uint8_t>(ControlTable::IMU_RECALIBRATION.m_address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::IMU_RECALIBRATION.m_address >> 8) & 0xFF));
	parameters.push_back(1U);

	if (m_config.m_is_imu_recalibration_ack_required)
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
		static_cast<int32_t>(command.m_linear_x_mps * 100.0),
		0,
		0,
		0,
		0,
		static_cast<int32_t>(command.m_angular_z_rps * 100.0)
	};

	parameters.push_back(static_cast<uint8_t>(ControlTable::CMD_VELOCITY_LINEAR_X.m_address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::CMD_VELOCITY_LINEAR_X.m_address >> 8) & 0xFF));

	for (std::size_t index = 0; index < 6U; ++index)
	{
		uint8_t *value_bytes = reinterpret_cast<uint8_t *>(&velocity_values[index]);
		parameters.insert(parameters.end(), value_bytes, value_bytes + sizeof(int32_t));
	}

	bool success = transact(DxlInstruction::Write, parameters, status_packet, true);
	if (success)
	{
		RCLCPP_DEBUG(
			m_logger,
			"OpenCR cmd_vel write succeeded: linear.x=%.3f angular.z=%.3f",
			command.m_linear_x_mps,
			command.m_angular_z_rps);
	}

	return success;
}

bool OpencrClient::writeProfileAcceleration()
{
	std::vector<uint8_t> parameters;
	parameters.reserve(2U + sizeof(int32_t) * 2U);

	double scaled_acceleration = 0.0;
	if (m_config.m_profile_acceleration_constant > std::numeric_limits<double>::epsilon())
	{
		scaled_acceleration = m_config.m_profile_acceleration / m_config.m_profile_acceleration_constant;
	}

	int32_t profile_values[2] = {
		static_cast<int32_t>(scaled_acceleration),
		static_cast<int32_t>(scaled_acceleration)
	};

	parameters.push_back(static_cast<uint8_t>(ControlTable::PROFILE_ACCELERATION_LEFT.m_address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::PROFILE_ACCELERATION_LEFT.m_address >> 8) & 0xFF));

	for (std::size_t index = 0; index < 2U; ++index)
	{
		uint8_t *value_bytes = reinterpret_cast<uint8_t *>(&profile_values[index]);
		parameters.insert(parameters.end(), value_bytes, value_bytes + sizeof(int32_t));
	}

	bool success = false;
	if (m_config.m_is_profile_acceleration_ack_required)
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
			m_logger,
			"OpenCR profile acceleration write succeeded: requested=%.3f raw=%.3f",
			m_config.m_profile_acceleration,
			scaled_acceleration);
	}

	return success;
}

bool OpencrClient::writeHeartbeat()
{
	std::vector<uint8_t> parameters;
	parameters.push_back(static_cast<uint8_t>(ControlTable::HEARTBEAT.m_address & 0xFF));
	parameters.push_back(static_cast<uint8_t>((ControlTable::HEARTBEAT.m_address >> 8) & 0xFF));
	parameters.push_back(m_heartbeat_counter);

	bool success = false;
	if (m_config.m_is_heartbeat_ack_required)
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
		++m_heartbeat_counter;
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

	if (status_packet.m_parameters.size() < ControlTable::READ_BLOCK_LENGTH)
	{
		RCLCPP_ERROR(
			m_logger,
			"OpenCR read returned %zu bytes but %u bytes were expected",
			status_packet.m_parameters.size(),
			ControlTable::READ_BLOCK_LENGTH);
		return false;
	}

	state.m_device_status = static_cast<int8_t>(status_packet.m_parameters[ControlTable::DEVICE_STATUS.m_address - ControlTable::READ_START_ADDRESS]);
	state.m_present_velocity_left = readInt32(status_packet.m_parameters, ControlTable::PRESENT_VELOCITY_LEFT.m_address);
	state.m_present_velocity_right = readInt32(status_packet.m_parameters, ControlTable::PRESENT_VELOCITY_RIGHT.m_address);
	state.m_present_position_left = readInt32(status_packet.m_parameters, ControlTable::PRESENT_POSITION_LEFT.m_address);
	state.m_present_position_right = readInt32(status_packet.m_parameters, ControlTable::PRESENT_POSITION_RIGHT.m_address);
	state.m_imu_angular_velocity_x = readFloat32(status_packet.m_parameters, ControlTable::IMU_ANGULAR_VELOCITY_X.m_address);
	state.m_imu_angular_velocity_y = readFloat32(status_packet.m_parameters, ControlTable::IMU_ANGULAR_VELOCITY_Y.m_address);
	state.m_imu_angular_velocity_z = readFloat32(status_packet.m_parameters, ControlTable::IMU_ANGULAR_VELOCITY_Z.m_address);
	state.m_imu_linear_acceleration_x = readFloat32(status_packet.m_parameters, ControlTable::IMU_LINEAR_ACCELERATION_X.m_address);
	state.m_imu_linear_acceleration_y = readFloat32(status_packet.m_parameters, ControlTable::IMU_LINEAR_ACCELERATION_Y.m_address);
	state.m_imu_linear_acceleration_z = readFloat32(status_packet.m_parameters, ControlTable::IMU_LINEAR_ACCELERATION_Z.m_address);
	state.m_imu_orientation_w = readFloat32(status_packet.m_parameters, ControlTable::IMU_ORIENTATION_W.m_address);
	state.m_imu_orientation_x = readFloat32(status_packet.m_parameters, ControlTable::IMU_ORIENTATION_X.m_address);
	state.m_imu_orientation_y = readFloat32(status_packet.m_parameters, ControlTable::IMU_ORIENTATION_Y.m_address);
	state.m_imu_orientation_z = readFloat32(status_packet.m_parameters, ControlTable::IMU_ORIENTATION_Z.m_address);
	state.m_has_imu_data = true;
	return true;
}

bool OpencrClient::readInitialState()
{
	int retries = std::max(1, m_config.m_startup_initial_state_read_retries);
	int retry_interval_ms = std::max(0, m_config.m_startup_initial_state_read_retry_interval_ms);

	for (int attempt = 1; attempt <= retries; ++attempt)
	{
		OpencrState initial_state;
		if (readState(initial_state))
		{
			RCLCPP_INFO(
				m_logger,
				"OpenCR initial state read succeeded on attempt %d/%d",
				attempt,
				retries);
			return true;
		}

		RCLCPP_WARN(
			m_logger,
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

bool OpencrClient::transact(
	DxlInstruction instruction,
	const std::vector<uint8_t> &parameters,
	DxlStatusPacket &status_packet,
	bool is_failure_fatal)
{
	prepareForTransaction();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		m_config.m_opencr_id,
		instruction,
		parameters);
	logSerialPacket("tx", packet);

	if (!m_serial_port->writeAll(packet.data(), packet.size(), m_config.m_response_timeout_ms))
	{
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(m_logger, "Serial write failed during OpenCR transaction");
		}
		else
		{
			RCLCPP_WARN(m_logger, "Serial write failed during optional OpenCR transaction");
		}
		return false;
	}

	if (!waitForStatusPacket(status_packet, instruction, m_config.m_opencr_id, parameters, is_failure_fatal))
	{
		return false;
	}

	if (status_packet.m_error != 0U)
	{
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(m_logger, "OpenCR status packet returned device error: 0x%02X", status_packet.m_error);
		}
		else
		{
			RCLCPP_WARN(m_logger, "Optional OpenCR status packet returned device error: 0x%02X", status_packet.m_error);
		}
		return false;
	}

	return true;
}

bool OpencrClient::transactWriteOnly(
	DxlInstruction instruction,
	const std::vector<uint8_t> &parameters,
	bool is_failure_fatal)
{
	prepareForTransaction();

	std::vector<uint8_t> packet = DxlPacketCodec::encodeInstructionPacket(
		m_config.m_opencr_id,
		instruction,
		parameters);
	logSerialPacket("tx", packet);

	if (!m_serial_port->writeAll(packet.data(), packet.size(), m_config.m_response_timeout_ms))
	{
		if (is_failure_fatal)
		{
			RCLCPP_ERROR(
				m_logger,
				"Serial write failed during OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(m_config.m_opencr_id));
		}
		else
		{
			RCLCPP_WARN(
				m_logger,
				"Serial write failed during optional OpenCR write-only transaction: instruction=%s id=%u",
				instructionToString(instruction),
				static_cast<unsigned int>(m_config.m_opencr_id));
		}
		return false;
	}

	discardOptionalResponses(m_config.m_opencr_id);
	return true;
}

bool OpencrClient::waitForStatusPacket(
	DxlStatusPacket &status_packet,
	DxlInstruction instruction,
	uint8_t target_id,
	const std::vector<uint8_t> &parameters,
	bool is_failure_fatal)
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_config.m_response_timeout_ms);
	bool packet_found = false;
	bool header_seen = DxlPacketCodec::containsPacketHeader(m_rx_buffer);

	while (m_is_running.load() || !m_rx_buffer.empty())
	{
		std::optional<DxlStatusPacket> decoded_packet = DxlPacketCodec::tryDecodeStatusPacket(m_rx_buffer, &packet_found);
		if (decoded_packet.has_value())
		{
			status_packet = decoded_packet.value();
			if (status_packet.m_id != target_id)
			{
				RCLCPP_WARN_THROTTLE(
					m_logger,
					m_throttle_clock,
					2000,
					"Ignoring OpenCR status packet from unexpected id=%u while waiting for %s id=%u",
					static_cast<unsigned int>(status_packet.m_id),
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
				RCLCPP_ERROR(m_logger, "Timed out waiting for OpenCR status packet");
			}
			else
			{
				RCLCPP_WARN(m_logger, "Timed out waiting for optional OpenCR status packet");
			}
			return false;
		}

		int timeout_ms = static_cast<int>(
			std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
		if (timeout_ms < 0)
		{
			timeout_ms = 0;
		}

		if (!m_serial_port->waitForReadable(timeout_ms))
		{
			continue;
		}

		uint8_t read_buffer[512];
		ssize_t read_size = m_serial_port->readSome(read_buffer, sizeof(read_buffer));
		if (read_size > 0)
		{
			appendReadBytes(read_buffer, static_cast<std::size_t>(read_size));
			header_seen = header_seen || DxlPacketCodec::containsPacketHeader(m_rx_buffer);
			logReadRate(static_cast<std::size_t>(read_size));
			continue;
		}

		if (read_size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			if (is_failure_fatal)
			{
				RCLCPP_ERROR(m_logger, "Serial read failed: errno=%d (%s)", errno, std::strerror(errno));
			}
			else
			{
				RCLCPP_WARN(m_logger, "Serial read failed during optional transaction: errno=%d (%s)", errno, std::strerror(errno));
			}
			return false;
		}
	}

	return false;
}

void OpencrClient::prepareForTransaction()
{
	if (m_rx_buffer.empty())
	{
		return;
	}

	if (!DxlPacketCodec::containsPacketHeader(m_rx_buffer))
	{
		RCLCPP_DEBUG(
			m_logger,
			"Clearing stale OpenCR rx buffer without a valid packet header: %zu bytes",
			m_rx_buffer.size());
		m_rx_buffer.clear();
	}
}

void OpencrClient::discardOptionalResponses(uint8_t target_id)
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

	while (std::chrono::steady_clock::now() < deadline)
	{
		if (!m_serial_port->waitForReadable(1))
		{
			break;
		}

		uint8_t read_buffer[256];
		ssize_t read_size = m_serial_port->readSome(read_buffer, sizeof(read_buffer));
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
		std::optional<DxlStatusPacket> decoded_packet = DxlPacketCodec::tryDecodeStatusPacket(m_rx_buffer, &packet_found);
		if (!decoded_packet.has_value())
		{
			break;
		}

		if (decoded_packet->m_id == target_id)
		{
			RCLCPP_DEBUG(
				m_logger,
				"Discarded optional OpenCR status packet from id=%u after write-only transaction",
				static_cast<unsigned int>(target_id));
		}
	}
}

void OpencrClient::appendReadBytes(const uint8_t *data, std::size_t size)
{
	m_rx_buffer.insert(m_rx_buffer.end(), data, data + size);
	logRawBytes("rx", data, size);
}

void OpencrClient::logTimeoutDiagnostics(
	DxlInstruction instruction,
	uint8_t target_id,
	const std::vector<uint8_t> &parameters,
	bool header_seen) const
{
	RCLCPP_WARN_THROTTLE(
		m_logger,
		m_throttle_clock,
		2000,
		"Timed out waiting for OpenCR status packet: instruction=%s id=%u parameters=[%s] rx_buffer_size=%zu header_seen=%s",
		instructionToString(instruction),
		static_cast<unsigned int>(target_id),
		formatBytes(parameters).c_str(),
		m_rx_buffer.size(),
		header_seen ? "true" : "false");
}

void OpencrClient::logRawBytes(const char *direction, const uint8_t *data, std::size_t size)
{
	if (!m_config.m_is_serial_packet_logging_enabled)
	{
		return;
	}

	RCLCPP_DEBUG_THROTTLE(
		m_logger,
		m_throttle_clock,
		500,
		"OpenCR %s raw bytes (%zu): %s",
		direction,
		size,
		formatBytes(data, size).c_str());
}

void OpencrClient::logSerialPacket(const char *direction, const std::vector<uint8_t> &packet)
{
	if (!m_config.m_is_serial_packet_logging_enabled)
	{
		return;
	}

	RCLCPP_DEBUG_THROTTLE(
		m_logger,
		m_throttle_clock,
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
	if (!m_config.m_is_read_rate_logging_enabled)
	{
		return;
	}

	m_read_rate_accumulator += size;
	auto now = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_read_rate_log_time).count();
	if (elapsed_ms >= 1000)
	{
		double bytes_per_sec = static_cast<double>(m_read_rate_accumulator) * 1000.0 / static_cast<double>(elapsed_ms);
		RCLCPP_INFO(m_logger, "OpenCR serial read throughput: %.1f B/s", bytes_per_sec);
		m_read_rate_accumulator = 0U;
		m_last_read_rate_log_time = now;
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
