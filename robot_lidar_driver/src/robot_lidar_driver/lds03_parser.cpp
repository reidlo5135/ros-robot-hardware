#include "robot_lidar_driver/lds03_parser.hpp"

using namespace robot::hw::lidar;

Lds03Parser::Lds03Parser(const rclcpp::Logger &logger, std::function<rclcpp::Time()> now_cb, bool log_raw_packet, bool log_packet_error)
: m_logger(logger),
	m_now_cb(now_cb),
	m_is_raw_packet_logging_enabled(log_raw_packet),
	m_is_packet_error_logging_enabled(log_packet_error),
	m_throttle_clock(RCL_STEADY_TIME),
	m_has_scan_sync(false),
	m_current_scan_frequency_hz(0.0),
	m_current_points()
{
}

bool Lds03Parser::consume(RingBuffer &buffer, std::vector<LidarScan> &completed_scans)
{
	bool made_progress = false;

	while (true)
	{
		if (!alignToPacketStart(buffer, made_progress))
		{
			return made_progress;
		}

		if (buffer.available() < HEADER_SIZE)
		{
			return made_progress;
		}

		std::array<uint8_t, HEADER_SIZE> header{};
		if (buffer.peek(header.data(), HEADER_SIZE) != HEADER_SIZE)
		{
			return made_progress;
		}

		const std::size_t sample_count = static_cast<std::size_t>(header[3]);
		if (sample_count == 0U || sample_count > MAX_SAMPLES_PER_PACKET)
		{
			logPacketWarning("Invalid LDS-03 sample count detected while parsing");
			buffer.consume(1U);
			made_progress = true;
			continue;
		}

		const std::size_t total_packet_size = HEADER_SIZE + (sample_count * SAMPLE_SIZE);
		if (buffer.available() < total_packet_size)
		{
			return made_progress;
		}

		std::vector<uint8_t> packet(total_packet_size, 0U);
		buffer.peek(packet.data(), total_packet_size);
		buffer.consume(total_packet_size);
		made_progress = true;

		std::vector<LidarPoint> packet_points;
		bool is_ring_start = false;
		double scan_frequency_hz = 0.0;
		if (!decodePacket(packet, packet_points, is_ring_start, scan_frequency_hz))
		{
			continue;
		}

		if (!m_has_scan_sync)
		{
			if (!is_ring_start)
			{
				continue;
			}

			m_has_scan_sync = true;
			m_current_scan_frequency_hz = scan_frequency_hz;
			m_current_points = packet_points;
			continue;
		}

		if (is_ring_start)
		{
			if (!m_current_points.empty())
			{
				LidarScan completed_scan;
				completed_scan.points = m_current_points;
				completed_scan.stamp = m_now_cb ? m_now_cb() : rclcpp::Clock(RCL_SYSTEM_TIME).now();
				completed_scan.scan_frequency_hz = m_current_scan_frequency_hz;
				completed_scans.push_back(completed_scan);
			}

			m_current_points = packet_points;
			m_current_scan_frequency_hz = scan_frequency_hz;
			continue;
		}

		m_current_points.insert(m_current_points.end(), packet_points.begin(), packet_points.end());
	}
}

void Lds03Parser::reset()
{
	m_has_scan_sync = false;
	m_current_scan_frequency_hz = 0.0;
	m_current_points.clear();
}

double Lds03Parser::degreesToRadians(double degrees)
{
	return degrees * PI / 180.0;
}

double Lds03Parser::normalizeRadians(double angle_rad)
{
	double normalized = std::fmod(angle_rad, 2.0 * PI);
	if (normalized < 0.0)
	{
		normalized += 2.0 * PI;
	}

	return normalized;
}

uint16_t Lds03Parser::readLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U));
}

std::string Lds03Parser::packetToHexString(const std::vector<uint8_t> &packet)
{
	std::ostringstream stream;
	stream << std::hex << std::setfill('0');

	for (std::size_t index = 0U; index < packet.size(); ++index)
	{
		stream << std::setw(2) << static_cast<int>(packet[index]);
		if ((index + 1U) < packet.size())
		{
			stream << ' ';
		}
	}

	return stream.str();
}

bool Lds03Parser::alignToPacketStart(RingBuffer &buffer, bool &made_progress)
{
	while (buffer.available() >= 2U)
	{
		uint8_t first_byte = 0U;
		uint8_t second_byte = 0U;
		buffer.peek(0U, first_byte);
		buffer.peek(1U, second_byte);

		if (first_byte == PACKET_SYNC_LOW && second_byte == PACKET_SYNC_HIGH)
		{
			return true;
		}

		buffer.consume(1U);
		made_progress = true;
	}

	if (buffer.available() == 1U)
	{
		uint8_t first_byte = 0U;
		buffer.peek(0U, first_byte);
		if (first_byte != PACKET_SYNC_LOW)
		{
			buffer.consume(1U);
			made_progress = true;
		}
	}

	return false;
}

bool Lds03Parser::decodePacket(const std::vector<uint8_t> &packet, std::vector<LidarPoint> &points, bool &is_ring_start, double &scan_frequency_hz)
{
	if (packet.size() < HEADER_SIZE)
	{
		logPacketWarning("Received a packet smaller than the LDS-03 header");
		return false;
	}

	if (m_is_raw_packet_logging_enabled)
	{
		logRawPacket(packet);
	}

	const std::size_t sample_count = static_cast<std::size_t>(packet[3]);
	if (packet.size() != HEADER_SIZE + (sample_count * SAMPLE_SIZE))
	{
		logPacketWarning("Packet size does not match LDS-03 sample count");
		return false;
	}

	if ((packet[4] & 0x01U) == 0U || (packet[6] & 0x01U) == 0U)
	{
		logPacketWarning("LDS-03 packet angle checkbit validation failed");
		return false;
	}

	const uint16_t first_angle_raw = readLe16(packet.data() + 4U);
	const uint16_t last_angle_raw = readLe16(packet.data() + 6U);
	const uint16_t target_checksum = readLe16(packet.data() + 8U);
	const uint16_t computed_checksum = computeChecksum(packet, sample_count);
	if (computed_checksum != target_checksum)
	{
		logPacketWarning("LDS-03 packet checksum validation failed");
		return false;
	}

	const uint8_t packet_type = packet[2] & 0x01U;
	if (packet_type > 1U)
	{
		logPacketWarning("Unsupported LDS-03 packet type");
		return false;
	}

	is_ring_start = packet_type == 1U;
	scan_frequency_hz = static_cast<double>((packet[2] & 0xFEU) >> 1U);

	const uint16_t first_angle_q6 = first_angle_raw >> 1U;
	const uint16_t last_angle_q6 = last_angle_raw >> 1U;
	const double interval_q6 = computeAngleIncrementQ6(first_angle_q6, last_angle_q6, sample_count);

	points.clear();
	points.reserve(sample_count);

	for (std::size_t sample_index = 0U; sample_index < sample_count; ++sample_index)
	{
		const std::size_t offset = HEADER_SIZE + (sample_index * SAMPLE_SIZE);
		const uint8_t sample_byte0 = packet[offset + 0U];
		const uint8_t sample_byte1 = packet[offset + 1U];
		const uint8_t sample_byte2 = packet[offset + 2U];

		const uint16_t distance_q2 = static_cast<uint16_t>((static_cast<uint16_t>(sample_byte2) * 64U) + (sample_byte1 >> 2U));
		const double quality = static_cast<double>(((sample_byte1 & 0x03U) * 64U) + (sample_byte0 >> 2U));
		const bool has_exposure = (sample_byte0 & 0x01U) != 0U;
		const double corrected_angle_q6 = computeCorrectedAngleQ6(first_angle_q6, interval_q6, sample_index, distance_q2);
		const double angle_rad = normalizeRadians(degreesToRadians(corrected_angle_q6 / 64.0));

		LidarPoint point;
		point.angle_rad = angle_rad;
		point.range_m = static_cast<double>(distance_q2) / 1000.0;
		point.intensity = has_exposure ? 255.0 : std::min<double>(quality, 254.0);
		points.push_back(point);
	}

	return true;
}

uint16_t Lds03Parser::computeChecksum(const std::vector<uint8_t> &packet, std::size_t sample_count) const
{
	const uint16_t sample_num_and_ct = static_cast<uint16_t>(
		static_cast<uint16_t>(packet[2]) |
		(static_cast<uint16_t>(packet[3]) << 8U));
	const uint16_t first_angle_raw = readLe16(packet.data() + 4U);
	const uint16_t last_angle_raw = readLe16(packet.data() + 6U);

	uint16_t checksum = PACKET_HEADER_VALUE;
	checksum ^= sample_num_and_ct;
	checksum ^= first_angle_raw;
	checksum ^= last_angle_raw;

	for (std::size_t sample_index = 0U; sample_index < sample_count; ++sample_index)
	{
		const std::size_t offset = HEADER_SIZE + (sample_index * SAMPLE_SIZE);
		checksum ^= static_cast<uint16_t>(packet[offset]);
		checksum ^= static_cast<uint16_t>(
			static_cast<uint16_t>(packet[offset + 1U]) |
			(static_cast<uint16_t>(packet[offset + 2U]) << 8U));
	}

	return checksum;
}

double Lds03Parser::computeAngleIncrementQ6(uint16_t first_angle_q6, uint16_t last_angle_q6, std::size_t sample_count) const
{
	if (sample_count <= 1U)
	{
		return 0.0;
	}

	double delta_q6 = 0.0;
	if (last_angle_q6 < first_angle_q6)
	{
		if (first_angle_q6 > WRAP_THRESHOLD_HIGH_Q6 && last_angle_q6 < WRAP_THRESHOLD_LOW_Q6)
		{
			delta_q6 = static_cast<double>(FULL_ROTATION_Q6 + last_angle_q6 - first_angle_q6);
		}
		else
		{
			delta_q6 = static_cast<double>(FULL_ROTATION_Q6 + last_angle_q6 - first_angle_q6);
		}
	}
	else
	{
		delta_q6 = static_cast<double>(last_angle_q6 - first_angle_q6);
	}

	return delta_q6 / static_cast<double>(sample_count - 1U);
}

double Lds03Parser::computeCorrectedAngleQ6(uint16_t first_angle_q6, double interval_q6, std::size_t sample_index, uint16_t distance_q2) const
{
	double angle_correction_q6 = 0.0;
	if (distance_q2 != 0U)
	{
		angle_correction_q6 = std::atan(
			ANGLE_CORRECTION_NUMERATOR *
			(static_cast<double>(distance_q2) - ANGLE_CORRECTION_DENOMINATOR) /
			(ANGLE_CORRECTION_DENOMINATOR * static_cast<double>(distance_q2))) * 64.0;
	}

	double corrected_angle_q6 = static_cast<double>(first_angle_q6) + (interval_q6 * static_cast<double>(sample_index)) + angle_correction_q6;
	while (corrected_angle_q6 < 0.0)
	{
		corrected_angle_q6 += static_cast<double>(FULL_ROTATION_Q6);
	}
	while (corrected_angle_q6 > static_cast<double>(FULL_ROTATION_Q6))
	{
		corrected_angle_q6 -= static_cast<double>(FULL_ROTATION_Q6);
	}

	return corrected_angle_q6;
}

void Lds03Parser::logRawPacket(const std::vector<uint8_t> &packet)
{
	RCLCPP_DEBUG_THROTTLE(
		m_logger,
		m_throttle_clock,
		2000,
		"LDS-03 raw packet: %s",
		packetToHexString(packet).c_str());
}

void Lds03Parser::logPacketWarning(const char *message)
{
	if (!m_is_packet_error_logging_enabled)
	{
		return;
	}

	RCLCPP_WARN_THROTTLE(m_logger, m_throttle_clock, 2000, "%s", message);
}
