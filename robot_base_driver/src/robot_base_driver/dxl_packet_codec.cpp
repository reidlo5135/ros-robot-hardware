#include "robot_base_driver/dxl_packet_codec.hpp"

using namespace robot::hw::base;

uint16_t DxlPacketCodec::computeCrc(const uint8_t *data, std::size_t size)
{
	uint16_t crc = 0;

	for (std::size_t index = 0; index < size; ++index)
	{
		crc ^= static_cast<uint16_t>(data[index]) << 8;

		for (int bit_index = 0; bit_index < 8; ++bit_index)
		{
			if ((crc & 0x8000) != 0U)
			{
				crc = static_cast<uint16_t>((crc << 1) ^ 0x8005);
			}
			else
			{
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}

	return crc;
}

std::vector<uint8_t> DxlPacketCodec::applyByteStuffing(const std::vector<uint8_t> &payload)
{
	std::vector<uint8_t> stuffed_payload;
	stuffed_payload.reserve(payload.size() + 8U);

	for (std::size_t index = 0; index < payload.size(); ++index)
	{
		stuffed_payload.push_back(payload[index]);

		if (stuffed_payload.size() >= 3U)
		{
			std::size_t size = stuffed_payload.size();
			if (stuffed_payload[size - 3U] == 0xFF &&
				stuffed_payload[size - 2U] == 0xFF &&
				stuffed_payload[size - 1U] == 0xFD)
			{
				stuffed_payload.push_back(0xFD);
			}
		}
	}

	return stuffed_payload;
}

std::vector<uint8_t> DxlPacketCodec::removeByteStuffing(const std::vector<uint8_t> &payload)
{
	std::vector<uint8_t> unstuffed_payload;
	unstuffed_payload.reserve(payload.size());

	for (std::size_t index = 0; index < payload.size(); ++index)
	{
		if (index >= 3U &&
			payload[index - 3U] == 0xFF &&
			payload[index - 2U] == 0xFF &&
			payload[index - 1U] == 0xFD &&
			payload[index] == 0xFD)
		{
			continue;
		}

		unstuffed_payload.push_back(payload[index]);
	}

	return unstuffed_payload;
}

bool DxlPacketCodec::containsPacketHeader(const std::vector<uint8_t> &buffer)
{
	if (buffer.size() < PACKET_HEADER.size())
	{
		return false;
	}

	for (std::size_t index = 0; index + PACKET_HEADER.size() <= buffer.size(); ++index)
	{
		if (buffer[index] == PACKET_HEADER[0] &&
			buffer[index + 1U] == PACKET_HEADER[1] &&
			buffer[index + 2U] == PACKET_HEADER[2] &&
			buffer[index + 3U] == PACKET_HEADER[3])
		{
			return true;
		}
	}

	return false;
}

std::vector<uint8_t> DxlPacketCodec::encodeInstructionPacket(uint8_t id, DxlInstruction instruction, const std::vector<uint8_t> &parameters)
{
	std::vector<uint8_t> payload;
	payload.reserve(parameters.size() + 1U);
	payload.push_back(static_cast<uint8_t>(instruction));
	payload.insert(payload.end(), parameters.begin(), parameters.end());

	std::vector<uint8_t> stuffed_payload = applyByteStuffing(payload);
	uint16_t length = static_cast<uint16_t>(stuffed_payload.size() + 2U);

	std::vector<uint8_t> packet;
	packet.reserve(PACKET_HEADER.size() + 3U + stuffed_payload.size() + 2U);
	packet.insert(packet.end(), PACKET_HEADER.begin(), PACKET_HEADER.end());
	packet.push_back(id);
	packet.push_back(static_cast<uint8_t>(length & 0xFF));
	packet.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
	packet.insert(packet.end(), stuffed_payload.begin(), stuffed_payload.end());

	uint16_t crc = computeCrc(packet.data(), packet.size());
	packet.push_back(static_cast<uint8_t>(crc & 0xFF));
	packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
	return packet;
}

std::optional<DxlStatusPacket> DxlPacketCodec::tryDecodeStatusPacket(std::vector<uint8_t> &buffer, bool *packet_found)
{
	if (packet_found != nullptr)
	{
		*packet_found = false;
	}

	while (buffer.size() >= PACKET_HEADER.size())
	{
		if (!(buffer[0] == PACKET_HEADER[0] &&
			buffer[1] == PACKET_HEADER[1] &&
			buffer[2] == PACKET_HEADER[2] &&
			buffer[3] == PACKET_HEADER[3]))
		{
			buffer.erase(buffer.begin());
			continue;
		}

		if (buffer.size() < 7U)
		{
			return std::nullopt;
		}

		uint16_t length = static_cast<uint16_t>(buffer[5]) |
			static_cast<uint16_t>(static_cast<uint16_t>(buffer[6]) << 8);
		std::size_t packet_size = 7U + static_cast<std::size_t>(length);

		if (buffer.size() < packet_size)
		{
			return std::nullopt;
		}

		if (packet_found != nullptr)
		{
			*packet_found = true;
		}

		std::vector<uint8_t> raw_packet_bytes(
			buffer.begin(),
			buffer.begin() + static_cast<std::ptrdiff_t>(packet_size));
		uint16_t received_crc = static_cast<uint16_t>(buffer[packet_size - 2U]) |
			static_cast<uint16_t>(static_cast<uint16_t>(buffer[packet_size - 1U]) << 8);
		uint16_t computed_crc = computeCrc(buffer.data(), packet_size - 2U);
		if (received_crc != computed_crc)
		{
			buffer.erase(buffer.begin());
			continue;
		}

		std::vector<uint8_t> stuffed_payload(
			buffer.begin() + 7,
			buffer.begin() + static_cast<std::ptrdiff_t>(packet_size - 2U));
		std::vector<uint8_t> payload = removeByteStuffing(stuffed_payload);
		uint8_t packet_id = buffer[4];
		buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(packet_size));

		if (payload.size() < 2U)
		{
			return std::nullopt;
		}

		if (payload[0] != static_cast<uint8_t>(DxlInstruction::Status))
		{
			return std::nullopt;
		}

		DxlStatusPacket status_packet;
		status_packet.id = packet_id;
		status_packet.error = payload[1];
		status_packet.parameters.assign(payload.begin() + 2, payload.end());
		status_packet.raw_bytes = raw_packet_bytes;
		return status_packet;
	}

	return std::nullopt;
}
