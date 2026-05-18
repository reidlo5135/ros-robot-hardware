#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robot::hw::base
{

enum class DxlInstruction : uint8_t
{
	Ping = 0x01,
	Read = 0x02,
	Write = 0x03,
	Status = 0x55
};

struct DxlStatusPacket
{
	uint8_t m_id;
	uint8_t m_error;
	std::vector<uint8_t> m_parameters;
};

class DxlPacketCodec
{
private:
	static constexpr std::array<uint8_t, 4> PACKET_HEADER = {0xFF, 0xFF, 0xFD, 0x00};

protected:
public:
	static uint16_t computeCrc(const uint8_t *data, std::size_t size);
	static std::vector<uint8_t> applyByteStuffing(const std::vector<uint8_t> &payload);
	static std::vector<uint8_t> removeByteStuffing(const std::vector<uint8_t> &payload);
	static bool containsPacketHeader(const std::vector<uint8_t> &buffer);
	static std::vector<uint8_t> encodeInstructionPacket(
		uint8_t id,
		DxlInstruction instruction,
		const std::vector<uint8_t> &parameters);
	static std::optional<DxlStatusPacket> tryDecodeStatusPacket(
		std::vector<uint8_t> &buffer,
		bool *packet_found);
};

}  // namespace robot::hw::base
