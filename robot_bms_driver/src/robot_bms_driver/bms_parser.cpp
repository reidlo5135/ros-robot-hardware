#include "robot_bms_driver/bms_parser.hpp"

#include <utility>

using namespace robot::hw::bms;

BmsParser::BmsParser(std::string protocol)
: protocol_(std::move(protocol)),
	buffer_()
{
}

void BmsParser::setProtocol(const std::string &protocol)
{
	protocol_ = protocol;
	reset();
}

const std::string &BmsParser::protocol() const
{
	return protocol_;
}

bool BmsParser::isSupportedProtocol() const
{
	return isPlaceholderProtocol();
}

bool BmsParser::isPlaceholderProtocol() const
{
	return protocol_ == "placeholder" || protocol_ == "none" || protocol_.empty();
}

void BmsParser::reset()
{
	buffer_.clear();
}

std::optional<BatterySample> BmsParser::parseBytes(
	const std::uint8_t *data,
	std::size_t size,
	std::string &reason)
{
	if (data != nullptr && size > 0U)
	{
		buffer_.insert(buffer_.end(), data, data + size);
	}

	if (isPlaceholderProtocol())
	{
		reason = "protocol_not_configured";
		return std::nullopt;
	}

	// Extension point: add vendor-specific frame boundary, checksum, and field parsing here
	// once the BMS protocol is documented for the target hardware.
	reason = "unsupported_protocol";
	return std::nullopt;
}