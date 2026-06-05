#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::hw::bms
{

struct BatterySample
{
	std::optional<double> voltage;
	std::optional<double> current;
	std::optional<double> charge;
	std::optional<double> capacity;
	std::optional<double> design_capacity;
	std::optional<double> percentage;
	std::optional<double> temperature;
	std::optional<bool> present;
	std::optional<std::uint8_t> power_supply_status;
	std::optional<std::uint8_t> power_supply_health;
	std::optional<std::uint8_t> power_supply_technology;
	std::vector<double> cell_voltage;
	std::vector<double> cell_temperature;
	std::string location;
	std::string serial_number;
};

class BmsParser
{
private:
	std::string protocol_;
	std::vector<std::uint8_t> buffer_;

public:
	explicit BmsParser(std::string protocol);

	void setProtocol(const std::string &protocol);
	const std::string &protocol() const;
	bool isSupportedProtocol() const;
	bool isPlaceholderProtocol() const;
	void reset();

	std::optional<BatterySample> parseBytes(
		const std::uint8_t *data,
		std::size_t size,
		std::string &reason);
};

}  // namespace robot::hw::bms