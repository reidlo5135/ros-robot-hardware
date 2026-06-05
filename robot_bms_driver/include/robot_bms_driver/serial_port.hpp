#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace robot::hw::bms
{

class SerialPort
{
private:
	static constexpr int INVALID_FD = -1;

	rclcpp::Logger logger_;
	rclcpp::Clock throttle_clock_;
	std::string port_;
	int baudrate_;
	int fd_;

	bool configurePort(int baudrate);
	bool baudrateToSpeed(int baudrate, speed_t &speed) const;

public:
	explicit SerialPort(const rclcpp::Logger &logger);
	~SerialPort();

	bool openPort(const std::string &port, int baudrate);
	void closePort();
	bool isOpen() const;
	ssize_t readSome(std::uint8_t *buffer, std::size_t max_size);
	bool flush();

	const std::string &port() const;
	int baudrate() const;
};

}  // namespace robot::hw::bms