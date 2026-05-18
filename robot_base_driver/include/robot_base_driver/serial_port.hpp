#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <rclcpp/rclcpp.hpp>

namespace robot::hw::base
{

class SerialPort
{
private:
	rclcpp::Logger logger_;
	int fd_;
	std::string port_;
	int baudrate_;

	bool configurePort(int baudrate);
	speed_t resolveBaudrate(int baudrate) const;

protected:
public:
	explicit SerialPort(const rclcpp::Logger &logger);
	virtual ~SerialPort();

	bool openPort(const std::string &port, int baudrate);
	void closePort();
	bool isOpen() const;
	int fd() const;
	ssize_t readSome(uint8_t *buffer, std::size_t max_size);
	ssize_t writeSome(const uint8_t *buffer, std::size_t size);
	bool writeAll(const uint8_t *buffer, std::size_t size, int timeout_ms);
	bool waitForReadable(int timeout_ms);
	bool waitForWritable(int timeout_ms);
	void flush();
	bool flushInput();
	bool drainOutput();
	bool reconnect();
	const std::string &getPort() const;
	int getBaudrate() const;
};

}  // namespace robot::hw::base
