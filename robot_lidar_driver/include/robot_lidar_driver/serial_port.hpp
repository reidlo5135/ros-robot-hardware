#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace robot::hw::lidar
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
	bool setBaudrate(int baudrate);
	bool applyRawMode();
	bool setModemLine(int line_flag, bool is_active, const char *line_name);

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
	bool writeAll(const uint8_t *buffer, std::size_t size);
	bool reconnect();
	bool flush();
	bool setDtr(bool is_active);
	bool setRts(bool is_active);

	const std::string &port() const;
	int baudrate() const;
};

}  // namespace robot::hw::lidar
