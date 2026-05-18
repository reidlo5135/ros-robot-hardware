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

	rclcpp::Logger m_logger;
	rclcpp::Clock m_throttle_clock;
	std::string m_port;
	int m_baudrate;
	int m_fd;

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
	bool reconnect();
	bool flush();
	bool setDtr(bool is_active);
	bool setRts(bool is_active);

	const std::string &port() const;
	int baudrate() const;
};

}  // namespace robot::hw::lidar
