#include "robot_base_driver/serial_port.hpp"

using namespace robot::hw::base;

SerialPort::SerialPort(const rclcpp::Logger &logger)
: m_logger(logger),
	m_fd(-1),
	m_port(""),
	m_baudrate(0)
{
}

SerialPort::~SerialPort()
{
	closePort();
}

bool SerialPort::openPort(const std::string &port, int baudrate)
{
	closePort();

	m_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (m_fd < 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to open serial port %s: errno=%d (%s)",
			port.c_str(),
			errno,
			std::strerror(errno));
		return false;
	}

	m_port = port;
	m_baudrate = baudrate;

	if (!configurePort(baudrate))
	{
		closePort();
		return false;
	}

	flush();
	RCLCPP_INFO(m_logger, "Opened serial port: %s @ %d", m_port.c_str(), m_baudrate);
	return true;
}

void SerialPort::closePort()
{
	if (m_fd >= 0)
	{
		close(m_fd);
		m_fd = -1;
	}
}

bool SerialPort::isOpen() const
{
	return m_fd >= 0;
}

int SerialPort::fd() const
{
	return m_fd;
}

ssize_t SerialPort::readSome(uint8_t *buffer, std::size_t max_size)
{
	if (!isOpen())
	{
		errno = EBADF;
		return -1;
	}

	return read(m_fd, buffer, max_size);
}

ssize_t SerialPort::writeSome(const uint8_t *buffer, std::size_t size)
{
	if (!isOpen())
	{
		errno = EBADF;
		return -1;
	}

	return write(m_fd, buffer, size);
}

bool SerialPort::writeAll(const uint8_t *buffer, std::size_t size, int timeout_ms)
{
	std::size_t bytes_written = 0;

	while (bytes_written < size)
	{
		if (!waitForWritable(timeout_ms))
		{
			RCLCPP_WARN(m_logger, "Timed out while waiting for serial port to become writable");
			return false;
		}

		ssize_t result = writeSome(buffer + bytes_written, size - bytes_written);
		if (result > 0)
		{
			bytes_written += static_cast<std::size_t>(result);
			continue;
		}

		if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		{
			continue;
		}

		RCLCPP_ERROR(m_logger, "Failed to write serial data: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::waitForReadable(int timeout_ms)
{
	if (!isOpen())
	{
		return false;
	}

	struct pollfd poll_fd;
	poll_fd.fd = m_fd;
	poll_fd.events = POLLIN;
	poll_fd.revents = 0;

	int result = poll(&poll_fd, 1, timeout_ms);
	if (result < 0)
	{
		RCLCPP_ERROR(m_logger, "poll(POLLIN) failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return result > 0;
}

bool SerialPort::waitForWritable(int timeout_ms)
{
	if (!isOpen())
	{
		return false;
	}

	struct pollfd poll_fd;
	poll_fd.fd = m_fd;
	poll_fd.events = POLLOUT;
	poll_fd.revents = 0;

	int result = poll(&poll_fd, 1, timeout_ms);
	if (result < 0)
	{
		RCLCPP_ERROR(m_logger, "poll(POLLOUT) failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return result > 0;
}

void SerialPort::flush()
{
	if (isOpen())
	{
		tcflush(m_fd, TCIOFLUSH);
	}
}

bool SerialPort::reconnect()
{
	if (m_port.empty() || m_baudrate <= 0)
	{
		RCLCPP_ERROR(m_logger, "Cannot reconnect serial port without saved port and baudrate");
		return false;
	}

	return openPort(m_port, m_baudrate);
}

const std::string &SerialPort::getPort() const
{
	return m_port;
}

int SerialPort::getBaudrate() const
{
	return m_baudrate;
}

bool SerialPort::configurePort(int baudrate)
{
	struct termios options;
	if (tcgetattr(m_fd, &options) != 0)
	{
		RCLCPP_ERROR(m_logger, "tcgetattr failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	speed_t speed = resolveBaudrate(baudrate);
	if (speed == static_cast<speed_t>(0))
	{
		RCLCPP_ERROR(m_logger, "Unsupported baudrate: %d", baudrate);
		return false;
	}

	cfmakeraw(&options);
	cfsetispeed(&options, speed);
	cfsetospeed(&options, speed);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;

	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 0;

	if (tcsetattr(m_fd, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(m_logger, "tcsetattr failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return true;
}

speed_t SerialPort::resolveBaudrate(int baudrate) const
{
	switch (baudrate)
	{
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 921600:
			return B921600;
#ifdef B1000000
		case 1000000:
			return B1000000;
#endif
		default:
			return static_cast<speed_t>(0);
	}
}
