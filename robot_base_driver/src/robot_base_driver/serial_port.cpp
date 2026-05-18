#include "robot_base_driver/serial_port.hpp"

using namespace robot::hw::base;

SerialPort::SerialPort(const rclcpp::Logger &logger)
: logger_(logger),
	fd_(-1),
	port_(""),
	baudrate_(0)
{
}

SerialPort::~SerialPort()
{
	closePort();
}

bool SerialPort::openPort(const std::string &port, int baudrate)
{
	closePort();

	fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_ < 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to open serial port %s: errno=%d (%s)",
			port.c_str(),
			errno,
			std::strerror(errno));
		return false;
	}

	port_ = port;
	baudrate_ = baudrate;

	if (!configurePort(baudrate))
	{
		closePort();
		return false;
	}

	flush();
	RCLCPP_INFO(logger_, "Opened serial port: %s @ %d", port_.c_str(), baudrate_);
	return true;
}

void SerialPort::closePort()
{
	if (fd_ >= 0)
	{
		close(fd_);
		fd_ = -1;
	}
}

bool SerialPort::isOpen() const
{
	return fd_ >= 0;
}

int SerialPort::fd() const
{
	return fd_;
}

ssize_t SerialPort::readSome(uint8_t *buffer, std::size_t max_size)
{
	if (!isOpen())
	{
		errno = EBADF;
		return -1;
	}

	return read(fd_, buffer, max_size);
}

ssize_t SerialPort::writeSome(const uint8_t *buffer, std::size_t size)
{
	if (!isOpen())
	{
		errno = EBADF;
		return -1;
	}

	return write(fd_, buffer, size);
}

bool SerialPort::writeAll(const uint8_t *buffer, std::size_t size, int timeout_ms)
{
	std::size_t bytes_written = 0;

	while (bytes_written < size)
	{
		if (!waitForWritable(timeout_ms))
		{
			RCLCPP_WARN(logger_, "Timed out while waiting for serial port to become writable");
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

		RCLCPP_ERROR(logger_, "Failed to write serial data: errno=%d (%s)", errno, std::strerror(errno));
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
	poll_fd.fd = fd_;
	poll_fd.events = POLLIN;
	poll_fd.revents = 0;

	int result = poll(&poll_fd, 1, timeout_ms);
	if (result < 0)
	{
		RCLCPP_ERROR(logger_, "poll(POLLIN) failed: errno=%d (%s)", errno, std::strerror(errno));
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
	poll_fd.fd = fd_;
	poll_fd.events = POLLOUT;
	poll_fd.revents = 0;

	int result = poll(&poll_fd, 1, timeout_ms);
	if (result < 0)
	{
		RCLCPP_ERROR(logger_, "poll(POLLOUT) failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return result > 0;
}

void SerialPort::flush()
{
	if (isOpen())
	{
		tcflush(fd_, TCIOFLUSH);
	}
}

bool SerialPort::flushInput()
{
	if (!isOpen())
	{
		return false;
	}

	if (tcflush(fd_, TCIFLUSH) != 0)
	{
		RCLCPP_ERROR(logger_, "tcflush(TCIFLUSH) failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::drainOutput()
{
	if (!isOpen())
	{
		return false;
	}

	if (tcdrain(fd_) != 0)
	{
		RCLCPP_ERROR(logger_, "tcdrain failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::reconnect()
{
	if (port_.empty() || baudrate_ <= 0)
	{
		RCLCPP_ERROR(logger_, "Cannot reconnect serial port without saved port and baudrate");
		return false;
	}

	return openPort(port_, baudrate_);
}

const std::string &SerialPort::getPort() const
{
	return port_;
}

int SerialPort::getBaudrate() const
{
	return baudrate_;
}

bool SerialPort::configurePort(int baudrate)
{
	struct termios options;
	if (tcgetattr(fd_, &options) != 0)
	{
		RCLCPP_ERROR(logger_, "tcgetattr failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	speed_t speed = resolveBaudrate(baudrate);
	if (speed == static_cast<speed_t>(0))
	{
		RCLCPP_ERROR(logger_, "Unsupported baudrate: %d", baudrate);
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

	if (tcsetattr(fd_, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(logger_, "tcsetattr failed: errno=%d (%s)", errno, std::strerror(errno));
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
#ifdef B921600
			return B921600;
#else
			return static_cast<speed_t>(0);
#endif
		case 1000000:
#ifdef B1000000
			return B1000000;
#else
			return static_cast<speed_t>(0);
#endif
		default:
			return static_cast<speed_t>(0);
	}
}
