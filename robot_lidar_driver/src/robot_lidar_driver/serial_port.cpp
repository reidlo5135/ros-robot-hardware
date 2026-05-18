#include "robot_lidar_driver/serial_port.hpp"

using namespace robot::hw::lidar;

SerialPort::SerialPort(const rclcpp::Logger &logger)
: logger_(logger),
	throttle_clock_(RCL_STEADY_TIME),
	port_(""),
	baudrate_(0),
	fd_(INVALID_FD)
{
}

SerialPort::~SerialPort()
{
	closePort();
}

bool SerialPort::openPort(const std::string &port, int baudrate)
{
	closePort();

	port_ = port;
	baudrate_ = baudrate;
	fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_ == INVALID_FD)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to open serial port %s: errno=%d (%s)",
			port_.c_str(),
			errno,
			std::strerror(errno));

		if (errno == EBUSY)
		{
			RCLCPP_ERROR(logger_, "Serial port %s is busy. Another process may already be using /dev/tb3_lidar or the backing device.", port_.c_str());
		}
		else if (errno == EACCES)
		{
			RCLCPP_ERROR(logger_, "Serial port %s cannot be opened due to a permission problem. Check device access rights for /dev/tb3_lidar or the backing device.", port_.c_str());
		}

		return false;
	}

	if (!flush())
	{
		closePort();
		return false;
	}

	if (!configurePort(baudrate_))
	{
		closePort();
		return false;
	}

	RCLCPP_INFO(
		logger_,
		"Opened serial port %s at %d baud",
		port_.c_str(),
		baudrate_);
	return true;
}

void SerialPort::closePort()
{
	if (fd_ != INVALID_FD)
	{
		::close(fd_);
		fd_ = INVALID_FD;
	}
}

bool SerialPort::isOpen() const
{
	return fd_ != INVALID_FD;
}

int SerialPort::fd() const
{
	return fd_;
}

ssize_t SerialPort::readSome(uint8_t *buffer, std::size_t max_size)
{
	if (!isOpen() || buffer == nullptr || max_size == 0U)
	{
		return -1;
	}

	const ssize_t read_size = ::read(fd_, buffer, max_size);
	if (read_size < 0)
	{
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		{
			return 0;
		}

		RCLCPP_ERROR_THROTTLE(
			logger_,
			throttle_clock_,
			2000,
			"Serial read failed on %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return -1;
	}

	return read_size;
}

ssize_t SerialPort::writeSome(const uint8_t *buffer, std::size_t size)
{
	if (!isOpen() || buffer == nullptr || size == 0U)
	{
		return -1;
	}

	const ssize_t written_size = ::write(fd_, buffer, size);
	if (written_size < 0)
	{
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		{
			return 0;
		}

		RCLCPP_ERROR_THROTTLE(
			logger_,
			throttle_clock_,
			2000,
			"Serial write failed on %s: errno=%d (%s)",
			port_.c_str(),
			errno,
			std::strerror(errno));
		return -1;
	}

	return written_size;
}

bool SerialPort::writeAll(const uint8_t *buffer, std::size_t size)
{
	if (buffer == nullptr || size == 0U)
	{
		return false;
	}

	std::size_t total_written = 0U;
	while (total_written < size)
	{
		const ssize_t written_size = writeSome(buffer + total_written, size - total_written);
		if (written_size < 0)
		{
			return false;
		}

		if (written_size == 0)
		{
			::usleep(1000);
			continue;
		}

		total_written += static_cast<std::size_t>(written_size);
	}

	return true;
}

bool SerialPort::reconnect()
{
	if (port_.empty() || baudrate_ <= 0)
	{
		RCLCPP_ERROR(logger_, "Reconnect requested without a valid serial configuration");
		return false;
	}

	closePort();
	return openPort(port_, baudrate_);
}

const std::string &SerialPort::port() const
{
	return port_;
}

int SerialPort::baudrate() const
{
	return baudrate_;
}

bool SerialPort::configurePort(int baudrate)
{
	if (!isOpen())
	{
		RCLCPP_ERROR(logger_, "Cannot configure a serial port that is not open");
		return false;
	}

	if (!applyRawMode())
	{
		return false;
	}

	if (!setBaudrate(baudrate))
	{
		return false;
	}

	int current_flags = ::fcntl(fd_, F_GETFL, 0);
	if (current_flags < 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to read serial port flags for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::fcntl(fd_, F_SETFL, current_flags | O_NONBLOCK) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to set non-blocking mode for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::setBaudrate(int baudrate)
{
	speed_t speed = 0;
	switch (baudrate)
	{
		case 115200:
			speed = B115200;
			break;
		case 230400:
			speed = B230400;
			break;
		case 460800:
			speed = B460800;
			break;
		case 512000:
#ifdef B512000
			speed = B512000;
			break;
#else
			RCLCPP_ERROR(logger_, "Baudrate 512000 is not supported on this platform");
			return false;
#endif
		case 921600:
#ifdef B921600
			speed = B921600;
			break;
#else
			RCLCPP_ERROR(logger_, "Baudrate 921600 is not supported on this platform");
			return false;
#endif
		default:
			RCLCPP_ERROR(logger_, "Unsupported serial baudrate: %d", baudrate);
			return false;
	}

	struct termios options;
	if (::tcgetattr(fd_, &options) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to get serial settings for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to apply baudrate %d on %s: %s",
			baudrate,
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::tcsetattr(fd_, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to update serial baudrate for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::applyRawMode()
{
	struct termios options;
	if (::tcgetattr(fd_, &options) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to get serial settings for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	::cfmakeraw(&options);
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

	if (::tcsetattr(fd_, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to configure raw mode for %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::flush()
{
	if (!isOpen())
	{
		return false;
	}

	if (::tcflush(fd_, TCIOFLUSH) != 0)
	{
		RCLCPP_ERROR(
			logger_,
			"Failed to flush serial port %s: %s",
			port_.c_str(),
			std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::setDtr(bool is_active)
{
	return setModemLine(TIOCM_DTR, is_active, "DTR");
}

bool SerialPort::setRts(bool is_active)
{
	return setModemLine(TIOCM_RTS, is_active, "RTS");
}

bool SerialPort::setModemLine(int line_flag, bool is_active, const char *line_name)
{
	if (!isOpen())
	{
		return false;
	}

	int modem_bits = 0;
	if (::ioctl(fd_, TIOCMGET, &modem_bits) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to read %s line state for %s: %s", line_name, port_.c_str(), std::strerror(errno));
		return false;
	}

	if (is_active)
	{
		modem_bits |= line_flag;
	}
	else
	{
		modem_bits &= ~line_flag;
	}

	if (::ioctl(fd_, TIOCMSET, &modem_bits) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to set %s line state for %s: %s", line_name, port_.c_str(), std::strerror(errno));
		return false;
	}

	RCLCPP_INFO(logger_, "Applied %s=%s on %s", line_name, is_active ? "active" : "inactive", port_.c_str());
	return true;
}
