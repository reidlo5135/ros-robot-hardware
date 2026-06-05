#include "robot_bms_driver/serial_port.hpp"

using namespace robot::hw::bms;

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
			"Failed to open BMS serial port %s: errno=%d (%s)",
			port_.c_str(),
			errno,
			std::strerror(errno));
		return false;
	}

	if (!flush() || !configurePort(baudrate_))
	{
		closePort();
		return false;
	}

	RCLCPP_INFO(logger_, "Opened BMS serial port %s at %d baud", port_.c_str(), baudrate_);
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

ssize_t SerialPort::readSome(std::uint8_t *buffer, std::size_t max_size)
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
			"BMS serial read failed on %s: errno=%d (%s)",
			port_.c_str(),
			errno,
			std::strerror(errno));
		return -1;
	}

	return read_size;
}

bool SerialPort::flush()
{
	if (!isOpen())
	{
		return false;
	}

	return ::tcflush(fd_, TCIOFLUSH) == 0;
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
	speed_t speed = 0;
	if (!baudrateToSpeed(baudrate, speed))
	{
		return false;
	}

	struct termios termios_options;
	if (::tcgetattr(fd_, &termios_options) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to read BMS serial attributes for %s: %s", port_.c_str(), std::strerror(errno));
		return false;
	}

	::cfmakeraw(&termios_options);
	termios_options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
	termios_options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
	termios_options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
	termios_options.c_cflag &= static_cast<tcflag_t>(~PARENB);
	termios_options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
	termios_options.c_cflag |= CS8;
	termios_options.c_cc[VMIN] = 0;
	termios_options.c_cc[VTIME] = 0;

	if (::cfsetispeed(&termios_options, speed) != 0 || ::cfsetospeed(&termios_options, speed) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to set BMS serial baudrate for %s: %s", port_.c_str(), std::strerror(errno));
		return false;
	}

	if (::tcsetattr(fd_, TCSANOW, &termios_options) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to apply BMS serial attributes for %s: %s", port_.c_str(), std::strerror(errno));
		return false;
	}

	const int current_flags = ::fcntl(fd_, F_GETFL, 0);
	if (current_flags < 0)
	{
		RCLCPP_ERROR(logger_, "Failed to read BMS serial flags for %s: %s", port_.c_str(), std::strerror(errno));
		return false;
	}

	if (::fcntl(fd_, F_SETFL, current_flags | O_NONBLOCK) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to set BMS serial non-blocking mode for %s: %s", port_.c_str(), std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::baudrateToSpeed(int baudrate, speed_t &speed) const
{
	switch (baudrate)
	{
		case 9600:
			speed = B9600;
			break;
		case 19200:
			speed = B19200;
			break;
		case 38400:
			speed = B38400;
			break;
		case 57600:
			speed = B57600;
			break;
		case 115200:
			speed = B115200;
			break;
		case 230400:
			speed = B230400;
			break;
		case 460800:
			speed = B460800;
			break;
		case 921600:
#ifdef B921600
			speed = B921600;
			break;
#else
			RCLCPP_ERROR(logger_, "Baudrate 921600 is not supported on this platform");
			return false;
#endif
		default:
			RCLCPP_ERROR(logger_, "Unsupported BMS baudrate: %d", baudrate);
			return false;
	}

	return true;
}