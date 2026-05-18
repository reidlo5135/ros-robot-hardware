#include "robot_lidar_driver/serial_port.hpp"

using namespace robot::hw::lidar;

SerialPort::SerialPort(const rclcpp::Logger &logger)
: m_logger(logger),
	m_throttle_clock(RCL_STEADY_TIME),
	m_port(""),
	m_baudrate(0),
	m_fd(INVALID_FD)
{
}

SerialPort::~SerialPort()
{
	closePort();
}

bool SerialPort::openPort(const std::string &port, int baudrate)
{
	closePort();

	m_port = port;
	m_baudrate = baudrate;
	m_fd = ::open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (m_fd == INVALID_FD)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to open serial port %s: errno=%d (%s)",
			m_port.c_str(),
			errno,
			std::strerror(errno));

		if (errno == EBUSY)
		{
			RCLCPP_ERROR(m_logger, "Serial port %s is busy. Another process may already be using /dev/tb3_lidar or the backing device.", m_port.c_str());
		}
		else if (errno == EACCES)
		{
			RCLCPP_ERROR(m_logger, "Serial port %s cannot be opened due to a permission problem. Check device access rights for /dev/tb3_lidar or the backing device.", m_port.c_str());
		}

		return false;
	}

	if (!flush())
	{
		closePort();
		return false;
	}

	if (!configurePort(m_baudrate))
	{
		closePort();
		return false;
	}

	RCLCPP_INFO(
		m_logger,
		"Opened serial port %s at %d baud",
		m_port.c_str(),
		m_baudrate);
	return true;
}

void SerialPort::closePort()
{
	if (m_fd != INVALID_FD)
	{
		::close(m_fd);
		m_fd = INVALID_FD;
	}
}

bool SerialPort::isOpen() const
{
	return m_fd != INVALID_FD;
}

int SerialPort::fd() const
{
	return m_fd;
}

ssize_t SerialPort::readSome(uint8_t *buffer, std::size_t max_size)
{
	if (!isOpen() || buffer == nullptr || max_size == 0U)
	{
		return -1;
	}

	const ssize_t read_size = ::read(m_fd, buffer, max_size);
	if (read_size < 0)
	{
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		{
			return 0;
		}

		RCLCPP_ERROR_THROTTLE(
			m_logger,
			m_throttle_clock,
			2000,
			"Serial read failed on %s: %s",
			m_port.c_str(),
			std::strerror(errno));
		return -1;
	}

	return read_size;
}

bool SerialPort::reconnect()
{
	if (m_port.empty() || m_baudrate <= 0)
	{
		RCLCPP_ERROR(m_logger, "Reconnect requested without a valid serial configuration");
		return false;
	}

	closePort();
	return openPort(m_port, m_baudrate);
}

const std::string &SerialPort::port() const
{
	return m_port;
}

int SerialPort::baudrate() const
{
	return m_baudrate;
}

bool SerialPort::configurePort(int baudrate)
{
	if (!isOpen())
	{
		RCLCPP_ERROR(m_logger, "Cannot configure a serial port that is not open");
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

	int current_flags = ::fcntl(m_fd, F_GETFL, 0);
	if (current_flags < 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to read serial port flags for %s: %s",
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::fcntl(m_fd, F_SETFL, current_flags | O_NONBLOCK) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to set non-blocking mode for %s: %s",
			m_port.c_str(),
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
			RCLCPP_ERROR(m_logger, "Baudrate 512000 is not supported on this platform");
			return false;
#endif
		case 921600:
#ifdef B921600
			speed = B921600;
			break;
#else
			RCLCPP_ERROR(m_logger, "Baudrate 921600 is not supported on this platform");
			return false;
#endif
		default:
			RCLCPP_ERROR(m_logger, "Unsupported serial baudrate: %d", baudrate);
			return false;
	}

	struct termios options;
	if (::tcgetattr(m_fd, &options) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to get termios attributes for %s: %s",
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to set baudrate %d for %s: %s",
			baudrate,
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	if (::tcsetattr(m_fd, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to apply baudrate %d for %s: %s",
			baudrate,
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	return true;
}

bool SerialPort::applyRawMode()
{
	struct termios options;
	if (::tcgetattr(m_fd, &options) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to read termios settings for %s: %s",
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	::cfmakeraw(&options);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
#ifdef CRTSCTS
	options.c_cflag &= ~CRTSCTS;
#endif
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 0;

	if (::tcsetattr(m_fd, TCSANOW, &options) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to apply raw serial mode for %s: %s",
			m_port.c_str(),
			std::strerror(errno));
		return false;
	}

	::tcflush(m_fd, TCIOFLUSH);
	return true;
}

bool SerialPort::flush()
{
	if (!isOpen())
	{
		RCLCPP_ERROR(m_logger, "Cannot flush a serial port that is not open");
		return false;
	}

	if (::tcflush(m_fd, TCIOFLUSH) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to flush serial port %s: errno=%d (%s)",
			m_port.c_str(),
			errno,
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
		RCLCPP_ERROR(m_logger, "Cannot set %s because the serial port is not open", line_name);
		return false;
	}

	int modem_bits = 0;
	if (::ioctl(m_fd, TIOCMGET, &modem_bits) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to read modem control lines for %s: errno=%d (%s)",
			m_port.c_str(),
			errno,
			std::strerror(errno));
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

	if (::ioctl(m_fd, TIOCMSET, &modem_bits) != 0)
	{
		RCLCPP_ERROR(
			m_logger,
			"Failed to set %s on %s: errno=%d (%s)",
			line_name,
			m_port.c_str(),
			errno,
			std::strerror(errno));
		return false;
	}

	RCLCPP_INFO(m_logger, "Set %s on %s to %s", line_name, m_port.c_str(), is_active ? "active" : "inactive");
	return true;
}
