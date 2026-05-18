#include "robot_lidar_driver/epoll_serial_reader.hpp"

using namespace robot::hw::lidar;

EpollSerialReader::EpollSerialReader(const rclcpp::Logger &logger, bool use_epoll, std::size_t read_buffer_size, int timeout_ms)
: logger_(logger),
	use_epoll_(use_epoll),
	read_buffer_size_(read_buffer_size == 0U ? 4096U : read_buffer_size),
	timeout_ms_(timeout_ms <= 0 ? 1000 : timeout_ms),
	is_stop_requested_(false),
	is_running_(false),
	thread_(),
	serial_port_(nullptr),
	epoll_fd_(-1),
	event_fd_(-1),
	read_buffer_(read_buffer_size_, 0U)
{
}

EpollSerialReader::~EpollSerialReader()
{
	stop();
}

bool EpollSerialReader::start(SerialPort *serial, DataCallback data_cb, ErrorCallback error_cb)
{
	if (serial == nullptr || !serial->isOpen())
	{
		RCLCPP_ERROR(logger_, "Cannot start serial reader without an open serial port");
		return false;
	}

	stop();
	cleanupThread();

	serial_port_ = serial;
	is_stop_requested_.store(false);
	read_buffer_.assign(read_buffer_size_, 0U);

	if (!setupDescriptors())
	{
		serial_port_ = nullptr;
		return false;
	}

	thread_ = std::thread(&EpollSerialReader::threadMain, this, data_cb, error_cb);
	return true;
}

void EpollSerialReader::stop()
{
	is_stop_requested_.store(true);
	wakeStopEvent();

	if (thread_.joinable())
	{
		if (std::this_thread::get_id() == thread_.get_id())
		{
			thread_.detach();
		}
		else
		{
			thread_.join();
		}
	}

	is_running_.store(false);
	cleanupDescriptors();
}

bool EpollSerialReader::isRunning() const
{
	return is_running_.load();
}

void EpollSerialReader::cleanupDescriptors()
{
	if (epoll_fd_ >= 0)
	{
		::close(epoll_fd_);
		epoll_fd_ = -1;
	}

	if (event_fd_ >= 0)
	{
		::close(event_fd_);
		event_fd_ = -1;
	}
}

void EpollSerialReader::cleanupThread()
{
	if (thread_.joinable() && std::this_thread::get_id() != thread_.get_id())
	{
		thread_.join();
	}
}

bool EpollSerialReader::setupDescriptors()
{
	cleanupDescriptors();

	event_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
	if (event_fd_ < 0)
	{
		RCLCPP_ERROR(logger_, "Failed to create eventfd: %s", std::strerror(errno));
		return false;
	}

	if (!use_epoll_)
	{
		return true;
	}

	epoll_fd_ = ::epoll_create1(EPOLL_CLOEXEC);
	if (epoll_fd_ < 0)
	{
		RCLCPP_ERROR(logger_, "Failed to create epoll instance: %s", std::strerror(errno));
		cleanupDescriptors();
		return false;
	}

	struct epoll_event stop_event;
	std::memset(&stop_event, 0, sizeof(stop_event));
	stop_event.events = EPOLLIN;
	stop_event.data.fd = event_fd_;
	if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &stop_event) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to add stop eventfd to epoll: %s", std::strerror(errno));
		cleanupDescriptors();
		return false;
	}

	struct epoll_event serial_event;
	std::memset(&serial_event, 0, sizeof(serial_event));
	serial_event.events = EPOLLIN | EPOLLERR | EPOLLHUP | EPOLLRDHUP;
	serial_event.data.fd = serial_port_->fd();
	if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_port_->fd(), &serial_event) != 0)
	{
		RCLCPP_ERROR(logger_, "Failed to add serial fd to epoll: %s", std::strerror(errno));
		cleanupDescriptors();
		return false;
	}

	return true;
}

void EpollSerialReader::wakeStopEvent()
{
	if (event_fd_ < 0)
	{
		return;
	}

	const uint64_t signal_value = 1U;
	(void)::write(event_fd_, &signal_value, sizeof(signal_value));
}

void EpollSerialReader::threadMain(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb)
{
	is_running_.store(true);

	if (use_epoll_)
	{
		epollLoop(data_cb, error_cb);
	}
	else
	{
		pollingLoop(data_cb, error_cb);
	}

	is_running_.store(false);
}

void EpollSerialReader::epollLoop(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb)
{
	struct epoll_event events[4];

	while (!is_stop_requested_.load())
	{
		const int ready_count = ::epoll_wait(epoll_fd_, events, 4, timeout_ms_);
		if (ready_count < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}

			reportReadError(error_cb, std::string("epoll_wait failed: ") + std::strerror(errno));
			return;
		}

		if (ready_count == 0)
		{
			continue;
		}

		for (int index = 0; index < ready_count; ++index)
		{
			const int fd = events[index].data.fd;
			const uint32_t event_mask = events[index].events;

			if (fd == event_fd_)
			{
				uint64_t event_value = 0U;
				(void)::read(event_fd_, &event_value, sizeof(event_value));
				continue;
			}

			if (event_mask & (EPOLLERR | EPOLLHUP | EPOLLRDHUP))
			{
				reportReadError(error_cb, "Serial device reported EPOLLERR/EPOLLHUP");
				return;
			}

			if ((event_mask & EPOLLIN) == 0U)
			{
				continue;
			}

			const ssize_t read_size = serial_port_->readSome(read_buffer_.data(), read_buffer_.size());
			if (read_size < 0)
			{
				reportReadError(error_cb, "Serial read failed");
				return;
			}

			if (read_size == 0)
			{
				continue;
			}

			if (data_cb)
			{
				data_cb(read_buffer_.data(), static_cast<std::size_t>(read_size));
			}
		}
	}
}

void EpollSerialReader::pollingLoop(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb)
{
	const std::chrono::milliseconds sleep_interval(2);

	while (!is_stop_requested_.load())
	{
		const ssize_t read_size = serial_port_->readSome(read_buffer_.data(), read_buffer_.size());
		if (read_size < 0)
		{
			reportReadError(error_cb, "Serial read failed");
			return;
		}

		if (read_size > 0 && data_cb)
		{
			data_cb(read_buffer_.data(), static_cast<std::size_t>(read_size));
		}

		std::this_thread::sleep_for(sleep_interval);
	}
}

void EpollSerialReader::reportReadError(const std::function<void(const std::string &)> &error_cb, const std::string &message)
{
	if (error_cb)
	{
		error_cb(message);
	}
}
