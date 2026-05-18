#pragma once

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_lidar_driver/serial_port.hpp"

namespace robot::hw::lidar
{

class EpollSerialReader
{
private:
	rclcpp::Logger m_logger;
	bool m_use_epoll;
	std::size_t m_read_buffer_size;
	int m_timeout_ms;
	std::atomic_bool m_is_stop_requested;
	std::atomic_bool m_is_running;
	std::thread m_thread;
	SerialPort *m_serial_port;
	int m_epoll_fd;
	int m_event_fd;
	std::vector<uint8_t> m_read_buffer;

	void cleanupDescriptors();
	void cleanupThread();
	bool setupDescriptors();
	void wakeStopEvent();
	void threadMain(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb);
	void epollLoop(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb);
	void pollingLoop(const std::function<void(const uint8_t *, std::size_t)> &data_cb, const std::function<void(const std::string &)> &error_cb);
	void reportReadError(const std::function<void(const std::string &)> &error_cb, const std::string &message);

protected:
public:
	explicit EpollSerialReader(const rclcpp::Logger &logger, bool use_epoll, std::size_t read_buffer_size, int timeout_ms);
	virtual ~EpollSerialReader();

	using DataCallback = std::function<void(const uint8_t *, std::size_t)>;
	using ErrorCallback = std::function<void(const std::string &)>;

	bool start(SerialPort *serial, DataCallback data_cb, ErrorCallback error_cb);
	void stop();
	bool isRunning() const;
};

}  // namespace robot::hw::lidar
