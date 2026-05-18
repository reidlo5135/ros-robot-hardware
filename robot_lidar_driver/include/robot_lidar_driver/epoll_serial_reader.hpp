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
	rclcpp::Logger logger_;
	bool use_epoll_;
	std::size_t read_buffer_size_;
	int timeout_ms_;
	std::atomic_bool is_stop_requested_;
	std::atomic_bool is_running_;
	std::thread thread_;
	SerialPort *serial_port_;
	int epoll_fd_;
	int event_fd_;
	std::vector<uint8_t> read_buffer_;

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
