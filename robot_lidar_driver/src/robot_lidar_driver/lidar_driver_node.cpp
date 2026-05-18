#include "robot_lidar_driver/lidar_driver_node.hpp"

using namespace robot::hw::lidar;

LidarDriverNode::LidarDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_lidar_driver", options),
	m_lidar_model("coin_d4_tof"),
	m_port("/dev/tb3_lidar"),
	m_baudrate(230400),
	m_frame_id("base_scan"),
	m_topic_name("/scan"),
	m_range_min(0.12),
	m_range_max(12.0),
	m_angle_min(-PI),
	m_angle_max(PI),
	m_is_scan_direction_reversed(false),
	m_publish_rate_hint_hz(10.0),
	m_read_buffer_size(4096),
	m_ring_buffer_size(65536),
	m_use_epoll(true),
	m_is_reconnect_on_error(true),
	m_reconnect_interval_ms(1000),
	m_serial_read_timeout_ms(1000),
	m_startup_delay_ms(1000),
	m_is_set_dtr(false),
	m_is_set_rts(false),
	m_is_dtr_active(true),
	m_is_rts_active(true),
	m_is_mock_mode(false),
	m_is_read_rate_logging_enabled(true),
	m_is_raw_packet_logging_enabled(false),
	m_is_packet_error_logging_enabled(true),
	m_scan_publisher(nullptr),
	m_serial_port(nullptr),
	m_reader(nullptr),
	m_parser(nullptr),
	m_scan_builder(nullptr),
	m_mock_timer(nullptr),
	m_reconnect_timer(nullptr),
	m_ring_buffer(0U),
	m_data_mutex(),
	m_is_shutdown_requested(false),
	m_is_reconnecting(false),
	m_throttle_clock(RCL_STEADY_TIME),
	m_read_bytes_accumulator(0U),
	m_last_read_rate_log_time(std::chrono::steady_clock::now()),
	m_has_logged_serial_read_success(false),
	m_has_logged_publish_success(false)
{
	declareParameters();
	loadParameters();
	validateParameters();
	logParameterSummary();
	setupPublisher();
	setupLaserScanBuilder();
	startDriver();
}

LidarDriverNode::~LidarDriverNode()
{
	m_is_shutdown_requested.store(true);
	cancelReconnect();
	stopMockMode();
	stopRealMode();
}

void LidarDriverNode::declareParameters()
{
	declare_parameter("lidar_model", m_lidar_model);
	declare_parameter("port", m_port);
	declare_parameter("baudrate", m_baudrate);
	declare_parameter("frame_id", m_frame_id);
	declare_parameter("topic_name", m_topic_name);
	declare_parameter("range_min", m_range_min);
	declare_parameter("range_max", m_range_max);
	declare_parameter("angle_min", m_angle_min);
	declare_parameter("angle_max", m_angle_max);
	declare_parameter("scan_direction_reversed", m_is_scan_direction_reversed);
	declare_parameter("publish_rate_hint_hz", m_publish_rate_hint_hz);
	declare_parameter("read_buffer_size", m_read_buffer_size);
	declare_parameter("ring_buffer_size", m_ring_buffer_size);
	declare_parameter("use_epoll", m_use_epoll);
	declare_parameter("reconnect_on_error", m_is_reconnect_on_error);
	declare_parameter("reconnect_interval_ms", m_reconnect_interval_ms);
	declare_parameter("serial_read_timeout_ms", m_serial_read_timeout_ms);
	declare_parameter("startup_delay_ms", m_startup_delay_ms);
	declare_parameter("set_dtr", m_is_set_dtr);
	declare_parameter("set_rts", m_is_set_rts);
	declare_parameter("dtr_active", m_is_dtr_active);
	declare_parameter("rts_active", m_is_rts_active);
	declare_parameter("mock_mode", m_is_mock_mode);
	declare_parameter("log_read_rate", m_is_read_rate_logging_enabled);
	declare_parameter("log_raw_packet", m_is_raw_packet_logging_enabled);
	declare_parameter("log_packet_error", m_is_packet_error_logging_enabled);
}

void LidarDriverNode::loadParameters()
{
	get_parameter("lidar_model", m_lidar_model);
	get_parameter("port", m_port);
	get_parameter("baudrate", m_baudrate);
	get_parameter("frame_id", m_frame_id);
	get_parameter("topic_name", m_topic_name);
	get_parameter("range_min", m_range_min);
	get_parameter("range_max", m_range_max);
	get_parameter("angle_min", m_angle_min);
	get_parameter("angle_max", m_angle_max);
	get_parameter("scan_direction_reversed", m_is_scan_direction_reversed);
	get_parameter("publish_rate_hint_hz", m_publish_rate_hint_hz);
	get_parameter("read_buffer_size", m_read_buffer_size);
	get_parameter("ring_buffer_size", m_ring_buffer_size);
	get_parameter("use_epoll", m_use_epoll);
	get_parameter("reconnect_on_error", m_is_reconnect_on_error);
	get_parameter("reconnect_interval_ms", m_reconnect_interval_ms);
	get_parameter("serial_read_timeout_ms", m_serial_read_timeout_ms);
	get_parameter("startup_delay_ms", m_startup_delay_ms);
	get_parameter("set_dtr", m_is_set_dtr);
	get_parameter("set_rts", m_is_set_rts);
	get_parameter("dtr_active", m_is_dtr_active);
	get_parameter("rts_active", m_is_rts_active);
	get_parameter("mock_mode", m_is_mock_mode);
	get_parameter("log_read_rate", m_is_read_rate_logging_enabled);
	get_parameter("log_raw_packet", m_is_raw_packet_logging_enabled);
	get_parameter("log_packet_error", m_is_packet_error_logging_enabled);
}

void LidarDriverNode::validateParameters()
{
	if (m_publish_rate_hint_hz <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "publish_rate_hint_hz must be positive. Resetting to 10.0");
		m_publish_rate_hint_hz = 10.0;
	}

	if (m_read_buffer_size <= 0)
	{
		RCLCPP_WARN(get_logger(), "read_buffer_size must be positive. Resetting to 4096");
		m_read_buffer_size = 4096;
	}

	if (m_ring_buffer_size < m_read_buffer_size)
	{
		RCLCPP_WARN(
			get_logger(),
			"ring_buffer_size must be at least read_buffer_size. Resetting to %d",
			m_read_buffer_size * 16);
		m_ring_buffer_size = m_read_buffer_size * 16;
	}

	if (m_reconnect_interval_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "reconnect_interval_ms must be positive. Resetting to 1000");
		m_reconnect_interval_ms = 1000;
	}

	if (m_serial_read_timeout_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "serial_read_timeout_ms must be positive. Resetting to 1000");
		m_serial_read_timeout_ms = 1000;
	}

	if (m_startup_delay_ms < 0)
	{
		RCLCPP_WARN(get_logger(), "startup_delay_ms cannot be negative. Resetting to 1000");
		m_startup_delay_ms = 1000;
	}

	if (m_range_min < 0.0)
	{
		RCLCPP_WARN(get_logger(), "range_min cannot be negative. Resetting to 0.0");
		m_range_min = 0.0;
	}

	if (m_range_max <= m_range_min)
	{
		RCLCPP_WARN(get_logger(), "range_max must be larger than range_min. Resetting to range_min + 10.0");
		m_range_max = m_range_min + 10.0;
	}

	if (m_angle_max <= m_angle_min)
	{
		RCLCPP_WARN(get_logger(), "angle_max must be larger than angle_min. Resetting to [-pi, pi]");
		m_angle_min = -PI;
		m_angle_max = PI;
	}

	m_ring_buffer.resize(static_cast<std::size_t>(m_ring_buffer_size));
}

void LidarDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"LiDAR parameters: model=%s port=%s baudrate=%d frame_id=%s topic_name=%s range=[%.3f, %.3f] angle=[%.3f, %.3f] reversed=%s publish_rate_hint_hz=%.2f read_buffer_size=%d ring_buffer_size=%d use_epoll=%s reconnect_on_error=%s reconnect_interval_ms=%d serial_read_timeout_ms=%d startup_delay_ms=%d set_dtr=%s set_rts=%s dtr_active=%s rts_active=%s mock_mode=%s log_read_rate=%s log_raw_packet=%s log_packet_error=%s",
		m_lidar_model.c_str(),
		m_port.c_str(),
		m_baudrate,
		m_frame_id.c_str(),
		m_topic_name.c_str(),
		m_range_min,
		m_range_max,
		m_angle_min,
		m_angle_max,
		m_is_scan_direction_reversed ? "true" : "false",
		m_publish_rate_hint_hz,
		m_read_buffer_size,
		m_ring_buffer_size,
		m_use_epoll ? "true" : "false",
		m_is_reconnect_on_error ? "true" : "false",
		m_reconnect_interval_ms,
		m_serial_read_timeout_ms,
		m_startup_delay_ms,
		m_is_set_dtr ? "true" : "false",
		m_is_set_rts ? "true" : "false",
		m_is_dtr_active ? "true" : "false",
		m_is_rts_active ? "true" : "false",
		m_is_mock_mode ? "true" : "false",
		m_is_read_rate_logging_enabled ? "true" : "false",
		m_is_raw_packet_logging_enabled ? "true" : "false",
		m_is_packet_error_logging_enabled ? "true" : "false");
}

void LidarDriverNode::setupPublisher()
{
	m_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>(resolveTopicName(), rclcpp::SensorDataQoS());
}

bool LidarDriverNode::setupParser()
{
	if (m_lidar_model == "coin_d4_tof" || m_lidar_model == "lds_03_coin_d4" || m_lidar_model == "lds_03")
	{
		if (m_lidar_model == "coin_d4_tof" || m_lidar_model == "lds_03_coin_d4")
		{
			RCLCPP_INFO(get_logger(), "version M1CT_TOF");
		}

		m_parser = std::make_shared<Lds03Parser>(
			get_logger(),
			[this]() -> rclcpp::Time
			{
				return now();
			},
			m_is_raw_packet_logging_enabled,
			m_is_packet_error_logging_enabled);
		return true;
	}

	RCLCPP_ERROR(get_logger(), "Unsupported lidar_model: %s", m_lidar_model.c_str());
	return false;
}

void LidarDriverNode::setupLaserScanBuilder()
{
	m_scan_builder = std::make_shared<LaserScanBuilder>(
		m_frame_id,
		m_angle_min,
		m_angle_max,
		m_range_min,
		m_range_max,
		m_is_scan_direction_reversed,
		m_publish_rate_hint_hz);
}

void LidarDriverNode::startDriver()
{
	if (m_is_mock_mode)
	{
		startMockMode();
		return;
	}

	if (!setupParser())
	{
		return;
	}

	startRealMode();
}

void LidarDriverNode::startRealMode()
{
	stopMockMode();

	{
		std::lock_guard<std::mutex> lock(m_data_mutex);
		m_ring_buffer.clear();
		if (m_parser)
		{
			m_parser->reset();
		}
	}

	m_has_logged_serial_read_success = false;
	m_has_logged_publish_success = false;

	if (!m_serial_port)
	{
		m_serial_port = std::make_shared<SerialPort>(get_logger());
	}

	if (!m_serial_port->openPort(m_port, m_baudrate))
	{
		RCLCPP_WARN(get_logger(), "Serial open failed for %s", m_port.c_str());
		scheduleReconnect("Initial serial open failed");
		return;
	}

	applySerialControlSignals();
	RCLCPP_INFO(get_logger(), "Activated lidar grab thread for port %s", m_port.c_str());
	RCLCPP_INFO(get_logger(), "Lidar status changed for %s : 0 -> 1", m_port.c_str());
	RCLCPP_INFO(get_logger(), "Activated lidar publish thread for port %s", m_port.c_str());
	waitForStartupDelayAndLog();
	(void)sendCoinD4StartCommand();

	cancelReconnect();

	m_reader = std::make_shared<EpollSerialReader>(
		get_logger(),
		m_use_epoll,
		static_cast<std::size_t>(m_read_buffer_size),
		m_serial_read_timeout_ms);

	const bool reader_started = m_reader->start(
		m_serial_port.get(),
		[this](const uint8_t *data, std::size_t size)
		{
			handleSerialBytes(data, size);
		},
		[this](const std::string &message)
		{
			handleReaderError(message);
		});

	if (!reader_started)
	{
		RCLCPP_WARN(get_logger(), "Failed to start serial reader thread");
		m_serial_port->closePort();
		scheduleReconnect("Failed to start reader thread");
		return;
	}

	RCLCPP_INFO(get_logger(), "Real LiDAR mode started on %s", m_port.c_str());
}

void LidarDriverNode::startMockMode()
{
	stopRealMode();
	cancelReconnect();

	const double period_seconds = 1.0 / m_publish_rate_hint_hz;
	const std::chrono::duration<double> period_duration(period_seconds);
	m_mock_timer = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(period_duration),
		[this]()
		{
			publishMockScan();
		});

	RCLCPP_INFO(get_logger(), "Mock LiDAR mode started");
}

bool LidarDriverNode::sendCoinD4StartCommand()
{
	if (!m_serial_port || !m_serial_port->isOpen())
	{
		return false;
	}

	if (!(m_lidar_model == "coin_d4_tof" || m_lidar_model == "lds_03_coin_d4"))
	{
		return false;
	}

	if (!m_serial_port->writeAll(COIN_D4_START_COMMAND.data(), COIN_D4_START_COMMAND.size()))
	{
		RCLCPP_ERROR(get_logger(), "Failed to send COIN-D4 TOF start command on %s", m_port.c_str());
		return false;
	}

	RCLCPP_INFO(get_logger(), "Sent COIN-D4 TOF start command on %s: aa 55 f0 0f", m_port.c_str());
	return true;
}

void LidarDriverNode::sendCoinD4StopCommand()
{
	if (!m_serial_port || !m_serial_port->isOpen())
	{
		return;
	}

	if (!(m_lidar_model == "coin_d4_tof" || m_lidar_model == "lds_03_coin_d4"))
	{
		return;
	}

	if (m_serial_port->writeAll(COIN_D4_STOP_COMMAND.data(), COIN_D4_STOP_COMMAND.size()))
	{
		RCLCPP_INFO(get_logger(), "Sent COIN-D4 TOF stop command on %s: aa 55 f5 0a", m_port.c_str());
	}
}

void LidarDriverNode::stopRealMode()
{
	sendCoinD4StopCommand();

	if (m_reader)
	{
		m_reader->stop();
	}

	if (m_serial_port)
	{
		m_serial_port->closePort();
	}
}

void LidarDriverNode::stopMockMode()
{
	if (m_mock_timer)
	{
		m_mock_timer->cancel();
		m_mock_timer.reset();
	}
}

void LidarDriverNode::scheduleReconnect(const std::string &reason)
{
	if (m_is_shutdown_requested.load() || m_is_mock_mode || !m_is_reconnect_on_error)
	{
		return;
	}

	if (m_is_reconnecting.exchange(true))
	{
		return;
	}

	RCLCPP_WARN(get_logger(), "Scheduling serial reconnect: %s", reason.c_str());

	if (m_serial_port)
	{
		m_serial_port->closePort();
	}

	if (m_reconnect_timer)
	{
		m_reconnect_timer->cancel();
	}

	m_is_reconnecting.store(true);
	m_reconnect_timer = create_wall_timer(
		std::chrono::milliseconds(m_reconnect_interval_ms),
		[this]()
		{
			attemptReconnect();
		});
}

void LidarDriverNode::cancelReconnect()
{
	m_is_reconnecting.store(false);
	if (m_reconnect_timer)
	{
		m_reconnect_timer->cancel();
		m_reconnect_timer.reset();
	}
}

void LidarDriverNode::attemptReconnect()
{
	if (m_is_shutdown_requested.load() || m_is_mock_mode)
	{
		cancelReconnect();
		return;
	}

	RCLCPP_INFO(get_logger(), "Attempting to reconnect LiDAR on %s", m_port.c_str());

	if (!m_parser && !setupParser())
	{
		cancelReconnect();
		return;
	}

	if (m_reader)
	{
		m_reader->stop();
	}

	if (!m_serial_port)
	{
		m_serial_port = std::make_shared<SerialPort>(get_logger());
	}

	if (!m_serial_port->openPort(m_port, m_baudrate))
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			m_throttle_clock,
			2000,
			"Reconnect open failed for %s",
			m_port.c_str());
		return;
	}

	applySerialControlSignals();
	RCLCPP_INFO(get_logger(), "Activated lidar grab thread for port %s", m_port.c_str());
	RCLCPP_INFO(get_logger(), "Lidar status changed for %s : 0 -> 1", m_port.c_str());
	RCLCPP_INFO(get_logger(), "Activated lidar publish thread for port %s", m_port.c_str());
	waitForStartupDelayAndLog();
	(void)sendCoinD4StartCommand();

	{
		std::lock_guard<std::mutex> lock(m_data_mutex);
		m_ring_buffer.clear();
		if (m_parser)
		{
			m_parser->reset();
		}
	}

	m_has_logged_serial_read_success = false;
	m_has_logged_publish_success = false;

	m_reader = std::make_shared<EpollSerialReader>(
		get_logger(),
		m_use_epoll,
		static_cast<std::size_t>(m_read_buffer_size),
		m_serial_read_timeout_ms);

	const bool reader_started = m_reader->start(
		m_serial_port.get(),
		[this](const uint8_t *data, std::size_t size)
		{
			handleSerialBytes(data, size);
		},
		[this](const std::string &message)
		{
			handleReaderError(message);
		});

	if (!reader_started)
	{
		m_serial_port->closePort();
		RCLCPP_WARN_THROTTLE(get_logger(), m_throttle_clock, 2000, "Reconnect reader start failed");
		return;
	}

	cancelReconnect();
	RCLCPP_INFO(get_logger(), "LiDAR reconnect succeeded on %s", m_port.c_str());
}

void LidarDriverNode::handleSerialBytes(const uint8_t *data, std::size_t size)
{
	if (m_is_shutdown_requested.load() || data == nullptr || size == 0U || !m_parser)
	{
		return;
	}

	logRawReadChunk(data, size);
	logReadRate(size);

	if (!m_has_logged_serial_read_success)
	{
		RCLCPP_INFO(get_logger(), "Serial read stream is active on %s", m_port.c_str());
		m_has_logged_serial_read_success = true;
	}

	std::vector<LidarScan> completed_scans;
	{
		std::lock_guard<std::mutex> lock(m_data_mutex);
		m_ring_buffer.push(data, size);
		if (m_ring_buffer.overflowed())
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				m_throttle_clock,
				2000,
				"Ring buffer overflow detected. Oldest bytes were discarded.");
			m_ring_buffer.resetOverflowFlag();
		}

		bool made_progress = false;
		do
		{
			made_progress = m_parser->consume(m_ring_buffer, completed_scans);
		} while (made_progress && m_ring_buffer.available() > 0U);
	}

	publishCompletedScans(completed_scans);
}

void LidarDriverNode::applySerialControlSignals()
{
	if (!m_serial_port || !m_serial_port->isOpen())
	{
		return;
	}

	if (m_is_set_dtr)
	{
		(void)m_serial_port->setDtr(m_is_dtr_active);
	}

	if (m_is_set_rts)
	{
		(void)m_serial_port->setRts(m_is_rts_active);
	}
}

void LidarDriverNode::waitForStartupDelayAndLog()
{
	if (m_startup_delay_ms > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(m_startup_delay_ms));
	}

	if (m_lidar_model == "coin_d4_tof" || m_lidar_model == "lds_03_coin_d4")
	{
		RCLCPP_INFO(get_logger(), "TOF version lidar start for %s", m_port.c_str());
	}
}

void LidarDriverNode::handleReaderError(const std::string &message)
{
	if (m_is_shutdown_requested.load())
	{
		return;
	}

	if (m_serial_port)
	{
		m_serial_port->closePort();
	}

	RCLCPP_WARN_THROTTLE(get_logger(), m_throttle_clock, 2000, "Serial reader error: %s", message.c_str());
	scheduleReconnect(message);
}

void LidarDriverNode::logRawReadChunk(const uint8_t *data, std::size_t size)
{
	if (!m_is_raw_packet_logging_enabled || data == nullptr || size == 0U)
	{
		return;
	}

	static constexpr std::size_t MAX_DUMP_BYTES = 32U;
	const std::size_t dump_size = size < MAX_DUMP_BYTES ? size : MAX_DUMP_BYTES;

	std::ostringstream stream;
	stream << std::hex << std::setfill('0');
	for (std::size_t index = 0U; index < dump_size; ++index)
	{
		stream << std::setw(2) << static_cast<int>(data[index]);
		if ((index + 1U) < dump_size)
		{
			stream << ' ';
		}
	}

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		m_throttle_clock,
		1000,
		"Raw serial read chunk: bytes=%zu dump[%zu]=%s",
		size,
		dump_size,
		stream.str().c_str());
}

void LidarDriverNode::logReadRate(std::size_t size)
{
	if (!m_is_read_rate_logging_enabled)
	{
		return;
	}

	m_read_bytes_accumulator += static_cast<std::uint64_t>(size);
	const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
	const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - m_last_read_rate_log_time);
	if (elapsed.count() < 1000)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"Serial read throughput: %llu bytes in %lld ms",
		static_cast<unsigned long long>(m_read_bytes_accumulator),
		static_cast<long long>(elapsed.count()));

	m_read_bytes_accumulator = 0U;
	m_last_read_rate_log_time = current_time;
}

void LidarDriverNode::publishCompletedScans(const std::vector<LidarScan> &completed_scans)
{
	if (!m_scan_publisher || !m_scan_builder)
	{
		return;
	}

	for (const LidarScan &completed_scan : completed_scans)
	{
		sensor_msgs::msg::LaserScan scan_message = m_scan_builder->buildScan(completed_scan, now());
		m_scan_publisher->publish(scan_message);
		if (!m_has_logged_publish_success)
		{
			RCLCPP_INFO(get_logger(), "LaserScan publish path is active on topic %s", resolveTopicName().c_str());
			m_has_logged_publish_success = true;
		}
	}
}

void LidarDriverNode::publishMockScan()
{
	if (!m_scan_builder || !m_scan_publisher)
	{
		return;
	}

	LidarScan mock_scan;
	mock_scan.stamp = now();
	mock_scan.scan_frequency_hz = m_publish_rate_hint_hz;
	mock_scan.points.reserve(360U);

	const double background_range = std::max(m_range_min, m_range_max * 0.8);
	for (std::size_t index = 0U; index < 360U; ++index)
	{
		const double angle_rad = (2.0 * PI * static_cast<double>(index)) / 360.0;
		double range_m = background_range;
		double intensity = 40.0;

		if (index >= 20U && index <= 40U)
		{
			range_m = std::max(m_range_min, 0.5);
			intensity = 180.0;
		}
		else if (index >= 120U && index <= 150U)
		{
			range_m = std::max(m_range_min, 1.2);
			intensity = 160.0;
		}
		else if (index >= 250U && index <= 290U)
		{
			range_m = std::max(m_range_min, 2.0);
			intensity = 140.0;
		}

		LidarPoint point;
		point.angle_rad = angle_rad;
		point.range_m = std::min(range_m, m_range_max);
		point.intensity = intensity;
		mock_scan.points.push_back(point);
	}

	sensor_msgs::msg::LaserScan scan_message = m_scan_builder->buildScan(mock_scan, now());
	m_scan_publisher->publish(scan_message);
}

std::string LidarDriverNode::resolveTopicName() const
{
	const std::string node_namespace = get_namespace();
	if (node_namespace != "/" && m_topic_name == "/scan")
	{
		return "scan";
	}

	return m_topic_name;
}
