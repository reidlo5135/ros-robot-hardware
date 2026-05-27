#include "robot_lidar_driver/lidar_driver_node.hpp"

using namespace robot::hw::lidar;

namespace
{

const char *boolToString(bool value)
{
	if (value)
	{
		return "true";
	}

	return "false";
}

}  // namespace

LidarDriverNode::LidarDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_lidar_driver", options),
	lidar_model_("coin_d4_tof"),
	port_("/dev/tb3_lidar"),
	baudrate_(230400),
	frame_id_("base_scan"),
	topic_name_("/scan"),
	range_min_(0.12),
	range_max_(12.0),
	angle_min_(-PI),
	angle_max_(PI),
	is_scan_direction_reversed_(false),
	publish_rate_hint_hz_(10.0),
	read_buffer_size_(4096),
	ring_buffer_size_(65536),
	use_epoll_(true),
	is_reconnect_on_error_(true),
	reconnect_interval_ms_(1000),
	serial_read_timeout_ms_(1000),
	startup_delay_ms_(1000),
	is_set_dtr_(false),
	is_set_rts_(false),
	is_dtr_active_(true),
	is_rts_active_(true),
	is_mock_mode_(false),
	is_read_rate_logging_enabled_(true),
	is_raw_packet_logging_enabled_(false),
	is_packet_error_logging_enabled_(true),
	scan_publisher_(nullptr),
	serial_port_(nullptr),
	reader_(nullptr),
	parser_(nullptr),
	scan_builder_(nullptr),
	mock_timer_(nullptr),
	reconnect_timer_(nullptr),
	ring_buffer_(0U),
	data_mutex_(),
	is_shutdown_requested_(false),
	is_reconnecting_(false),
	throttle_clock_(RCL_STEADY_TIME),
	read_bytes_accumulator_(0U),
	last_read_rate_log_time_(std::chrono::steady_clock::now()),
	has_logged_serial_read_success_(false),
	has_logged_publish_success_(false)
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
	is_shutdown_requested_.store(true);
	cancelReconnect();
	stopMockMode();
	stopRealMode();
}

void LidarDriverNode::declareParameters()
{
	declare_parameter("lidar_model", lidar_model_);
	declare_parameter("port", port_);
	declare_parameter("baudrate", baudrate_);
	declare_parameter("frame_id", frame_id_);
	declare_parameter("topic_name", topic_name_);
	declare_parameter("range_min", range_min_);
	declare_parameter("range_max", range_max_);
	declare_parameter("angle_min", angle_min_);
	declare_parameter("angle_max", angle_max_);
	declare_parameter("scan_direction_reversed", is_scan_direction_reversed_);
	declare_parameter("publish_rate_hint_hz", publish_rate_hint_hz_);
	declare_parameter("read_buffer_size", read_buffer_size_);
	declare_parameter("ring_buffer_size", ring_buffer_size_);
	declare_parameter("use_epoll", use_epoll_);
	declare_parameter("reconnect_on_error", is_reconnect_on_error_);
	declare_parameter("reconnect_interval_ms", reconnect_interval_ms_);
	declare_parameter("serial_read_timeout_ms", serial_read_timeout_ms_);
	declare_parameter("startup_delay_ms", startup_delay_ms_);
	declare_parameter("set_dtr", is_set_dtr_);
	declare_parameter("set_rts", is_set_rts_);
	declare_parameter("dtr_active", is_dtr_active_);
	declare_parameter("rts_active", is_rts_active_);
	declare_parameter("mock_mode", is_mock_mode_);
	declare_parameter("log_read_rate", is_read_rate_logging_enabled_);
	declare_parameter("log_raw_packet", is_raw_packet_logging_enabled_);
	declare_parameter("log_packet_error", is_packet_error_logging_enabled_);
}

void LidarDriverNode::loadParameters()
{
	get_parameter("lidar_model", lidar_model_);
	get_parameter("port", port_);
	get_parameter("baudrate", baudrate_);
	get_parameter("frame_id", frame_id_);
	get_parameter("topic_name", topic_name_);
	get_parameter("range_min", range_min_);
	get_parameter("range_max", range_max_);
	get_parameter("angle_min", angle_min_);
	get_parameter("angle_max", angle_max_);
	get_parameter("scan_direction_reversed", is_scan_direction_reversed_);
	get_parameter("publish_rate_hint_hz", publish_rate_hint_hz_);
	get_parameter("read_buffer_size", read_buffer_size_);
	get_parameter("ring_buffer_size", ring_buffer_size_);
	get_parameter("use_epoll", use_epoll_);
	get_parameter("reconnect_on_error", is_reconnect_on_error_);
	get_parameter("reconnect_interval_ms", reconnect_interval_ms_);
	get_parameter("serial_read_timeout_ms", serial_read_timeout_ms_);
	get_parameter("startup_delay_ms", startup_delay_ms_);
	get_parameter("set_dtr", is_set_dtr_);
	get_parameter("set_rts", is_set_rts_);
	get_parameter("dtr_active", is_dtr_active_);
	get_parameter("rts_active", is_rts_active_);
	get_parameter("mock_mode", is_mock_mode_);
	get_parameter("log_read_rate", is_read_rate_logging_enabled_);
	get_parameter("log_raw_packet", is_raw_packet_logging_enabled_);
	get_parameter("log_packet_error", is_packet_error_logging_enabled_);
}

void LidarDriverNode::validateParameters()
{
	if (publish_rate_hint_hz_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "publish_rate_hint_hz must be positive. Resetting to 10.0");
		publish_rate_hint_hz_ = 10.0;
	}

	if (read_buffer_size_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "read_buffer_size must be positive. Resetting to 4096");
		read_buffer_size_ = 4096;
	}

	if (ring_buffer_size_ < read_buffer_size_)
	{
		RCLCPP_WARN(
			get_logger(),
			"ring_buffer_size must be at least read_buffer_size. Resetting to %d",
			read_buffer_size_ * 16);
		ring_buffer_size_ = read_buffer_size_ * 16;
	}

	if (reconnect_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "reconnect_interval_ms must be positive. Resetting to 1000");
		reconnect_interval_ms_ = 1000;
	}

	if (serial_read_timeout_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "serial_read_timeout_ms must be positive. Resetting to 1000");
		serial_read_timeout_ms_ = 1000;
	}

	if (startup_delay_ms_ < 0)
	{
		RCLCPP_WARN(get_logger(), "startup_delay_ms cannot be negative. Resetting to 1000");
		startup_delay_ms_ = 1000;
	}

	if (range_min_ < 0.0)
	{
		RCLCPP_WARN(get_logger(), "range_min cannot be negative. Resetting to 0.0");
		range_min_ = 0.0;
	}

	if (range_max_ <= range_min_)
	{
		RCLCPP_WARN(get_logger(), "range_max must be larger than range_min. Resetting to range_min + 10.0");
		range_max_ = range_min_ + 10.0;
	}

	if (angle_max_ <= angle_min_)
	{
		RCLCPP_WARN(get_logger(), "angle_max must be larger than angle_min. Resetting to [-pi, pi]");
		angle_min_ = -PI;
		angle_max_ = PI;
	}

	ring_buffer_.resize(static_cast<std::size_t>(ring_buffer_size_));
}

void LidarDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"LiDAR parameters: model=%s port=%s baudrate=%d frame_id=%s topic_name=%s range=[%.3f, %.3f] angle=[%.3f, %.3f] reversed=%s publish_rate_hint_hz=%.2f read_buffer_size=%d ring_buffer_size=%d use_epoll=%s reconnect_on_error=%s reconnect_interval_ms=%d serial_read_timeout_ms=%d startup_delay_ms=%d set_dtr=%s set_rts=%s dtr_active=%s rts_active=%s mock_mode=%s log_read_rate=%s log_raw_packet=%s log_packet_error=%s",
		lidar_model_.c_str(),
		port_.c_str(),
		baudrate_,
		frame_id_.c_str(),
		topic_name_.c_str(),
		range_min_,
		range_max_,
		angle_min_,
		angle_max_,
		boolToString(is_scan_direction_reversed_),
		publish_rate_hint_hz_,
		read_buffer_size_,
		ring_buffer_size_,
		boolToString(use_epoll_),
		boolToString(is_reconnect_on_error_),
		reconnect_interval_ms_,
		serial_read_timeout_ms_,
		startup_delay_ms_,
		boolToString(is_set_dtr_),
		boolToString(is_set_rts_),
		boolToString(is_dtr_active_),
		boolToString(is_rts_active_),
		boolToString(is_mock_mode_),
		boolToString(is_read_rate_logging_enabled_),
		boolToString(is_raw_packet_logging_enabled_),
		boolToString(is_packet_error_logging_enabled_));
}

void LidarDriverNode::setupPublisher()
{
	scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(resolveTopicName(), rclcpp::SensorDataQoS());
}

bool LidarDriverNode::setupParser()
{
	if (lidar_model_ == "coin_d4_tof" || lidar_model_ == "lds_03_coin_d4" || lidar_model_ == "lds_03")
	{
		if (lidar_model_ == "coin_d4_tof" || lidar_model_ == "lds_03_coin_d4")
		{
			RCLCPP_INFO(get_logger(), "version M1CT_TOF");
		}

		parser_ = std::make_shared<Lds03Parser>(
			get_logger(),
			[this]() -> rclcpp::Time
			{
				return now();
			},
			is_raw_packet_logging_enabled_,
			is_packet_error_logging_enabled_);
		return true;
	}

	RCLCPP_ERROR(get_logger(), "Unsupported lidar_model: %s", lidar_model_.c_str());
	return false;
}

void LidarDriverNode::setupLaserScanBuilder()
{
	scan_builder_ = std::make_shared<LaserScanBuilder>(
		frame_id_,
		angle_min_,
		angle_max_,
		range_min_,
		range_max_,
		is_scan_direction_reversed_,
		publish_rate_hint_hz_);
}

void LidarDriverNode::startDriver()
{
	if (is_mock_mode_)
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
		std::lock_guard<std::mutex> lock(data_mutex_);
		ring_buffer_.clear();
		if (parser_)
		{
			parser_->reset();
		}
	}

	has_logged_serial_read_success_ = false;
	has_logged_publish_success_ = false;

	if (!serial_port_)
	{
		serial_port_ = std::make_shared<SerialPort>(get_logger());
	}

	if (!serial_port_->openPort(port_, baudrate_))
	{
		RCLCPP_WARN(get_logger(), "Serial open failed for %s", port_.c_str());
		scheduleReconnect("Initial serial open failed");
		return;
	}

	applySerialControlSignals();
	RCLCPP_INFO(get_logger(), "Activated lidar grab thread for port %s", port_.c_str());
	RCLCPP_INFO(get_logger(), "Lidar status changed for %s : 0 -> 1", port_.c_str());
	RCLCPP_INFO(get_logger(), "Activated lidar publish thread for port %s", port_.c_str());
	waitForStartupDelayAndLog();
	(void)sendCoinD4StartCommand();

	cancelReconnect();

	reader_ = std::make_shared<EpollSerialReader>(
		get_logger(),
		use_epoll_,
		static_cast<std::size_t>(read_buffer_size_),
		serial_read_timeout_ms_);

	const bool reader_started = reader_->start(
		serial_port_.get(),
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
		serial_port_->closePort();
		scheduleReconnect("Failed to start reader thread");
		return;
	}

	RCLCPP_INFO(get_logger(), "Real LiDAR mode started on %s", port_.c_str());
}

void LidarDriverNode::startMockMode()
{
	stopRealMode();
	cancelReconnect();

	const double period_seconds = 1.0 / publish_rate_hint_hz_;
	const std::chrono::duration<double> period_duration(period_seconds);
	mock_timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(period_duration),
		[this]()
		{
			publishMockScan();
		});

	RCLCPP_INFO(get_logger(), "Mock LiDAR mode started");
}

bool LidarDriverNode::sendCoinD4StartCommand()
{
	if (!serial_port_ || !serial_port_->isOpen())
	{
		return false;
	}

	if (!(lidar_model_ == "coin_d4_tof" || lidar_model_ == "lds_03_coin_d4"))
	{
		return false;
	}

	if (!serial_port_->writeAll(COIN_D4_START_COMMAND.data(), COIN_D4_START_COMMAND.size()))
	{
		RCLCPP_ERROR(get_logger(), "Failed to send COIN-D4 TOF start command on %s", port_.c_str());
		return false;
	}

	RCLCPP_INFO(get_logger(), "Sent COIN-D4 TOF start command on %s: aa 55 f0 0f", port_.c_str());
	return true;
}

void LidarDriverNode::sendCoinD4StopCommand()
{
	if (!serial_port_ || !serial_port_->isOpen())
	{
		return;
	}

	if (!(lidar_model_ == "coin_d4_tof" || lidar_model_ == "lds_03_coin_d4"))
	{
		return;
	}

	if (serial_port_->writeAll(COIN_D4_STOP_COMMAND.data(), COIN_D4_STOP_COMMAND.size()))
	{
		RCLCPP_INFO(get_logger(), "Sent COIN-D4 TOF stop command on %s: aa 55 f5 0a", port_.c_str());
	}
}

void LidarDriverNode::stopRealMode()
{
	sendCoinD4StopCommand();

	if (reader_)
	{
		reader_->stop();
	}

	if (serial_port_)
	{
		serial_port_->closePort();
	}
}

void LidarDriverNode::stopMockMode()
{
	if (mock_timer_)
	{
		mock_timer_->cancel();
		mock_timer_.reset();
	}
}

void LidarDriverNode::scheduleReconnect(const std::string &reason)
{
	if (is_shutdown_requested_.load() || is_mock_mode_ || !is_reconnect_on_error_)
	{
		return;
	}

	if (is_reconnecting_.exchange(true))
	{
		return;
	}

	RCLCPP_WARN(get_logger(), "Scheduling serial reconnect: %s", reason.c_str());

	if (serial_port_)
	{
		serial_port_->closePort();
	}

	if (reconnect_timer_)
	{
		reconnect_timer_->cancel();
	}

	is_reconnecting_.store(true);
	reconnect_timer_ = create_wall_timer(
		std::chrono::milliseconds(reconnect_interval_ms_),
		[this]()
		{
			attemptReconnect();
		});
}

void LidarDriverNode::cancelReconnect()
{
	is_reconnecting_.store(false);
	if (reconnect_timer_)
	{
		reconnect_timer_->cancel();
		reconnect_timer_.reset();
	}
}

void LidarDriverNode::attemptReconnect()
{
	if (is_shutdown_requested_.load() || is_mock_mode_)
	{
		cancelReconnect();
		return;
	}

	RCLCPP_INFO(get_logger(), "Attempting to reconnect LiDAR on %s", port_.c_str());

	if (!parser_ && !setupParser())
	{
		cancelReconnect();
		return;
	}

	if (reader_)
	{
		reader_->stop();
	}

	if (!serial_port_)
	{
		serial_port_ = std::make_shared<SerialPort>(get_logger());
	}

	if (!serial_port_->openPort(port_, baudrate_))
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"Reconnect open failed for %s",
			port_.c_str());
		return;
	}

	applySerialControlSignals();
	RCLCPP_INFO(get_logger(), "Activated lidar grab thread for port %s", port_.c_str());
	RCLCPP_INFO(get_logger(), "Lidar status changed for %s : 0 -> 1", port_.c_str());
	RCLCPP_INFO(get_logger(), "Activated lidar publish thread for port %s", port_.c_str());
	waitForStartupDelayAndLog();
	(void)sendCoinD4StartCommand();

	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		ring_buffer_.clear();
		if (parser_)
		{
			parser_->reset();
		}
	}

	has_logged_serial_read_success_ = false;
	has_logged_publish_success_ = false;

	reader_ = std::make_shared<EpollSerialReader>(
		get_logger(),
		use_epoll_,
		static_cast<std::size_t>(read_buffer_size_),
		serial_read_timeout_ms_);

	const bool reader_started = reader_->start(
		serial_port_.get(),
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
		serial_port_->closePort();
		RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000, "Reconnect reader start failed");
		return;
	}

	cancelReconnect();
	RCLCPP_INFO(get_logger(), "LiDAR reconnect succeeded on %s", port_.c_str());
}

void LidarDriverNode::handleSerialBytes(const uint8_t *data, std::size_t size)
{
	if (is_shutdown_requested_.load() || data == nullptr || size == 0U || !parser_)
	{
		return;
	}

	logRawReadChunk(data, size);
	logReadRate(size);

	if (!has_logged_serial_read_success_)
	{
		RCLCPP_INFO(get_logger(), "Serial read stream is active on %s", port_.c_str());
		has_logged_serial_read_success_ = true;
	}

	std::vector<LidarScan> completed_scans;
	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		ring_buffer_.push(data, size);
		if (ring_buffer_.overflowed())
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				throttle_clock_,
				2000,
				"Ring buffer overflow detected. Oldest bytes were discarded.");
			ring_buffer_.resetOverflowFlag();
		}

		bool made_progress = false;
		do
		{
			made_progress = parser_->consume(ring_buffer_, completed_scans);
		} while (made_progress && ring_buffer_.available() > 0U);
	}

	publishCompletedScans(completed_scans);
}

void LidarDriverNode::applySerialControlSignals()
{
	if (!serial_port_ || !serial_port_->isOpen())
	{
		return;
	}

	if (is_set_dtr_)
	{
		(void)serial_port_->setDtr(is_dtr_active_);
	}

	if (is_set_rts_)
	{
		(void)serial_port_->setRts(is_rts_active_);
	}
}

void LidarDriverNode::waitForStartupDelayAndLog()
{
	if (startup_delay_ms_ > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));
	}

	if (lidar_model_ == "coin_d4_tof" || lidar_model_ == "lds_03_coin_d4")
	{
		RCLCPP_INFO(get_logger(), "TOF version lidar start for %s", port_.c_str());
	}
}

void LidarDriverNode::handleReaderError(const std::string &message)
{
	if (is_shutdown_requested_.load())
	{
		return;
	}

	if (serial_port_)
	{
		serial_port_->closePort();
	}

	RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000, "Serial reader error: %s", message.c_str());
	scheduleReconnect(message);
}

void LidarDriverNode::logRawReadChunk(const uint8_t *data, std::size_t size)
{
	if (!is_raw_packet_logging_enabled_ || data == nullptr || size == 0U)
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
		throttle_clock_,
		1000,
		"Raw serial read chunk: bytes=%zu dump[%zu]=%s",
		size,
		dump_size,
		stream.str().c_str());
}

void LidarDriverNode::logReadRate(std::size_t size)
{
	if (!is_read_rate_logging_enabled_)
	{
		return;
	}

	read_bytes_accumulator_ += static_cast<std::uint64_t>(size);
	const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
	const std::chrono::milliseconds elapsed =
		std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_read_rate_log_time_);
	if (elapsed.count() < 1000)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"Serial read throughput: %llu bytes in %lld ms",
		static_cast<unsigned long long>(read_bytes_accumulator_),
		static_cast<long long>(elapsed.count()));

	read_bytes_accumulator_ = 0U;
	last_read_rate_log_time_ = current_time;
}

void LidarDriverNode::publishCompletedScans(const std::vector<LidarScan> &completed_scans)
{
	if (!scan_publisher_ || !scan_builder_)
	{
		return;
	}

	for (const LidarScan &completed_scan : completed_scans)
	{
		sensor_msgs::msg::LaserScan scan_message = scan_builder_->buildScan(
			completed_scan,
			completed_scan.stamp);
		scan_publisher_->publish(scan_message);
		if (!has_logged_publish_success_)
		{
			RCLCPP_INFO(get_logger(), "LaserScan publish path is active on topic %s", resolveTopicName().c_str());
			has_logged_publish_success_ = true;
		}
	}
}

void LidarDriverNode::publishMockScan()
{
	if (!scan_builder_ || !scan_publisher_)
	{
		return;
	}

	LidarScan mock_scan;
	mock_scan.stamp = now();
	mock_scan.scan_frequency_hz = publish_rate_hint_hz_;
	mock_scan.points.reserve(360U);

	const double background_range = std::max(range_min_, range_max_ * 0.8);
	for (std::size_t index = 0U; index < 360U; ++index)
	{
		const double angle_rad = (2.0 * PI * static_cast<double>(index)) / 360.0;
		double range_m = background_range;
		double intensity = 40.0;

		if (index >= 20U && index <= 40U)
		{
			range_m = std::max(range_min_, 0.5);
			intensity = 180.0;
		}
		else if (index >= 120U && index <= 150U)
		{
			range_m = std::max(range_min_, 1.2);
			intensity = 160.0;
		}
		else if (index >= 250U && index <= 290U)
		{
			range_m = std::max(range_min_, 2.0);
			intensity = 140.0;
		}

		LidarPoint point;
		point.angle_rad = angle_rad;
		point.range_m = std::min(range_m, range_max_);
		point.intensity = intensity;
		mock_scan.points.push_back(point);
	}

	sensor_msgs::msg::LaserScan scan_message = scan_builder_->buildScan(mock_scan, mock_scan.stamp);
	scan_publisher_->publish(scan_message);
}

std::string LidarDriverNode::resolveTopicName() const
{
	const std::string namespace_value = get_namespace();
	if (namespace_value == "/" || namespace_value.empty())
	{
		return topic_name_;
	}

	if (topic_name_ == "/scan")
	{
		return "scan";
	}

	return topic_name_;
}
