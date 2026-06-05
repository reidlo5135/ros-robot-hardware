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

int secondsToMilliseconds(double seconds, int fallback_ms)
{
	if (!std::isfinite(seconds) || seconds <= 0.0)
	{
		return fallback_ms;
	}

	return static_cast<int>(std::max(1.0, seconds * 1000.0));
}

std::string sanitizeLogValue(const std::string &value)
{
	if (value.empty())
	{
		return "none";
	}

	std::string sanitized = value;
	for (char &character : sanitized)
	{
		if (character == ' ' || character == '\t' || character == '\n' || character == '\r' || character == '=')
		{
			character = '_';
		}
	}

	return sanitized;
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
	scan_angle_offset_(0.0),
	is_scan_direction_reversed_(false),
	reverse_scan_(false),
	debug_scan_geometry_(false),
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
	is_structured_logging_enabled_(true),
	sensor_state_throttle_sec_(1.0),
	scan_geometry_throttle_sec_(1.0),
	serial_state_throttle_sec_(2.0),
	packet_error_throttle_sec_(1.0),
	is_publish_summary_enabled_(true),
	is_frame_diagnostics_enabled_(true),
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
	reconnect_count_(0U),
	serial_error_count_(0U),
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
	declare_parameter("scan_angle_offset", scan_angle_offset_);
	declare_parameter("scan_direction_reversed", is_scan_direction_reversed_);
	declare_parameter("reverse_scan", reverse_scan_);
	declare_parameter("debug_scan_geometry", debug_scan_geometry_);
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
	declare_parameter("logging.structured_enabled", is_structured_logging_enabled_);
	declare_parameter("logging.sensor_state_throttle_sec", sensor_state_throttle_sec_);
	declare_parameter("logging.scan_geometry_throttle_sec", scan_geometry_throttle_sec_);
	declare_parameter("logging.serial_state_throttle_sec", serial_state_throttle_sec_);
	declare_parameter("logging.packet_error_throttle_sec", packet_error_throttle_sec_);
	declare_parameter("logging.publish_summary_enabled", is_publish_summary_enabled_);
	declare_parameter("logging.frame_diagnostics_enabled", is_frame_diagnostics_enabled_);
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
	get_parameter("scan_angle_offset", scan_angle_offset_);
	get_parameter("scan_direction_reversed", is_scan_direction_reversed_);
	get_parameter("reverse_scan", reverse_scan_);
	get_parameter("debug_scan_geometry", debug_scan_geometry_);
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
	get_parameter("logging.structured_enabled", is_structured_logging_enabled_);
	get_parameter("logging.sensor_state_throttle_sec", sensor_state_throttle_sec_);
	get_parameter("logging.scan_geometry_throttle_sec", scan_geometry_throttle_sec_);
	get_parameter("logging.serial_state_throttle_sec", serial_state_throttle_sec_);
	get_parameter("logging.packet_error_throttle_sec", packet_error_throttle_sec_);
	get_parameter("logging.publish_summary_enabled", is_publish_summary_enabled_);
	get_parameter("logging.frame_diagnostics_enabled", is_frame_diagnostics_enabled_);
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

	if (!std::isfinite(scan_angle_offset_))
	{
		RCLCPP_WARN(get_logger(), "scan_angle_offset must be finite. Resetting to 0.0");
		scan_angle_offset_ = 0.0;
	}

	if (!std::isfinite(sensor_state_throttle_sec_) || sensor_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.sensor_state_throttle_sec must be positive. Resetting to 1.0");
		sensor_state_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(scan_geometry_throttle_sec_) || scan_geometry_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.scan_geometry_throttle_sec must be positive. Resetting to 1.0");
		scan_geometry_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(serial_state_throttle_sec_) || serial_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.serial_state_throttle_sec must be positive. Resetting to 2.0");
		serial_state_throttle_sec_ = 2.0;
	}

	if (!std::isfinite(packet_error_throttle_sec_) || packet_error_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.packet_error_throttle_sec must be positive. Resetting to 1.0");
		packet_error_throttle_sec_ = 1.0;
	}

	ring_buffer_.resize(static_cast<std::size_t>(ring_buffer_size_));
}

void LidarDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"LiDAR parameters: model=%s port=%s baudrate=%d frame_id=%s topic_name=%s range=[%.3f, %.3f] angle=[%.3f, %.3f] scan_angle_offset=%.3f reversed=%s reverse_scan=%s debug_scan_geometry=%s publish_rate_hint_hz=%.2f read_buffer_size=%d ring_buffer_size=%d use_epoll=%s reconnect_on_error=%s reconnect_interval_ms=%d serial_read_timeout_ms=%d startup_delay_ms=%d set_dtr=%s set_rts=%s dtr_active=%s rts_active=%s mock_mode=%s log_read_rate=%s log_raw_packet=%s log_packet_error=%s",
		lidar_model_.c_str(),
		port_.c_str(),
		baudrate_,
		resolveFrameId().c_str(),
		topic_name_.c_str(),
		range_min_,
		range_max_,
		angle_min_,
		angle_max_,
		scan_angle_offset_,
		boolToString(is_scan_direction_reversed_),
		boolToString(reverse_scan_),
		boolToString(debug_scan_geometry_),
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

	if (!is_structured_logging_enabled_)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=sensor_config node=%s namespace=%s lidar_model=%s port=%s baudrate=%d topic=%s frame_id=%s range_min_m=%.3f range_max_m=%.3f angle_min_rad=%.6f angle_max_rad=%.6f scan_angle_offset_rad=%.6f scan_direction_reversed=%s reverse_scan=%s mock_mode=%s use_epoll=%s reconnect_on_error=%s read_buffer_size=%d ring_buffer_size=%d structured_enabled=%s publish_summary_enabled=%s frame_diagnostics_enabled=%s result=ok",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		sanitizeLogValue(lidar_model_).c_str(),
		sanitizeLogValue(port_).c_str(),
		baudrate_,
		resolveTopicName().c_str(),
		resolveFrameId().c_str(),
		range_min_,
		range_max_,
		angle_min_,
		angle_max_,
		scan_angle_offset_,
		boolToString(is_scan_direction_reversed_),
		boolToString(reverse_scan_),
		boolToString(is_mock_mode_),
		boolToString(use_epoll_),
		boolToString(is_reconnect_on_error_),
		read_buffer_size_,
		ring_buffer_size_,
		boolToString(is_structured_logging_enabled_),
		boolToString(is_publish_summary_enabled_),
		boolToString(is_frame_diagnostics_enabled_));

	logFrameConfig();
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
			is_packet_error_logging_enabled_,
			secondsToMilliseconds(packet_error_throttle_sec_, 1000));
		return true;
	}

	RCLCPP_ERROR(get_logger(), "Unsupported lidar_model: %s", lidar_model_.c_str());
	return false;
}

void LidarDriverNode::setupLaserScanBuilder()
{
	scan_builder_ = std::make_shared<LaserScanBuilder>(
		resolveFrameId(),
		angle_min_,
		angle_max_,
		range_min_,
		range_max_,
		scan_angle_offset_,
		is_scan_direction_reversed_ != reverse_scan_,
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
		serial_error_count_ += 1U;
		RCLCPP_WARN(get_logger(), "Serial open failed for %s", port_.c_str());
		if (is_structured_logging_enabled_)
		{
			RCLCPP_WARN(
				get_logger(),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_open node=%s namespace=%s port=%s baudrate=%d read_buffer_size=%d ring_buffer_size=%d reconnect_count=%llu error_count=%llu result=failed reason=open_failed",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_,
				read_buffer_size_,
				ring_buffer_size_,
				static_cast<unsigned long long>(reconnect_count_),
				static_cast<unsigned long long>(serial_error_count_));
		}
		scheduleReconnect("Initial serial open failed");
		return;
	}

	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_open node=%s namespace=%s port=%s baudrate=%d read_buffer_size=%d ring_buffer_size=%d reconnect_count=%llu error_count=%llu result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_,
			read_buffer_size_,
			ring_buffer_size_,
			static_cast<unsigned long long>(reconnect_count_),
			static_cast<unsigned long long>(serial_error_count_));
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
		serial_error_count_ += 1U;
		RCLCPP_WARN(get_logger(), "Failed to start serial reader thread");
		if (is_structured_logging_enabled_)
		{
			RCLCPP_WARN(
				get_logger(),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_error node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=failed reason=reader_start_failed",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_,
				static_cast<unsigned long long>(reconnect_count_),
				static_cast<unsigned long long>(serial_error_count_));
		}
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
	if (is_structured_logging_enabled_)
	{
		RCLCPP_WARN(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_reconnect node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=scheduled reason=%s",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_,
			static_cast<unsigned long long>(reconnect_count_),
			static_cast<unsigned long long>(serial_error_count_),
			sanitizeLogValue(reason).c_str());
	}

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
	reconnect_count_ += 1U;

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
		serial_error_count_ += 1U;
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"Reconnect open failed for %s",
			port_.c_str());
		if (is_structured_logging_enabled_)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				throttle_clock_,
				secondsToMilliseconds(serial_state_throttle_sec_, 2000),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_reconnect node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=failed reason=open_failed throttle_sec=%.3f",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_,
				static_cast<unsigned long long>(reconnect_count_),
				static_cast<unsigned long long>(serial_error_count_),
				serial_state_throttle_sec_);
		}
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
		serial_error_count_ += 1U;
		serial_port_->closePort();
		RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000, "Reconnect reader start failed");
		if (is_structured_logging_enabled_)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				throttle_clock_,
				secondsToMilliseconds(serial_state_throttle_sec_, 2000),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_reconnect node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=failed reason=reader_start_failed throttle_sec=%.3f",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_,
				static_cast<unsigned long long>(reconnect_count_),
				static_cast<unsigned long long>(serial_error_count_),
				serial_state_throttle_sec_);
		}
		return;
	}

	cancelReconnect();
	RCLCPP_INFO(get_logger(), "LiDAR reconnect succeeded on %s", port_.c_str());
	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_reconnect node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_,
			static_cast<unsigned long long>(reconnect_count_),
			static_cast<unsigned long long>(serial_error_count_));
	}
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
		if (is_structured_logging_enabled_)
		{
			RCLCPP_INFO(
				get_logger(),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_state node=%s namespace=%s port=%s baudrate=%d read_buffer_size=%d ring_buffer_size=%d reconnect_count=%llu error_count=%llu result=active",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_,
				read_buffer_size_,
				ring_buffer_size_,
				static_cast<unsigned long long>(reconnect_count_),
				static_cast<unsigned long long>(serial_error_count_));
		}
		has_logged_serial_read_success_ = true;
	}

	std::vector<LidarScan> completed_scans;
	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		ring_buffer_.push(data, size);
		if (ring_buffer_.overflowed())
		{
			serial_error_count_ += 1U;
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				throttle_clock_,
				2000,
				"Ring buffer overflow detected. Oldest bytes were discarded.");
			if (is_structured_logging_enabled_)
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					throttle_clock_,
					secondsToMilliseconds(serial_state_throttle_sec_, 2000),
					"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_error node=%s namespace=%s port=%s baudrate=%d read_buffer_size=%d ring_buffer_size=%d reconnect_count=%llu error_count=%llu result=failed reason=ring_buffer_overflow throttle_sec=%.3f",
					get_name(),
					sanitizeLogValue(get_namespace()).c_str(),
					sanitizeLogValue(port_).c_str(),
					baudrate_,
					read_buffer_size_,
					ring_buffer_size_,
					static_cast<unsigned long long>(reconnect_count_),
					static_cast<unsigned long long>(serial_error_count_),
					serial_state_throttle_sec_);
			}
			ring_buffer_.resetOverflowFlag();
		}

		bool made_progress = false;
		do
		{
			made_progress = parser_->consume(ring_buffer_, completed_scans);
		} while (made_progress && ring_buffer_.available() > 0U);
	}

	logPacketParserState();

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

	serial_error_count_ += 1U;
	RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000, "Serial reader error: %s", message.c_str());
	if (is_structured_logging_enabled_)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(serial_state_throttle_sec_, 2000),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_error node=%s namespace=%s port=%s baudrate=%d reconnect_count=%llu error_count=%llu result=failed reason=%s throttle_sec=%.3f",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_,
			static_cast<unsigned long long>(reconnect_count_),
			static_cast<unsigned long long>(serial_error_count_),
			sanitizeLogValue(message).c_str(),
			serial_state_throttle_sec_);
	}
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
	const int throttle_ms = secondsToMilliseconds(serial_state_throttle_sec_, 2000);
	if (elapsed.count() < throttle_ms)
	{
		return;
	}

	const double bytes_per_sec = static_cast<double>(read_bytes_accumulator_) * 1000.0 /
		static_cast<double>(elapsed.count());
	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=lidar event=serial_read_rate node=%s namespace=%s port=%s baudrate=%d bytes_per_sec=%.1f read_buffer_size=%d ring_buffer_size=%d reconnect_count=%llu error_count=%llu throttle_sec=%.3f result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_,
			bytes_per_sec,
			read_buffer_size_,
			ring_buffer_size_,
			static_cast<unsigned long long>(reconnect_count_),
			static_cast<unsigned long long>(serial_error_count_),
			serial_state_throttle_sec_);
	}
	else
	{
		RCLCPP_INFO(
			get_logger(),
			"Serial read throughput: %llu bytes in %lld ms",
			static_cast<unsigned long long>(read_bytes_accumulator_),
			static_cast<long long>(elapsed.count()));
	}

	read_bytes_accumulator_ = 0U;
	last_read_rate_log_time_ = current_time;
}

void LidarDriverNode::logFrameConfig() const
{
	if (!is_structured_logging_enabled_ || !is_frame_diagnostics_enabled_)
	{
		return;
	}

	std::string expected_scan_frame = "base_scan";
	const std::string sanitized_namespace = getSanitizedNamespace();
	if (!sanitized_namespace.empty())
	{
		expected_scan_frame = sanitized_namespace + "/base_scan";
	}

	const std::string resolved_frame_id = resolveFrameId();
	const bool is_frame_match = resolved_frame_id == expected_scan_frame;
	if (is_frame_match)
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=TF component=lidar event=frame_config node=%s namespace=%s scan_frame_id=%s robot_description_expected=%s result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolved_frame_id.c_str(),
			expected_scan_frame.c_str());
		return;
	}

	RCLCPP_WARN(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=TF component=lidar event=frame_config node=%s namespace=%s scan_frame_id=%s robot_description_expected=%s result=warn reason=scan_frame_mismatch",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		resolved_frame_id.c_str(),
		expected_scan_frame.c_str());
}

void LidarDriverNode::logPacketParserState() const
{
	if (!is_structured_logging_enabled_ || !parser_)
	{
		return;
	}

	const LidarParserStats stats = parser_->getStats();
	const char *result = stats.invalid_packet_count == 0U ? "ok" : "warn";
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(packet_error_throttle_sec_, 1000),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=packet_parse node=%s namespace=%s packet_count=%llu valid_packet_count=%llu invalid_packet_count=%llu checksum_error_count=%llu malformed_packet_count=%llu dropped_bytes=%llu throttle_sec=%.3f result=%s",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		static_cast<unsigned long long>(stats.packet_count),
		static_cast<unsigned long long>(stats.valid_packet_count),
		static_cast<unsigned long long>(stats.invalid_packet_count),
		static_cast<unsigned long long>(stats.checksum_error_count),
		static_cast<unsigned long long>(stats.malformed_packet_count),
		static_cast<unsigned long long>(stats.dropped_bytes),
		packet_error_throttle_sec_,
		result);

	if (stats.invalid_packet_count == 0U)
	{
		return;
	}

	RCLCPP_WARN_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(packet_error_throttle_sec_, 1000),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=packet_error node=%s namespace=%s packet_count=%llu valid_packet_count=%llu invalid_packet_count=%llu checksum_error_count=%llu malformed_packet_count=%llu dropped_bytes=%llu throttle_sec=%.3f result=warn reason=%s",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		static_cast<unsigned long long>(stats.packet_count),
		static_cast<unsigned long long>(stats.valid_packet_count),
		static_cast<unsigned long long>(stats.invalid_packet_count),
		static_cast<unsigned long long>(stats.checksum_error_count),
		static_cast<unsigned long long>(stats.malformed_packet_count),
		static_cast<unsigned long long>(stats.dropped_bytes),
		packet_error_throttle_sec_,
		stats.checksum_error_count > 0U ? "checksum_or_malformed" : "malformed_packet");
}

void LidarDriverNode::logScanPublishSummary(
	const LidarScan &completed_scan,
	const sensor_msgs::msg::LaserScan &scan_message) const
{
	if (!is_structured_logging_enabled_ || !is_publish_summary_enabled_)
	{
		return;
	}

	std::size_t valid_ranges = 0U;
	double min_range = std::numeric_limits<double>::infinity();
	double max_range = 0.0;
	for (float range : scan_message.ranges)
	{
		const double range_value = static_cast<double>(range);
		if (!std::isfinite(range_value) || range_value < range_min_ || range_value > range_max_)
		{
			continue;
		}

		valid_ranges += 1U;
		min_range = std::min(min_range, range_value);
		max_range = std::max(max_range, range_value);
	}

	if (valid_ranges == 0U)
	{
		min_range = std::numeric_limits<double>::quiet_NaN();
		max_range = std::numeric_limits<double>::quiet_NaN();
	}

	const std::size_t invalid_ranges = scan_message.ranges.size() - valid_ranges;
	const double stamp_age_sec = (now() - scan_message.header.stamp).seconds();
	const double publish_rate_hz = completed_scan.scan_frequency_hz > 0.0 ? completed_scan.scan_frequency_hz : publish_rate_hint_hz_;
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(sensor_state_throttle_sec_, 1000),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=scan_publish node=%s namespace=%s topic=%s frame_id=%s stamp_age_sec=%.6f ranges=%zu valid_ranges=%zu invalid_ranges=%zu range_min_m=%.3f range_max_m=%.3f min_range_m=%.3f max_range_m=%.3f angle_min_rad=%.6f angle_max_rad=%.6f angle_increment_rad=%.9f scan_time_sec=%.6f publish_rate_hz=%.3f throttle_sec=%.3f result=ok",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		resolveTopicName().c_str(),
		scan_message.header.frame_id.c_str(),
		stamp_age_sec,
		scan_message.ranges.size(),
		valid_ranges,
		invalid_ranges,
		range_min_,
		range_max_,
		min_range,
		max_range,
		static_cast<double>(scan_message.angle_min),
		static_cast<double>(scan_message.angle_max),
		static_cast<double>(scan_message.angle_increment),
		static_cast<double>(scan_message.scan_time),
		publish_rate_hz,
		sensor_state_throttle_sec_);
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
		logScanPublishSummary(completed_scan, scan_message);
		if (debug_scan_geometry_)
		{
			logScanGeometry(completed_scan, scan_message);
		}
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
	logScanPublishSummary(mock_scan, scan_message);
	if (debug_scan_geometry_)
	{
		logScanGeometry(mock_scan, scan_message);
	}
}

void LidarDriverNode::logScanGeometry(
	const LidarScan &completed_scan,
	const sensor_msgs::msg::LaserScan &scan_message) const
{
	struct RawDirectionSample
	{
		double requested_angle_rad;
		double point_angle_rad;
		double angle_error_rad;
		double range_m;
		double intensity;
		bool found;
	};

	const int front_index = computeScanIndexForAngle(scan_message, 0.0);
	const int left_index = computeScanIndexForAngle(scan_message, PI * 0.5);
	const int right_index = computeScanIndexForAngle(scan_message, -PI * 0.5);
	const int rear_index = computeScanIndexForAngle(scan_message, PI);
	const double sector_half_width_rad = 15.0 * PI / 180.0;

	const auto normalize_angle = [](double angle_rad) -> double
	{
		double normalized = std::fmod(angle_rad, 2.0 * PI);
		if (normalized <= -PI)
		{
			normalized += 2.0 * PI;
		}
		if (normalized > PI)
		{
			normalized -= 2.0 * PI;
		}
		return normalized;
	};

	const auto range_for_index = [&scan_message](int index) -> double
	{
		if (index < 0 || static_cast<std::size_t>(index) >= scan_message.ranges.size())
		{
			return std::numeric_limits<double>::quiet_NaN();
		}

		return static_cast<double>(scan_message.ranges[static_cast<std::size_t>(index)]);
	};

	const auto angle_for_index = [&scan_message, &normalize_angle](int index) -> double
	{
		if (index < 0 || static_cast<std::size_t>(index) >= scan_message.ranges.size())
		{
			return std::numeric_limits<double>::quiet_NaN();
		}

		return normalize_angle(
			static_cast<double>(scan_message.angle_min) +
			(static_cast<double>(index) * static_cast<double>(scan_message.angle_increment)));
	};

	const auto min_range_in_sector = [&](double center_angle_rad) -> double
	{
		double best_range = std::numeric_limits<double>::infinity();

		for (std::size_t index = 0U; index < scan_message.ranges.size(); ++index)
		{
			const double range = static_cast<double>(scan_message.ranges[index]);
			if (!std::isfinite(range))
			{
				continue;
			}

			const double sample_angle = static_cast<double>(scan_message.angle_min) +
				(static_cast<double>(index) * static_cast<double>(scan_message.angle_increment));
			const double error = std::abs(normalize_angle(sample_angle - center_angle_rad));
			if (error > sector_half_width_rad)
			{
				continue;
			}

			if (range < best_range)
			{
				best_range = range;
			}
		}

		return std::isfinite(best_range) ? best_range : std::numeric_limits<double>::quiet_NaN();
	};

	const auto find_raw_sample_for_angle = [&](double requested_angle_rad) -> RawDirectionSample
	{
		RawDirectionSample result{};
		result.requested_angle_rad = requested_angle_rad;
		result.point_angle_rad = std::numeric_limits<double>::quiet_NaN();
		result.angle_error_rad = std::numeric_limits<double>::infinity();
		result.range_m = std::numeric_limits<double>::quiet_NaN();
		result.intensity = std::numeric_limits<double>::quiet_NaN();
		result.found = false;

		for (const LidarPoint &point : completed_scan.points)
		{
			const double point_angle = normalize_angle(point.angle_rad);
			const double error = std::abs(normalize_angle(point_angle - requested_angle_rad));
			if (!result.found || error < result.angle_error_rad)
			{
				result.requested_angle_rad = requested_angle_rad;
				result.point_angle_rad = point_angle;
				result.angle_error_rad = error;
				result.range_m = point.range_m;
				result.intensity = point.intensity;
				result.found = true;
			}
		}

		return result;
	};

	const auto find_global_nearest_hit = [&]() -> std::tuple<int, double, double>
	{
		int best_index = -1;
		double best_angle = std::numeric_limits<double>::quiet_NaN();
		double best_range = std::numeric_limits<double>::infinity();

		for (std::size_t index = 0U; index < scan_message.ranges.size(); ++index)
		{
			const double range = static_cast<double>(scan_message.ranges[index]);
			if (!std::isfinite(range) || range >= best_range)
			{
				continue;
			}

			best_index = static_cast<int>(index);
			best_range = range;
			best_angle = static_cast<double>(scan_message.angle_min) +
				(static_cast<double>(index) * static_cast<double>(scan_message.angle_increment));
		}

		if (best_index < 0)
		{
			return std::make_tuple(-1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
		}

		return std::make_tuple(best_index, normalize_angle(best_angle), best_range);
	};

	const RawDirectionSample raw_front = find_raw_sample_for_angle(0.0);
	const RawDirectionSample raw_left = find_raw_sample_for_angle(PI * 0.5);
	const RawDirectionSample raw_right = find_raw_sample_for_angle(-PI * 0.5);
	const RawDirectionSample raw_rear = find_raw_sample_for_angle(PI);
	const auto [nearest_index, nearest_angle, nearest_range] = find_global_nearest_hit();
	const int first_index = scan_message.ranges.empty() ? -1 : 0;
	const int center_index = scan_message.ranges.empty() ? -1 : static_cast<int>(scan_message.ranges.size() / 2U);
	const int last_index = scan_message.ranges.empty() ? -1 : static_cast<int>(scan_message.ranges.size() - 1U);
	const double first_angle = static_cast<double>(scan_message.angle_min);
	const double last_angle = last_index < 0 ? std::numeric_limits<double>::quiet_NaN() :
		static_cast<double>(scan_message.angle_min) + (static_cast<double>(last_index) * static_cast<double>(scan_message.angle_increment));

	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(scan_geometry_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=scan_geometry node=%s namespace=%s frame_id=%s scan_angle_offset_rad=%.6f scan_direction_reversed=%s reverse_scan=%s first_angle_rad=%.6f last_angle_rad=%.6f first_range_m=%.3f center_range_m=%.3f last_range_m=%.3f front_angle_rad=%.3f left_angle_rad=%.3f right_angle_rad=%.3f rear_angle_rad=%.3f front_range_m=%.3f left_range_m=%.3f right_range_m=%.3f rear_range_m=%.3f expected_forward_index=%d front_index=%d left_index=%d right_index=%d rear_index=%d nearest_index=%d nearest_angle_rad=%.3f nearest_range_m=%.3f raw_front_angle_rad=%.3f raw_front_range_m=%.3f raw_left_angle_rad=%.3f raw_right_angle_rad=%.3f raw_rear_angle_rad=%.3f throttle_sec=%.3f result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			scan_message.header.frame_id.c_str(),
			scan_angle_offset_,
			boolToString(is_scan_direction_reversed_),
			boolToString(reverse_scan_),
			first_angle,
			last_angle,
			range_for_index(first_index),
			range_for_index(center_index),
			range_for_index(last_index),
			angle_for_index(front_index),
			angle_for_index(left_index),
			angle_for_index(right_index),
			angle_for_index(rear_index),
			range_for_index(front_index),
			range_for_index(left_index),
			range_for_index(right_index),
			range_for_index(rear_index),
			front_index,
			front_index,
			left_index,
			right_index,
			rear_index,
			nearest_index,
			nearest_angle,
			nearest_range,
			raw_front.point_angle_rad,
			raw_front.range_m,
			raw_left.point_angle_rad,
			raw_right.point_angle_rad,
			raw_rear.point_angle_rad,
			scan_geometry_throttle_sec_);
		return;
	}

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(scan_geometry_throttle_sec_, 1000),
		"Scan geometry: angle_min=%.6f angle_max=%.6f angle_increment=%.6f ranges=%zu index(front=%d left=%d right=%d rear=%d) range(front=%.3f left=%.3f right=%.3f rear=%.3f) sector_min(front=%.3f left=%.3f right=%.3f rear=%.3f) nearest_hit(index=%d angle=%.3f range=%.3f) raw_front(angle=%.3f err=%.3f range=%.3f intensity=%.1f)",
		static_cast<double>(scan_message.angle_min),
		static_cast<double>(scan_message.angle_max),
		static_cast<double>(scan_message.angle_increment),
		scan_message.ranges.size(),
		front_index,
		left_index,
		right_index,
		rear_index,
		range_for_index(front_index),
		range_for_index(left_index),
		range_for_index(right_index),
		range_for_index(rear_index),
		min_range_in_sector(0.0),
		min_range_in_sector(PI * 0.5),
		min_range_in_sector(-PI * 0.5),
		min_range_in_sector(PI),
		nearest_index,
		nearest_angle,
		nearest_range,
		raw_front.point_angle_rad,
		raw_front.angle_error_rad,
		raw_front.range_m,
		raw_front.intensity);
}

int LidarDriverNode::computeScanIndexForAngle(
	const sensor_msgs::msg::LaserScan &scan_message,
	double angle_rad) const
{
	if (scan_message.ranges.empty() || scan_message.angle_increment == 0.0F)
	{
		return -1;
	}

	const double angle_increment = static_cast<double>(scan_message.angle_increment);
	if (angle_increment <= 0.0)
	{
		return -1;
	}

	const double angle_min = static_cast<double>(scan_message.angle_min);
	const double angle_max = static_cast<double>(scan_message.angle_max);
	const double angle_span = angle_max - angle_min;
	const bool is_full_circle = angle_span >= ((2.0 * PI) - 1e-6);
	double relative_angle = 0.0;

	if (is_full_circle)
	{
		relative_angle = std::fmod(angle_rad - angle_min, 2.0 * PI);
		if (relative_angle < 0.0)
		{
			relative_angle += 2.0 * PI;
		}
	}
	else
	{
		while (angle_rad < angle_min)
		{
			angle_rad += 2.0 * PI;
		}
		while (angle_rad > angle_max)
		{
			angle_rad -= 2.0 * PI;
		}
		if (angle_rad < angle_min || angle_rad > angle_max)
		{
			return -1;
		}
		relative_angle = angle_rad - angle_min;
	}

	std::size_t index = static_cast<std::size_t>(std::llround(relative_angle / angle_increment));
	if (index >= scan_message.ranges.size())
	{
		index = scan_message.ranges.size() - 1U;
	}

	return static_cast<int>(index);
}

std::string LidarDriverNode::resolveTopicName() const
{
	const std::string sanitized_namespace = getSanitizedNamespace();
	if (sanitized_namespace.empty())
	{
		return topic_name_;
	}

	if (topic_name_ == "/scan")
	{
		return "scan";
	}

	return topic_name_;
}

std::string LidarDriverNode::resolveFrameId() const
{
	const std::string sanitized_namespace = getSanitizedNamespace();
	if (sanitized_namespace.empty())
	{
		return frame_id_;
	}

	if (frame_id_.find('/') != std::string::npos)
	{
		return frame_id_;
	}

	return sanitized_namespace + "/" + frame_id_;
}

std::string LidarDriverNode::getSanitizedNamespace() const
{
	std::string namespace_value = get_namespace();
	if (namespace_value == "/" || namespace_value.empty())
	{
		return "";
	}

	if (!namespace_value.empty() && namespace_value.front() == '/')
	{
		namespace_value.erase(namespace_value.begin());
	}

	if (!namespace_value.empty() && namespace_value.back() == '/')
	{
		namespace_value.pop_back();
	}

	return namespace_value;
}
