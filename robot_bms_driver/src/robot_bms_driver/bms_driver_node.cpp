#include "robot_bms_driver/bms_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

using namespace robot::hw::bms;

namespace
{

const char *boolToString(bool value)
{
	return value ? "true" : "false";
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

std::string bytesToHex(const std::vector<std::uint8_t> &bytes)
{
	std::ostringstream stream;
	stream << std::hex << std::setfill('0');
	for (std::uint8_t byte : bytes)
	{
		stream << std::setw(2) << static_cast<int>(byte);
	}

	return stream.str();
}

}  // namespace

BmsDriverNode::BmsDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_bms_driver", options),
	enabled_(false),
	port_("/dev/robot/bms"),
	baudrate_(9600),
	frame_id_("base_link"),
	topic_name_("/battery_state"),
	poll_interval_ms_(1000),
	read_timeout_ms_(100),
	frame_timeout_ms_(250),
	protocol_("placeholder"),
	publish_diagnostics_(true),
	log_raw_frames_(false),
	warn_timeout_ms_(5000),
	read_buffer_size_(512U),
	structured_logging_enabled_(true),
	diagnostics_throttle_sec_(5.0),
	battery_publisher_(nullptr),
	serial_port_(nullptr),
	parser_(nullptr),
	poll_timer_(nullptr),
	throttle_clock_(RCL_STEADY_TIME),
	start_time_(std::chrono::steady_clock::now()),
	last_rx_time_(start_time_),
	last_valid_frame_time_(start_time_),
	has_rx_bytes_(false),
	has_valid_frame_(false)
{
	declareParameters();
	loadParameters();
	validateParameters();
	setup();
}

BmsDriverNode::~BmsDriverNode()
{
	if (serial_port_)
	{
		serial_port_->closePort();
	}
}

void BmsDriverNode::declareParameters()
{
	declare_parameter("bms.enabled", enabled_);
	declare_parameter("bms.port", port_);
	declare_parameter("bms.baudrate", baudrate_);
	declare_parameter("bms.frame_id", frame_id_);
	declare_parameter("bms.topic_name", topic_name_);
	declare_parameter("bms.poll_interval_ms", poll_interval_ms_);
	declare_parameter("bms.read_timeout_ms", read_timeout_ms_);
	declare_parameter("bms.frame_timeout_ms", frame_timeout_ms_);
	declare_parameter("bms.protocol", protocol_);
	declare_parameter("bms.publish_diagnostics", publish_diagnostics_);
	declare_parameter("bms.log_raw_frames", log_raw_frames_);
	declare_parameter("bms.warn_timeout_ms", warn_timeout_ms_);
	declare_parameter("bms.read_buffer_size", static_cast<int>(read_buffer_size_));
	declare_parameter("logging.structured_enabled", structured_logging_enabled_);
	declare_parameter("logging.diagnostics_throttle_sec", diagnostics_throttle_sec_);
}

void BmsDriverNode::loadParameters()
{
	int read_buffer_size = static_cast<int>(read_buffer_size_);

	get_parameter("bms.enabled", enabled_);
	get_parameter("bms.port", port_);
	get_parameter("bms.baudrate", baudrate_);
	get_parameter("bms.frame_id", frame_id_);
	get_parameter("bms.topic_name", topic_name_);
	get_parameter("bms.poll_interval_ms", poll_interval_ms_);
	get_parameter("bms.read_timeout_ms", read_timeout_ms_);
	get_parameter("bms.frame_timeout_ms", frame_timeout_ms_);
	get_parameter("bms.protocol", protocol_);
	get_parameter("bms.publish_diagnostics", publish_diagnostics_);
	get_parameter("bms.log_raw_frames", log_raw_frames_);
	get_parameter("bms.warn_timeout_ms", warn_timeout_ms_);
	get_parameter("bms.read_buffer_size", read_buffer_size);
	get_parameter("logging.structured_enabled", structured_logging_enabled_);
	get_parameter("logging.diagnostics_throttle_sec", diagnostics_throttle_sec_);

	read_buffer_size_ = static_cast<std::size_t>(std::max(1, read_buffer_size));
}

void BmsDriverNode::validateParameters()
{
	if (baudrate_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "bms.baudrate must be positive. Resetting to 9600");
		baudrate_ = 9600;
	}

	if (frame_id_.empty())
	{
		RCLCPP_WARN(get_logger(), "bms.frame_id is empty. Resetting to base_link");
		frame_id_ = "base_link";
	}

	if (topic_name_.empty())
	{
		RCLCPP_WARN(get_logger(), "bms.topic_name is empty. Resetting to /battery_state");
		topic_name_ = "/battery_state";
	}

	if (poll_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "bms.poll_interval_ms must be positive. Resetting to 1000");
		poll_interval_ms_ = 1000;
	}

	if (read_timeout_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "bms.read_timeout_ms must be positive. Resetting to 100");
		read_timeout_ms_ = 100;
	}

	if (frame_timeout_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "bms.frame_timeout_ms must be positive. Resetting to 250");
		frame_timeout_ms_ = 250;
	}

	if (warn_timeout_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "bms.warn_timeout_ms must be positive. Resetting to 5000");
		warn_timeout_ms_ = 5000;
	}

	if (!std::isfinite(diagnostics_throttle_sec_) || diagnostics_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.diagnostics_throttle_sec must be positive. Resetting to 5.0");
		diagnostics_throttle_sec_ = 5.0;
	}
}

void BmsDriverNode::setup()
{
	logConfig();

	if (!enabled_)
	{
		RCLCPP_INFO(get_logger(), "BMS driver is disabled. Set use_bms:=true or bms.enabled:=true to enable it.");
		return;
	}

	battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(resolveTopicName(), rclcpp::QoS(10));
	serial_port_ = std::make_unique<SerialPort>(get_logger());
	parser_ = std::make_unique<BmsParser>(protocol_);

	if (parser_->isPlaceholderProtocol())
	{
		RCLCPP_WARN(
			get_logger(),
			"BMS protocol is placeholder; serial bytes may be read but /battery_state will not be published until a concrete parser is configured");
	}
	else if (!parser_->isSupportedProtocol())
	{
		RCLCPP_WARN(get_logger(), "Unsupported BMS protocol '%s'; no battery values will be published", protocol_.c_str());
	}

	poll_timer_ = create_wall_timer(std::chrono::milliseconds(poll_interval_ms_), [this]() { pollSerial(); });
}

void BmsDriverNode::pollSerial()
{
	if (!enabled_ || !parser_ || !openSerialIfNeeded())
	{
		logTimeoutIfNeeded();
		return;
	}

	std::vector<std::uint8_t> buffer(read_buffer_size_, 0U);
	const ssize_t read_size = serial_port_->readSome(buffer.data(), buffer.size());
	if (read_size < 0)
	{
		serial_port_->closePort();
		logTimeoutIfNeeded();
		return;
	}

	if (read_size == 0)
	{
		logTimeoutIfNeeded();
		return;
	}

	buffer.resize(static_cast<std::size_t>(read_size));
	has_rx_bytes_ = true;
	last_rx_time_ = std::chrono::steady_clock::now();
	logRawFrame(buffer);

	std::string reason;
	const std::optional<BatterySample> sample = parser_->parseBytes(buffer.data(), buffer.size(), reason);
	if (!sample.has_value())
	{
		logFrameRejected(reason, buffer.size());
		logTimeoutIfNeeded();
		return;
	}

	has_valid_frame_ = true;
	last_valid_frame_time_ = std::chrono::steady_clock::now();
	publishBatteryState(*sample);
}

void BmsDriverNode::publishBatteryState(const BatterySample &sample)
{
	if (!battery_publisher_)
	{
		return;
	}

	sensor_msgs::msg::BatteryState message;
	message.header.stamp = now();
	message.header.frame_id = frame_id_;
	message.voltage = optionalToFloat(sample.voltage);
	message.temperature = optionalToFloat(sample.temperature);
	message.current = optionalToFloat(sample.current);
	message.charge = optionalToFloat(sample.charge);
	message.capacity = optionalToFloat(sample.capacity);
	message.design_capacity = optionalToFloat(sample.design_capacity);
	message.percentage = optionalToFloat(sample.percentage);
	message.power_supply_status = sample.power_supply_status.value_or(sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
	message.power_supply_health = sample.power_supply_health.value_or(sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
	message.power_supply_technology = sample.power_supply_technology.value_or(sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
	message.present = sample.present.value_or(true);
	message.cell_voltage = toFloatVector(sample.cell_voltage);
	message.cell_temperature = toFloatVector(sample.cell_temperature);
	message.location = sample.location;
	message.serial_number = sample.serial_number;

	battery_publisher_->publish(message);

	if (structured_logging_enabled_ && publish_diagnostics_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(diagnostics_throttle_sec_, 5000),
			"ROBOT_HW_LOG schema=v1 tag=SENSOR component=bms event=battery_publish node=%s namespace=%s topic=%s frame_id=%s protocol=%s voltage_v=%.3f current_a=%.3f percentage=%.3f present=%s cell_count=%zu result=published",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolveTopicName().c_str(),
			sanitizeLogValue(frame_id_).c_str(),
			sanitizeLogValue(protocol_).c_str(),
			static_cast<double>(message.voltage),
			static_cast<double>(message.current),
			static_cast<double>(message.percentage),
			boolToString(message.present),
			message.cell_voltage.size());
	}
}

void BmsDriverNode::logConfig() const
{
	if (!structured_logging_enabled_)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=bms event=bms_config node=%s namespace=%s enabled=%s port=%s baudrate=%d topic=%s frame_id=%s protocol=%s poll_interval_ms=%d read_timeout_ms=%d frame_timeout_ms=%d publish_diagnostics=%s log_raw_frames=%s warn_timeout_ms=%d result=%s reason=%s",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		boolToString(enabled_),
		sanitizeLogValue(port_).c_str(),
		baudrate_,
		sanitizeLogValue(resolveTopicName()).c_str(),
		sanitizeLogValue(frame_id_).c_str(),
		sanitizeLogValue(protocol_).c_str(),
		poll_interval_ms_,
		read_timeout_ms_,
		frame_timeout_ms_,
		boolToString(publish_diagnostics_),
		boolToString(log_raw_frames_),
		warn_timeout_ms_,
		enabled_ ? "configured" : "disabled",
		enabled_ ? "none" : "bms_disabled");
}

void BmsDriverNode::logFrameRejected(const std::string &reason, std::size_t bytes_received) const
{
	if (!structured_logging_enabled_ || !publish_diagnostics_)
	{
		return;
	}

	RCLCPP_WARN_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(diagnostics_throttle_sec_, 5000),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=bms event=bms_frame node=%s namespace=%s protocol=%s bytes=%zu result=rejected reason=%s",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		sanitizeLogValue(protocol_).c_str(),
		bytes_received,
		sanitizeLogValue(reason).c_str());
}

void BmsDriverNode::logTimeoutIfNeeded() const
{
	if (!structured_logging_enabled_ || !publish_diagnostics_)
	{
		return;
	}

	const auto now_time = std::chrono::steady_clock::now();
	const auto reference_time = has_valid_frame_ ? last_valid_frame_time_ : start_time_;
	const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - reference_time).count();
	if (elapsed_ms < warn_timeout_ms_)
	{
		return;
	}

	const char *reason = has_rx_bytes_ ? "timeout_waiting_for_valid_bms_frame" : "timeout_waiting_for_bms_bytes";
	RCLCPP_WARN_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(diagnostics_throttle_sec_, 5000),
		"ROBOT_HW_LOG schema=v1 tag=SENSOR component=bms event=bms_timeout node=%s namespace=%s protocol=%s port=%s elapsed_ms=%ld has_rx_bytes=%s has_valid_frame=%s result=warn reason=%s",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		sanitizeLogValue(protocol_).c_str(),
		sanitizeLogValue(port_).c_str(),
		static_cast<long>(elapsed_ms),
		boolToString(has_rx_bytes_),
		boolToString(has_valid_frame_),
		reason);
}

void BmsDriverNode::logRawFrame(const std::vector<std::uint8_t> &bytes) const
{
	if (!log_raw_frames_)
	{
		return;
	}

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(diagnostics_throttle_sec_, 5000),
		"ROBOT_HW_LOG schema=v1 tag=SERIAL component=bms event=bms_raw_frame node=%s namespace=%s bytes=%zu data_hex=%s result=received",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		bytes.size(),
		bytesToHex(bytes).c_str());
}

bool BmsDriverNode::openSerialIfNeeded()
{
	if (serial_port_ && serial_port_->isOpen())
	{
		return true;
	}

	if (!serial_port_)
	{
		serial_port_ = std::make_unique<SerialPort>(get_logger());
	}

	const bool opened = serial_port_->openPort(port_, baudrate_);
	if (!opened && structured_logging_enabled_ && publish_diagnostics_)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(diagnostics_throttle_sec_, 5000),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=bms event=bms_serial_open node=%s namespace=%s port=%s baudrate=%d result=failed reason=open_failed",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_);
	}

	return opened;
}

std::string BmsDriverNode::resolveTopicName() const
{
	if (std::string(get_namespace()) != "/" && topic_name_ == "/battery_state")
	{
		return "battery_state";
	}

	return topic_name_;
}

float BmsDriverNode::optionalToFloat(const std::optional<double> &value)
{
	if (!value.has_value())
	{
		return std::numeric_limits<float>::quiet_NaN();
	}

	return static_cast<float>(*value);
}

std::vector<float> BmsDriverNode::toFloatVector(const std::vector<double> &values)
{
	std::vector<float> result;
	result.reserve(values.size());
	for (double value : values)
	{
		result.push_back(static_cast<float>(value));
	}

	return result;
}