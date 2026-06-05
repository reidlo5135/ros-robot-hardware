#include "robot_base_driver/robot_base_driver_node.hpp"

#include <stdexcept>

#include <rcutils/logging.h>

using namespace robot::hw::base;

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

bool isValidCovarianceDiagonal(const std::vector<double> &diagonal)
{
	if (diagonal.size() != 6U)
	{
		return false;
	}

	for (double value : diagonal)
	{
		if (value < 0.0 || !std::isfinite(value))
		{
			return false;
		}
	}

	return true;
}

bool isValidCovarianceMatrix(const std::vector<double> &covariance)
{
	if (covariance.size() != 9U)
	{
		return false;
	}

	for (double value : covariance)
	{
		if (!std::isfinite(value))
		{
			return false;
		}
	}

	return true;
}

void applyCovarianceDiagonal(
	const std::vector<double> &diagonal,
	std::array<double, 36> &covariance)
{
	covariance.fill(0.0);
	for (std::size_t index = 0; index < 6U && index < diagonal.size(); ++index)
	{
		covariance[(index * 6U) + index] = diagonal[index];
	}
}

void applyCovarianceMatrix(
	const std::vector<double> &values,
	std::array<double, 9> &covariance)
{
	covariance.fill(0.0);
	for (std::size_t index = 0; index < 9U && index < values.size(); ++index)
	{
		covariance[index] = values[index];
	}
}

}  // namespace

RobotBaseDriverNode::RobotBaseDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_base_driver", options),
	port_("/dev/ttyACM0"),
	baudrate_(1000000),
	opencr_id_(ControlTable::OPENCR_ID),
	protocol_version_(2.0),
	cmd_vel_topic_(DEFAULT_CMD_VEL_TOPIC),
	cmd_vel_stamped_topic_(DEFAULT_CMD_VEL_STAMPED_TOPIC),
	odom_topic_(DEFAULT_ODOM_TOPIC),
	imu_topic_(DEFAULT_IMU_TOPIC),
	joint_states_topic_(DEFAULT_JOINT_STATES_TOPIC),
	odom_frame_id_("odom"),
	base_frame_id_("base_footprint"),
	imu_frame_id_("imu_link"),
	scan_frame_id_("base_scan"),
	wheel_left_joint_name_("wheel_left_joint"),
	wheel_right_joint_name_("wheel_right_joint"),
	wheel_separation_m_(DEFAULT_WHEEL_SEPARATION_M),
	wheel_radius_m_(DEFAULT_WHEEL_RADIUS_M),
	left_encoder_sign_(1),
	right_encoder_sign_(1),
	swap_wheel_encoders_(false),
	profile_acceleration_constant_(DEFAULT_PROFILE_ACCELERATION_CONSTANT),
	profile_acceleration_(0.0),
	odom_pose_covariance_diagonal_(
		DEFAULT_ODOM_POSE_COVARIANCE_DIAGONAL.begin(),
		DEFAULT_ODOM_POSE_COVARIANCE_DIAGONAL.end()),
	odom_twist_covariance_diagonal_(
		DEFAULT_ODOM_TWIST_COVARIANCE_DIAGONAL.begin(),
		DEFAULT_ODOM_TWIST_COVARIANCE_DIAGONAL.end()),
	imu_orientation_covariance_(
		DEFAULT_IMU_ORIENTATION_COVARIANCE.begin(),
		DEFAULT_IMU_ORIENTATION_COVARIANCE.end()),
	imu_angular_velocity_covariance_(
		DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE.begin(),
		DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE.end()),
	imu_linear_acceleration_covariance_(
		DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE.begin(),
		DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE.end()),
	command_mode_(DEFAULT_COMMAND_MODE),
	is_publish_tf_(true),
	is_using_imu_for_yaw_(false),
	is_publishing_imu_(true),
	is_publishing_joint_states_(true),
	is_heartbeat_enabled_(true),
	heartbeat_interval_ms_(100),
	poll_interval_ms_(DEFAULT_POLL_INTERVAL_MS),
	startup_delay_ms_(1000),
	is_reconnect_on_error_(true),
	reconnect_interval_ms_(1000),
	debug_motor_command_(true),
	is_stamped_cmd_vel_enabled_(false),
	is_motor_torque_enable_on_startup_(true),
	is_motor_torque_enable_ack_required_(true),
	is_imu_recalibration_on_startup_(false),
	is_imu_recalibration_ack_required_(false),
	is_profile_acceleration_ack_required_(false),
	is_profile_acceleration_on_startup_(false),
	is_heartbeat_ack_required_(false),
	is_startup_initial_state_read_required_(true),
	startup_initial_state_read_retries_(5),
	startup_initial_state_read_retry_interval_ms_(200),
	is_serial_packet_logging_enabled_(false),
	is_read_rate_logging_enabled_(true),
	response_timeout_ms_(500),
	transaction_gap_us_(10000),
	poll_mode_("full"),
	max_consecutive_poll_failures_(5),
	is_polling_device_status_(false),
	require_device_status_(false),
	require_imu_(false),
	reconnect_on_poll_failure_(false),
	reopen_serial_on_poll_failure_(false),
	probe_registers_on_startup_(true),
	debug_odom_(false),
	debug_odom_interval_ms_(1000),
	debug_tf_(false),
	debug_poll_timing_(false),
	target_odom_rate_hz_(20.0),
	debug_odom_auto_enabled_(false),
	debug_tf_auto_enabled_(false),
	is_structured_logging_enabled_(true),
	base_state_throttle_sec_(1.0),
	cmd_vel_throttle_sec_(1.0),
	odom_throttle_sec_(1.0),
	tf_throttle_sec_(1.0),
	imu_throttle_sec_(1.0),
	joint_state_throttle_sec_(1.0),
	serial_state_throttle_sec_(2.0),
	opencr_state_throttle_sec_(1.0),
	poll_timing_throttle_sec_(2.0),
	is_frame_diagnostics_enabled_(true),
	is_topic_diagnostics_enabled_(true),
	serial_port_(nullptr),
	opencr_client_(nullptr),
	odometry_integrator_(nullptr),
	odom_publisher_(nullptr),
	imu_publisher_(nullptr),
	joint_state_publisher_(nullptr),
	tf_broadcaster_(nullptr),
	cmd_vel_subscription_(nullptr),
	cmd_vel_stamped_subscription_(nullptr),
	reconnect_timer_(nullptr),
	reset_odometry_service_(nullptr),
	state_mutex_(),
	is_shutdown_requested_(false),
	is_reconnecting_(false),
	throttle_clock_(RCL_STEADY_TIME),
	has_logged_publish_success_(false),
	last_device_status_(0),
	has_seen_device_status_(false),
	last_motor_torque_enabled_(false),
	has_seen_motor_torque_enabled_(false),
	has_recent_cmd_vel_(false),
	last_cmd_vel_linear_x_(0.0),
	last_cmd_vel_angular_z_(0.0)
{
	declareParameters();
	loadParameters();
	validateParameters();
	autoConfigureDiagnosticsFromLogLevel();
	logParameterSummary();
	logStartupFrameSanity();
	logFrameConfig();
	logTopicConfig();
	setupPublishers();
	setupSubscriptions();
	setupServices();
	setupOdometryIntegrator();
	startDriver();
}

RobotBaseDriverNode::~RobotBaseDriverNode()
{
	is_shutdown_requested_.store(true);
	cancelReconnect();
	stopDriver();
}

void RobotBaseDriverNode::declareParameters()
{
	declare_parameter("port", port_);
	declare_parameter("baudrate", baudrate_);
	declare_parameter("opencr_id", opencr_id_);
	declare_parameter("protocol_version", protocol_version_);
	declare_parameter("cmd_vel_topic", cmd_vel_topic_);
	declare_parameter("cmd_vel_stamped_topic", cmd_vel_stamped_topic_);
	declare_parameter("odom_topic", odom_topic_);
	declare_parameter("imu_topic", imu_topic_);
	declare_parameter("joint_states_topic", joint_states_topic_);
	declare_parameter("odom_frame_id", odom_frame_id_);
	declare_parameter("base_frame_id", base_frame_id_);
	declare_parameter("imu_frame_id", imu_frame_id_);
	declare_parameter("scan_frame_id", scan_frame_id_);
	declare_parameter("wheel_left_joint_name", wheel_left_joint_name_);
	declare_parameter("wheel_right_joint_name", wheel_right_joint_name_);
	declare_parameter("wheel_separation", wheel_separation_m_);
	declare_parameter("wheel_radius", wheel_radius_m_);
	declare_parameter("left_encoder_sign", left_encoder_sign_);
	declare_parameter("right_encoder_sign", right_encoder_sign_);
	declare_parameter("swap_wheel_encoders", swap_wheel_encoders_);
	declare_parameter("motors.profile_acceleration_constant", profile_acceleration_constant_);
	declare_parameter("motors.profile_acceleration", profile_acceleration_);
	declare_parameter("odom_pose_covariance_diagonal", odom_pose_covariance_diagonal_);
	declare_parameter("odom_twist_covariance_diagonal", odom_twist_covariance_diagonal_);
	declare_parameter("imu_orientation_covariance", imu_orientation_covariance_);
	declare_parameter("imu_angular_velocity_covariance", imu_angular_velocity_covariance_);
	declare_parameter("imu_linear_acceleration_covariance", imu_linear_acceleration_covariance_);
	declare_parameter("command_mode", command_mode_);
	declare_parameter("publish_tf", is_publish_tf_);
	declare_parameter("use_imu_for_yaw", is_using_imu_for_yaw_);
	declare_parameter("publish_imu", is_publishing_imu_);
	declare_parameter("publish_joint_states", is_publishing_joint_states_);
	declare_parameter("heartbeat_enabled", is_heartbeat_enabled_);
	declare_parameter("heartbeat_interval_ms", heartbeat_interval_ms_);
	declare_parameter("poll_interval_ms", poll_interval_ms_);
	declare_parameter("startup_delay_ms", startup_delay_ms_);
	declare_parameter("reconnect_on_error", is_reconnect_on_error_);
	declare_parameter("reconnect_interval_ms", reconnect_interval_ms_);
	declare_parameter("debug_motor_command", debug_motor_command_);
	declare_parameter("enable_stamped_cmd_vel", is_stamped_cmd_vel_enabled_);
	declare_parameter("motor_torque_enable_on_startup", is_motor_torque_enable_on_startup_);
	declare_parameter("motor_torque_enable_requires_ack", is_motor_torque_enable_ack_required_);
	declare_parameter("imu_recalibration_on_startup", is_imu_recalibration_on_startup_);
	declare_parameter("imu_recalibration_requires_ack", is_imu_recalibration_ack_required_);
	declare_parameter("profile_acceleration_requires_ack", is_profile_acceleration_ack_required_);
	declare_parameter("profile_acceleration_on_startup", is_profile_acceleration_on_startup_);
	declare_parameter("heartbeat_requires_ack", is_heartbeat_ack_required_);
	declare_parameter("startup_require_initial_state_read", is_startup_initial_state_read_required_);
	declare_parameter("startup_initial_state_read_retries", startup_initial_state_read_retries_);
	declare_parameter("startup_initial_state_read_retry_interval_ms", startup_initial_state_read_retry_interval_ms_);
	declare_parameter("log_serial_packets", is_serial_packet_logging_enabled_);
	declare_parameter("log_read_rate", is_read_rate_logging_enabled_);
	declare_parameter("response_timeout_ms", response_timeout_ms_);
	declare_parameter("transaction_gap_us", transaction_gap_us_);
	declare_parameter("poll_mode", poll_mode_);
	declare_parameter("max_consecutive_poll_failures", max_consecutive_poll_failures_);
	declare_parameter("poll_device_status", is_polling_device_status_);
	declare_parameter("require_device_status", require_device_status_);
	declare_parameter("require_imu", require_imu_);
	declare_parameter("reconnect_on_poll_failure", reconnect_on_poll_failure_);
	declare_parameter("reopen_serial_on_poll_failure", reopen_serial_on_poll_failure_);
	declare_parameter("probe_registers_on_startup", probe_registers_on_startup_);
	declare_parameter("debug_odom", debug_odom_);
	declare_parameter("debug_odom_interval_ms", debug_odom_interval_ms_);
	declare_parameter("debug_tf", debug_tf_);
	declare_parameter("debug_poll_timing", debug_poll_timing_);
	declare_parameter("target_odom_rate_hz", target_odom_rate_hz_);
	declare_parameter("logging.structured_enabled", is_structured_logging_enabled_);
	declare_parameter("logging.base_state_throttle_sec", base_state_throttle_sec_);
	declare_parameter("logging.cmd_vel_throttle_sec", cmd_vel_throttle_sec_);
	declare_parameter("logging.odom_throttle_sec", odom_throttle_sec_);
	declare_parameter("logging.tf_throttle_sec", tf_throttle_sec_);
	declare_parameter("logging.imu_throttle_sec", imu_throttle_sec_);
	declare_parameter("logging.joint_state_throttle_sec", joint_state_throttle_sec_);
	declare_parameter("logging.serial_state_throttle_sec", serial_state_throttle_sec_);
	declare_parameter("logging.opencr_state_throttle_sec", opencr_state_throttle_sec_);
	declare_parameter("logging.poll_timing_throttle_sec", poll_timing_throttle_sec_);
	declare_parameter("logging.frame_diagnostics_enabled", is_frame_diagnostics_enabled_);
	declare_parameter("logging.topic_diagnostics_enabled", is_topic_diagnostics_enabled_);
}

void RobotBaseDriverNode::loadParameters()
{
	get_parameter("port", port_);
	get_parameter("baudrate", baudrate_);
	get_parameter("opencr_id", opencr_id_);
	get_parameter("protocol_version", protocol_version_);
	get_parameter("cmd_vel_topic", cmd_vel_topic_);
	get_parameter("cmd_vel_stamped_topic", cmd_vel_stamped_topic_);
	get_parameter("odom_topic", odom_topic_);
	get_parameter("imu_topic", imu_topic_);
	get_parameter("joint_states_topic", joint_states_topic_);
	get_parameter("odom_frame_id", odom_frame_id_);
	get_parameter("base_frame_id", base_frame_id_);
	get_parameter("imu_frame_id", imu_frame_id_);
	get_parameter("scan_frame_id", scan_frame_id_);
	get_parameter("wheel_left_joint_name", wheel_left_joint_name_);
	get_parameter("wheel_right_joint_name", wheel_right_joint_name_);
	get_parameter("wheel_separation", wheel_separation_m_);
	get_parameter("wheel_radius", wheel_radius_m_);
	get_parameter("left_encoder_sign", left_encoder_sign_);
	get_parameter("right_encoder_sign", right_encoder_sign_);
	get_parameter("swap_wheel_encoders", swap_wheel_encoders_);
	get_parameter("motors.profile_acceleration_constant", profile_acceleration_constant_);
	get_parameter("motors.profile_acceleration", profile_acceleration_);
	get_parameter("odom_pose_covariance_diagonal", odom_pose_covariance_diagonal_);
	get_parameter("odom_twist_covariance_diagonal", odom_twist_covariance_diagonal_);
	get_parameter("imu_orientation_covariance", imu_orientation_covariance_);
	get_parameter("imu_angular_velocity_covariance", imu_angular_velocity_covariance_);
	get_parameter("imu_linear_acceleration_covariance", imu_linear_acceleration_covariance_);
	get_parameter("command_mode", command_mode_);
	get_parameter("publish_tf", is_publish_tf_);
	get_parameter("use_imu_for_yaw", is_using_imu_for_yaw_);
	get_parameter("publish_imu", is_publishing_imu_);
	get_parameter("publish_joint_states", is_publishing_joint_states_);
	get_parameter("heartbeat_enabled", is_heartbeat_enabled_);
	get_parameter("heartbeat_interval_ms", heartbeat_interval_ms_);
	get_parameter("poll_interval_ms", poll_interval_ms_);
	get_parameter("startup_delay_ms", startup_delay_ms_);
	get_parameter("reconnect_on_error", is_reconnect_on_error_);
	get_parameter("reconnect_interval_ms", reconnect_interval_ms_);
	get_parameter("debug_motor_command", debug_motor_command_);
	get_parameter("enable_stamped_cmd_vel", is_stamped_cmd_vel_enabled_);
	get_parameter("motor_torque_enable_on_startup", is_motor_torque_enable_on_startup_);
	get_parameter("motor_torque_enable_requires_ack", is_motor_torque_enable_ack_required_);
	get_parameter("imu_recalibration_on_startup", is_imu_recalibration_on_startup_);
	get_parameter("imu_recalibration_requires_ack", is_imu_recalibration_ack_required_);
	get_parameter("profile_acceleration_requires_ack", is_profile_acceleration_ack_required_);
	get_parameter("profile_acceleration_on_startup", is_profile_acceleration_on_startup_);
	get_parameter("heartbeat_requires_ack", is_heartbeat_ack_required_);
	get_parameter("startup_require_initial_state_read", is_startup_initial_state_read_required_);
	get_parameter("startup_initial_state_read_retries", startup_initial_state_read_retries_);
	get_parameter("startup_initial_state_read_retry_interval_ms", startup_initial_state_read_retry_interval_ms_);
	get_parameter("log_serial_packets", is_serial_packet_logging_enabled_);
	get_parameter("log_read_rate", is_read_rate_logging_enabled_);
	get_parameter("response_timeout_ms", response_timeout_ms_);
	get_parameter("transaction_gap_us", transaction_gap_us_);
	get_parameter("poll_mode", poll_mode_);
	get_parameter("max_consecutive_poll_failures", max_consecutive_poll_failures_);
	get_parameter("poll_device_status", is_polling_device_status_);
	get_parameter("require_device_status", require_device_status_);
	get_parameter("require_imu", require_imu_);
	get_parameter("reconnect_on_poll_failure", reconnect_on_poll_failure_);
	get_parameter("reopen_serial_on_poll_failure", reopen_serial_on_poll_failure_);
	get_parameter("probe_registers_on_startup", probe_registers_on_startup_);
	get_parameter("debug_odom", debug_odom_);
	get_parameter("debug_odom_interval_ms", debug_odom_interval_ms_);
	get_parameter("debug_tf", debug_tf_);
	get_parameter("debug_poll_timing", debug_poll_timing_);
	get_parameter("target_odom_rate_hz", target_odom_rate_hz_);
	get_parameter("logging.structured_enabled", is_structured_logging_enabled_);
	get_parameter("logging.base_state_throttle_sec", base_state_throttle_sec_);
	get_parameter("logging.cmd_vel_throttle_sec", cmd_vel_throttle_sec_);
	get_parameter("logging.odom_throttle_sec", odom_throttle_sec_);
	get_parameter("logging.tf_throttle_sec", tf_throttle_sec_);
	get_parameter("logging.imu_throttle_sec", imu_throttle_sec_);
	get_parameter("logging.joint_state_throttle_sec", joint_state_throttle_sec_);
	get_parameter("logging.serial_state_throttle_sec", serial_state_throttle_sec_);
	get_parameter("logging.opencr_state_throttle_sec", opencr_state_throttle_sec_);
	get_parameter("logging.poll_timing_throttle_sec", poll_timing_throttle_sec_);
	get_parameter("logging.frame_diagnostics_enabled", is_frame_diagnostics_enabled_);
	get_parameter("logging.topic_diagnostics_enabled", is_topic_diagnostics_enabled_);
}

void RobotBaseDriverNode::validateParameters()
{
	if (opencr_id_ < 0 || opencr_id_ > 252)
	{
		RCLCPP_WARN(
			get_logger(),
			"opencr_id must be between 0 and 252. Resetting to %u",
			ControlTable::OPENCR_ID);
		opencr_id_ = ControlTable::OPENCR_ID;
	}

	if (std::abs(protocol_version_ - 2.0) > 0.01)
	{
		RCLCPP_WARN(get_logger(), "Only Dynamixel Protocol 2.0 is supported. Resetting protocol_version to 2.0");
		protocol_version_ = 2.0;
	}

	if (wheel_separation_m_ <= 0.0)
	{
		RCLCPP_WARN(
			get_logger(),
			"wheel_separation must be positive. Resetting to %.3f",
			DEFAULT_WHEEL_SEPARATION_M);
		wheel_separation_m_ = DEFAULT_WHEEL_SEPARATION_M;
	}

	if (wheel_radius_m_ <= 0.0)
	{
		RCLCPP_WARN(
			get_logger(),
			"wheel_radius must be positive. Resetting to %.3f",
			DEFAULT_WHEEL_RADIUS_M);
		wheel_radius_m_ = DEFAULT_WHEEL_RADIUS_M;
	}

	if (!(left_encoder_sign_ == -1 || left_encoder_sign_ == 1))
	{
		RCLCPP_WARN(get_logger(), "left_encoder_sign must be either -1 or 1. Resetting to 1.");
		left_encoder_sign_ = 1;
	}

	if (!(right_encoder_sign_ == -1 || right_encoder_sign_ == 1))
	{
		RCLCPP_WARN(get_logger(), "right_encoder_sign must be either -1 or 1. Resetting to 1.");
		right_encoder_sign_ = 1;
	}

	if (!isValidCovarianceDiagonal(odom_pose_covariance_diagonal_))
	{
		RCLCPP_WARN(
			get_logger(),
			"odom_pose_covariance_diagonal must contain exactly 6 non-negative finite values. Resetting to defaults.");
		odom_pose_covariance_diagonal_ = std::vector<double>(
			DEFAULT_ODOM_POSE_COVARIANCE_DIAGONAL.begin(),
			DEFAULT_ODOM_POSE_COVARIANCE_DIAGONAL.end());
	}

	if (!isValidCovarianceDiagonal(odom_twist_covariance_diagonal_))
	{
		RCLCPP_WARN(
			get_logger(),
			"odom_twist_covariance_diagonal must contain exactly 6 non-negative finite values. Resetting to defaults.");
		odom_twist_covariance_diagonal_ = std::vector<double>(
			DEFAULT_ODOM_TWIST_COVARIANCE_DIAGONAL.begin(),
			DEFAULT_ODOM_TWIST_COVARIANCE_DIAGONAL.end());
	}

	if (!isValidCovarianceMatrix(imu_orientation_covariance_))
	{
		RCLCPP_WARN(
			get_logger(),
			"imu_orientation_covariance must contain exactly 9 finite values. Resetting to defaults.");
		imu_orientation_covariance_ = std::vector<double>(
			DEFAULT_IMU_ORIENTATION_COVARIANCE.begin(),
			DEFAULT_IMU_ORIENTATION_COVARIANCE.end());
	}

	if (!isValidCovarianceMatrix(imu_angular_velocity_covariance_))
	{
		RCLCPP_WARN(
			get_logger(),
			"imu_angular_velocity_covariance must contain exactly 9 finite values. Resetting to defaults.");
		imu_angular_velocity_covariance_ = std::vector<double>(
			DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE.begin(),
			DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE.end());
	}

	if (!isValidCovarianceMatrix(imu_linear_acceleration_covariance_))
	{
		RCLCPP_WARN(
			get_logger(),
			"imu_linear_acceleration_covariance must contain exactly 9 finite values. Resetting to defaults.");
		imu_linear_acceleration_covariance_ = std::vector<double>(
			DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE.begin(),
			DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE.end());
	}

	if (heartbeat_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "heartbeat_interval_ms must be positive. Resetting to 100");
		heartbeat_interval_ms_ = 100;
	}

	if (!is_heartbeat_enabled_)
	{
		RCLCPP_WARN(
			get_logger(),
			"heartbeat_enabled is false. Official TurtleBot3 bringup keeps the OpenCR heartbeat active; motor commands may be ignored without it.");
	}

	if (poll_interval_ms_ <= 0)
	{
		RCLCPP_WARN(
			get_logger(),
			"poll_interval_ms must be positive. Resetting to %d",
			DEFAULT_POLL_INTERVAL_MS);
		poll_interval_ms_ = DEFAULT_POLL_INTERVAL_MS;
	}

	if (debug_odom_interval_ms_ <= 0)
	{
		RCLCPP_WARN(
			get_logger(),
			"debug_odom_interval_ms must be positive. Resetting to 1000");
		debug_odom_interval_ms_ = 1000;
	}

	if (!(command_mode_ == "body_twist" || command_mode_ == "wheel_velocity"))
	{
		RCLCPP_ERROR(
			get_logger(),
			"Invalid command_mode '%s'. Allowed values are 'body_twist' and 'wheel_velocity'.",
			command_mode_.c_str());
		throw std::runtime_error("Invalid robot_base_driver command_mode");
	}

	if (startup_delay_ms_ < 0)
	{
		RCLCPP_WARN(get_logger(), "startup_delay_ms cannot be negative. Resetting to 1000");
		startup_delay_ms_ = 1000;
	}

	if (reconnect_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "reconnect_interval_ms must be positive. Resetting to 1000");
		reconnect_interval_ms_ = 1000;
	}

	if (response_timeout_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "response_timeout_ms must be positive. Resetting to 500");
		response_timeout_ms_ = 500;
	}

	if (transaction_gap_us_ < 0)
	{
		RCLCPP_WARN(get_logger(), "transaction_gap_us cannot be negative. Resetting to 10000");
		transaction_gap_us_ = 10000;
	}

	if (startup_initial_state_read_retries_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "startup_initial_state_read_retries must be positive. Resetting to 5");
		startup_initial_state_read_retries_ = 5;
	}

	if (startup_initial_state_read_retry_interval_ms_ < 0)
	{
		RCLCPP_WARN(
			get_logger(),
			"startup_initial_state_read_retry_interval_ms cannot be negative. Resetting to 200");
		startup_initial_state_read_retry_interval_ms_ = 200;
	}

	if (!(poll_mode_ == "minimal" || poll_mode_ == "full" || poll_mode_ == "odom"))
	{
		RCLCPP_WARN(
			get_logger(),
			"poll_mode must be one of 'minimal', 'full', or 'odom'. Resetting to 'minimal'");
		poll_mode_ = "minimal";
	}

	if ((is_using_imu_for_yaw_ || require_imu_) && poll_mode_ != "full")
	{
		RCLCPP_WARN(
			get_logger(),
			"use_imu_for_yaw/require_imu requires poll_mode='full'. Upgrading poll_mode from '%s' to 'full'.",
			poll_mode_.c_str());
		poll_mode_ = "full";
	}

	if (is_publishing_imu_ && poll_mode_ != "full")
	{
		RCLCPP_WARN(
			get_logger(),
			"publish_imu is enabled while poll_mode='%s'. IMU data is only polled in 'full' mode, so /imu may remain idle until poll_mode is switched to 'full'.",
			poll_mode_.c_str());
	}

	if (max_consecutive_poll_failures_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "max_consecutive_poll_failures must be positive. Resetting to 5");
		max_consecutive_poll_failures_ = 5;
	}

	if (require_device_status_ && !is_polling_device_status_)
	{
		RCLCPP_WARN(
			get_logger(),
			"require_device_status is true while poll_device_status is false. Enabling poll_device_status.");
		is_polling_device_status_ = true;
	}

	if (poll_mode_ == "odom" && (is_polling_device_status_ || require_device_status_))
	{
		RCLCPP_WARN(
			get_logger(),
			"poll_mode='odom' reads wheel feedback only. DEVICE_STATUS and torque-state reads are skipped in this mode even if poll_device_status/require_device_status are enabled.");
	}

	if (!reconnect_on_poll_failure_)
	{
		reopen_serial_on_poll_failure_ = false;
	}

	if (target_odom_rate_hz_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "target_odom_rate_hz must be positive. Resetting to 20.0");
		target_odom_rate_hz_ = 20.0;
	}

	if (!std::isfinite(base_state_throttle_sec_) || base_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.base_state_throttle_sec must be positive. Resetting to 1.0");
		base_state_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(cmd_vel_throttle_sec_) || cmd_vel_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.cmd_vel_throttle_sec must be positive. Resetting to 1.0");
		cmd_vel_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(odom_throttle_sec_) || odom_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.odom_throttle_sec must be positive. Resetting to 1.0");
		odom_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(tf_throttle_sec_) || tf_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.tf_throttle_sec must be positive. Resetting to 1.0");
		tf_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(imu_throttle_sec_) || imu_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.imu_throttle_sec must be positive. Resetting to 1.0");
		imu_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(joint_state_throttle_sec_) || joint_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.joint_state_throttle_sec must be positive. Resetting to 1.0");
		joint_state_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(serial_state_throttle_sec_) || serial_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.serial_state_throttle_sec must be positive. Resetting to 2.0");
		serial_state_throttle_sec_ = 2.0;
	}

	if (!std::isfinite(opencr_state_throttle_sec_) || opencr_state_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.opencr_state_throttle_sec must be positive. Resetting to 1.0");
		opencr_state_throttle_sec_ = 1.0;
	}

	if (!std::isfinite(poll_timing_throttle_sec_) || poll_timing_throttle_sec_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "logging.poll_timing_throttle_sec must be positive. Resetting to 2.0");
		poll_timing_throttle_sec_ = 2.0;
	}
}

void RobotBaseDriverNode::autoConfigureDiagnosticsFromLogLevel()
{
	const int effective_level =
		rcutils_logging_get_logger_effective_level(get_logger().get_name());
	const bool is_debug_log_level = effective_level <= RCUTILS_LOG_SEVERITY_DEBUG;

	debug_odom_auto_enabled_ = false;
	debug_tf_auto_enabled_ = false;

	if (is_debug_log_level && !debug_odom_)
	{
		debug_odom_ = true;
		debug_odom_auto_enabled_ = true;
	}

	if (is_debug_log_level && !debug_tf_)
	{
		debug_tf_ = true;
		debug_tf_auto_enabled_ = true;
	}
}

void RobotBaseDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"Base parameters: port=%s baudrate=%d opencr_id=%d protocol_version=%.1f cmd_vel_topic=%s cmd_vel_stamped_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s wheel_separation=%.3f wheel_radius=%.3f left_encoder_sign=%d right_encoder_sign=%d swap_wheel_encoders=%s command_mode=%s publish_tf=%s use_imu_for_yaw=%s publish_imu=%s publish_joint_states=%s heartbeat_enabled=%s heartbeat_interval_ms=%d poll_interval_ms=%d startup_delay_ms=%d reconnect_on_error=%s reconnect_interval_ms=%d debug_motor_command=%s debug_odom=%s debug_odom_interval_ms=%d debug_tf=%s debug_poll_timing=%s target_odom_rate_hz=%.1f enable_stamped_cmd_vel=%s motor_torque_enable_on_startup=%s motor_torque_enable_requires_ack=%s imu_recalibration_on_startup=%s imu_recalibration_requires_ack=%s profile_acceleration_requires_ack=%s profile_acceleration_on_startup=%s heartbeat_requires_ack=%s startup_require_initial_state_read=%s startup_initial_state_read_retries=%d startup_initial_state_read_retry_interval_ms=%d log_serial_packets=%s log_read_rate=%s response_timeout_ms=%d transaction_gap_us=%d poll_mode=%s max_consecutive_poll_failures=%d poll_device_status=%s require_device_status=%s require_imu=%s reconnect_on_poll_failure=%s reopen_serial_on_poll_failure=%s probe_registers_on_startup=%s",
		port_.c_str(),
		baudrate_,
		opencr_id_,
		protocol_version_,
		cmd_vel_topic_.c_str(),
		cmd_vel_stamped_topic_.c_str(),
		odom_topic_.c_str(),
		imu_topic_.c_str(),
		joint_states_topic_.c_str(),
		odom_frame_id_.c_str(),
		base_frame_id_.c_str(),
		imu_frame_id_.c_str(),
		wheel_separation_m_,
		wheel_radius_m_,
		left_encoder_sign_,
		right_encoder_sign_,
		boolToString(swap_wheel_encoders_),
		command_mode_.c_str(),
		boolToString(is_publish_tf_),
		boolToString(is_using_imu_for_yaw_),
		boolToString(is_publishing_imu_),
		boolToString(is_publishing_joint_states_),
		boolToString(is_heartbeat_enabled_),
		heartbeat_interval_ms_,
		poll_interval_ms_,
		startup_delay_ms_,
		boolToString(is_reconnect_on_error_),
		reconnect_interval_ms_,
		boolToString(debug_motor_command_),
		boolToString(debug_odom_),
		debug_odom_interval_ms_,
		boolToString(debug_tf_),
		boolToString(debug_poll_timing_),
		target_odom_rate_hz_,
		boolToString(is_stamped_cmd_vel_enabled_),
		boolToString(is_motor_torque_enable_on_startup_),
		boolToString(is_motor_torque_enable_ack_required_),
		boolToString(is_imu_recalibration_on_startup_),
		boolToString(is_imu_recalibration_ack_required_),
		boolToString(is_profile_acceleration_ack_required_),
		boolToString(is_profile_acceleration_on_startup_),
		boolToString(is_heartbeat_ack_required_),
		boolToString(is_startup_initial_state_read_required_),
		startup_initial_state_read_retries_,
		startup_initial_state_read_retry_interval_ms_,
		boolToString(is_serial_packet_logging_enabled_),
		boolToString(is_read_rate_logging_enabled_),
		response_timeout_ms_,
		transaction_gap_us_,
		poll_mode_.c_str(),
		max_consecutive_poll_failures_,
		boolToString(is_polling_device_status_),
		boolToString(require_device_status_),
		boolToString(require_imu_),
		boolToString(reconnect_on_poll_failure_),
		boolToString(reopen_serial_on_poll_failure_),
		boolToString(probe_registers_on_startup_));

	RCLCPP_INFO(
		get_logger(),
		"Odom covariance diagonals: pose=[%.6f %.6f %.6f %.6f %.6f %.6f] twist=[%.6f %.6f %.6f %.6f %.6f %.6f]",
		odom_pose_covariance_diagonal_[0],
		odom_pose_covariance_diagonal_[1],
		odom_pose_covariance_diagonal_[2],
		odom_pose_covariance_diagonal_[3],
		odom_pose_covariance_diagonal_[4],
		odom_pose_covariance_diagonal_[5],
		odom_twist_covariance_diagonal_[0],
		odom_twist_covariance_diagonal_[1],
		odom_twist_covariance_diagonal_[2],
		odom_twist_covariance_diagonal_[3],
		odom_twist_covariance_diagonal_[4],
		odom_twist_covariance_diagonal_[5]);

	if (!is_structured_logging_enabled_)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=BASE component=opencr event=base_config node=%s namespace=%s port=%s baudrate=%d opencr_id=%d protocol_version=%.1f cmd_vel_topic=%s cmd_vel_stamped_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s scan_frame_id=%s publish_tf=%s use_imu_for_yaw=%s publish_imu=%s publish_joint_states=%s wheel_separation_m=%.3f wheel_radius_m=%.3f left_encoder_sign=%d right_encoder_sign=%d swap_wheel_encoders=%s command_mode=%s poll_mode=%s target_odom_rate_hz=%.1f structured_enabled=%s result=ok",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		sanitizeLogValue(port_).c_str(),
		baudrate_,
		opencr_id_,
		protocol_version_,
		resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC).c_str(),
		resolveTopicName(cmd_vel_stamped_topic_, DEFAULT_CMD_VEL_STAMPED_TOPIC).c_str(),
		resolveTopicName(odom_topic_, DEFAULT_ODOM_TOPIC).c_str(),
		resolveTopicName(imu_topic_, DEFAULT_IMU_TOPIC).c_str(),
		resolveTopicName(joint_states_topic_, DEFAULT_JOINT_STATES_TOPIC).c_str(),
		resolveFrameId(odom_frame_id_).c_str(),
		resolveFrameId(base_frame_id_).c_str(),
		resolveFrameId(imu_frame_id_).c_str(),
		resolveFrameId(scan_frame_id_).c_str(),
		boolToString(is_publish_tf_),
		boolToString(is_using_imu_for_yaw_),
		boolToString(is_publishing_imu_),
		boolToString(is_publishing_joint_states_),
		wheel_separation_m_,
		wheel_radius_m_,
		left_encoder_sign_,
		right_encoder_sign_,
		boolToString(swap_wheel_encoders_),
		command_mode_.c_str(),
		poll_mode_.c_str(),
		target_odom_rate_hz_,
		boolToString(is_structured_logging_enabled_));
}

void RobotBaseDriverNode::logStartupFrameSanity() const
{
	const char *debug_odom_mode = "disabled_by_default";
	if (debug_odom_auto_enabled_)
	{
		debug_odom_mode = "auto_enabled_from_debug_log_level";
	}
	else if (debug_odom_)
	{
		debug_odom_mode = "explicitly_enabled";
	}

	const char *debug_tf_mode = "disabled_by_default";
	if (debug_tf_auto_enabled_)
	{
		debug_tf_mode = "auto_enabled_from_debug_log_level";
	}
	else if (debug_tf_)
	{
		debug_tf_mode = "explicitly_enabled";
	}

	RCLCPP_INFO(
		get_logger(),
		"Startup frame sanity: odom_frame_id=%s base_frame_id=%s imu_frame_id=%s wheel_left_joint_name=%s wheel_right_joint_name=%s publish_tf=%s command_mode=%s poll_interval_ms=%d wheel_radius=%.3f wheel_separation=%.3f left_encoder_sign=%d right_encoder_sign=%d swap_wheel_encoders=%s debug_odom=%s(%s) debug_tf=%s(%s)",
		resolveFrameId(odom_frame_id_).c_str(),
		resolveFrameId(base_frame_id_).c_str(),
		resolveFrameId(imu_frame_id_).c_str(),
		resolveJointName(wheel_left_joint_name_).c_str(),
		resolveJointName(wheel_right_joint_name_).c_str(),
		boolToString(is_publish_tf_),
		command_mode_.c_str(),
		poll_interval_ms_,
		wheel_radius_m_,
		wheel_separation_m_,
		left_encoder_sign_,
		right_encoder_sign_,
		boolToString(swap_wheel_encoders_),
		boolToString(debug_odom_),
		debug_odom_mode,
		boolToString(debug_tf_),
		debug_tf_mode);
}

void RobotBaseDriverNode::logFrameConfig() const
{
	if (!is_structured_logging_enabled_ || !is_frame_diagnostics_enabled_)
	{
		return;
	}

	std::string expected_base_frame = "base_footprint";
	std::string expected_scan_frame = "base_scan";
	std::string expected_imu_frame = "imu_link";
	const std::string sanitized_namespace = getSanitizedNamespace();
	if (!sanitized_namespace.empty())
	{
		expected_base_frame = sanitized_namespace + "/base_footprint";
		expected_scan_frame = sanitized_namespace + "/base_scan";
		expected_imu_frame = sanitized_namespace + "/imu_link";
	}

	const std::string resolved_odom_frame = resolveFrameId(odom_frame_id_);
	const std::string resolved_base_frame = resolveFrameId(base_frame_id_);
	const std::string resolved_imu_frame = resolveFrameId(imu_frame_id_);
	const std::string resolved_scan_frame = resolveFrameId(scan_frame_id_);
	std::string reason;
	const auto append_reason = [&reason](const char *value) -> void
	{
		if (!reason.empty())
		{
			reason += ",";
		}
		reason += value;
	};

	if (!is_publish_tf_)
	{
		append_reason("publish_tf_false");
	}
	if (odom_frame_id_.empty())
	{
		append_reason("odom_frame_empty");
	}
	if (base_frame_id_.empty())
	{
		append_reason("base_frame_empty");
	}
	if (!odom_frame_id_.empty() && !base_frame_id_.empty() && resolved_odom_frame == resolved_base_frame)
	{
		append_reason("odom_frame_equals_base_frame");
	}
	if (resolved_base_frame != expected_base_frame)
	{
		append_reason("base_frame_mismatch");
	}
	if (is_publishing_imu_ && imu_frame_id_.empty())
	{
		append_reason("imu_frame_empty");
	}
	if (is_publishing_imu_ && !imu_frame_id_.empty() && resolved_imu_frame != expected_imu_frame)
	{
		append_reason("imu_frame_mismatch");
	}
	if (resolved_scan_frame != expected_scan_frame)
	{
		append_reason("scan_frame_mismatch");
	}
	if (!sanitized_namespace.empty() && base_frame_id_.find('/') != std::string::npos && base_frame_id_.rfind(sanitized_namespace + "/", 0U) != 0U)
	{
		append_reason("namespace_frame_mismatch");
	}

	if (reason.empty())
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=frame_config node=%s namespace=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s scan_frame_id=%s publish_tf=%s use_imu_for_yaw=%s robot_description_expected=%s result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolved_odom_frame.c_str(),
			resolved_base_frame.c_str(),
			resolved_imu_frame.c_str(),
			resolved_scan_frame.c_str(),
			boolToString(is_publish_tf_),
			boolToString(is_using_imu_for_yaw_),
			expected_base_frame.c_str());
	}
	else
	{
		RCLCPP_WARN(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=frame_config node=%s namespace=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s scan_frame_id=%s publish_tf=%s use_imu_for_yaw=%s robot_description_expected=%s result=warn reason=%s",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolved_odom_frame.c_str(),
			resolved_base_frame.c_str(),
			resolved_imu_frame.c_str(),
			resolved_scan_frame.c_str(),
			boolToString(is_publish_tf_),
			boolToString(is_using_imu_for_yaw_),
			expected_base_frame.c_str(),
			reason.c_str());
	}

	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=tf_chain_expected node=%s namespace=%s map_to_odom=external_localization odom_to_base_footprint=robot_base_driver base_footprint_to_base_link=robot_state_publisher base_link_to_base_scan=robot_state_publisher base_link_to_imu_link=robot_state_publisher yaw_source=%s robot_hardware_publishes_map_to_odom=false result=configured",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		is_using_imu_for_yaw_ ? "imu" : "wheel_odom");
}

void RobotBaseDriverNode::logTopicConfig() const
{
	if (!is_structured_logging_enabled_ || !is_topic_diagnostics_enabled_)
	{
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=DIAG component=bringup event=topic_config node=%s namespace=%s cmd_vel_topic=%s cmd_vel_stamped_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s tf_topic=/tf tf_static_topic=/tf_static publish_imu=%s publish_joint_states=%s enable_stamped_cmd_vel=%s result=ok",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC).c_str(),
		resolveTopicName(cmd_vel_stamped_topic_, DEFAULT_CMD_VEL_STAMPED_TOPIC).c_str(),
		resolveTopicName(odom_topic_, DEFAULT_ODOM_TOPIC).c_str(),
		resolveTopicName(imu_topic_, DEFAULT_IMU_TOPIC).c_str(),
		resolveTopicName(joint_states_topic_, DEFAULT_JOINT_STATES_TOPIC).c_str(),
		boolToString(is_publishing_imu_),
		boolToString(is_publishing_joint_states_),
		boolToString(is_stamped_cmd_vel_enabled_));
}

void RobotBaseDriverNode::setupPublishers()
{
	const std::string resolved_odom_topic = resolveTopicName(odom_topic_, DEFAULT_ODOM_TOPIC);
	odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
		resolved_odom_topic,
		rclcpp::SystemDefaultsQoS());
	RCLCPP_INFO(
		get_logger(),
		"Publishing odom topic: topic=%s type=nav_msgs/msg/Odometry",
		resolved_odom_topic.c_str());

	if (is_publishing_imu_)
	{
		const std::string resolved_imu_topic = resolveTopicName(imu_topic_, DEFAULT_IMU_TOPIC);
		imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
			resolved_imu_topic,
			rclcpp::SensorDataQoS());
		RCLCPP_INFO(
			get_logger(),
			"Publishing imu topic: topic=%s type=sensor_msgs/msg/Imu frame_id=%s qos=sensor_data",
			resolved_imu_topic.c_str(),
			resolveFrameId(imu_frame_id_).c_str());
	}
	else
	{
		RCLCPP_WARN(get_logger(), "IMU publishing is disabled by parameter publish_imu=false");
	}

	if (is_publishing_joint_states_)
	{
		const std::string resolved_joint_states_topic =
			resolveTopicName(joint_states_topic_, DEFAULT_JOINT_STATES_TOPIC);
		joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
			resolved_joint_states_topic,
			rclcpp::SystemDefaultsQoS());
		RCLCPP_INFO(
			get_logger(),
			"Publishing joint_states topic: topic=%s type=sensor_msgs/msg/JointState",
			resolved_joint_states_topic.c_str());
	}

	if (is_publish_tf_)
	{
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	}
}

void RobotBaseDriverNode::setupSubscriptions()
{
	rclcpp::QoS cmd_vel_qos(rclcpp::KeepLast(10));
	cmd_vel_qos.reliable();

	const std::string resolved_cmd_vel_topic = resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC);
	if (is_stamped_cmd_vel_enabled_)
	{
		const std::string resolved_cmd_vel_stamped_topic =
			resolveTopicName(cmd_vel_stamped_topic_, DEFAULT_CMD_VEL_STAMPED_TOPIC);
		if (resolved_cmd_vel_stamped_topic == resolved_cmd_vel_topic)
		{
			RCLCPP_WARN(
				get_logger(),
				"Stamped cmd_vel topic matches Twist cmd_vel topic (%s). This can confuse ROS topic type discovery; prefer a separate stamped topic name.",
				resolved_cmd_vel_topic.c_str());
		}
		cmd_vel_stamped_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
			resolved_cmd_vel_stamped_topic,
			cmd_vel_qos,
			[this](const geometry_msgs::msg::TwistStamped::SharedPtr message) -> void
			{
				handleStampedVelocityCommand(*message);
			});

		RCLCPP_INFO(
			get_logger(),
			"Subscribed to optional stamped cmd_vel topic: topic=%s type=geometry_msgs/msg/TwistStamped qos=reliable depth=10",
			resolved_cmd_vel_stamped_topic.c_str());
	}

	cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
		resolved_cmd_vel_topic,
		cmd_vel_qos,
		[this](const geometry_msgs::msg::Twist::SharedPtr message) -> void
		{
			handleVelocityCommand(*message);
		});

	RCLCPP_INFO(
		get_logger(),
		"Subscribed to cmd_vel topic: topic=%s type=geometry_msgs/msg/Twist qos=reliable depth=10",
		resolved_cmd_vel_topic.c_str());
}

void RobotBaseDriverNode::setupServices()
{
	reset_odometry_service_ = create_service<std_srvs::srv::Trigger>(
		"reset_odometry",
		[this](
			const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
			std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
		{
			handleResetOdometry(request, response);
		});
}

void RobotBaseDriverNode::setupOdometryIntegrator()
{
	odometry_integrator_ = std::make_shared<OdometryIntegrator>(
		wheel_separation_m_,
		wheel_radius_m_,
		is_using_imu_for_yaw_,
		left_encoder_sign_,
		right_encoder_sign_,
		swap_wheel_encoders_);
}

void RobotBaseDriverNode::startDriver()
{
	RCLCPP_INFO(
		get_logger(),
		"No cmd_vel timeout_stop watchdog is configured in robot_base_driver. Motor commands are sent only from source=cmd_vel and optional source=shutdown_stop is not implemented.");
	if (!startRealMode())
	{
		scheduleReconnect("Initial OpenCR bringup failed");
	}
}

bool RobotBaseDriverNode::startRealMode()
{
	stopDriver();

	serial_port_ = std::make_shared<SerialPort>(get_logger());
	if (!serial_port_->openPort(port_, baudrate_))
	{
		RCLCPP_ERROR(get_logger(), "Serial open failed for %s", port_.c_str());
		if (is_structured_logging_enabled_)
		{
			RCLCPP_ERROR(
				get_logger(),
				"ROBOT_HW_LOG schema=v1 tag=SERIAL component=opencr event=serial_state node=%s namespace=%s port=%s baudrate=%d result=failed reason=open_failed",
				get_name(),
				sanitizeLogValue(get_namespace()).c_str(),
				sanitizeLogValue(port_).c_str(),
				baudrate_);
		}
		return false;
	}
	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=SERIAL component=opencr event=serial_state node=%s namespace=%s port=%s baudrate=%d result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(port_).c_str(),
			baudrate_);
	}

	OpencrClientConfig config;
	config.opencr_id = static_cast<uint8_t>(opencr_id_);
	config.port = port_;
	config.baudrate = baudrate_;
	config.protocol_version = protocol_version_;
	config.response_timeout_ms = response_timeout_ms_;
	config.transaction_gap_us = transaction_gap_us_;
	config.startup_delay_ms = startup_delay_ms_;
	config.poll_interval_ms = poll_interval_ms_;
	config.heartbeat_interval_ms = heartbeat_interval_ms_;
	config.is_heartbeat_enabled = is_heartbeat_enabled_;
	config.is_motor_torque_enable_on_startup = is_motor_torque_enable_on_startup_;
	config.is_motor_torque_enable_ack_required = is_motor_torque_enable_ack_required_;
	config.is_imu_recalibration_on_startup = is_imu_recalibration_on_startup_;
	config.is_imu_recalibration_ack_required = is_imu_recalibration_ack_required_;
	config.is_profile_acceleration_ack_required = is_profile_acceleration_ack_required_;
	config.is_profile_acceleration_on_startup = is_profile_acceleration_on_startup_;
	config.is_heartbeat_ack_required = is_heartbeat_ack_required_;
	config.is_startup_initial_state_read_required = is_startup_initial_state_read_required_;
	config.startup_initial_state_read_retries = startup_initial_state_read_retries_;
	config.startup_initial_state_read_retry_interval_ms = startup_initial_state_read_retry_interval_ms_;
	config.is_serial_packet_logging_enabled = is_serial_packet_logging_enabled_;
	config.is_read_rate_logging_enabled = is_read_rate_logging_enabled_;
	config.is_structured_logging_enabled = is_structured_logging_enabled_;
	config.serial_state_throttle_sec = serial_state_throttle_sec_;
	config.opencr_state_throttle_sec = opencr_state_throttle_sec_;
	config.poll_timing_throttle_sec = poll_timing_throttle_sec_;
	config.debug_motor_command = debug_motor_command_;
	config.debug_poll_timing = debug_poll_timing_;
	config.wheel_separation_m = wheel_separation_m_;
	config.wheel_radius_m = wheel_radius_m_;
	config.profile_acceleration_constant = profile_acceleration_constant_;
	config.profile_acceleration = profile_acceleration_;
	config.target_odom_rate_hz = target_odom_rate_hz_;
	config.command_mode = command_mode_ == "wheel_velocity"
		? OpencrCommandMode::WheelVelocity
		: OpencrCommandMode::BodyTwist;
	config.poll_mode = OpencrPollMode::Minimal;
	if (poll_mode_ == "full")
	{
		config.poll_mode = OpencrPollMode::Full;
	}
	else if (poll_mode_ == "odom")
	{
		config.poll_mode = OpencrPollMode::Odom;
	}
	config.max_consecutive_poll_failures = max_consecutive_poll_failures_;
	config.poll_device_status = is_polling_device_status_;
	config.require_device_status = require_device_status_;
	config.require_imu = require_imu_;
	config.reconnect_on_poll_failure = reconnect_on_poll_failure_;
	config.reopen_serial_on_poll_failure = reopen_serial_on_poll_failure_;
	config.probe_registers_on_startup = probe_registers_on_startup_;

	opencr_client_ = std::make_shared<OpencrClient>(get_logger(), serial_port_.get(), config);
	bool started = opencr_client_->start(
		[this](const OpencrState &state) -> void
		{
			handleOpencrState(state);
		},
		[this]() -> void
		{
			handleClientConnected();
		},
		[this](const std::string &reason) -> void
		{
			handleClientError(reason);
		});

	if (!started)
	{
		opencr_client_.reset();
		serial_port_->closePort();
		return false;
	}

	return true;
}

void RobotBaseDriverNode::stopDriver()
{
	if (opencr_client_)
	{
		opencr_client_->stop();
		opencr_client_.reset();
	}

	if (serial_port_)
	{
		serial_port_->closePort();
		serial_port_.reset();
	}
}

void RobotBaseDriverNode::scheduleReconnect(const std::string &reason)
{
	if (!is_reconnect_on_error_ || is_shutdown_requested_.load())
	{
		return;
	}

	if (is_reconnecting_.exchange(true))
	{
		return;
	}

	RCLCPP_WARN(get_logger(), "Scheduling OpenCR reconnect: %s", reason.c_str());
	reconnect_timer_ = create_wall_timer(
		std::chrono::milliseconds(reconnect_interval_ms_),
		[this]() -> void
		{
			attemptReconnect();
		});
}

void RobotBaseDriverNode::cancelReconnect()
{
	if (reconnect_timer_)
	{
		reconnect_timer_->cancel();
		reconnect_timer_.reset();
	}

	is_reconnecting_.store(false);
}

void RobotBaseDriverNode::attemptReconnect()
{
	RCLCPP_INFO(get_logger(), "Attempting OpenCR reconnect on %s", port_.c_str());
	if (!startRealMode())
	{
		RCLCPP_WARN(get_logger(), "OpenCR reconnect attempt failed");
		return;
	}

	RCLCPP_INFO(get_logger(), "OpenCR reconnect succeeded");
	cancelReconnect();
}

void RobotBaseDriverNode::handleVelocityCommand(const geometry_msgs::msg::Twist &message)
{
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		has_recent_cmd_vel_ = true;
		last_cmd_vel_linear_x_ = message.linear.x;
		last_cmd_vel_angular_z_ = message.angular.z;
	}

	if (!opencr_client_ || !opencr_client_->isRunning())
	{
		logCommandInput(message, "cmd_vel_rejected", 0.0, "rejected", "opencr_not_running");
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"Ignoring cmd_vel because OpenCR client is not running: linear.x=%.3f angular.z=%.3f",
			message.linear.x,
			message.angular.z);
		return;
	}

	const double expected_left_wheel_linear_mps =
		message.linear.x - (message.angular.z * wheel_separation_m_ * 0.5);
	const double expected_right_wheel_linear_mps =
		message.linear.x + (message.angular.z * wheel_separation_m_ * 0.5);
	const double expected_left_wheel_radps = expected_left_wheel_linear_mps / wheel_radius_m_;
	const double expected_right_wheel_radps = expected_right_wheel_linear_mps / wheel_radius_m_;
	const double velocity_constant = 1263.632956882;
	const int expected_left_goal_velocity = static_cast<int>(
		std::clamp(expected_left_wheel_linear_mps * velocity_constant, -337.0, 337.0));
	const int expected_right_goal_velocity = static_cast<int>(
		std::clamp(expected_right_wheel_linear_mps * velocity_constant, -337.0, 337.0));

	if (debug_motor_command_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			1000,
			"cmd_vel callback entered: source=cmd_vel topic=%s command_mode=%s linear.x=%.3f angular.z=%.3f expected_left_wheel_mps=%.3f expected_right_wheel_mps=%.3f expected_left_wheel_radps=%.3f expected_right_wheel_radps=%.3f expected_left_goal_velocity=%d expected_right_goal_velocity=%d",
			resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC).c_str(),
			command_mode_.c_str(),
			message.linear.x,
			message.angular.z,
			expected_left_wheel_linear_mps,
			expected_right_wheel_linear_mps,
			expected_left_wheel_radps,
			expected_right_wheel_radps,
			expected_left_goal_velocity,
			expected_right_goal_velocity);

		if (std::abs(expected_left_goal_velocity) < 3 && std::abs(expected_right_goal_velocity) < 3 &&
			(std::abs(message.linear.x) > 0.0 || std::abs(message.angular.z) > 0.0))
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				throttle_clock_,
				2000,
				"cmd_vel converted to very small expected wheel goal velocities: left=%d right=%d. In body_twist mode these are diagnostic only. Use >=0.05 m/s or >=0.5 rad/s for deadband testing.",
				expected_left_goal_velocity,
				expected_right_goal_velocity);
		}
	}

	if (has_seen_device_status_ && last_device_status_ == -1)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"cmd_vel received while OpenCR device_status=-1. Command will still be sent, but motor power/torque may be disabled.");
	}

	if (has_seen_motor_torque_enabled_ && !last_motor_torque_enabled_)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"cmd_vel command sent but motor not ready: device_status=%d torque_enable=0 source=cmd_vel",
			has_seen_device_status_ ? static_cast<int>(last_device_status_) : 0);
	}

	VelocityCommand command;
	command.linear_x_mps = message.linear.x;
	command.angular_z_rps = message.angular.z;
	command.source = "cmd_vel";
	opencr_client_->setVelocityCommand(command);
	logCommandInput(message, "cmd_vel_received", 0.0, "accepted", "none");
	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(cmd_vel_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=CMD component=opencr event=cmd_vel_write node=%s namespace=%s linear_x=%.3f angular_z=%.3f command_mode=%s wheel_left_target=%d wheel_right_target=%d last_cmd_age_sec=%.3f throttle_sec=%.3f result=queued",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			message.linear.x,
			message.angular.z,
			command_mode_.c_str(),
			expected_left_goal_velocity,
			expected_right_goal_velocity,
			0.0,
			cmd_vel_throttle_sec_);
	}

	RCLCPP_DEBUG(
		get_logger(),
		"Queued cmd_vel for OpenCR write: linear.x=%.3f angular.z=%.3f",
		command.linear_x_mps,
		command.angular_z_rps);
}

void RobotBaseDriverNode::handleStampedVelocityCommand(const geometry_msgs::msg::TwistStamped &message)
{
	const double age_sec = (now() - message.header.stamp).seconds();
	logCommandInput(message.twist, "cmd_vel_stamped_received", age_sec, "accepted", "none");
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		1000,
		"TwistStamped cmd_vel received: topic=%s stamp=%u.%u",
		resolveTopicName(cmd_vel_stamped_topic_, DEFAULT_CMD_VEL_STAMPED_TOPIC).c_str(),
		static_cast<unsigned int>(message.header.stamp.sec),
		static_cast<unsigned int>(message.header.stamp.nanosec));
	handleVelocityCommand(message.twist);
}

void RobotBaseDriverNode::handleOpencrState(const OpencrState &state)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	rclcpp::Time stamp = now();
	if (state.has_device_status)
	{
		last_device_status_ = state.device_status;
		has_seen_device_status_ = true;
	}
	else
	{
		has_seen_device_status_ = false;
	}

	if (state.has_motor_torque_enable)
	{
		last_motor_torque_enabled_ = state.motor_torque_enabled;
		has_seen_motor_torque_enabled_ = true;
	}
	else
	{
		has_seen_motor_torque_enabled_ = false;
	}

	if (state.has_device_status && state.device_status != 0 && state.device_status != -1)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"OpenCR reported non-zero device_status=%d",
			static_cast<int>(state.device_status));
	}

	if (state.has_device_status && state.device_status == -1)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"OpenCR reported device_status=-1. OpenCR communication is up, but motor power/torque is not ready or the motor driver is faulted.");
	}
	else if (!state.has_device_status)
	{
		RCLCPP_DEBUG_THROTTLE(
			get_logger(),
			throttle_clock_,
			5000,
			"DEVICE_STATUS polling is disabled, so motor power fault detection is unavailable.");
	}

	if (state.has_motor_torque_enable && !state.motor_torque_enabled)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"OpenCR reports motor_torque_enable=0. Command packets may be acknowledged while the motors remain disabled.");
	}

	if (is_structured_logging_enabled_)
	{
		const char *state_result = "ok";
		const char *state_reason = "none";
		if (state.has_device_status && state.device_status != 0)
		{
			state_result = "warn";
			state_reason = "device_status_nonzero";
		}
		else if (state.has_motor_torque_enable && !state.motor_torque_enabled)
		{
			state_result = "warn";
			state_reason = "motor_torque_disabled";
		}

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(opencr_state_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=BASE component=opencr event=base_state node=%s namespace=%s device_status=%d has_device_status=%s motor_torque_enabled=%s has_motor_torque=%s poll_mode=%s target_odom_rate_hz=%.3f throttle_sec=%.3f result=%s reason=%s",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			state.has_device_status ? static_cast<int>(state.device_status) : 0,
			boolToString(state.has_device_status),
			boolToString(state.motor_torque_enabled),
			boolToString(state.has_motor_torque_enable),
			poll_mode_.c_str(),
			target_odom_rate_hz_,
			opencr_state_throttle_sec_,
			state_result,
			state_reason);
	}

	bool updated = odometry_integrator_->update(
		state.present_position_left,
		state.present_position_right,
		state.present_velocity_left,
		state.present_velocity_right,
		state.has_imu_data,
		state.imu_orientation_w,
		state.imu_orientation_x,
		state.imu_orientation_y,
		state.imu_orientation_z,
		stamp);

	if (is_publishing_imu_ && state.has_imu_data)
	{
		publishImu(state, stamp);
	}
	else if (
		is_publishing_imu_ &&
		!state.has_imu_data &&
		(poll_mode_ == "full" || require_imu_ || is_using_imu_for_yaw_))
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"IMU publisher is enabled but no valid IMU data is available from OpenCR. Check poll_mode, OpenCR IMU registers, and firmware state.");
	}

	if (is_publishing_joint_states_)
	{
		publishJointStates(state, stamp);
	}

	if (updated)
	{
		publishOdometry(stamp);
		if (!has_logged_publish_success_)
		{
			RCLCPP_INFO(get_logger(), "OpenCR data pipeline active: imu/joint_states/odom publishing");
			has_logged_publish_success_ = true;
		}
	}
}

void RobotBaseDriverNode::publishImu(const OpencrState &state, const rclcpp::Time &stamp)
{
	if (!imu_publisher_)
	{
		return;
	}

	sensor_msgs::msg::Imu message;
	message.header.stamp = stamp;
	message.header.frame_id = resolveFrameId(imu_frame_id_);
	message.orientation.w = state.imu_orientation_w;
	message.orientation.x = state.imu_orientation_x;
	message.orientation.y = state.imu_orientation_y;
	message.orientation.z = state.imu_orientation_z;
	message.angular_velocity.x = state.imu_angular_velocity_x;
	message.angular_velocity.y = state.imu_angular_velocity_y;
	message.angular_velocity.z = state.imu_angular_velocity_z;
	message.linear_acceleration.x = state.imu_linear_acceleration_x;
	message.linear_acceleration.y = state.imu_linear_acceleration_y;
	message.linear_acceleration.z = state.imu_linear_acceleration_z;
	applyCovarianceMatrix(imu_orientation_covariance_, message.orientation_covariance);
	applyCovarianceMatrix(imu_angular_velocity_covariance_, message.angular_velocity_covariance);
	applyCovarianceMatrix(imu_linear_acceleration_covariance_, message.linear_acceleration_covariance);
	imu_publisher_->publish(message);

	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(imu_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=IMU component=opencr event=imu_publish node=%s namespace=%s topic=%s frame_id=%s orientation_yaw_rad=%.6f angular_velocity_z=%.6f linear_acceleration_x=%.6f linear_acceleration_y=%.6f linear_acceleration_z=%.6f source=opencr publish_rate_hz=%.3f throttle_sec=%.3f result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolveTopicName(imu_topic_, DEFAULT_IMU_TOPIC).c_str(),
			message.header.frame_id.c_str(),
			quaternionToYaw(message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z),
			message.angular_velocity.z,
			message.linear_acceleration.x,
			message.linear_acceleration.y,
			message.linear_acceleration.z,
			target_odom_rate_hz_,
			imu_throttle_sec_);
	}
}

void RobotBaseDriverNode::publishJointStates(const OpencrState &state, const rclcpp::Time &stamp)
{
	(void)state;

	if (!joint_state_publisher_)
	{
		return;
	}

	sensor_msgs::msg::JointState message;
	message.header.stamp = stamp;
	message.header.frame_id = resolveFrameId(base_frame_id_);
	message.name.push_back(resolveJointName(wheel_left_joint_name_));
	message.name.push_back(resolveJointName(wheel_right_joint_name_));

	std::array<double, 2> positions = odometry_integrator_->getJointPositionsRad();
	std::array<double, 2> velocities = odometry_integrator_->getJointVelocitiesRadps();
	message.position.push_back(positions[0]);
	message.position.push_back(positions[1]);
	message.velocity.push_back(velocities[0]);
	message.velocity.push_back(velocities[1]);
	joint_state_publisher_->publish(message);

	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(joint_state_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=JOINT component=opencr event=joint_state_publish node=%s namespace=%s topic=%s left_joint=%s right_joint=%s left_position=%.6f right_position=%.6f left_velocity=%.6f right_velocity=%.6f publish_rate_hz=%.3f throttle_sec=%.3f result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolveTopicName(joint_states_topic_, DEFAULT_JOINT_STATES_TOPIC).c_str(),
			message.name[0].c_str(),
			message.name[1].c_str(),
			message.position[0],
			message.position[1],
			message.velocity[0],
			message.velocity[1],
			target_odom_rate_hz_,
			joint_state_throttle_sec_);
	}
}

void RobotBaseDriverNode::publishOdometry(const rclcpp::Time &stamp)
{
	nav_msgs::msg::Odometry message = odometry_integrator_->buildOdometryMessage(
		stamp,
		resolveFrameId(odom_frame_id_),
		resolveFrameId(base_frame_id_));
	applyCovarianceDiagonal(odom_pose_covariance_diagonal_, message.pose.covariance);
	applyCovarianceDiagonal(odom_twist_covariance_diagonal_, message.twist.covariance);
	odom_publisher_->publish(message);

	const OdometryDebugSnapshot snapshot = odometry_integrator_->getDebugSnapshot();
	const double odom_yaw = quaternionToYaw(
		message.pose.pose.orientation.w,
		message.pose.pose.orientation.x,
		message.pose.pose.orientation.y,
		message.pose.pose.orientation.z);
	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(odom_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=ODOM component=opencr event=odom_publish node=%s namespace=%s topic=%s frame_id=%s child_frame_id=%s x=%.6f y=%.6f yaw_rad=%.6f vx=%.6f wz=%.6f left_wheel_position=%.6f right_wheel_position=%.6f left_wheel_velocity=%.6f right_wheel_velocity=%.6f source=%s publish_rate_hz=%.3f throttle_sec=%.3f result=ok",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			resolveTopicName(odom_topic_, DEFAULT_ODOM_TOPIC).c_str(),
			message.header.frame_id.c_str(),
			message.child_frame_id.c_str(),
			message.pose.pose.position.x,
			message.pose.pose.position.y,
			odom_yaw,
			message.twist.twist.linear.x,
			message.twist.twist.angular.z,
			snapshot.left_delta_rad,
			snapshot.right_delta_rad,
			snapshot.left_velocity_radps,
			snapshot.right_velocity_radps,
			is_using_imu_for_yaw_ ? "imu" : "wheel_odom",
			target_odom_rate_hz_,
			odom_throttle_sec_);
	}

	if (debug_odom_)
	{
		logOdomDiagnostics(snapshot, message);
	}

	if (is_publish_tf_ && tf_broadcaster_)
	{
		geometry_msgs::msg::TransformStamped transform = odometry_integrator_->buildTransformMessage(
			stamp,
			resolveFrameId(odom_frame_id_),
			resolveFrameId(base_frame_id_));
		tf_broadcaster_->sendTransform(transform);

		if (debug_tf_ || is_structured_logging_enabled_)
		{
			logTfDiagnostics(transform, snapshot);
		}
	}
}

void RobotBaseDriverNode::logOdomDiagnostics(
	const OdometryDebugSnapshot &snapshot,
	const nav_msgs::msg::Odometry &message) const
{
	const double odom_quaternion_yaw = quaternionToYaw(
		message.pose.pose.orientation.w,
		message.pose.pose.orientation.x,
		message.pose.pose.orientation.y,
		message.pose.pose.orientation.z);

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		debug_odom_interval_ms_,
		"odom debug: raw_left_pos=%d raw_right_pos=%d raw_left_vel=%d raw_right_vel=%d adjusted_left_pos=%d adjusted_right_pos=%d adjusted_left_vel=%d adjusted_right_vel=%d left_tick_delta=%d right_tick_delta=%d left_delta_rad=%.6f right_delta_rad=%.6f left_delta_m=%.6f right_delta_m=%.6f dt=%.6f delta_s=%.6f delta_theta=%.6f v=%.6f w=%.6f x=%.6f y=%.6f yaw=%.6f odom_quat_yaw=%.6f left_joint_radps=%.6f right_joint_radps=%.6f",
		snapshot.raw_left_ticks,
		snapshot.raw_right_ticks,
		snapshot.raw_left_velocity,
		snapshot.raw_right_velocity,
		snapshot.adjusted_left_ticks,
		snapshot.adjusted_right_ticks,
		snapshot.adjusted_left_velocity,
		snapshot.adjusted_right_velocity,
		snapshot.left_tick_delta,
		snapshot.right_tick_delta,
		snapshot.left_delta_rad,
		snapshot.right_delta_rad,
		snapshot.left_delta_m,
		snapshot.right_delta_m,
		snapshot.delta_time,
		snapshot.delta_s,
		snapshot.delta_theta,
		snapshot.linear_x,
		snapshot.angular_z,
		snapshot.x,
		snapshot.y,
		snapshot.yaw,
		odom_quaternion_yaw,
		snapshot.left_velocity_radps,
		snapshot.right_velocity_radps);

	if (!has_recent_cmd_vel_)
	{
		return;
	}

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		debug_odom_interval_ms_,
		"odom sign check: cmd_linear_x=%.3f cmd_angular_z=%.3f expected_motion=%s encoder_signs(left=%s,right=%s) odom_signs(delta_s=%s,delta_theta=%s)",
		last_cmd_vel_linear_x_,
		last_cmd_vel_angular_z_,
		describeExpectedMotionType(last_cmd_vel_linear_x_, last_cmd_vel_angular_z_).c_str(),
		describeSign(static_cast<double>(snapshot.left_tick_delta)),
		describeSign(static_cast<double>(snapshot.right_tick_delta)),
		describeSign(snapshot.delta_s),
		describeSign(snapshot.delta_theta));

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		debug_odom_interval_ms_,
		"odom cmd compare: cmd_linear_x=%.3f cmd_angular_z=%.3f odom_linear_x=%.3f odom_angular_z=%.3f",
		last_cmd_vel_linear_x_,
		last_cmd_vel_angular_z_,
		message.twist.twist.linear.x,
		message.twist.twist.angular.z);
}

void RobotBaseDriverNode::logTfDiagnostics(
	const geometry_msgs::msg::TransformStamped &transform,
	const OdometryDebugSnapshot &snapshot) const
{
	const double tf_quaternion_yaw = quaternionToYaw(
		transform.transform.rotation.w,
		transform.transform.rotation.x,
		transform.transform.rotation.y,
		transform.transform.rotation.z);

	if (is_structured_logging_enabled_)
	{
		RCLCPP_INFO_THROTTLE(
			get_logger(),
			throttle_clock_,
			secondsToMilliseconds(tf_throttle_sec_, 1000),
			"ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=tf_publish node=%s namespace=%s parent_frame=%s child_frame=%s x=%.6f y=%.6f z=%.6f roll_rad=%.6f pitch_rad=%.6f yaw_rad=%.6f source=%s publish_tf=%s stamp_age_sec=%.6f publish_rate_hz=%.3f throttle_sec=%.3f result=published",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			transform.header.frame_id.c_str(),
			transform.child_frame_id.c_str(),
			transform.transform.translation.x,
			transform.transform.translation.y,
			transform.transform.translation.z,
			0.0,
			0.0,
			tf_quaternion_yaw,
			is_using_imu_for_yaw_ ? "imu" : "wheel_odom",
			boolToString(is_publish_tf_),
			(now() - transform.header.stamp).seconds(),
			target_odom_rate_hz_,
			tf_throttle_sec_);

		if (!debug_tf_)
		{
			return;
		}
	}

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		debug_odom_interval_ms_,
		"tf debug: parent=%s child=%s x=%.6f y=%.6f qx=%.6f qy=%.6f qz=%.6f qw=%.6f yaw=%.6f integrated_yaw=%.6f stamp=%u.%09u",
		transform.header.frame_id.c_str(),
		transform.child_frame_id.c_str(),
		transform.transform.translation.x,
		transform.transform.translation.y,
		transform.transform.rotation.x,
		transform.transform.rotation.y,
		transform.transform.rotation.z,
		transform.transform.rotation.w,
		tf_quaternion_yaw,
		snapshot.yaw,
		transform.header.stamp.sec,
		transform.header.stamp.nanosec);
}

void RobotBaseDriverNode::logCommandInput(
	const geometry_msgs::msg::Twist &message,
	const char *event,
	double age_sec,
	const char *result,
	const char *reason) const
{
	if (!is_structured_logging_enabled_)
	{
		return;
	}

	const double expected_left_wheel_linear_mps =
		message.linear.x - (message.angular.z * wheel_separation_m_ * 0.5);
	const double expected_right_wheel_linear_mps =
		message.linear.x + (message.angular.z * wheel_separation_m_ * 0.5);
	const double velocity_constant = 1263.632956882;
	const int expected_left_goal_velocity = static_cast<int>(
		std::clamp(expected_left_wheel_linear_mps * velocity_constant, -337.0, 337.0));
	const int expected_right_goal_velocity = static_cast<int>(
		std::clamp(expected_right_wheel_linear_mps * velocity_constant, -337.0, 337.0));

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		throttle_clock_,
		secondsToMilliseconds(cmd_vel_throttle_sec_, 1000),
		"ROBOT_HW_LOG schema=v1 tag=CMD component=opencr event=%s node=%s namespace=%s linear_x=%.3f angular_z=%.3f command_mode=%s wheel_left_target=%d wheel_right_target=%d last_cmd_age_sec=%.3f throttle_sec=%.3f result=%s reason=%s",
		event,
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		message.linear.x,
		message.angular.z,
		command_mode_.c_str(),
		expected_left_goal_velocity,
		expected_right_goal_velocity,
		age_sec,
		cmd_vel_throttle_sec_,
		result,
		sanitizeLogValue(reason).c_str());
}

void RobotBaseDriverNode::handleClientConnected()
{
	RCLCPP_INFO(get_logger(), "OpenCR client started successfully");
	cancelReconnect();
}

void RobotBaseDriverNode::handleClientError(const std::string &reason)
{
	if (is_shutdown_requested_.load())
	{
		return;
	}

	if (reason.rfind("poll_failure:", 0) == 0)
	{
		RCLCPP_WARN(get_logger(), "OpenCR poll failure callback: %s", reason.c_str());
		if (!reconnect_on_poll_failure_)
		{
			return;
		}

		if (!reopen_serial_on_poll_failure_)
		{
			RCLCPP_WARN(get_logger(), "Poll failure reconnect requested but reopen_serial_on_poll_failure is disabled");
			return;
		}
	}

	RCLCPP_ERROR(get_logger(), "OpenCR client error: %s", reason.c_str());
	scheduleReconnect(reason);
}

void RobotBaseDriverNode::handleResetOdometry(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;
	std::lock_guard<std::mutex> lock(state_mutex_);
	odometry_integrator_->reset();
	response->success = true;
	response->message = "Odometry reset";
}

std::string RobotBaseDriverNode::resolveTopicName(const std::string &configured_topic, const char *default_topic) const
{
	std::string sanitized_namespace = getSanitizedNamespace();
	if (sanitized_namespace.empty())
	{
		return configured_topic;
	}

	if (configured_topic == default_topic && !configured_topic.empty() && configured_topic.front() == '/')
	{
		return configured_topic.substr(1);
	}

	return configured_topic;
}

std::string RobotBaseDriverNode::resolveFrameId(const std::string &configured_frame_id) const
{
	std::string sanitized_namespace = getSanitizedNamespace();
	if (sanitized_namespace.empty())
	{
		return configured_frame_id;
	}

	if (configured_frame_id.find('/') != std::string::npos)
	{
		return configured_frame_id;
	}

	return sanitized_namespace + "/" + configured_frame_id;
}

std::string RobotBaseDriverNode::resolveJointName(const std::string &configured_joint_name) const
{
	std::string sanitized_namespace = getSanitizedNamespace();
	if (sanitized_namespace.empty())
	{
		return configured_joint_name;
	}

	if (configured_joint_name.find('/') != std::string::npos)
	{
		return configured_joint_name;
	}

	return sanitized_namespace + "/" + configured_joint_name;
}

std::string RobotBaseDriverNode::getSanitizedNamespace() const
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

	return namespace_value;
}

std::string RobotBaseDriverNode::describeExpectedMotionType(double linear_x, double angular_z) const
{
	static constexpr double EPSILON = 1e-6;

	if (std::abs(linear_x) <= EPSILON && std::abs(angular_z) <= EPSILON)
	{
		return "stop";
	}

	if (std::abs(angular_z) <= EPSILON)
	{
		return linear_x > 0.0 ? "forward" : "backward";
	}

	if (std::abs(linear_x) <= EPSILON)
	{
		return angular_z > 0.0 ? "rotate_left" : "rotate_right";
	}

	return "arc";
}

const char *RobotBaseDriverNode::describeSign(double value) const
{
	static constexpr double EPSILON = 1e-9;

	if (value > EPSILON)
	{
		return "+";
	}

	if (value < -EPSILON)
	{
		return "-";
	}

	return "0";
}

double RobotBaseDriverNode::quaternionToYaw(
	double orientation_w,
	double orientation_x,
	double orientation_y,
	double orientation_z) const
{
	return std::atan2(
		2.0 * ((orientation_w * orientation_z) + (orientation_x * orientation_y)),
		1.0 - (2.0 * ((orientation_y * orientation_y) + (orientation_z * orientation_z))));
}
