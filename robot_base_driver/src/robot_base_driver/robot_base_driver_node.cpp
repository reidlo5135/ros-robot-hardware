#include "robot_base_driver/robot_base_driver_node.hpp"

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

}  // namespace

RobotBaseDriverNode::RobotBaseDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_base_driver", options),
	port_("/dev/ttyACM0"),
	baudrate_(1000000),
	opencr_id_(ControlTable::OPENCR_ID),
	protocol_version_(2.0),
	cmd_vel_topic_(DEFAULT_CMD_VEL_TOPIC),
	odom_topic_(DEFAULT_ODOM_TOPIC),
	imu_topic_(DEFAULT_IMU_TOPIC),
	joint_states_topic_(DEFAULT_JOINT_STATES_TOPIC),
	odom_frame_id_("odom"),
	base_frame_id_("base_footprint"),
	imu_frame_id_("imu_link"),
	wheel_left_joint_name_("wheel_left_joint"),
	wheel_right_joint_name_("wheel_right_joint"),
	wheel_separation_m_(DEFAULT_WHEEL_SEPARATION_M),
	wheel_radius_m_(DEFAULT_WHEEL_RADIUS_M),
	profile_acceleration_constant_(DEFAULT_PROFILE_ACCELERATION_CONSTANT),
	profile_acceleration_(0.0),
	is_publish_tf_(true),
	is_using_imu_for_yaw_(false),
	is_publishing_imu_(false),
	is_publishing_joint_states_(true),
	is_heartbeat_enabled_(false),
	heartbeat_interval_ms_(100),
	poll_interval_ms_(300),
	startup_delay_ms_(1000),
	is_reconnect_on_error_(true),
	reconnect_interval_ms_(1000),
	is_stamped_cmd_vel_enabled_(true),
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
	poll_mode_("minimal"),
	max_consecutive_poll_failures_(5),
	is_polling_device_status_(false),
	require_device_status_(false),
	require_imu_(false),
	reconnect_on_poll_failure_(false),
	reopen_serial_on_poll_failure_(false),
	probe_registers_on_startup_(true),
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
	has_logged_publish_success_(false)
{
	declareParameters();
	loadParameters();
	validateParameters();
	logParameterSummary();
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
	declare_parameter("odom_topic", odom_topic_);
	declare_parameter("imu_topic", imu_topic_);
	declare_parameter("joint_states_topic", joint_states_topic_);
	declare_parameter("odom_frame_id", odom_frame_id_);
	declare_parameter("base_frame_id", base_frame_id_);
	declare_parameter("imu_frame_id", imu_frame_id_);
	declare_parameter("wheel_left_joint_name", wheel_left_joint_name_);
	declare_parameter("wheel_right_joint_name", wheel_right_joint_name_);
	declare_parameter("wheel_separation", wheel_separation_m_);
	declare_parameter("wheel_radius", wheel_radius_m_);
	declare_parameter("motors.profile_acceleration_constant", profile_acceleration_constant_);
	declare_parameter("motors.profile_acceleration", profile_acceleration_);
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
	declare_parameter("enable_stamped_cmd_vel", is_stamped_cmd_vel_enabled_);
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
}

void RobotBaseDriverNode::loadParameters()
{
	get_parameter("port", port_);
	get_parameter("baudrate", baudrate_);
	get_parameter("opencr_id", opencr_id_);
	get_parameter("protocol_version", protocol_version_);
	get_parameter("cmd_vel_topic", cmd_vel_topic_);
	get_parameter("odom_topic", odom_topic_);
	get_parameter("imu_topic", imu_topic_);
	get_parameter("joint_states_topic", joint_states_topic_);
	get_parameter("odom_frame_id", odom_frame_id_);
	get_parameter("base_frame_id", base_frame_id_);
	get_parameter("imu_frame_id", imu_frame_id_);
	get_parameter("wheel_left_joint_name", wheel_left_joint_name_);
	get_parameter("wheel_right_joint_name", wheel_right_joint_name_);
	get_parameter("wheel_separation", wheel_separation_m_);
	get_parameter("wheel_radius", wheel_radius_m_);
	get_parameter("motors.profile_acceleration_constant", profile_acceleration_constant_);
	get_parameter("motors.profile_acceleration", profile_acceleration_);
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
	get_parameter("enable_stamped_cmd_vel", is_stamped_cmd_vel_enabled_);
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
}

void RobotBaseDriverNode::validateParameters()
{
	if (opencr_id_ < 0 || opencr_id_ > 252)
	{
		RCLCPP_WARN(get_logger(), "opencr_id must be between 0 and 252. Resetting to %u", ControlTable::OPENCR_ID);
		opencr_id_ = ControlTable::OPENCR_ID;
	}

	if (std::abs(protocol_version_ - 2.0) > 0.01)
	{
		RCLCPP_WARN(get_logger(), "Only Dynamixel Protocol 2.0 is supported. Resetting protocol_version to 2.0");
		protocol_version_ = 2.0;
	}

	if (wheel_separation_m_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "wheel_separation must be positive. Resetting to %.3f", DEFAULT_WHEEL_SEPARATION_M);
		wheel_separation_m_ = DEFAULT_WHEEL_SEPARATION_M;
	}

	if (wheel_radius_m_ <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "wheel_radius must be positive. Resetting to %.3f", DEFAULT_WHEEL_RADIUS_M);
		wheel_radius_m_ = DEFAULT_WHEEL_RADIUS_M;
	}

	if (heartbeat_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "heartbeat_interval_ms must be positive. Resetting to 100");
		heartbeat_interval_ms_ = 100;
	}

	if (poll_interval_ms_ <= 0)
	{
		RCLCPP_WARN(get_logger(), "poll_interval_ms must be positive. Resetting to 300");
		poll_interval_ms_ = 300;
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

	if (!(poll_mode_ == "minimal" || poll_mode_ == "full"))
	{
		RCLCPP_WARN(get_logger(), "poll_mode must be either 'minimal' or 'full'. Resetting to 'minimal'");
		poll_mode_ = "minimal";
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

	if (!reconnect_on_poll_failure_)
	{
		reopen_serial_on_poll_failure_ = false;
	}
}

void RobotBaseDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"Base parameters: port=%s baudrate=%d opencr_id=%d protocol_version=%.1f cmd_vel_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s wheel_separation=%.3f wheel_radius=%.3f publish_tf=%s use_imu_for_yaw=%s publish_imu=%s publish_joint_states=%s heartbeat_enabled=%s heartbeat_interval_ms=%d poll_interval_ms=%d startup_delay_ms=%d reconnect_on_error=%s reconnect_interval_ms=%d enable_stamped_cmd_vel=%s imu_recalibration_on_startup=%s imu_recalibration_requires_ack=%s profile_acceleration_requires_ack=%s profile_acceleration_on_startup=%s heartbeat_requires_ack=%s startup_require_initial_state_read=%s startup_initial_state_read_retries=%d startup_initial_state_read_retry_interval_ms=%d log_serial_packets=%s log_read_rate=%s response_timeout_ms=%d transaction_gap_us=%d poll_mode=%s max_consecutive_poll_failures=%d poll_device_status=%s require_device_status=%s require_imu=%s reconnect_on_poll_failure=%s reopen_serial_on_poll_failure=%s probe_registers_on_startup=%s",
		port_.c_str(),
		baudrate_,
		opencr_id_,
		protocol_version_,
		cmd_vel_topic_.c_str(),
		odom_topic_.c_str(),
		imu_topic_.c_str(),
		joint_states_topic_.c_str(),
		odom_frame_id_.c_str(),
		base_frame_id_.c_str(),
		imu_frame_id_.c_str(),
		wheel_separation_m_,
		wheel_radius_m_,
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
		boolToString(is_stamped_cmd_vel_enabled_),
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
}

void RobotBaseDriverNode::setupPublishers()
{
	odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
		resolveTopicName(odom_topic_, DEFAULT_ODOM_TOPIC),
		rclcpp::SystemDefaultsQoS());

	if (is_publishing_imu_)
	{
		imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
			resolveTopicName(imu_topic_, DEFAULT_IMU_TOPIC),
			rclcpp::SensorDataQoS());
	}

	if (is_publishing_joint_states_)
	{
		joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
			resolveTopicName(joint_states_topic_, DEFAULT_JOINT_STATES_TOPIC),
			rclcpp::SystemDefaultsQoS());
	}

	if (is_publish_tf_)
	{
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	}
}

void RobotBaseDriverNode::setupSubscriptions()
{
	if (is_stamped_cmd_vel_enabled_)
	{
		cmd_vel_stamped_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
			resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC),
			rclcpp::SystemDefaultsQoS(),
			[this](const geometry_msgs::msg::TwistStamped::SharedPtr message) -> void
			{
				handleStampedVelocityCommand(*message);
			});
		return;
	}

	cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
		resolveTopicName(cmd_vel_topic_, DEFAULT_CMD_VEL_TOPIC),
		rclcpp::SystemDefaultsQoS(),
		[this](const geometry_msgs::msg::Twist::SharedPtr message) -> void
		{
			handleVelocityCommand(*message);
		});
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
		is_using_imu_for_yaw_);
}

void RobotBaseDriverNode::startDriver()
{
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
		return false;
	}

	OpencrClientConfig config;
	config.opencr_id = static_cast<uint8_t>(opencr_id_);
	config.protocol_version = protocol_version_;
	config.response_timeout_ms = response_timeout_ms_;
	config.transaction_gap_us = transaction_gap_us_;
	config.startup_delay_ms = startup_delay_ms_;
	config.poll_interval_ms = poll_interval_ms_;
	config.heartbeat_interval_ms = heartbeat_interval_ms_;
	config.is_heartbeat_enabled = is_heartbeat_enabled_;
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
	config.profile_acceleration_constant = profile_acceleration_constant_;
	config.profile_acceleration = profile_acceleration_;
	config.poll_mode = OpencrPollMode::Minimal;
	if (poll_mode_ == "full" && (is_publishing_imu_ || is_using_imu_for_yaw_))
	{
		config.poll_mode = OpencrPollMode::Full;
	}
	config.max_consecutive_poll_failures = max_consecutive_poll_failures_;
	config.poll_device_status = is_polling_device_status_;
	config.require_device_status = require_device_status_;
	config.require_imu = require_imu_;
	config.reconnect_on_poll_failure = reconnect_on_poll_failure_;
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
	if (!opencr_client_ || !opencr_client_->isRunning())
	{
		return;
	}

	VelocityCommand command;
	command.linear_x_mps = message.linear.x;
	command.angular_z_rps = message.angular.z;
	opencr_client_->setVelocityCommand(command);
}

void RobotBaseDriverNode::handleStampedVelocityCommand(const geometry_msgs::msg::TwistStamped &message)
{
	handleVelocityCommand(message.twist);
}

void RobotBaseDriverNode::handleOpencrState(const OpencrState &state)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	rclcpp::Time stamp = now();

	if (state.device_status == -1)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			throttle_clock_,
			2000,
			"OpenCR reported device_status = -1, please check motor and power");
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
	imu_publisher_->publish(message);
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
	std::array<double, 2> velocities = odometry_integrator_->getJointVelocitiesMps();
	message.position.push_back(positions[0]);
	message.position.push_back(positions[1]);
	message.velocity.push_back(velocities[0]);
	message.velocity.push_back(velocities[1]);
	joint_state_publisher_->publish(message);
}

void RobotBaseDriverNode::publishOdometry(const rclcpp::Time &stamp)
{
	nav_msgs::msg::Odometry message = odometry_integrator_->buildOdometryMessage(
		stamp,
		resolveFrameId(odom_frame_id_),
		resolveFrameId(base_frame_id_));
	odom_publisher_->publish(message);

	if (is_publish_tf_ && tf_broadcaster_)
	{
		geometry_msgs::msg::TransformStamped transform = odometry_integrator_->buildTransformMessage(
			stamp,
			resolveFrameId(odom_frame_id_),
			resolveFrameId(base_frame_id_));
		tf_broadcaster_->sendTransform(transform);
	}
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
