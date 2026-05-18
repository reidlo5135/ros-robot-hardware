#include "robot_base_driver/robot_base_driver_node.hpp"

using namespace robot::hw::base;

RobotBaseDriverNode::RobotBaseDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_base_driver", options),
	m_port("/dev/ttyACM0"),
	m_baudrate(1000000),
	m_opencr_id(ControlTable::OPENCR_ID),
	m_protocol_version(2.0),
	m_cmd_vel_topic(DEFAULT_CMD_VEL_TOPIC),
	m_odom_topic(DEFAULT_ODOM_TOPIC),
	m_imu_topic(DEFAULT_IMU_TOPIC),
	m_joint_states_topic(DEFAULT_JOINT_STATES_TOPIC),
	m_odom_frame_id("odom"),
	m_base_frame_id("base_footprint"),
	m_imu_frame_id("imu_link"),
	m_wheel_left_joint_name("wheel_left_joint"),
	m_wheel_right_joint_name("wheel_right_joint"),
	m_wheel_separation_m(DEFAULT_WHEEL_SEPARATION_M),
	m_wheel_radius_m(DEFAULT_WHEEL_RADIUS_M),
	m_profile_acceleration_constant(DEFAULT_PROFILE_ACCELERATION_CONSTANT),
	m_profile_acceleration(0.0),
	m_is_publish_tf(true),
	m_is_using_imu_for_yaw(true),
	m_is_publishing_imu(true),
	m_is_publishing_joint_states(true),
	m_is_heartbeat_enabled(false),
	m_heartbeat_interval_ms(100),
	m_poll_interval_ms(50),
	m_startup_delay_ms(1000),
	m_is_reconnect_on_error(true),
	m_reconnect_interval_ms(1000),
	m_is_stamped_cmd_vel_enabled(true),
	m_is_imu_recalibration_on_startup(false),
	m_is_imu_recalibration_ack_required(false),
	m_is_profile_acceleration_ack_required(false),
	m_is_heartbeat_ack_required(false),
	m_is_startup_initial_state_read_required(true),
	m_startup_initial_state_read_retries(5),
	m_startup_initial_state_read_retry_interval_ms(200),
	m_is_serial_packet_logging_enabled(false),
	m_is_read_rate_logging_enabled(true),
	m_response_timeout_ms(500),
	m_serial_port(nullptr),
	m_opencr_client(nullptr),
	m_odometry_integrator(nullptr),
	m_odom_publisher(nullptr),
	m_imu_publisher(nullptr),
	m_joint_state_publisher(nullptr),
	m_tf_broadcaster(nullptr),
	m_cmd_vel_subscription(nullptr),
	m_cmd_vel_stamped_subscription(nullptr),
	m_reconnect_timer(nullptr),
	m_reset_odometry_service(nullptr),
	m_state_mutex(),
	m_is_shutdown_requested(false),
	m_is_reconnecting(false),
	m_throttle_clock(RCL_STEADY_TIME),
	m_has_logged_publish_success(false)
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
	m_is_shutdown_requested.store(true);
	cancelReconnect();
	stopDriver();
}

void RobotBaseDriverNode::declareParameters()
{
	declare_parameter("port", m_port);
	declare_parameter("baudrate", m_baudrate);
	declare_parameter("opencr_id", m_opencr_id);
	declare_parameter("protocol_version", m_protocol_version);
	declare_parameter("cmd_vel_topic", m_cmd_vel_topic);
	declare_parameter("odom_topic", m_odom_topic);
	declare_parameter("imu_topic", m_imu_topic);
	declare_parameter("joint_states_topic", m_joint_states_topic);
	declare_parameter("odom_frame_id", m_odom_frame_id);
	declare_parameter("base_frame_id", m_base_frame_id);
	declare_parameter("imu_frame_id", m_imu_frame_id);
	declare_parameter("wheel_left_joint_name", m_wheel_left_joint_name);
	declare_parameter("wheel_right_joint_name", m_wheel_right_joint_name);
	declare_parameter("wheel_separation", m_wheel_separation_m);
	declare_parameter("wheel_radius", m_wheel_radius_m);
	declare_parameter("motors.profile_acceleration_constant", m_profile_acceleration_constant);
	declare_parameter("motors.profile_acceleration", m_profile_acceleration);
	declare_parameter("publish_tf", m_is_publish_tf);
	declare_parameter("use_imu_for_yaw", m_is_using_imu_for_yaw);
	declare_parameter("publish_imu", m_is_publishing_imu);
	declare_parameter("publish_joint_states", m_is_publishing_joint_states);
	declare_parameter("heartbeat_enabled", m_is_heartbeat_enabled);
	declare_parameter("heartbeat_interval_ms", m_heartbeat_interval_ms);
	declare_parameter("poll_interval_ms", m_poll_interval_ms);
	declare_parameter("startup_delay_ms", m_startup_delay_ms);
	declare_parameter("reconnect_on_error", m_is_reconnect_on_error);
	declare_parameter("reconnect_interval_ms", m_reconnect_interval_ms);
	declare_parameter("enable_stamped_cmd_vel", m_is_stamped_cmd_vel_enabled);
	declare_parameter("imu_recalibration_on_startup", m_is_imu_recalibration_on_startup);
	declare_parameter("imu_recalibration_requires_ack", m_is_imu_recalibration_ack_required);
	declare_parameter("profile_acceleration_requires_ack", m_is_profile_acceleration_ack_required);
	declare_parameter("heartbeat_requires_ack", m_is_heartbeat_ack_required);
	declare_parameter("startup_require_initial_state_read", m_is_startup_initial_state_read_required);
	declare_parameter("startup_initial_state_read_retries", m_startup_initial_state_read_retries);
	declare_parameter("startup_initial_state_read_retry_interval_ms", m_startup_initial_state_read_retry_interval_ms);
	declare_parameter("log_serial_packets", m_is_serial_packet_logging_enabled);
	declare_parameter("log_read_rate", m_is_read_rate_logging_enabled);
	declare_parameter("response_timeout_ms", m_response_timeout_ms);
}

void RobotBaseDriverNode::loadParameters()
{
	get_parameter("port", m_port);
	get_parameter("baudrate", m_baudrate);
	get_parameter("opencr_id", m_opencr_id);
	get_parameter("protocol_version", m_protocol_version);
	get_parameter("cmd_vel_topic", m_cmd_vel_topic);
	get_parameter("odom_topic", m_odom_topic);
	get_parameter("imu_topic", m_imu_topic);
	get_parameter("joint_states_topic", m_joint_states_topic);
	get_parameter("odom_frame_id", m_odom_frame_id);
	get_parameter("base_frame_id", m_base_frame_id);
	get_parameter("imu_frame_id", m_imu_frame_id);
	get_parameter("wheel_left_joint_name", m_wheel_left_joint_name);
	get_parameter("wheel_right_joint_name", m_wheel_right_joint_name);
	get_parameter("wheel_separation", m_wheel_separation_m);
	get_parameter("wheel_radius", m_wheel_radius_m);
	get_parameter("motors.profile_acceleration_constant", m_profile_acceleration_constant);
	get_parameter("motors.profile_acceleration", m_profile_acceleration);
	get_parameter("publish_tf", m_is_publish_tf);
	get_parameter("use_imu_for_yaw", m_is_using_imu_for_yaw);
	get_parameter("publish_imu", m_is_publishing_imu);
	get_parameter("publish_joint_states", m_is_publishing_joint_states);
	get_parameter("heartbeat_enabled", m_is_heartbeat_enabled);
	get_parameter("heartbeat_interval_ms", m_heartbeat_interval_ms);
	get_parameter("poll_interval_ms", m_poll_interval_ms);
	get_parameter("startup_delay_ms", m_startup_delay_ms);
	get_parameter("reconnect_on_error", m_is_reconnect_on_error);
	get_parameter("reconnect_interval_ms", m_reconnect_interval_ms);
	get_parameter("enable_stamped_cmd_vel", m_is_stamped_cmd_vel_enabled);
	get_parameter("imu_recalibration_on_startup", m_is_imu_recalibration_on_startup);
	get_parameter("imu_recalibration_requires_ack", m_is_imu_recalibration_ack_required);
	get_parameter("profile_acceleration_requires_ack", m_is_profile_acceleration_ack_required);
	get_parameter("heartbeat_requires_ack", m_is_heartbeat_ack_required);
	get_parameter("startup_require_initial_state_read", m_is_startup_initial_state_read_required);
	get_parameter("startup_initial_state_read_retries", m_startup_initial_state_read_retries);
	get_parameter("startup_initial_state_read_retry_interval_ms", m_startup_initial_state_read_retry_interval_ms);
	get_parameter("log_serial_packets", m_is_serial_packet_logging_enabled);
	get_parameter("log_read_rate", m_is_read_rate_logging_enabled);
	get_parameter("response_timeout_ms", m_response_timeout_ms);
}

void RobotBaseDriverNode::validateParameters()
{
	if (m_opencr_id < 0 || m_opencr_id > 252)
	{
		RCLCPP_WARN(get_logger(), "opencr_id must be between 0 and 252. Resetting to %u", ControlTable::OPENCR_ID);
		m_opencr_id = ControlTable::OPENCR_ID;
	}

	if (std::abs(m_protocol_version - 2.0) > 0.01)
	{
		RCLCPP_WARN(get_logger(), "Only Dynamixel Protocol 2.0 is supported. Resetting protocol_version to 2.0");
		m_protocol_version = 2.0;
	}

	if (m_wheel_separation_m <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "wheel_separation must be positive. Resetting to %.3f", DEFAULT_WHEEL_SEPARATION_M);
		m_wheel_separation_m = DEFAULT_WHEEL_SEPARATION_M;
	}

	if (m_wheel_radius_m <= 0.0)
	{
		RCLCPP_WARN(get_logger(), "wheel_radius must be positive. Resetting to %.3f", DEFAULT_WHEEL_RADIUS_M);
		m_wheel_radius_m = DEFAULT_WHEEL_RADIUS_M;
	}

	if (m_heartbeat_interval_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "heartbeat_interval_ms must be positive. Resetting to 100");
		m_heartbeat_interval_ms = 100;
	}

	if (m_poll_interval_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "poll_interval_ms must be positive. Resetting to 50");
		m_poll_interval_ms = 50;
	}

	if (m_startup_delay_ms < 0)
	{
		RCLCPP_WARN(get_logger(), "startup_delay_ms cannot be negative. Resetting to 1000");
		m_startup_delay_ms = 1000;
	}

	if (m_reconnect_interval_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "reconnect_interval_ms must be positive. Resetting to 1000");
		m_reconnect_interval_ms = 1000;
	}

	if (m_response_timeout_ms <= 0)
	{
		RCLCPP_WARN(get_logger(), "response_timeout_ms must be positive. Resetting to 500");
		m_response_timeout_ms = 500;
	}

	if (m_startup_initial_state_read_retries <= 0)
	{
		RCLCPP_WARN(get_logger(), "startup_initial_state_read_retries must be positive. Resetting to 5");
		m_startup_initial_state_read_retries = 5;
	}

	if (m_startup_initial_state_read_retry_interval_ms < 0)
	{
		RCLCPP_WARN(
			get_logger(),
			"startup_initial_state_read_retry_interval_ms cannot be negative. Resetting to 200");
		m_startup_initial_state_read_retry_interval_ms = 200;
	}
}

void RobotBaseDriverNode::logParameterSummary() const
{
	RCLCPP_INFO(
		get_logger(),
		"Base parameters: port=%s baudrate=%d opencr_id=%d protocol_version=%.1f cmd_vel_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s odom_frame_id=%s base_frame_id=%s imu_frame_id=%s wheel_separation=%.3f wheel_radius=%.3f publish_tf=%s use_imu_for_yaw=%s publish_imu=%s publish_joint_states=%s heartbeat_enabled=%s heartbeat_interval_ms=%d poll_interval_ms=%d startup_delay_ms=%d reconnect_on_error=%s reconnect_interval_ms=%d enable_stamped_cmd_vel=%s imu_recalibration_on_startup=%s imu_recalibration_requires_ack=%s profile_acceleration_requires_ack=%s heartbeat_requires_ack=%s startup_require_initial_state_read=%s startup_initial_state_read_retries=%d startup_initial_state_read_retry_interval_ms=%d log_serial_packets=%s log_read_rate=%s response_timeout_ms=%d",
		m_port.c_str(),
		m_baudrate,
		m_opencr_id,
		m_protocol_version,
		m_cmd_vel_topic.c_str(),
		m_odom_topic.c_str(),
		m_imu_topic.c_str(),
		m_joint_states_topic.c_str(),
		m_odom_frame_id.c_str(),
		m_base_frame_id.c_str(),
		m_imu_frame_id.c_str(),
		m_wheel_separation_m,
		m_wheel_radius_m,
		m_is_publish_tf ? "true" : "false",
		m_is_using_imu_for_yaw ? "true" : "false",
		m_is_publishing_imu ? "true" : "false",
		m_is_publishing_joint_states ? "true" : "false",
		m_is_heartbeat_enabled ? "true" : "false",
		m_heartbeat_interval_ms,
		m_poll_interval_ms,
		m_startup_delay_ms,
		m_is_reconnect_on_error ? "true" : "false",
		m_reconnect_interval_ms,
		m_is_stamped_cmd_vel_enabled ? "true" : "false",
		m_is_imu_recalibration_on_startup ? "true" : "false",
		m_is_imu_recalibration_ack_required ? "true" : "false",
		m_is_profile_acceleration_ack_required ? "true" : "false",
		m_is_heartbeat_ack_required ? "true" : "false",
		m_is_startup_initial_state_read_required ? "true" : "false",
		m_startup_initial_state_read_retries,
		m_startup_initial_state_read_retry_interval_ms,
		m_is_serial_packet_logging_enabled ? "true" : "false",
		m_is_read_rate_logging_enabled ? "true" : "false",
		m_response_timeout_ms);
}

void RobotBaseDriverNode::setupPublishers()
{
	m_odom_publisher = create_publisher<nav_msgs::msg::Odometry>(
		resolveTopicName(m_odom_topic, DEFAULT_ODOM_TOPIC),
		rclcpp::SystemDefaultsQoS());

	if (m_is_publishing_imu)
	{
		m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>(
			resolveTopicName(m_imu_topic, DEFAULT_IMU_TOPIC),
			rclcpp::SensorDataQoS());
	}

	if (m_is_publishing_joint_states)
	{
		m_joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>(
			resolveTopicName(m_joint_states_topic, DEFAULT_JOINT_STATES_TOPIC),
			rclcpp::SystemDefaultsQoS());
	}

	if (m_is_publish_tf)
	{
		m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	}
}

void RobotBaseDriverNode::setupSubscriptions()
{
	if (m_is_stamped_cmd_vel_enabled)
	{
		m_cmd_vel_stamped_subscription = create_subscription<geometry_msgs::msg::TwistStamped>(
			resolveTopicName(m_cmd_vel_topic, DEFAULT_CMD_VEL_TOPIC),
			rclcpp::SystemDefaultsQoS(),
			[this](const geometry_msgs::msg::TwistStamped::SharedPtr message) -> void
			{
				handleStampedVelocityCommand(*message);
			});
		return;
	}

	m_cmd_vel_subscription = create_subscription<geometry_msgs::msg::Twist>(
		resolveTopicName(m_cmd_vel_topic, DEFAULT_CMD_VEL_TOPIC),
		rclcpp::SystemDefaultsQoS(),
		[this](const geometry_msgs::msg::Twist::SharedPtr message) -> void
		{
			handleVelocityCommand(*message);
		});
}

void RobotBaseDriverNode::setupServices()
{
	m_reset_odometry_service = create_service<std_srvs::srv::Trigger>(
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
	m_odometry_integrator = std::make_shared<OdometryIntegrator>(
		m_wheel_separation_m,
		m_wheel_radius_m,
		m_is_using_imu_for_yaw);
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

	m_serial_port = std::make_shared<SerialPort>(get_logger());
	if (!m_serial_port->openPort(m_port, m_baudrate))
	{
		RCLCPP_ERROR(get_logger(), "Serial open failed for %s", m_port.c_str());
		return false;
	}

	OpencrClientConfig config;
	config.m_opencr_id = static_cast<uint8_t>(m_opencr_id);
	config.m_protocol_version = m_protocol_version;
	config.m_response_timeout_ms = m_response_timeout_ms;
	config.m_startup_delay_ms = m_startup_delay_ms;
	config.m_poll_interval_ms = m_poll_interval_ms;
	config.m_heartbeat_interval_ms = m_heartbeat_interval_ms;
	config.m_is_heartbeat_enabled = m_is_heartbeat_enabled;
	config.m_is_imu_recalibration_on_startup = m_is_imu_recalibration_on_startup;
	config.m_is_imu_recalibration_ack_required = m_is_imu_recalibration_ack_required;
	config.m_is_profile_acceleration_ack_required = m_is_profile_acceleration_ack_required;
	config.m_is_heartbeat_ack_required = m_is_heartbeat_ack_required;
	config.m_is_startup_initial_state_read_required = m_is_startup_initial_state_read_required;
	config.m_startup_initial_state_read_retries = m_startup_initial_state_read_retries;
	config.m_startup_initial_state_read_retry_interval_ms = m_startup_initial_state_read_retry_interval_ms;
	config.m_is_serial_packet_logging_enabled = m_is_serial_packet_logging_enabled;
	config.m_is_read_rate_logging_enabled = m_is_read_rate_logging_enabled;
	config.m_profile_acceleration_constant = m_profile_acceleration_constant;
	config.m_profile_acceleration = m_profile_acceleration;

	m_opencr_client = std::make_shared<OpencrClient>(get_logger(), m_serial_port.get(), config);
	bool started = m_opencr_client->start(
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
		m_opencr_client.reset();
		m_serial_port->closePort();
		return false;
	}

	return true;
}

void RobotBaseDriverNode::stopDriver()
{
	if (m_opencr_client)
	{
		m_opencr_client->stop();
		m_opencr_client.reset();
	}

	if (m_serial_port)
	{
		m_serial_port->closePort();
		m_serial_port.reset();
	}
}

void RobotBaseDriverNode::scheduleReconnect(const std::string &reason)
{
	if (!m_is_reconnect_on_error || m_is_shutdown_requested.load())
	{
		return;
	}

	if (m_is_reconnecting.exchange(true))
	{
		return;
	}

	RCLCPP_WARN(get_logger(), "Scheduling OpenCR reconnect: %s", reason.c_str());
	m_reconnect_timer = create_wall_timer(
		std::chrono::milliseconds(m_reconnect_interval_ms),
		[this]() -> void
		{
			attemptReconnect();
		});
}

void RobotBaseDriverNode::cancelReconnect()
{
	if (m_reconnect_timer)
	{
		m_reconnect_timer->cancel();
		m_reconnect_timer.reset();
	}

	m_is_reconnecting.store(false);
}

void RobotBaseDriverNode::attemptReconnect()
{
	RCLCPP_INFO(get_logger(), "Attempting OpenCR reconnect on %s", m_port.c_str());
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
	if (!m_opencr_client || !m_opencr_client->isRunning())
	{
		return;
	}

	VelocityCommand command;
	command.m_linear_x_mps = message.linear.x;
	command.m_angular_z_rps = message.angular.z;
	m_opencr_client->setVelocityCommand(command);
}

void RobotBaseDriverNode::handleStampedVelocityCommand(const geometry_msgs::msg::TwistStamped &message)
{
	handleVelocityCommand(message.twist);
}

void RobotBaseDriverNode::handleOpencrState(const OpencrState &state)
{
	std::lock_guard<std::mutex> lock(m_state_mutex);
	rclcpp::Time stamp = now();

	if (state.m_device_status == -1)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			m_throttle_clock,
			2000,
			"OpenCR reported device_status = -1, please check motor and power");
	}

	bool updated = m_odometry_integrator->update(
		state.m_present_position_left,
		state.m_present_position_right,
		state.m_present_velocity_left,
		state.m_present_velocity_right,
		state.m_has_imu_data,
		state.m_imu_orientation_w,
		state.m_imu_orientation_x,
		state.m_imu_orientation_y,
		state.m_imu_orientation_z,
		stamp);

	if (m_is_publishing_imu)
	{
		publishImu(state, stamp);
	}

	if (m_is_publishing_joint_states)
	{
		publishJointStates(state, stamp);
	}

	if (updated)
	{
		publishOdometry(stamp);
		if (!m_has_logged_publish_success)
		{
			RCLCPP_INFO(get_logger(), "OpenCR data pipeline active: imu/joint_states/odom publishing");
			m_has_logged_publish_success = true;
		}
	}
}

void RobotBaseDriverNode::publishImu(const OpencrState &state, const rclcpp::Time &stamp)
{
	if (!m_imu_publisher)
	{
		return;
	}

	sensor_msgs::msg::Imu message;
	message.header.stamp = stamp;
	message.header.frame_id = resolveFrameId(m_imu_frame_id);
	message.orientation.w = state.m_imu_orientation_w;
	message.orientation.x = state.m_imu_orientation_x;
	message.orientation.y = state.m_imu_orientation_y;
	message.orientation.z = state.m_imu_orientation_z;
	message.angular_velocity.x = state.m_imu_angular_velocity_x;
	message.angular_velocity.y = state.m_imu_angular_velocity_y;
	message.angular_velocity.z = state.m_imu_angular_velocity_z;
	message.linear_acceleration.x = state.m_imu_linear_acceleration_x;
	message.linear_acceleration.y = state.m_imu_linear_acceleration_y;
	message.linear_acceleration.z = state.m_imu_linear_acceleration_z;
	m_imu_publisher->publish(message);
}

void RobotBaseDriverNode::publishJointStates(const OpencrState &state, const rclcpp::Time &stamp)
{
	(void)state;

	if (!m_joint_state_publisher)
	{
		return;
	}

	sensor_msgs::msg::JointState message;
	message.header.stamp = stamp;
	message.header.frame_id = resolveFrameId(m_base_frame_id);
	message.name.push_back(resolveJointName(m_wheel_left_joint_name));
	message.name.push_back(resolveJointName(m_wheel_right_joint_name));

	std::array<double, 2> positions = m_odometry_integrator->getJointPositionsRad();
	std::array<double, 2> velocities = m_odometry_integrator->getJointVelocitiesMps();
	message.position.push_back(positions[0]);
	message.position.push_back(positions[1]);
	message.velocity.push_back(velocities[0]);
	message.velocity.push_back(velocities[1]);
	m_joint_state_publisher->publish(message);
}

void RobotBaseDriverNode::publishOdometry(const rclcpp::Time &stamp)
{
	nav_msgs::msg::Odometry message = m_odometry_integrator->buildOdometryMessage(
		stamp,
		resolveFrameId(m_odom_frame_id),
		resolveFrameId(m_base_frame_id));
	m_odom_publisher->publish(message);

	if (m_is_publish_tf && m_tf_broadcaster)
	{
		geometry_msgs::msg::TransformStamped transform = m_odometry_integrator->buildTransformMessage(
			stamp,
			resolveFrameId(m_odom_frame_id),
			resolveFrameId(m_base_frame_id));
		m_tf_broadcaster->sendTransform(transform);
	}
}

void RobotBaseDriverNode::handleClientConnected()
{
	RCLCPP_INFO(get_logger(), "OpenCR client started successfully");
	cancelReconnect();
}

void RobotBaseDriverNode::handleClientError(const std::string &reason)
{
	if (m_is_shutdown_requested.load())
	{
		return;
	}

	RCLCPP_ERROR(get_logger(), "OpenCR client error: %s", reason.c_str());
	scheduleReconnect(reason);
}

void RobotBaseDriverNode::handleResetOdometry(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;
	std::lock_guard<std::mutex> lock(m_state_mutex);
	m_odometry_integrator->reset();
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
