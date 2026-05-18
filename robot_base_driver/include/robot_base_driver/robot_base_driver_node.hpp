#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "robot_base_driver/odometry_integrator.hpp"
#include "robot_base_driver/opencr_client.hpp"
#include "robot_base_driver/serial_port.hpp"

namespace robot::hw::base
{

class RobotBaseDriverNode : public rclcpp::Node
{
private:
	static constexpr double DEFAULT_WHEEL_SEPARATION_M = 0.160;
	static constexpr double DEFAULT_WHEEL_RADIUS_M = 0.033;
	static constexpr double DEFAULT_PROFILE_ACCELERATION_CONSTANT = 214.577;
	static constexpr const char *DEFAULT_CMD_VEL_TOPIC = "/cmd_vel";
	static constexpr const char *DEFAULT_ODOM_TOPIC = "/odom";
	static constexpr const char *DEFAULT_IMU_TOPIC = "/imu";
	static constexpr const char *DEFAULT_JOINT_STATES_TOPIC = "/joint_states";

	std::string m_port;
	int m_baudrate;
	int m_opencr_id;
	double m_protocol_version;
	std::string m_cmd_vel_topic;
	std::string m_odom_topic;
	std::string m_imu_topic;
	std::string m_joint_states_topic;
	std::string m_odom_frame_id;
	std::string m_base_frame_id;
	std::string m_imu_frame_id;
	std::string m_wheel_left_joint_name;
	std::string m_wheel_right_joint_name;
	double m_wheel_separation_m;
	double m_wheel_radius_m;
	double m_profile_acceleration_constant;
	double m_profile_acceleration;
	bool m_is_publish_tf;
	bool m_is_using_imu_for_yaw;
	bool m_is_publishing_imu;
	bool m_is_publishing_joint_states;
	bool m_is_heartbeat_enabled;
	int m_heartbeat_interval_ms;
	int m_poll_interval_ms;
	int m_startup_delay_ms;
	bool m_is_reconnect_on_error;
	int m_reconnect_interval_ms;
	bool m_is_stamped_cmd_vel_enabled;
	bool m_is_imu_recalibration_on_startup;
	bool m_is_serial_packet_logging_enabled;
	bool m_is_read_rate_logging_enabled;
	int m_response_timeout_ms;

	std::shared_ptr<SerialPort> m_serial_port;
	std::shared_ptr<OpencrClient> m_opencr_client;
	std::shared_ptr<OdometryIntegrator> m_odometry_integrator;
	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odom_publisher;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imu_publisher;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_joint_state_publisher;
	std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> m_cmd_vel_subscription;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> m_cmd_vel_stamped_subscription;
	std::shared_ptr<rclcpp::TimerBase> m_reconnect_timer;
	std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_reset_odometry_service;

	std::mutex m_state_mutex;
	std::atomic_bool m_is_shutdown_requested;
	std::atomic_bool m_is_reconnecting;
	rclcpp::Clock m_throttle_clock;
	bool m_has_logged_publish_success;

	void declareParameters();
	void loadParameters();
	void validateParameters();
	void logParameterSummary() const;
	void setupPublishers();
	void setupSubscriptions();
	void setupServices();
	void setupOdometryIntegrator();
	void startDriver();
	bool startRealMode();
	void stopDriver();
	void scheduleReconnect(const std::string &reason);
	void cancelReconnect();
	void attemptReconnect();
	void handleVelocityCommand(const geometry_msgs::msg::Twist &message);
	void handleStampedVelocityCommand(const geometry_msgs::msg::TwistStamped &message);
	void handleOpencrState(const OpencrState &state);
	void publishImu(const OpencrState &state, const rclcpp::Time &stamp);
	void publishJointStates(const OpencrState &state, const rclcpp::Time &stamp);
	void publishOdometry(const rclcpp::Time &stamp);
	void handleClientConnected();
	void handleClientError(const std::string &reason);
	void handleResetOdometry(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	std::string resolveTopicName(const std::string &configured_topic, const char *default_topic) const;
	std::string resolveFrameId(const std::string &configured_frame_id) const;
	std::string resolveJointName(const std::string &configured_joint_name) const;
	std::string getSanitizedNamespace() const;

protected:
public:
	explicit RobotBaseDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	virtual ~RobotBaseDriverNode();
};

}  // namespace robot::hw::base
