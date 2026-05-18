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

	std::string port_;
	int baudrate_;
	int opencr_id_;
	double protocol_version_;
	std::string cmd_vel_topic_;
	std::string odom_topic_;
	std::string imu_topic_;
	std::string joint_states_topic_;
	std::string odom_frame_id_;
	std::string base_frame_id_;
	std::string imu_frame_id_;
	std::string wheel_left_joint_name_;
	std::string wheel_right_joint_name_;
	double wheel_separation_m_;
	double wheel_radius_m_;
	double profile_acceleration_constant_;
	double profile_acceleration_;
	bool is_publish_tf_;
	bool is_using_imu_for_yaw_;
	bool is_publishing_imu_;
	bool is_publishing_joint_states_;
	bool is_heartbeat_enabled_;
	int heartbeat_interval_ms_;
	int poll_interval_ms_;
	int startup_delay_ms_;
	bool is_reconnect_on_error_;
	int reconnect_interval_ms_;
	bool is_stamped_cmd_vel_enabled_;
	bool is_imu_recalibration_on_startup_;
	bool is_imu_recalibration_ack_required_;
	bool is_profile_acceleration_ack_required_;
	bool is_profile_acceleration_on_startup_;
	bool is_heartbeat_ack_required_;
	bool is_startup_initial_state_read_required_;
	int startup_initial_state_read_retries_;
	int startup_initial_state_read_retry_interval_ms_;
	bool is_serial_packet_logging_enabled_;
	bool is_read_rate_logging_enabled_;
	int response_timeout_ms_;
	int transaction_gap_us_;
	std::string poll_mode_;
	int max_consecutive_poll_failures_;
	bool is_polling_device_status_;
	bool require_device_status_;
	bool require_imu_;
	bool reconnect_on_poll_failure_;
	bool reopen_serial_on_poll_failure_;
	bool probe_registers_on_startup_;

	std::shared_ptr<SerialPort> serial_port_;
	std::shared_ptr<OpencrClient> opencr_client_;
	std::shared_ptr<OdometryIntegrator> odometry_integrator_;
	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscription_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> cmd_vel_stamped_subscription_;
	std::shared_ptr<rclcpp::TimerBase> reconnect_timer_;
	std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_odometry_service_;

	std::mutex state_mutex_;
	std::atomic_bool is_shutdown_requested_;
	std::atomic_bool is_reconnecting_;
	rclcpp::Clock throttle_clock_;
	bool has_logged_publish_success_;

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

	using SharedPtr = std::shared_ptr<RobotBaseDriverNode>;
};

}  // namespace robot::hw::base
