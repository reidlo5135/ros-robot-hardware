#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
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
	static constexpr int DEFAULT_POLL_INTERVAL_MS = 50;
	static constexpr const char *DEFAULT_CMD_VEL_TOPIC = "/cmd_vel";
	static constexpr const char *DEFAULT_CMD_VEL_STAMPED_TOPIC = "/cmd_vel_stamped";
	static constexpr const char *DEFAULT_ODOM_TOPIC = "/odom";
	static constexpr const char *DEFAULT_IMU_TOPIC = "/imu";
	static constexpr const char *DEFAULT_JOINT_STATES_TOPIC = "/joint_states";
	static constexpr const char *DEFAULT_BATTERY_STATE_TOPIC = "/battery_state";
	static constexpr const char *DEFAULT_COMMAND_MODE = "body_twist";
	static constexpr std::array<double, 6> DEFAULT_ODOM_POSE_COVARIANCE_DIAGONAL = {
		0.01,
		0.01,
		1000000.0,
		1000000.0,
		1000000.0,
		0.05};
	static constexpr std::array<double, 6> DEFAULT_ODOM_TWIST_COVARIANCE_DIAGONAL = {
		0.01,
		0.01,
		1000000.0,
		1000000.0,
		1000000.0,
		0.05};
	static constexpr std::array<double, 9> DEFAULT_IMU_ORIENTATION_COVARIANCE = {
		0.0025, 0.0, 0.0,
		0.0, 0.0025, 0.0,
		0.0, 0.0, 0.01};
	static constexpr std::array<double, 9> DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE = {
		0.02, 0.0, 0.0,
		0.0, 0.02, 0.0,
		0.0, 0.0, 0.04};
	static constexpr std::array<double, 9> DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE = {
		0.04, 0.0, 0.0,
		0.0, 0.04, 0.0,
		0.0, 0.0, 0.04};

	std::string port_;
	int baudrate_;
	int opencr_id_;
	double protocol_version_;
	std::string cmd_vel_topic_;
	std::string cmd_vel_stamped_topic_;
	std::string odom_topic_;
	std::string imu_topic_;
	std::string joint_states_topic_;
	bool is_publishing_battery_state_;
	std::string battery_frame_id_;
	bool is_battery_percentage_enabled_;
	double battery_min_voltage_;
	double battery_max_voltage_;
	bool warn_low_battery_voltage_;
	double battery_low_voltage_;
	bool log_battery_state_;
	std::string odom_frame_id_;
	std::string base_frame_id_;
	std::string imu_frame_id_;
	std::string scan_frame_id_;
	std::string wheel_left_joint_name_;
	std::string wheel_right_joint_name_;
	double wheel_separation_m_;
	double wheel_radius_m_;
	double odom_linear_scale_;
	double odom_angular_scale_;
	bool tb3_odom_zero_covariance_;
	int left_encoder_sign_;
	int right_encoder_sign_;
	bool swap_wheel_encoders_;
	double profile_acceleration_constant_;
	double profile_acceleration_;
	std::vector<double> odom_pose_covariance_diagonal_;
	std::vector<double> odom_twist_covariance_diagonal_;
	std::vector<double> imu_orientation_covariance_;
	std::vector<double> imu_angular_velocity_covariance_;
	std::vector<double> imu_linear_acceleration_covariance_;
	std::string command_mode_;
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
	bool debug_motor_command_;
	bool is_stamped_cmd_vel_enabled_;
	bool is_motor_torque_enable_on_startup_;
	bool is_motor_torque_enable_ack_required_;
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
	bool debug_odom_;
	int debug_odom_interval_ms_;
	bool debug_tf_;
	bool debug_poll_timing_;
	double target_odom_rate_hz_;
	bool debug_odom_auto_enabled_;
	bool debug_tf_auto_enabled_;
	bool is_structured_logging_enabled_;
	double base_state_throttle_sec_;
	double cmd_vel_throttle_sec_;
	double odom_throttle_sec_;
	double tf_throttle_sec_;
	double imu_throttle_sec_;
	double joint_state_throttle_sec_;
	double serial_state_throttle_sec_;
	double opencr_state_throttle_sec_;
	double poll_timing_throttle_sec_;
	double rotation_diagnostics_throttle_sec_;
	bool is_frame_diagnostics_enabled_;
	bool is_topic_diagnostics_enabled_;

	std::shared_ptr<SerialPort> serial_port_;
	std::shared_ptr<OpencrClient> opencr_client_;
	std::shared_ptr<OdometryIntegrator> odometry_integrator_;
	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_state_publisher_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscription_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> cmd_vel_stamped_subscription_;
	std::shared_ptr<rclcpp::TimerBase> reconnect_timer_;
	std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_odometry_service_;

	std::mutex state_mutex_;
	std::atomic_bool is_shutdown_requested_;
	std::atomic_bool is_reconnecting_;
	mutable rclcpp::Clock throttle_clock_;
	bool has_logged_publish_success_;
	int8_t last_device_status_;
	bool has_seen_device_status_;
	bool last_motor_torque_enabled_;
	bool has_seen_motor_torque_enabled_;
	bool has_recent_cmd_vel_;
	double last_cmd_vel_linear_x_;
	double last_cmd_vel_angular_z_;
	bool has_last_imu_yaw_;
	double last_imu_yaw_rad_;
	double last_imu_angular_velocity_z_;

	void declareParameters();
	void loadParameters();
	void validateParameters();
	void autoConfigureDiagnosticsFromLogLevel();
	void logParameterSummary() const;
	void logBatteryConfig() const;
	void logStartupFrameSanity() const;
	void logFrameConfig() const;
	void logTopicConfig() const;
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
	void publishJointStates(
		const OpencrState &state,
		const rclcpp::Time &stamp);
	void publishBatteryState(
		const OpencrState &state,
		const rclcpp::Time &stamp);
	void publishOdometry(const rclcpp::Time &stamp);
	void logOdomDiagnostics(
		const OdometryDebugSnapshot &snapshot,
		const nav_msgs::msg::Odometry &message) const;
	void logTfDiagnostics(
		const geometry_msgs::msg::TransformStamped &transform,
		const OdometryDebugSnapshot &snapshot) const;
	void logRotationDiagnostics(
		const OdometryDebugSnapshot &snapshot,
		const nav_msgs::msg::Odometry &message) const;
	void logImuCompatibility(const nav_msgs::msg::Odometry &message) const;
	void logOdomCompatibility() const;
	void logCommandInput(
		const geometry_msgs::msg::Twist &message,
		const char *event,
		double age_sec,
		const char *result,
		const char *reason) const;
	void handleClientConnected();
	void handleClientError(const std::string &reason);
	void handleResetOdometry(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	std::string resolveTopicName(
		const std::string &configured_topic,
		const char *default_topic) const;
	std::string resolveFrameId(const std::string &configured_frame_id) const;
	std::string resolveJointName(const std::string &configured_joint_name) const;
	std::string getSanitizedNamespace() const;
	std::string describeExpectedMotionType(double linear_x, double angular_z) const;
	const char *describeSign(double value) const;
	const char *describeSignMatch(double first_value, double second_value) const;
	double normalizeAngle(double angle_rad) const;
	double calculateBatteryPercentage(double voltage) const;
	bool hasValidBatteryVoltage(const OpencrState &state) const;
	double quaternionToYaw(
		double orientation_w,
		double orientation_x,
		double orientation_y,
		double orientation_z) const;

protected:
public:
	explicit RobotBaseDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	virtual ~RobotBaseDriverNode();

	using SharedPtr = std::shared_ptr<RobotBaseDriverNode>;
};

}  // namespace robot::hw::base
