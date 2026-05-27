#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace robot::hw::base
{

struct OdometryDebugSnapshot
{
	int32_t raw_left_ticks;
	int32_t raw_right_ticks;
	int32_t raw_left_velocity;
	int32_t raw_right_velocity;
	int32_t left_tick_delta;
	int32_t right_tick_delta;
	double left_delta_rad;
	double right_delta_rad;
	double delta_s;
	double delta_theta;
	double delta_time;
	double x;
	double y;
	double yaw;
	double linear_x;
	double angular_z;
	bool has_valid_dt;
};

class OdometryIntegrator
{
private:
	static constexpr double TICK_TO_RAD = 0.001533981;
	static constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

	double wheel_separation_m_;
	double wheel_radius_m_;
	bool use_imu_for_yaw_;
	bool has_last_joint_ticks_;
	bool has_last_stamp_;
	bool has_last_imu_yaw_;
	int32_t last_left_ticks_;
	int32_t last_right_ticks_;
	rclcpp::Time last_stamp_;
	double last_imu_yaw_rad_;
	std::array<double, 2> joint_positions_rad_;
	std::array<double, 2> joint_velocities_mps_;
	std::array<double, 3> pose_;
	std::array<double, 3> velocity_;
	OdometryDebugSnapshot debug_snapshot_;

	double quaternionToYaw(
		double orientation_w,
		double orientation_x,
		double orientation_y,
		double orientation_z) const;
	double normalizeAngle(double angle_rad) const;

protected:
public:
	explicit OdometryIntegrator(double wheel_separation_m, double wheel_radius_m, bool use_imu_for_yaw);
	virtual ~OdometryIntegrator();

	void reset();
	bool update(
		int32_t raw_left_ticks,
		int32_t raw_right_ticks,
		int32_t raw_left_velocity,
		int32_t raw_right_velocity,
		bool has_imu_orientation,
		double imu_orientation_w,
		double imu_orientation_x,
		double imu_orientation_y,
		double imu_orientation_z,
		const rclcpp::Time &stamp);
	std::array<double, 2> getJointPositionsRad() const;
	std::array<double, 2> getJointVelocitiesMps() const;
	OdometryDebugSnapshot getDebugSnapshot() const;
	nav_msgs::msg::Odometry buildOdometryMessage(
		const rclcpp::Time &stamp,
		const std::string &odom_frame_id,
		const std::string &base_frame_id) const;
	geometry_msgs::msg::TransformStamped buildTransformMessage(
		const rclcpp::Time &stamp,
		const std::string &odom_frame_id,
		const std::string &base_frame_id) const;
};

}  // namespace robot::hw::base
