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

class OdometryIntegrator
{
private:
	static constexpr double TICK_TO_RAD = 0.001533981;
	static constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

	double m_wheel_separation_m;
	double m_wheel_radius_m;
	bool m_is_using_imu_for_yaw;
	bool m_has_last_joint_ticks;
	bool m_has_last_stamp;
	bool m_has_last_imu_yaw;
	int32_t m_last_left_ticks;
	int32_t m_last_right_ticks;
	rclcpp::Time m_last_stamp;
	double m_last_imu_yaw_rad;
	std::array<double, 2> m_joint_positions_rad;
	std::array<double, 2> m_joint_velocities_mps;
	std::array<double, 3> m_pose;
	std::array<double, 3> m_velocity;

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
