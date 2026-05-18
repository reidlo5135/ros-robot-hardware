#include "robot_base_driver/odometry_integrator.hpp"

using namespace robot::hw::base;

OdometryIntegrator::OdometryIntegrator(double wheel_separation_m, double wheel_radius_m, bool use_imu_for_yaw)
: m_wheel_separation_m(wheel_separation_m),
	m_wheel_radius_m(wheel_radius_m),
	m_is_using_imu_for_yaw(use_imu_for_yaw),
	m_has_last_joint_ticks(false),
	m_has_last_stamp(false),
	m_has_last_imu_yaw(false),
	m_last_left_ticks(0),
	m_last_right_ticks(0),
	m_last_stamp(0, 0, RCL_ROS_TIME),
	m_last_imu_yaw_rad(0.0),
	m_joint_positions_rad({0.0, 0.0}),
	m_joint_velocities_mps({0.0, 0.0}),
	m_pose({0.0, 0.0, 0.0}),
	m_velocity({0.0, 0.0, 0.0})
{
}

OdometryIntegrator::~OdometryIntegrator()
{
}

void OdometryIntegrator::reset()
{
	m_has_last_joint_ticks = false;
	m_has_last_stamp = false;
	m_has_last_imu_yaw = false;
	m_last_left_ticks = 0;
	m_last_right_ticks = 0;
	m_last_imu_yaw_rad = 0.0;
	m_joint_positions_rad = {0.0, 0.0};
	m_joint_velocities_mps = {0.0, 0.0};
	m_pose = {0.0, 0.0, 0.0};
	m_velocity = {0.0, 0.0, 0.0};
}

bool OdometryIntegrator::update(
	int32_t raw_left_ticks,
	int32_t raw_right_ticks,
	int32_t raw_left_velocity,
	int32_t raw_right_velocity,
	bool has_imu_orientation,
	double imu_orientation_w,
	double imu_orientation_x,
	double imu_orientation_y,
	double imu_orientation_z,
	const rclcpp::Time &stamp)
{
	if (!m_has_last_joint_ticks)
	{
		m_last_left_ticks = raw_left_ticks;
		m_last_right_ticks = raw_right_ticks;
		m_last_stamp = stamp;
		m_has_last_joint_ticks = true;
		m_has_last_stamp = true;
	}

	int32_t left_tick_delta = raw_left_ticks - m_last_left_ticks;
	int32_t right_tick_delta = raw_right_ticks - m_last_right_ticks;

	double left_delta_rad = static_cast<double>(left_tick_delta) * TICK_TO_RAD;
	double right_delta_rad = static_cast<double>(right_tick_delta) * TICK_TO_RAD;
	m_joint_positions_rad[0] += left_delta_rad;
	m_joint_positions_rad[1] += right_delta_rad;
	m_joint_velocities_mps[0] = static_cast<double>(raw_left_velocity) * RPM_TO_MS;
	m_joint_velocities_mps[1] = static_cast<double>(raw_right_velocity) * RPM_TO_MS;

	double delta_time = 0.0;
	if (m_has_last_stamp)
	{
		delta_time = (stamp - m_last_stamp).seconds();
	}

	if (delta_time <= 0.0)
	{
		m_last_left_ticks = raw_left_ticks;
		m_last_right_ticks = raw_right_ticks;
		m_last_stamp = stamp;
		return false;
	}

	double delta_s = m_wheel_radius_m * (right_delta_rad + left_delta_rad) / 2.0;
	double delta_theta = m_wheel_radius_m * (right_delta_rad - left_delta_rad) / m_wheel_separation_m;

	if (m_is_using_imu_for_yaw && has_imu_orientation)
	{
		double current_yaw = quaternionToYaw(
			imu_orientation_w,
			imu_orientation_x,
			imu_orientation_y,
			imu_orientation_z);

		if (m_has_last_imu_yaw)
		{
			delta_theta = normalizeAngle(current_yaw - m_last_imu_yaw_rad);
		}
		else
		{
			delta_theta = 0.0;
			m_has_last_imu_yaw = true;
		}

		m_last_imu_yaw_rad = current_yaw;
	}

	m_pose[0] += delta_s * std::cos(m_pose[2] + (delta_theta / 2.0));
	m_pose[1] += delta_s * std::sin(m_pose[2] + (delta_theta / 2.0));
	m_pose[2] = normalizeAngle(m_pose[2] + delta_theta);
	m_velocity[0] = delta_s / delta_time;
	m_velocity[1] = 0.0;
	m_velocity[2] = delta_theta / delta_time;

	m_last_left_ticks = raw_left_ticks;
	m_last_right_ticks = raw_right_ticks;
	m_last_stamp = stamp;
	return true;
}

std::array<double, 2> OdometryIntegrator::getJointPositionsRad() const
{
	return m_joint_positions_rad;
}

std::array<double, 2> OdometryIntegrator::getJointVelocitiesMps() const
{
	return m_joint_velocities_mps;
}

nav_msgs::msg::Odometry OdometryIntegrator::buildOdometryMessage(
	const rclcpp::Time &stamp,
	const std::string &odom_frame_id,
	const std::string &base_frame_id) const
{
	nav_msgs::msg::Odometry message;
	message.header.stamp = stamp;
	message.header.frame_id = odom_frame_id;
	message.child_frame_id = base_frame_id;
	message.pose.pose.position.x = m_pose[0];
	message.pose.pose.position.y = m_pose[1];
	message.pose.pose.position.z = 0.0;

	tf2::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, m_pose[2]);
	message.pose.pose.orientation.x = quaternion.x();
	message.pose.pose.orientation.y = quaternion.y();
	message.pose.pose.orientation.z = quaternion.z();
	message.pose.pose.orientation.w = quaternion.w();
	message.twist.twist.linear.x = m_velocity[0];
	message.twist.twist.linear.y = m_velocity[1];
	message.twist.twist.angular.z = m_velocity[2];
	return message;
}

geometry_msgs::msg::TransformStamped OdometryIntegrator::buildTransformMessage(
	const rclcpp::Time &stamp,
	const std::string &odom_frame_id,
	const std::string &base_frame_id) const
{
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = stamp;
	transform.header.frame_id = odom_frame_id;
	transform.child_frame_id = base_frame_id;
	transform.transform.translation.x = m_pose[0];
	transform.transform.translation.y = m_pose[1];
	transform.transform.translation.z = 0.0;

	tf2::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, m_pose[2]);
	transform.transform.rotation.x = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();
	transform.transform.rotation.w = quaternion.w();
	return transform;
}

double OdometryIntegrator::quaternionToYaw(
	double orientation_w,
	double orientation_x,
	double orientation_y,
	double orientation_z) const
{
	return std::atan2(
		2.0 * ((orientation_w * orientation_z) + (orientation_x * orientation_y)),
		1.0 - (2.0 * ((orientation_y * orientation_y) + (orientation_z * orientation_z))));
}

double OdometryIntegrator::normalizeAngle(double angle_rad) const
{
	static constexpr double PI = 3.14159265358979323846;

	while (angle_rad > PI)
	{
		angle_rad -= 2.0 * PI;
	}

	while (angle_rad < -PI)
	{
		angle_rad += 2.0 * PI;
	}

	return angle_rad;
}
