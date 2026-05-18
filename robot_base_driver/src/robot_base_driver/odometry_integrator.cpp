#include "robot_base_driver/odometry_integrator.hpp"

using namespace robot::hw::base;

OdometryIntegrator::OdometryIntegrator(double wheel_separation_m, double wheel_radius_m, bool use_imu_for_yaw)
: wheel_separation_m_(wheel_separation_m),
	wheel_radius_m_(wheel_radius_m),
	use_imu_for_yaw_(use_imu_for_yaw),
	has_last_joint_ticks_(false),
	has_last_stamp_(false),
	has_last_imu_yaw_(false),
	last_left_ticks_(0),
	last_right_ticks_(0),
	last_stamp_(0, 0, RCL_ROS_TIME),
	last_imu_yaw_rad_(0.0),
	joint_positions_rad_({0.0, 0.0}),
	joint_velocities_mps_({0.0, 0.0}),
	pose_({0.0, 0.0, 0.0}),
	velocity_({0.0, 0.0, 0.0})
{
}

OdometryIntegrator::~OdometryIntegrator()
{
}

void OdometryIntegrator::reset()
{
	has_last_joint_ticks_ = false;
	has_last_stamp_ = false;
	has_last_imu_yaw_ = false;
	last_left_ticks_ = 0;
	last_right_ticks_ = 0;
	last_imu_yaw_rad_ = 0.0;
	joint_positions_rad_ = {0.0, 0.0};
	joint_velocities_mps_ = {0.0, 0.0};
	pose_ = {0.0, 0.0, 0.0};
	velocity_ = {0.0, 0.0, 0.0};
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
	if (!has_last_joint_ticks_)
	{
		last_left_ticks_ = raw_left_ticks;
		last_right_ticks_ = raw_right_ticks;
		last_stamp_ = stamp;
		has_last_joint_ticks_ = true;
		has_last_stamp_ = true;
	}

	int32_t left_tick_delta = raw_left_ticks - last_left_ticks_;
	int32_t right_tick_delta = raw_right_ticks - last_right_ticks_;

	double left_delta_rad = static_cast<double>(left_tick_delta) * TICK_TO_RAD;
	double right_delta_rad = static_cast<double>(right_tick_delta) * TICK_TO_RAD;
	joint_positions_rad_[0] += left_delta_rad;
	joint_positions_rad_[1] += right_delta_rad;
	joint_velocities_mps_[0] = static_cast<double>(raw_left_velocity) * RPM_TO_MS;
	joint_velocities_mps_[1] = static_cast<double>(raw_right_velocity) * RPM_TO_MS;

	double delta_time = 0.0;
	if (has_last_stamp_)
	{
		delta_time = (stamp - last_stamp_).seconds();
	}

	if (delta_time <= 0.0)
	{
		last_left_ticks_ = raw_left_ticks;
		last_right_ticks_ = raw_right_ticks;
		last_stamp_ = stamp;
		return false;
	}

	double delta_s = wheel_radius_m_ * (right_delta_rad + left_delta_rad) / 2.0;
	double delta_theta = wheel_radius_m_ * (right_delta_rad - left_delta_rad) / wheel_separation_m_;

	if (use_imu_for_yaw_ && has_imu_orientation)
	{
		double current_yaw = quaternionToYaw(
			imu_orientation_w,
			imu_orientation_x,
			imu_orientation_y,
			imu_orientation_z);

		if (has_last_imu_yaw_)
		{
			delta_theta = normalizeAngle(current_yaw - last_imu_yaw_rad_);
		}
		else
		{
			delta_theta = 0.0;
			has_last_imu_yaw_ = true;
		}

		last_imu_yaw_rad_ = current_yaw;
	}

	pose_[0] += delta_s * std::cos(pose_[2] + (delta_theta / 2.0));
	pose_[1] += delta_s * std::sin(pose_[2] + (delta_theta / 2.0));
	pose_[2] = normalizeAngle(pose_[2] + delta_theta);
	velocity_[0] = delta_s / delta_time;
	velocity_[1] = 0.0;
	velocity_[2] = delta_theta / delta_time;

	last_left_ticks_ = raw_left_ticks;
	last_right_ticks_ = raw_right_ticks;
	last_stamp_ = stamp;
	return true;
}

std::array<double, 2> OdometryIntegrator::getJointPositionsRad() const
{
	return joint_positions_rad_;
}

std::array<double, 2> OdometryIntegrator::getJointVelocitiesMps() const
{
	return joint_velocities_mps_;
}

nav_msgs::msg::Odometry OdometryIntegrator::buildOdometryMessage(const rclcpp::Time &stamp, const std::string &odom_frame_id, const std::string &base_frame_id) const
{
	nav_msgs::msg::Odometry message;
	message.header.stamp = stamp;
	message.header.frame_id = odom_frame_id;
	message.child_frame_id = base_frame_id;
	message.pose.pose.position.x = pose_[0];
	message.pose.pose.position.y = pose_[1];
	message.pose.pose.position.z = 0.0;

	tf2::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, pose_[2]);
	message.pose.pose.orientation.x = quaternion.x();
	message.pose.pose.orientation.y = quaternion.y();
	message.pose.pose.orientation.z = quaternion.z();
	message.pose.pose.orientation.w = quaternion.w();
	message.twist.twist.linear.x = velocity_[0];
	message.twist.twist.linear.y = velocity_[1];
	message.twist.twist.angular.z = velocity_[2];
	return message;
}

geometry_msgs::msg::TransformStamped OdometryIntegrator::buildTransformMessage(const rclcpp::Time &stamp, const std::string &odom_frame_id, const std::string &base_frame_id) const
{
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = stamp;
	transform.header.frame_id = odom_frame_id;
	transform.child_frame_id = base_frame_id;
	transform.transform.translation.x = pose_[0];
	transform.transform.translation.y = pose_[1];
	transform.transform.translation.z = 0.0;

	tf2::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, pose_[2]);
	transform.transform.rotation.x = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();
	transform.transform.rotation.w = quaternion.w();
	return transform;
}

double OdometryIntegrator::quaternionToYaw(double orientation_w, double orientation_x, double orientation_y, double orientation_z) const
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
