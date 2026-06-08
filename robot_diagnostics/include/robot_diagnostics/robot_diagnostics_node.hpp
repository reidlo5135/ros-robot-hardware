#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robot::hw::diagnostics
{

enum class CheckStatus
{
	Pass,
	Warn,
	Fail,
};

struct TfEdgeExpectation
{
	std::string parent_frame;
	std::string child_frame;
};

struct StaticTfExpectation
{
	TfEdgeExpectation edge;
	double translation_x_m{0.0};
	double translation_y_m{0.0};
	double translation_z_m{0.0};
	double roll_rad{0.0};
	double pitch_rad{0.0};
	double yaw_rad{0.0};
};

struct ScanExpectation
{
	std::string frame_id{"base_scan"};
	std::size_t samples{400U};
	double angle_min_rad{0.0};
	double angle_max_rad{6.283185307179586};
	double angle_increment_rad{0.015707963267948967};
	int front_index{0};
	int left_index{100};
	int rear_index{200};
	int right_index{300};
};

struct OdomExpectation
{
	std::string frame_id{"odom"};
	std::string child_frame_id{"base_footprint"};
};

struct ImuExpectation
{
	std::string frame_id{"imu_link"};
};

struct ContractTolerances
{
	double static_translation_m{0.005};
	double static_rotation_rad{0.010};
	double scan_angle_rad{0.001};
	double scan_increment_rad{0.0001};
	double odom_tf_translation_m{0.020};
	double odom_tf_yaw_rad{0.020};
};

struct RobotContract
{
	std::string schema{"v1"};
	std::string profile{"tb3_burger_opencr_lds03"};
	std::vector<TfEdgeExpectation> dynamic_tf_edges;
	std::vector<StaticTfExpectation> static_tf_edges;
	std::vector<TfEdgeExpectation> optional_external_tf_edges;
	std::map<std::string, std::string> topics;
	ScanExpectation scan;
	OdomExpectation odom;
	ImuExpectation imu;
	std::vector<std::string> required_joint_names;
	ContractTolerances tolerances;
};

struct TransformRecord
{
	geometry_msgs::msg::TransformStamped transform;
	rclcpp::Time received_time;
	bool is_static{false};
};

struct ScanBaseline
{
	bool initialized{false};
	std::size_t ranges_size{0U};
	double angle_min_rad{0.0};
	double angle_max_rad{0.0};
	double angle_increment_rad{0.0};
};

struct CheckResult
{
	std::string event;
	CheckStatus status{CheckStatus::Pass};
	std::string reason{"none"};
};

class RobotDiagnosticsNode : public rclcpp::Node
{
public:
	explicit RobotDiagnosticsNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
	void declareParameters();
	void loadParameters();
	void loadContractFile();
	void setupSubscriptions();
	void handleTfMessage(const tf2_msgs::msg::TFMessage &message, bool is_static);
	void handleScan(const sensor_msgs::msg::LaserScan &message);
	void handleOdom(const nav_msgs::msg::Odometry &message);
	void handleImu(const sensor_msgs::msg::Imu &message);
	void handleJointStates(const sensor_msgs::msg::JointState &message);
	void runSummary();
	std::vector<CheckResult> checkTopicEndpoints();
	std::vector<CheckResult> checkRequiredTfEdges();
	std::vector<CheckResult> checkStaticTfGeometry();
	std::vector<CheckResult> checkOptionalExternalTf();
	CheckResult checkOdomFrames();
	CheckResult checkOdomTfConsistency();
	CheckResult checkScanGeometry();
	CheckResult checkImuFrame();
	CheckResult checkJointStates();
	void logContractConfig() const;
	void logCheck(const std::string &event, CheckStatus status, const std::string &fields, const std::string &reason) const;
	void logSummary(const std::vector<CheckResult> &results) const;
	CheckStatus aggregateStatus(const std::vector<CheckResult> &results) const;
	bool hasTransformEdge(const TfEdgeExpectation &edge, bool is_static) const;
	std::optional<TransformRecord> getTransformRecord(const TfEdgeExpectation &edge, bool is_static) const;
	std::optional<geometry_msgs::msg::TransformStamped> lookupLatestTransform(
		const std::string &parent_frame,
		const std::string &child_frame) const;
	bool appearsRobotHardwareMapOdomPublisher(bool is_static_edge) const;
	std::string describeTfPublishers(const std::string &topic_name) const;
	std::string topicName(const std::string &key) const;
	std::string makeEdgeKey(const std::string &parent_frame, const std::string &child_frame) const;
	std::string normalizeFrameId(const std::string &frame_id) const;
	std::string sanitizeLogValue(const std::string &value) const;
	std::string statusToText(CheckStatus status) const;
	std::string statusToResult(CheckStatus status) const;
	double normalizeAngle(double angle_rad) const;
	double yawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion) const;
	void rpyFromQuaternion(
		const geometry_msgs::msg::Quaternion &quaternion,
		double &roll_rad,
		double &pitch_rad,
		double &yaw_rad) const;

	RobotContract contract_;
	std::string contract_file_;
	double summary_period_sec_{2.0};
	bool once_{false};

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
	rclcpp::TimerBase::SharedPtr summary_timer_;

	std::map<std::string, TransformRecord> dynamic_transforms_;
	std::map<std::string, TransformRecord> static_transforms_;
	std::optional<sensor_msgs::msg::LaserScan> latest_scan_;
	std::optional<nav_msgs::msg::Odometry> latest_odom_;
	std::optional<sensor_msgs::msg::Imu> latest_imu_;
	std::optional<sensor_msgs::msg::JointState> latest_joint_states_;
	ScanBaseline scan_baseline_;
	bool scan_geometry_unstable_{false};
	std::string scan_instability_reason_{"none"};
};

}  // namespace robot::hw::diagnostics