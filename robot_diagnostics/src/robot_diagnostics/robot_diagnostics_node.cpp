#include "robot_diagnostics/robot_diagnostics_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/qos.hpp>
#include <yaml-cpp/yaml.h>

namespace robot::hw::diagnostics
{
namespace
{
constexpr double TWO_PI = 6.28318530717958647692;

RobotContract makeDefaultContract()
{
	RobotContract contract;
	contract.dynamic_tf_edges.push_back({"odom", "base_footprint"});
	contract.static_tf_edges.push_back({{"base_footprint", "base_link"}, 0.0, 0.0, 0.010, 0.0, 0.0, 0.0});
	contract.static_tf_edges.push_back({{"base_link", "base_scan"}, -0.032, 0.0, 0.172, 0.0, 0.0, 0.0});
	contract.static_tf_edges.push_back({{"base_link", "imu_link"}, -0.032, 0.0, 0.068, 0.0, 0.0, 0.0});
	contract.optional_external_tf_edges.push_back({"map", "odom"});
	contract.topics = {
		{"scan", "/scan"},
		{"odom", "/odom"},
		{"imu", "/imu"},
		{"joint_states", "/joint_states"},
		{"cmd_vel", "/cmd_vel"},
	};
	contract.scan.angle_max_rad = TWO_PI;
	contract.scan.angle_increment_rad = TWO_PI / static_cast<double>(contract.scan.samples);
	contract.required_joint_names = {"wheel_left_joint", "wheel_right_joint"};
	return contract;
}

std::string yamlString(const YAML::Node &node, const char *key, const std::string &default_value)
{
	if (!node || !node[key])
	{
		return default_value;
	}
	return node[key].as<std::string>();
}

double yamlDouble(const YAML::Node &node, const char *key, double default_value)
{
	if (!node || !node[key])
	{
		return default_value;
	}
	return node[key].as<double>();
}

int yamlInt(const YAML::Node &node, const char *key, int default_value)
{
	if (!node || !node[key])
	{
		return default_value;
	}
	return node[key].as<int>();
}

TfEdgeExpectation loadTfEdge(const YAML::Node &node)
{
	TfEdgeExpectation edge;
	edge.parent_frame = yamlString(node, "parent", "");
	edge.child_frame = yamlString(node, "child", "");
	if (edge.parent_frame.empty() || edge.child_frame.empty())
	{
		throw std::runtime_error("TF edge requires parent and child fields");
	}
	return edge;
}

std::vector<TfEdgeExpectation> loadTfEdges(const YAML::Node &node)
{
	std::vector<TfEdgeExpectation> edges;
	if (!node)
	{
		return edges;
	}
	for (const auto &item : node)
	{
		edges.push_back(loadTfEdge(item));
	}
	return edges;
}

std::vector<StaticTfExpectation> loadStaticTfEdges(const YAML::Node &node)
{
	std::vector<StaticTfExpectation> edges;
	if (!node)
	{
		return edges;
	}
	for (const auto &item : node)
	{
		StaticTfExpectation expectation;
		expectation.edge = loadTfEdge(item);
		const YAML::Node translation = item["translation"];
		const YAML::Node rotation = item["rotation_rpy"];
		expectation.translation_x_m = yamlDouble(translation, "x", expectation.translation_x_m);
		expectation.translation_y_m = yamlDouble(translation, "y", expectation.translation_y_m);
		expectation.translation_z_m = yamlDouble(translation, "z", expectation.translation_z_m);
		expectation.roll_rad = yamlDouble(rotation, "roll", expectation.roll_rad);
		expectation.pitch_rad = yamlDouble(rotation, "pitch", expectation.pitch_rad);
		expectation.yaw_rad = yamlDouble(rotation, "yaw", expectation.yaw_rad);
		edges.push_back(expectation);
	}
	return edges;
}

std::string joinNames(const std::vector<std::string> &names)
{
	if (names.empty())
	{
		return "none";
	}
	std::ostringstream stream;
	for (std::size_t index = 0U; index < names.size(); ++index)
	{
		if (index > 0U)
		{
			stream << ",";
		}
		stream << names[index];
	}
	return stream.str();
}

std::string formatDouble(double value, int precision = 6)
{
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}

}  // namespace

RobotDiagnosticsNode::RobotDiagnosticsNode(const rclcpp::NodeOptions &options)
: Node("robot_diagnostics", options),
	contract_(makeDefaultContract()),
	tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock()))
{
	declareParameters();
	loadParameters();
	loadContractFile();
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	setupSubscriptions();
	logContractConfig();

	const auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::duration<double>(summary_period_sec_));
	summary_timer_ = create_wall_timer(timer_period, [this]() { runSummary(); });
}

void RobotDiagnosticsNode::declareParameters()
{
	declare_parameter<std::string>("contract_file", "");
	declare_parameter<double>("summary_period_sec", 2.0);
	declare_parameter<bool>("once", false);
}

void RobotDiagnosticsNode::loadParameters()
{
	contract_file_ = get_parameter("contract_file").as_string();
	summary_period_sec_ = get_parameter("summary_period_sec").as_double();
	once_ = get_parameter("once").as_bool();
	if (summary_period_sec_ < 0.2)
	{
		summary_period_sec_ = 0.2;
	}
}

void RobotDiagnosticsNode::loadContractFile()
{
	if (contract_file_.empty())
	{
		return;
	}

	try
	{
		RobotContract loaded_contract = makeDefaultContract();
		const YAML::Node root = YAML::LoadFile(contract_file_);
		loaded_contract.schema = yamlString(root, "schema", loaded_contract.schema);
		loaded_contract.profile = yamlString(root, "profile", loaded_contract.profile);

		const YAML::Node tf = root["tf"];
		if (tf && tf["dynamic"])
		{
			loaded_contract.dynamic_tf_edges = loadTfEdges(tf["dynamic"]);
		}
		if (tf && tf["static"])
		{
			loaded_contract.static_tf_edges = loadStaticTfEdges(tf["static"]);
		}
		if (tf && tf["optional_external"])
		{
			loaded_contract.optional_external_tf_edges = loadTfEdges(tf["optional_external"]);
		}

		const YAML::Node topics = root["topics"];
		for (const auto &topic_pair : loaded_contract.topics)
		{
			loaded_contract.topics[topic_pair.first] = yamlString(topics, topic_pair.first.c_str(), topic_pair.second);
		}

		const YAML::Node scan = root["scan"];
		loaded_contract.scan.frame_id = yamlString(scan, "frame_id", loaded_contract.scan.frame_id);
		loaded_contract.scan.samples = static_cast<std::size_t>(yamlInt(scan, "samples", static_cast<int>(loaded_contract.scan.samples)));
		loaded_contract.scan.angle_min_rad = yamlDouble(scan, "angle_min", loaded_contract.scan.angle_min_rad);
		loaded_contract.scan.angle_max_rad = yamlDouble(scan, "angle_max", loaded_contract.scan.angle_max_rad);
		loaded_contract.scan.angle_increment_rad = yamlDouble(scan, "angle_increment", loaded_contract.scan.angle_increment_rad);
		const YAML::Node cardinal_indices = scan["cardinal_indices"];
		loaded_contract.scan.front_index = yamlInt(cardinal_indices, "front", loaded_contract.scan.front_index);
		loaded_contract.scan.left_index = yamlInt(cardinal_indices, "left", loaded_contract.scan.left_index);
		loaded_contract.scan.rear_index = yamlInt(cardinal_indices, "rear", loaded_contract.scan.rear_index);
		loaded_contract.scan.right_index = yamlInt(cardinal_indices, "right", loaded_contract.scan.right_index);

		const YAML::Node odom = root["odom"];
		loaded_contract.odom.frame_id = yamlString(odom, "frame_id", loaded_contract.odom.frame_id);
		loaded_contract.odom.child_frame_id = yamlString(odom, "child_frame_id", loaded_contract.odom.child_frame_id);

		const YAML::Node imu = root["imu"];
		loaded_contract.imu.frame_id = yamlString(imu, "frame_id", loaded_contract.imu.frame_id);

		const YAML::Node joint_states = root["joint_states"];
		if (joint_states && joint_states["required_joints"])
		{
			loaded_contract.required_joint_names.clear();
			for (const auto &joint_name : joint_states["required_joints"])
			{
				loaded_contract.required_joint_names.push_back(joint_name.as<std::string>());
			}
		}

		const YAML::Node tolerances = root["tolerances"];
		loaded_contract.tolerances.static_translation_m = yamlDouble(
			tolerances, "static_translation_m", loaded_contract.tolerances.static_translation_m);
		loaded_contract.tolerances.static_rotation_rad = yamlDouble(
			tolerances, "static_rotation_rad", loaded_contract.tolerances.static_rotation_rad);
		loaded_contract.tolerances.scan_angle_rad = yamlDouble(
			tolerances, "scan_angle_rad", loaded_contract.tolerances.scan_angle_rad);
		loaded_contract.tolerances.scan_increment_rad = yamlDouble(
			tolerances, "scan_increment_rad", loaded_contract.tolerances.scan_increment_rad);
		loaded_contract.tolerances.odom_tf_translation_m = yamlDouble(
			tolerances, "odom_tf_translation_m", loaded_contract.tolerances.odom_tf_translation_m);
		loaded_contract.tolerances.odom_tf_yaw_rad = yamlDouble(
			tolerances, "odom_tf_yaw_rad", loaded_contract.tolerances.odom_tf_yaw_rad);

		contract_ = std::move(loaded_contract);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			get_logger(),
			"ROBOT_HW_LOG schema=v1 tag=DIAG component=diagnostics event=contract_load node=%s namespace=%s contract_file=%s result=fail reason=%s",
			get_name(),
			sanitizeLogValue(get_namespace()).c_str(),
			sanitizeLogValue(contract_file_).c_str(),
			sanitizeLogValue(exception.what()).c_str());
	}
}

void RobotDiagnosticsNode::setupSubscriptions()
{
	tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
		"/tf",
		tf2_ros::DynamicListenerQoS(),
		[this](const tf2_msgs::msg::TFMessage::SharedPtr message) { handleTfMessage(*message, false); });
	tf_static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
		"/tf_static",
		tf2_ros::StaticListenerQoS(),
		[this](const tf2_msgs::msg::TFMessage::SharedPtr message) { handleTfMessage(*message, true); });
	scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
		topicName("scan"),
		rclcpp::SensorDataQoS(),
		[this](const sensor_msgs::msg::LaserScan::SharedPtr message) { handleScan(*message); });
	odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		topicName("odom"),
		rclcpp::SystemDefaultsQoS(),
		[this](const nav_msgs::msg::Odometry::SharedPtr message) { handleOdom(*message); });
	imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
		topicName("imu"),
		rclcpp::SensorDataQoS(),
		[this](const sensor_msgs::msg::Imu::SharedPtr message) { handleImu(*message); });
	joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
		topicName("joint_states"),
		rclcpp::SystemDefaultsQoS(),
		[this](const sensor_msgs::msg::JointState::SharedPtr message) { handleJointStates(*message); });
}

void RobotDiagnosticsNode::handleTfMessage(const tf2_msgs::msg::TFMessage &message, bool is_static)
{
	for (const auto &transform : message.transforms)
	{
		geometry_msgs::msg::TransformStamped normalized_transform = transform;
		normalized_transform.header.frame_id = normalizeFrameId(transform.header.frame_id);
		normalized_transform.child_frame_id = normalizeFrameId(transform.child_frame_id);
		if (normalized_transform.header.frame_id.empty() || normalized_transform.child_frame_id.empty())
		{
			continue;
		}

		TransformRecord record;
		record.transform = normalized_transform;
		record.received_time = now();
		record.is_static = is_static;
		const std::string key = makeEdgeKey(normalized_transform.header.frame_id, normalized_transform.child_frame_id);
		if (is_static)
		{
			static_transforms_[key] = record;
		}
		else
		{
			dynamic_transforms_[key] = record;
		}
	}
}

void RobotDiagnosticsNode::handleScan(const sensor_msgs::msg::LaserScan &message)
{
	latest_scan_ = message;
	if (!scan_baseline_.initialized)
	{
		scan_baseline_.initialized = true;
		scan_baseline_.ranges_size = message.ranges.size();
		scan_baseline_.angle_min_rad = message.angle_min;
		scan_baseline_.angle_max_rad = message.angle_max;
		scan_baseline_.angle_increment_rad = message.angle_increment;
		return;
	}

	const bool size_changed = scan_baseline_.ranges_size != message.ranges.size();
	const bool angle_min_changed = std::fabs(scan_baseline_.angle_min_rad - message.angle_min) > contract_.tolerances.scan_angle_rad;
	const bool angle_max_changed = std::fabs(scan_baseline_.angle_max_rad - message.angle_max) > contract_.tolerances.scan_angle_rad;
	const bool increment_changed = std::fabs(scan_baseline_.angle_increment_rad - message.angle_increment) > contract_.tolerances.scan_increment_rad;
	if (size_changed || angle_min_changed || angle_max_changed || increment_changed)
	{
		scan_geometry_unstable_ = true;
		scan_instability_reason_ = "scan_geometry_changed";
	}
}

void RobotDiagnosticsNode::handleOdom(const nav_msgs::msg::Odometry &message)
{
	latest_odom_ = message;
}

void RobotDiagnosticsNode::handleImu(const sensor_msgs::msg::Imu &message)
{
	latest_imu_ = message;
}

void RobotDiagnosticsNode::handleJointStates(const sensor_msgs::msg::JointState &message)
{
	latest_joint_states_ = message;
}

void RobotDiagnosticsNode::runSummary()
{
	std::vector<CheckResult> results;
	auto append_results = [&results](const std::vector<CheckResult> &new_results) {
		results.insert(results.end(), new_results.begin(), new_results.end());
	};
	append_results(checkTopicEndpoints());
	append_results(checkRequiredTfEdges());
	append_results(checkStaticTfGeometry());
	append_results(checkOptionalExternalTf());
	results.push_back(checkOdomFrames());
	results.push_back(checkOdomTfConsistency());
	results.push_back(checkScanGeometry());
	results.push_back(checkImuFrame());
	results.push_back(checkJointStates());
	logSummary(results);

	if (once_)
	{
		RCLCPP_INFO(get_logger(), "one-shot diagnostics completed");
		rclcpp::shutdown();
	}
}

std::vector<CheckResult> RobotDiagnosticsNode::checkTopicEndpoints()
{
	std::vector<CheckResult> results;
	const std::vector<std::pair<std::string, std::string>> publisher_topics = {
		{"scan", topicName("scan")},
		{"odom", topicName("odom")},
		{"imu", topicName("imu")},
		{"joint_states", topicName("joint_states")},
	};

	for (const auto &topic : publisher_topics)
	{
		const auto publishers = get_publishers_info_by_topic(topic.second);
		const auto subscriptions = get_subscriptions_info_by_topic(topic.second);
		const CheckStatus status = publishers.empty() ? CheckStatus::Fail : CheckStatus::Pass;
		const std::string reason = publishers.empty() ? "no_publisher" : "none";
		std::ostringstream fields;
		fields << "topic=" << sanitizeLogValue(topic.second)
			   << " role=publisher"
			   << " publisher_count=" << publishers.size()
			   << " subscription_count=" << subscriptions.size();
		logCheck("topic_endpoint", status, fields.str(), reason);
		results.push_back({"topic_endpoint." + topic.first, status, reason});
	}

	const std::string cmd_vel_topic = topicName("cmd_vel");
	const auto cmd_publishers = get_publishers_info_by_topic(cmd_vel_topic);
	const auto cmd_subscriptions = get_subscriptions_info_by_topic(cmd_vel_topic);
	const CheckStatus cmd_status = cmd_subscriptions.empty() ? CheckStatus::Fail : CheckStatus::Pass;
	const std::string cmd_reason = cmd_subscriptions.empty() ? "no_subscriber" : "none";
	std::ostringstream cmd_fields;
	cmd_fields << "topic=" << sanitizeLogValue(cmd_vel_topic)
			   << " role=subscriber"
			   << " publisher_count=" << cmd_publishers.size()
			   << " subscription_count=" << cmd_subscriptions.size();
	logCheck("topic_endpoint", cmd_status, cmd_fields.str(), cmd_reason);
	results.push_back({"topic_endpoint.cmd_vel", cmd_status, cmd_reason});

	return results;
}

std::vector<CheckResult> RobotDiagnosticsNode::checkRequiredTfEdges()
{
	std::vector<CheckResult> results;
	for (const auto &edge : contract_.dynamic_tf_edges)
	{
		const bool observed = hasTransformEdge(edge, false);
		const CheckStatus status = observed ? CheckStatus::Pass : CheckStatus::Fail;
		const std::string reason = observed ? "none" : "missing_dynamic_tf_edge";
		std::ostringstream fields;
		fields << "parent_frame=" << sanitizeLogValue(edge.parent_frame)
			   << " child_frame=" << sanitizeLogValue(edge.child_frame)
			   << " expected_source=dynamic observed=" << (observed ? "true" : "false");
		logCheck("tf_edge", status, fields.str(), reason);
		results.push_back({"tf_edge." + edge.parent_frame + "." + edge.child_frame, status, reason});
	}

	for (const auto &edge : contract_.static_tf_edges)
	{
		const bool observed = hasTransformEdge(edge.edge, true);
		const CheckStatus status = observed ? CheckStatus::Pass : CheckStatus::Fail;
		const std::string reason = observed ? "none" : "missing_static_tf_edge";
		std::ostringstream fields;
		fields << "parent_frame=" << sanitizeLogValue(edge.edge.parent_frame)
			   << " child_frame=" << sanitizeLogValue(edge.edge.child_frame)
			   << " expected_source=static observed=" << (observed ? "true" : "false");
		logCheck("tf_edge", status, fields.str(), reason);
		results.push_back({"tf_edge." + edge.edge.parent_frame + "." + edge.edge.child_frame, status, reason});
	}
	return results;
}

std::vector<CheckResult> RobotDiagnosticsNode::checkStaticTfGeometry()
{
	std::vector<CheckResult> results;
	for (const auto &expectation : contract_.static_tf_edges)
	{
		const auto record = getTransformRecord(expectation.edge, true);
		if (!record)
		{
			std::ostringstream fields;
			fields << "parent_frame=" << sanitizeLogValue(expectation.edge.parent_frame)
				   << " child_frame=" << sanitizeLogValue(expectation.edge.child_frame);
			logCheck("tf_static_geometry", CheckStatus::Fail, fields.str(), "missing_static_tf_edge");
			results.push_back({"tf_static_geometry." + expectation.edge.child_frame, CheckStatus::Fail, "missing_static_tf_edge"});
			continue;
		}

		const auto &translation = record->transform.transform.translation;
		double roll_rad = 0.0;
		double pitch_rad = 0.0;
		double yaw_rad = 0.0;
		rpyFromQuaternion(record->transform.transform.rotation, roll_rad, pitch_rad, yaw_rad);
		const double dx_m = translation.x - expectation.translation_x_m;
		const double dy_m = translation.y - expectation.translation_y_m;
		const double dz_m = translation.z - expectation.translation_z_m;
		const double translation_error_m = std::sqrt((dx_m * dx_m) + (dy_m * dy_m) + (dz_m * dz_m));
		const double roll_error_rad = std::fabs(normalizeAngle(roll_rad - expectation.roll_rad));
		const double pitch_error_rad = std::fabs(normalizeAngle(pitch_rad - expectation.pitch_rad));
		const double yaw_error_rad = std::fabs(normalizeAngle(yaw_rad - expectation.yaw_rad));
		const double rotation_error_rad = std::max({roll_error_rad, pitch_error_rad, yaw_error_rad});
		const bool translation_ok = translation_error_m <= contract_.tolerances.static_translation_m;
		const bool rotation_ok = rotation_error_rad <= contract_.tolerances.static_rotation_rad;
		const CheckStatus status = (translation_ok && rotation_ok) ? CheckStatus::Pass : CheckStatus::Fail;
		const std::string reason = status == CheckStatus::Pass ? "none" : "static_geometry_mismatch";

		std::ostringstream fields;
		fields << "parent_frame=" << sanitizeLogValue(expectation.edge.parent_frame)
			   << " child_frame=" << sanitizeLogValue(expectation.edge.child_frame)
			   << " translation_error_m=" << formatDouble(translation_error_m)
			   << " rotation_error_rad=" << formatDouble(rotation_error_rad)
			   << " tolerance_translation_m=" << formatDouble(contract_.tolerances.static_translation_m)
			   << " tolerance_rotation_rad=" << formatDouble(contract_.tolerances.static_rotation_rad);
		logCheck("tf_static_geometry", status, fields.str(), reason);
		results.push_back({"tf_static_geometry." + expectation.edge.child_frame, status, reason});
	}
	return results;
}

std::vector<CheckResult> RobotDiagnosticsNode::checkOptionalExternalTf()
{
	std::vector<CheckResult> results;
	for (const auto &edge : contract_.optional_external_tf_edges)
	{
		const bool dynamic_observed = hasTransformEdge(edge, false);
		const bool static_observed = hasTransformEdge(edge, true);
		const bool observed = dynamic_observed || static_observed;
		const bool suspected_robot_hw = observed && (appearsRobotHardwareMapOdomPublisher(false) || appearsRobotHardwareMapOdomPublisher(true));
		const CheckStatus status = suspected_robot_hw ? CheckStatus::Warn : CheckStatus::Pass;
		std::string reason = "absent_optional_external";
		if (observed && suspected_robot_hw)
		{
			reason = "robot_hardware_map_to_odom_candidate";
		}
		else if (observed)
		{
			reason = "external_or_unknown_map_to_odom_present";
		}

		std::ostringstream fields;
		fields << "parent_frame=" << sanitizeLogValue(edge.parent_frame)
			   << " child_frame=" << sanitizeLogValue(edge.child_frame)
			   << " optional=true observed=" << (observed ? "true" : "false")
			   << " dynamic_observed=" << (dynamic_observed ? "true" : "false")
			   << " static_observed=" << (static_observed ? "true" : "false")
			   << " tf_publishers=" << sanitizeLogValue(describeTfPublishers("/tf"))
			   << " tf_static_publishers=" << sanitizeLogValue(describeTfPublishers("/tf_static"));
		logCheck("tf_optional_external", status, fields.str(), reason);
		results.push_back({"tf_optional_external." + edge.parent_frame + "." + edge.child_frame, status, reason});
	}
	return results;
}

CheckResult RobotDiagnosticsNode::checkOdomFrames()
{
	if (!latest_odom_)
	{
		logCheck("odom_frame", CheckStatus::Fail, "topic=/odom", "no_odom_message");
		return {"odom_frame", CheckStatus::Fail, "no_odom_message"};
	}

	const std::string frame_id = normalizeFrameId(latest_odom_->header.frame_id);
	const std::string child_frame_id = normalizeFrameId(latest_odom_->child_frame_id);
	const bool frame_ok = frame_id == contract_.odom.frame_id;
	const bool child_ok = child_frame_id == contract_.odom.child_frame_id;
	const CheckStatus status = (frame_ok && child_ok) ? CheckStatus::Pass : CheckStatus::Fail;
	std::string reason = "none";
	if (!frame_ok && !child_ok)
	{
		reason = "odom_frame_and_child_mismatch";
	}
	else if (!frame_ok)
	{
		reason = "odom_frame_mismatch";
	}
	else if (!child_ok)
	{
		reason = "odom_child_frame_mismatch";
	}

	std::ostringstream fields;
	fields << "topic=" << sanitizeLogValue(topicName("odom"))
		   << " frame_id=" << sanitizeLogValue(frame_id)
		   << " expected_frame_id=" << sanitizeLogValue(contract_.odom.frame_id)
		   << " child_frame_id=" << sanitizeLogValue(child_frame_id)
		   << " expected_child_frame_id=" << sanitizeLogValue(contract_.odom.child_frame_id);
	logCheck("odom_frame", status, fields.str(), reason);
	return {"odom_frame", status, reason};
}

CheckResult RobotDiagnosticsNode::checkOdomTfConsistency()
{
	if (!latest_odom_)
	{
		logCheck("odom_tf_consistency", CheckStatus::Fail, "topic=/odom", "no_odom_message");
		return {"odom_tf_consistency", CheckStatus::Fail, "no_odom_message"};
	}

	const std::string parent_frame = normalizeFrameId(latest_odom_->header.frame_id);
	const std::string child_frame = normalizeFrameId(latest_odom_->child_frame_id);
	const auto &odom_stamp = latest_odom_->header.stamp;
	const rclcpp::Time odom_time(odom_stamp);

	std::optional<geometry_msgs::msg::TransformStamped> transform;
	std::string tf_lookup_mode = "stamp";
	std::string lookup_error = "none";
	try
	{
		transform = tf_buffer_->lookupTransform(parent_frame, child_frame, odom_time);
	}
	catch (const tf2::TransformException &exception)
	{
		lookup_error = exception.what();
		tf_lookup_mode = "unavailable";
		transform = lookupLatestTransform(parent_frame, child_frame);
		if (transform)
		{
			tf_lookup_mode = "latest_fallback";
		}
		else
		{
			const auto record = getTransformRecord({parent_frame, child_frame}, false);
			if (record)
			{
				transform = record->transform;
				tf_lookup_mode = "latest_fallback";
			}
		}
	}

	const auto latest_record = getTransformRecord({parent_frame, child_frame}, false);
	std::ostringstream fields;
	fields << "parent_frame=" << sanitizeLogValue(parent_frame)
		   << " child_frame=" << sanitizeLogValue(child_frame)
		   << " odom_stamp_sec=" << odom_stamp.sec
		   << " odom_stamp_nanosec=" << odom_stamp.nanosec
		   << " tf_lookup_mode=" << tf_lookup_mode;
	if (latest_record)
	{
		fields << " latest_available_sec=" << latest_record->transform.header.stamp.sec
			   << " latest_available_nanosec=" << latest_record->transform.header.stamp.nanosec;
	}

	if (!transform)
	{
		fields << " translation_error_m=unavailable"
			   << " yaw_error_rad=unavailable"
			   << " tolerance_translation_m=" << formatDouble(contract_.tolerances.odom_tf_translation_m)
			   << " tolerance_yaw_rad=" << formatDouble(contract_.tolerances.odom_tf_yaw_rad)
			   << " tf_lookup_error=" << sanitizeLogValue(lookup_error);
		logCheck("odom_tf_consistency", CheckStatus::Warn, fields.str(), "tf_unavailable_at_odom_stamp");
		return {"odom_tf_consistency", CheckStatus::Warn, "tf_unavailable_at_odom_stamp"};
	}

	const auto &pose_position = latest_odom_->pose.pose.position;
	const auto &tf_translation = transform->transform.translation;
	const double dx_m = pose_position.x - tf_translation.x;
	const double dy_m = pose_position.y - tf_translation.y;
	const double dz_m = pose_position.z - tf_translation.z;
	const double translation_error_m = std::sqrt((dx_m * dx_m) + (dy_m * dy_m) + (dz_m * dz_m));
	const double odom_yaw_rad = yawFromQuaternion(latest_odom_->pose.pose.orientation);
	const double tf_yaw_rad = yawFromQuaternion(transform->transform.rotation);
	const double yaw_error_rad = std::fabs(normalizeAngle(odom_yaw_rad - tf_yaw_rad));
	const bool translation_ok = translation_error_m <= contract_.tolerances.odom_tf_translation_m;
	const bool yaw_ok = yaw_error_rad <= contract_.tolerances.odom_tf_yaw_rad;
	CheckStatus status = CheckStatus::Pass;
	std::string reason = "none";
	if (tf_lookup_mode != "stamp")
	{
		status = CheckStatus::Warn;
		reason = "tf_unavailable_at_odom_stamp";
	}
	else if (!translation_ok || !yaw_ok)
	{
		status = CheckStatus::Warn;
		reason = "odom_pose_tf_mismatch";
	}

	fields << " translation_error_m=" << formatDouble(translation_error_m)
		   << " yaw_error_rad=" << formatDouble(yaw_error_rad)
		   << " tolerance_translation_m=" << formatDouble(contract_.tolerances.odom_tf_translation_m)
		   << " tolerance_yaw_rad=" << formatDouble(contract_.tolerances.odom_tf_yaw_rad)
		   << " tf_lookup_error=" << sanitizeLogValue(lookup_error);
	logCheck("odom_tf_consistency", status, fields.str(), reason);
	return {"odom_tf_consistency", status, reason};
}

CheckResult RobotDiagnosticsNode::checkScanGeometry()
{
	if (!latest_scan_)
	{
		logCheck("scan_geometry", CheckStatus::Fail, "topic=/scan", "no_scan_message");
		return {"scan_geometry", CheckStatus::Fail, "no_scan_message"};
	}

	const std::string frame_id = normalizeFrameId(latest_scan_->header.frame_id);
	const bool frame_ok = frame_id == contract_.scan.frame_id;
	const bool sample_count_ok = latest_scan_->ranges.size() == contract_.scan.samples;
	const bool angle_min_ok = std::fabs(latest_scan_->angle_min - contract_.scan.angle_min_rad) <= contract_.tolerances.scan_angle_rad;
	const bool angle_max_ok = std::fabs(latest_scan_->angle_max - contract_.scan.angle_max_rad) <= contract_.tolerances.scan_angle_rad;
	const bool increment_ok = std::fabs(latest_scan_->angle_increment - contract_.scan.angle_increment_rad) <= contract_.tolerances.scan_increment_rad;
	const auto index_in_range = [this](int index) {
		return index >= 0 && static_cast<std::size_t>(index) < contract_.scan.samples;
	};
	const bool cardinal_indices_ok = index_in_range(contract_.scan.front_index) && index_in_range(contract_.scan.left_index) &&
		index_in_range(contract_.scan.rear_index) && index_in_range(contract_.scan.right_index);

	std::vector<std::string> reasons;
	if (!frame_ok)
	{
		reasons.push_back("scan_frame_mismatch");
	}
	if (!sample_count_ok)
	{
		reasons.push_back("scan_sample_count_mismatch");
	}
	if (!angle_min_ok || !angle_max_ok || !increment_ok)
	{
		reasons.push_back("scan_angle_mismatch");
	}
	if (scan_geometry_unstable_)
	{
		reasons.push_back(scan_instability_reason_);
	}
	if (!cardinal_indices_ok)
	{
		reasons.push_back("cardinal_index_out_of_range");
	}

	const CheckStatus status = reasons.empty() ? CheckStatus::Pass : CheckStatus::Fail;
	const std::string reason = reasons.empty() ? "none" : joinNames(reasons);
	std::ostringstream fields;
	fields << "topic=" << sanitizeLogValue(topicName("scan"))
		   << " frame_id=" << sanitizeLogValue(frame_id)
		   << " expected_frame_id=" << sanitizeLogValue(contract_.scan.frame_id)
		   << " ranges=" << latest_scan_->ranges.size()
		   << " expected_ranges=" << contract_.scan.samples
		   << " angle_min_rad=" << formatDouble(latest_scan_->angle_min)
		   << " angle_max_rad=" << formatDouble(latest_scan_->angle_max)
		   << " angle_increment_rad=" << formatDouble(latest_scan_->angle_increment, 9)
		   << " expected_angle_min_rad=" << formatDouble(contract_.scan.angle_min_rad)
		   << " expected_angle_max_rad=" << formatDouble(contract_.scan.angle_max_rad)
		   << " expected_angle_increment_rad=" << formatDouble(contract_.scan.angle_increment_rad, 9)
		   << " front_index=" << contract_.scan.front_index
		   << " left_index=" << contract_.scan.left_index
		   << " rear_index=" << contract_.scan.rear_index
		   << " right_index=" << contract_.scan.right_index
		   << " stable=" << (scan_geometry_unstable_ ? "false" : "true");
	logCheck("scan_geometry", status, fields.str(), reason);
	return {"scan_geometry", status, reason};
}

CheckResult RobotDiagnosticsNode::checkImuFrame()
{
	if (!latest_imu_)
	{
		logCheck("imu_frame", CheckStatus::Fail, "topic=/imu", "no_imu_message");
		return {"imu_frame", CheckStatus::Fail, "no_imu_message"};
	}

	const std::string frame_id = normalizeFrameId(latest_imu_->header.frame_id);
	const bool frame_ok = frame_id == contract_.imu.frame_id;
	const CheckStatus status = frame_ok ? CheckStatus::Pass : CheckStatus::Fail;
	const std::string reason = frame_ok ? "none" : "imu_frame_mismatch";
	std::ostringstream fields;
	fields << "topic=" << sanitizeLogValue(topicName("imu"))
		   << " frame_id=" << sanitizeLogValue(frame_id)
		   << " expected_frame_id=" << sanitizeLogValue(contract_.imu.frame_id);
	logCheck("imu_frame", status, fields.str(), reason);
	return {"imu_frame", status, reason};
}

CheckResult RobotDiagnosticsNode::checkJointStates()
{
	if (!latest_joint_states_)
	{
		logCheck("joint_states", CheckStatus::Fail, "topic=/joint_states", "no_joint_states_message");
		return {"joint_states", CheckStatus::Fail, "no_joint_states_message"};
	}

	const std::set<std::string> observed_joint_names(latest_joint_states_->name.begin(), latest_joint_states_->name.end());
	std::vector<std::string> missing_joint_names;
	for (const auto &joint_name : contract_.required_joint_names)
	{
		if (observed_joint_names.find(joint_name) == observed_joint_names.end())
		{
			missing_joint_names.push_back(joint_name);
		}
	}

	const CheckStatus status = missing_joint_names.empty() ? CheckStatus::Pass : CheckStatus::Fail;
	const std::string reason = missing_joint_names.empty() ? "none" : "missing_required_wheel_joint";
	std::ostringstream fields;
	fields << "topic=" << sanitizeLogValue(topicName("joint_states"))
		   << " observed_count=" << observed_joint_names.size()
		   << " required_joints=" << sanitizeLogValue(joinNames(contract_.required_joint_names))
		   << " missing_joints=" << sanitizeLogValue(joinNames(missing_joint_names));
	logCheck("joint_states", status, fields.str(), reason);
	return {"joint_states", status, reason};
}

void RobotDiagnosticsNode::logContractConfig() const
{
	RCLCPP_INFO(
		get_logger(),
		"ROBOT_HW_LOG schema=v1 tag=DIAG component=diagnostics event=contract_config node=%s namespace=%s contract_file=%s schema=%s profile=%s summary_period_sec=%.3f once=%s scan_topic=%s odom_topic=%s imu_topic=%s joint_states_topic=%s cmd_vel_topic=%s result=configured",
		get_name(),
		sanitizeLogValue(get_namespace()).c_str(),
		sanitizeLogValue(contract_file_.empty() ? "built_in_default" : contract_file_).c_str(),
		sanitizeLogValue(contract_.schema).c_str(),
		sanitizeLogValue(contract_.profile).c_str(),
		summary_period_sec_,
		once_ ? "true" : "false",
		sanitizeLogValue(topicName("scan")).c_str(),
		sanitizeLogValue(topicName("odom")).c_str(),
		sanitizeLogValue(topicName("imu")).c_str(),
		sanitizeLogValue(topicName("joint_states")).c_str(),
		sanitizeLogValue(topicName("cmd_vel")).c_str());
}

void RobotDiagnosticsNode::logCheck(
	const std::string &event,
	CheckStatus status,
	const std::string &fields,
	const std::string &reason) const
{
	std::ostringstream line;
	line << "ROBOT_HW_LOG schema=v1 tag=DIAG component=diagnostics event=" << event
		 << " node=" << sanitizeLogValue(get_name())
		 << " namespace=" << sanitizeLogValue(get_namespace());
	if (!fields.empty())
	{
		line << " " << fields;
	}
	line << " result=" << statusToResult(status)
		 << " reason=" << sanitizeLogValue(reason);

	const std::string text = line.str();
	if (status == CheckStatus::Fail)
	{
		RCLCPP_ERROR(get_logger(), "%s", text.c_str());
	}
	else if (status == CheckStatus::Warn)
	{
		RCLCPP_WARN(get_logger(), "%s", text.c_str());
	}
	else
	{
		RCLCPP_INFO(get_logger(), "%s", text.c_str());
	}
}

void RobotDiagnosticsNode::logSummary(const std::vector<CheckResult> &results) const
{
	std::size_t pass_count = 0U;
	std::size_t warn_count = 0U;
	std::size_t fail_count = 0U;
	std::vector<std::string> detail_reasons;
	for (const auto &result : results)
	{
		if (result.status == CheckStatus::Fail)
		{
			++fail_count;
			detail_reasons.push_back(result.event + ":" + result.reason);
		}
		else if (result.status == CheckStatus::Warn)
		{
			++warn_count;
			detail_reasons.push_back(result.event + ":" + result.reason);
		}
		else
		{
			++pass_count;
		}
	}

	const CheckStatus overall_status = aggregateStatus(results);
	const std::string detail = detail_reasons.empty() ? "none" : joinNames(detail_reasons);
	std::ostringstream structured_fields;
	structured_fields << "checks=" << results.size()
				  << " pass=" << pass_count
				  << " warn=" << warn_count
				  << " fail=" << fail_count
				  << " detail=" << sanitizeLogValue(detail);
	logCheck("summary", overall_status, structured_fields.str(), detail_reasons.empty() ? "none" : "see_detail");

	std::ostringstream human_summary;
	human_summary << "Diagnostics summary: overall=" << statusToText(overall_status)
			  << " pass=" << pass_count
			  << " warn=" << warn_count
			  << " fail=" << fail_count;
	if (!detail_reasons.empty())
	{
		human_summary << " detail=" << detail;
	}

	const std::string human_text = human_summary.str();
	if (overall_status == CheckStatus::Fail)
	{
		RCLCPP_ERROR(get_logger(), "%s", human_text.c_str());
	}
	else if (overall_status == CheckStatus::Warn)
	{
		RCLCPP_WARN(get_logger(), "%s", human_text.c_str());
	}
	else
	{
		RCLCPP_INFO(get_logger(), "%s", human_text.c_str());
	}
}

CheckStatus RobotDiagnosticsNode::aggregateStatus(const std::vector<CheckResult> &results) const
{
	for (const auto &result : results)
	{
		if (result.status == CheckStatus::Fail)
		{
			return CheckStatus::Fail;
		}
	}
	for (const auto &result : results)
	{
		if (result.status == CheckStatus::Warn)
		{
			return CheckStatus::Warn;
		}
	}
	return CheckStatus::Pass;
}

bool RobotDiagnosticsNode::hasTransformEdge(const TfEdgeExpectation &edge, bool is_static) const
{
	return static_cast<bool>(getTransformRecord(edge, is_static));
}

std::optional<TransformRecord> RobotDiagnosticsNode::getTransformRecord(const TfEdgeExpectation &edge, bool is_static) const
{
	const std::string key = makeEdgeKey(edge.parent_frame, edge.child_frame);
	const auto &transforms = is_static ? static_transforms_ : dynamic_transforms_;
	const auto record = transforms.find(key);
	if (record == transforms.end())
	{
		return std::nullopt;
	}
	return record->second;
}

std::optional<geometry_msgs::msg::TransformStamped> RobotDiagnosticsNode::lookupLatestTransform(
	const std::string &parent_frame,
	const std::string &child_frame) const
{
	try
	{
		return tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
	}
	catch (const tf2::TransformException &)
	{
		return std::nullopt;
	}
}

bool RobotDiagnosticsNode::appearsRobotHardwareMapOdomPublisher(bool is_static_edge) const
{
	const std::string topic_name = is_static_edge ? "/tf_static" : "/tf";
	const auto publishers = get_publishers_info_by_topic(topic_name);
	if (publishers.size() != 1U)
	{
		return false;
	}
	const std::string node_name = publishers.front().node_name();
	return node_name.find("robot_base_driver") != std::string::npos ||
		node_name.find("robot_state_publisher") != std::string::npos ||
		node_name.find("robot_bringup") != std::string::npos ||
		node_name.find("robot_hardware") != std::string::npos;
}

std::string RobotDiagnosticsNode::describeTfPublishers(const std::string &topic_name) const
{
	std::vector<std::string> names;
	for (const auto &publisher : get_publishers_info_by_topic(topic_name))
	{
		std::string qualified_name = publisher.node_namespace();
		if (!qualified_name.empty() && qualified_name != "/")
		{
			qualified_name += "/";
		}
		else
		{
			qualified_name = "/";
		}
		qualified_name += publisher.node_name();
		names.push_back(qualified_name);
	}
	return joinNames(names);
}

std::string RobotDiagnosticsNode::topicName(const std::string &key) const
{
	const auto topic = contract_.topics.find(key);
	if (topic == contract_.topics.end())
	{
		return "/" + key;
	}
	return topic->second;
}

std::string RobotDiagnosticsNode::makeEdgeKey(const std::string &parent_frame, const std::string &child_frame) const
{
	return normalizeFrameId(parent_frame) + "->" + normalizeFrameId(child_frame);
}

std::string RobotDiagnosticsNode::normalizeFrameId(const std::string &frame_id) const
{
	std::string normalized = frame_id;
	while (!normalized.empty() && normalized.front() == '/')
	{
		normalized.erase(normalized.begin());
	}
	return normalized;
}

std::string RobotDiagnosticsNode::sanitizeLogValue(const std::string &value) const
{
	if (value.empty())
	{
		return "none";
	}
	std::string sanitized = value;
	for (auto &character : sanitized)
	{
		if (std::isspace(static_cast<unsigned char>(character)))
		{
			character = '_';
		}
	}
	return sanitized;
}

std::string RobotDiagnosticsNode::statusToText(CheckStatus status) const
{
	if (status == CheckStatus::Fail)
	{
		return "FAIL";
	}
	if (status == CheckStatus::Warn)
	{
		return "WARN";
	}
	return "PASS";
}

std::string RobotDiagnosticsNode::statusToResult(CheckStatus status) const
{
	if (status == CheckStatus::Fail)
	{
		return "fail";
	}
	if (status == CheckStatus::Warn)
	{
		return "warn";
	}
	return "ok";
}

double RobotDiagnosticsNode::normalizeAngle(double angle_rad) const
{
	while (angle_rad > M_PI)
	{
		angle_rad -= TWO_PI;
	}
	while (angle_rad < -M_PI)
	{
		angle_rad += TWO_PI;
	}
	return angle_rad;
}

double RobotDiagnosticsNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion) const
{
	double roll_rad = 0.0;
	double pitch_rad = 0.0;
	double yaw_rad = 0.0;
	rpyFromQuaternion(quaternion, roll_rad, pitch_rad, yaw_rad);
	return yaw_rad;
}

void RobotDiagnosticsNode::rpyFromQuaternion(
	const geometry_msgs::msg::Quaternion &quaternion,
	double &roll_rad,
	double &pitch_rad,
	double &yaw_rad) const
{
	tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	tf2::Matrix3x3(tf_quaternion).getRPY(roll_rad, pitch_rad, yaw_rad);
}

}  // namespace robot::hw::diagnostics
