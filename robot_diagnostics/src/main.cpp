#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_diagnostics/robot_diagnostics_node.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<robot::hw::diagnostics::RobotDiagnosticsNode>());
	rclcpp::shutdown();
	return 0;
}