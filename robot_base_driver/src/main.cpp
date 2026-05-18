#include "robot_base_driver/robot_base_driver_node.hpp"

using namespace robot::hw::base;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotBaseDriverNode>());
	rclcpp::shutdown();
	return 0;
}
