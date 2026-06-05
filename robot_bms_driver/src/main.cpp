#include "robot_bms_driver/bms_driver_node.hpp"

using namespace robot::hw::bms;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BmsDriverNode>());
	rclcpp::shutdown();
	return 0;
}