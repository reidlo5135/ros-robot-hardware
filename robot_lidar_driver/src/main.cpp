#include "robot_lidar_driver/lidar_driver_node.hpp"

using namespace robot::hw::lidar;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarDriverNode>());
	rclcpp::shutdown();
	return 0;
}
