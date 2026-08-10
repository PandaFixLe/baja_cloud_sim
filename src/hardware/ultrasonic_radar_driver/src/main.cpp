#include "ultrasonic_radar_driver/ultrasonic_radar_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ultrasonic_radar_driver::UltrasonicRadarNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}