#include <iostream>

// ROS2 dependencies
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;
  
//   rclcpp::NodeOptions node_options;
//   std::vector<std::thread> thrs;

  rclcpp::shutdown();
  std::cout << "rubis_main_runner launched 222." << std::endl;
  return 0;
}