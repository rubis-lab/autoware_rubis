// std
#include <iostream>
#include <thread>
#include <string>
#include <fstream>

// ROS2 dependencies
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

// rubis dependencies
#include "rubis_drive/rubis_drive_node.hpp"

#include "rubis_main/rubis_main_parsed.hpp"

using rclcpp::executors::SingleThreadedExecutor;

int main(int argc, char * argv[])
{
  std::cout << "main start" << std::endl;
  
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;
  std::vector<std::thread> thrs;

  //rubis_drive_nodes
  using autoware::rubis_drive::RubisDriveNode;

  auto ptr_rubis_drive_nodes = std::make_shared<RubisDriveNode>(
    configure_rubis_drive());
  SingleThreadedExecutor exec_rubis_drive_nodes;
  exec_rubis_drive_nodes.add_node(ptr_rubis_drive_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_rubis_drive_nodes.spin();
  }));
  std::cout << "rubis_drive_nodes" << std::endl;

  for(int i=0; i<thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();

  rclcpp::shutdown();
  std::cout << "rubis_main_runner launched." << std::endl;
  return 0;
}