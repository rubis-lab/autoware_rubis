#include <iostream>

// ROS2 dependencies
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

// rubis dependencies
#include "rubis_drive/rubis_drive_node.hpp"

// rubis_drive
rclcpp::NodeOptions configure_rubis_drive_nodes(void) {
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("target_vel", 10.0F);
  params.emplace_back("cur2tar", 1.0F);
  params.emplace_back("safe_dist", 30.0F);
  params.emplace_back("danger_scale", 20);
   
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  
  return node_options;
}


int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;
  std::vector<std::thread> thrs;

  //rubis_drive_nodes
  using autoware::rubis_drive::RubisDriveNode;

  rclcpp::NodeOptions rubis_drive_nodes_node_options = configure_rubis_drive_nodes();
  std::shared_ptr<RubisDriveNode> ptr_rubis_drive_nodes;
  ptr_rubis_drive_nodes = std::make_shared<RubisDriveNode>(rubis_drive_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_rubis_drive_nodes;
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
  std::cout << "rubis_main_runner launched 222." << std::endl;
  return 0;
}