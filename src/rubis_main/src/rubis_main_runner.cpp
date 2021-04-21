// std
#include <iostream>
#include <thread>
#include <string>
#include <fstream>

// ROS2 dependencies
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

// rubis dependencies
#include "rubis_main/rubis_main_parsed.hpp"

using rclcpp::executors::SingleThreadedExecutor;

int main(int argc, char * argv[])
{
  std::cout << "main start" << std::endl;
  
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;
  std::vector<std::thread> thrs;

  //point_cloud_filter_transform_nodes
  using autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode;

  auto ptr_point_cloud_filter_transform_lidar_front_nodes = std::make_shared<PointCloud2FilterTransformNode>(
    configure_point_cloud_filter_transform_lidar_front_nodes());
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_front_nodes;
  exec_point_cloud_filter_transform_lidar_front_nodes.add_node(ptr_point_cloud_filter_transform_lidar_front_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_lidar_front_nodes.spin();
  }));
  std::cout << "point_cloud_filter_transform_lidar_front_nodes" << std::endl;  

  auto ptr_point_cloud_filter_transform_lidar_rear_nodes = std::make_shared<PointCloud2FilterTransformNode>(
    configure_point_cloud_filter_transform_lidar_rear_nodes());
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_rear_nodes;
  exec_point_cloud_filter_transform_lidar_rear_nodes.add_node(ptr_point_cloud_filter_transform_lidar_rear_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_lidar_rear_nodes.spin();
  }));
  std::cout << "point_cloud_filter_transform_lidar_rear_nodes" << std::endl;  


  //rubis_drive
  using autoware::rubis_drive::RubisDriveNode;

  auto ptr_rubis_drive_nodes = std::make_shared<RubisDriveNode>(
    configure_rubis_drive());
  SingleThreadedExecutor exec_rubis_drive_nodes;
  exec_rubis_drive_nodes.add_node(ptr_rubis_drive_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_rubis_drive_nodes.spin();
  }));
  std::cout << "rubis_drive_nodes" << std::endl;

  //rubis_detect
  using autoware::rubis_detect::RubisDetectNode;

  auto ptr_rubis_detect_nodes = std::make_shared<RubisDetectNode>(
    configure_rubis_detect());
  SingleThreadedExecutor exec_rubis_detect_nodes;
  exec_rubis_detect_nodes.add_node(ptr_rubis_detect_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_rubis_detect_nodes.spin();
  }));
  std::cout << "rubis_detect_nodes" << std::endl;

  for(int i=0; i<thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();

  rclcpp::shutdown();
  std::cout << "rubis_main_runner launched." << std::endl;
  return 0;
}