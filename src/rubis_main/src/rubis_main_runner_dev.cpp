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
#include "rubis_rt/sched.hpp" 

using rclcpp::executors::SingleThreadedExecutor;
using rubis::sched::set_sched_deadline;
using autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode;

void routine_point_cloud_filter_transform_lidar_front_nodes() {
  // node
  auto ptr_point_cloud_filter_transform_lidar_front_nodes = 
    std::make_shared<PointCloud2FilterTransformNode>(
      "point_cloud_filter_transform_front", "lidar_front",
      configure_point_cloud_filter_transform_lidar_front_nodes());

  // executor
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_front_nodes;
  exec_point_cloud_filter_transform_lidar_front_nodes.add_node(
    ptr_point_cloud_filter_transform_lidar_front_nodes);

  // routine
  exec_point_cloud_filter_transform_lidar_front_nodes.spin();
  return;
}

void routine_point_cloud_filter_transform_lidar_rear_nodes() {
  auto ptr_point_cloud_filter_transform_lidar_rear_nodes =
    std::make_shared<PointCloud2FilterTransformNode>(
      "point_cloud_filter_transform_rear", "lidar_rear",
      configure_point_cloud_filter_transform_lidar_rear_nodes());
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_rear_nodes;
  exec_point_cloud_filter_transform_lidar_rear_nodes.add_node(
    ptr_point_cloud_filter_transform_lidar_rear_nodes);

  int tid = gettid();
  std::cout << "(point_cloud_filter_transform_lidar_rear_nodes) tid: " << tid << std::endl;
  set_sched_deadline(tid, 1000000, 2000000, 3000000);
  exec_point_cloud_filter_transform_lidar_rear_nodes.spin();
  std::cout << "point_cloud_filter_transform_lidar_rear_nodes" << std::endl; 
  return;
}

int main(int argc, char * argv[])
{
  std::cout << "main start (dev)" << std::endl;
  
  rclcpp::init(0, nullptr);
  std::vector<std::thread> thrs;

  thrs.emplace_back(std::thread(
    routine_point_cloud_filter_transform_lidar_front_nodes));
  thrs.emplace_back(std::thread(
    routine_point_cloud_filter_transform_lidar_rear_nodes));

  // main end
  for(int i = 0; i < thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();

  rclcpp::shutdown();
  return 0;
}