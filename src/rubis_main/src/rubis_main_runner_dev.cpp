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
using autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode;
using autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode;
using autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;
using autoware::localization::ndt_nodes::P2DNDTLocalizerNode;
using autoware::rubis_detect::RubisDetectNode;
using autoware::rubis_drive::RubisDriveNode;

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
  // node
  auto ptr_point_cloud_filter_transform_lidar_rear_nodes =
    std::make_shared<PointCloud2FilterTransformNode>(
      "point_cloud_filter_transform_rear", "lidar_rear",
      configure_point_cloud_filter_transform_lidar_rear_nodes());

  // executor
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_rear_nodes;
  exec_point_cloud_filter_transform_lidar_rear_nodes.add_node(
    ptr_point_cloud_filter_transform_lidar_rear_nodes);

  // rt param
  int tid = gettid();
  std::cout << "(point_cloud_filter_transform_lidar_rear_nodes) tid: " << tid << std::endl;
  set_sched_deadline(tid, 1000000, 2000000, 3000000);

  // routine
  exec_point_cloud_filter_transform_lidar_rear_nodes.spin();
  return;
}

void routine_point_cloud_fusion_nodes() {
  // node
  auto ptr_point_cloud_fusion_nodes =
    std::make_shared<PointCloudFusionNode>(
      "point_cloud_fusion_nodes", "lidars",
      configure_point_cloud_fusion_nodes());

  // executor
  SingleThreadedExecutor exec_point_cloud_fusion_nodes;
  exec_point_cloud_fusion_nodes.add_node(ptr_point_cloud_fusion_nodes);

  // routine
  exec_point_cloud_fusion_nodes.spin();
  return;
}

void routine_rubis_drive_nodes() {
  // node
  auto ptr_rubis_drive_nodes =
    std::make_shared<RubisDriveNode>(
      configure_rubis_drive_nodes());

  // executor
  SingleThreadedExecutor exec_rubis_drive_nodes;
  exec_rubis_drive_nodes.add_node(ptr_rubis_drive_nodes);

  // routine
  exec_rubis_drive_nodes.spin();
  return;
}

void routine_voxel_grid_nodes() {
  // node
  auto ptr_voxel_grid_nodes =
    std::make_shared<VoxelCloudNode>(
      "voxel_grid_cloud_node", "lidars",
      configure_voxel_grid_nodes());
      //name = "euclidean_cluster_node" from euclidean_cluster_node.cpp

  // executor
  SingleThreadedExecutor exec_voxel_grid_nodes;
  exec_voxel_grid_nodes.add_node(ptr_voxel_grid_nodes);

  // routine
  exec_voxel_grid_nodes.spin();
  return;
}

void routine_ray_ground_classifier_nodes() {
  // node
  auto ptr_ray_ground_classifier_nodes =
    std::make_shared<RayGroundClassifierCloudNode>(
      "ray_ground_classifier", "perception",
      configure_ray_ground_classifier_nodes());

  // executor
  SingleThreadedExecutor exec_ray_ground_classifier_nodes;
  exec_ray_ground_classifier_nodes.add_node(ptr_ray_ground_classifier_nodes);

  // routine
  exec_ray_ground_classifier_nodes.spin();
  return;
}

void routine_euclidean_cluster_nodes() {
  // node
  auto ptr_euclidean_cluster_nodes =
    std::make_shared<EuclideanClusterNode>(
      "euclidean_cluster_node", "perception",
      configure_euclidean_cluster_nodes());
    // name = "euclidean_cluster_node" from euclidean_cluster_node.cpp

  // executor
  SingleThreadedExecutor exec_euclidean_cluster_nodes;
  exec_euclidean_cluster_nodes.add_node(ptr_euclidean_cluster_nodes);

  // routine
  exec_euclidean_cluster_nodes.spin();
  return;
}

void routine_ndt_localizer_nodes() {
  // node
  auto ptr_ndt_localizer_nodes =
    std::make_shared<P2DNDTLocalizerNode<>>(
      "p2d_ndt_localizer_node", "localization",
      configure_ndt_localizer_nodes(),
      autoware::localization::ndt_nodes::PoseInitializer_{});
  
  // executor
  SingleThreadedExecutor exec_ndt_localizer_nodes;
  exec_ndt_localizer_nodes.add_node(ptr_ndt_localizer_nodes);

  // routine
  exec_ndt_localizer_nodes.spin();
  return;
}

void routine_rubis_detect_nodes() {
  // node
  auto ptr_rubis_detect_nodes =
    std::make_shared<RubisDetectNode>(
      configure_rubis_detect_nodes());

  // executor
  SingleThreadedExecutor exec_rubis_detect_nodes;
  exec_rubis_detect_nodes.add_node(ptr_rubis_detect_nodes);

  // routine
  exec_rubis_detect_nodes.spin();
  return;
}

int main(int argc, char * argv[])
{
  std::cout << "[main] start (dev)" << std::endl;
  
  rclcpp::init(0, nullptr);
  std::vector<std::thread> thrs;

  thrs.emplace_back(std::thread(
    routine_point_cloud_filter_transform_lidar_front_nodes));
  std::cout << "[main] point_cloud_filter_transform_lidar_front_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_point_cloud_filter_transform_lidar_rear_nodes));
  std::cout << "[main] point_cloud_filter_transform_lidar_rear_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_point_cloud_fusion_nodes));
  std::cout << "[main] point_cloud_fusion_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_voxel_grid_nodes));
  std::cout << "[main] voxel_grid_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_ray_ground_classifier_nodes));
  std::cout << "[main] ray_ground_classifier_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_euclidean_cluster_nodes));
  std::cout << "[main] euclidean_cluster_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_ndt_localizer_nodes));
  std::cout << "[main] ndt_localizer_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_rubis_detect_nodes));
  std::cout << "[main] rubis_detect_nodes" << std::endl;

  thrs.emplace_back(std::thread(
    routine_rubis_drive_nodes));
  std::cout << "[main] rubis_drive_nodes" << std::endl;

  // main end
  for(int i = 0; i < thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();

  rclcpp::shutdown();
  std::cout << "[main] shutdown (dev)" << std::endl;
  return 0;
}