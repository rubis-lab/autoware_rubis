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
    "point_cloud_filter_transform_front", "lidar_front", configure_point_cloud_filter_transform_lidar_front_nodes());
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_front_nodes;
  exec_point_cloud_filter_transform_lidar_front_nodes.add_node(ptr_point_cloud_filter_transform_lidar_front_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_lidar_front_nodes.spin();
  }));
  std::cout << "point_cloud_filter_transform_lidar_front_nodes" << std::endl;  

  auto ptr_point_cloud_filter_transform_lidar_rear_nodes = std::make_shared<PointCloud2FilterTransformNode>(
    "point_cloud_filter_transform_rear", "lidar_rear", configure_point_cloud_filter_transform_lidar_rear_nodes());
  SingleThreadedExecutor exec_point_cloud_filter_transform_lidar_rear_nodes;
  exec_point_cloud_filter_transform_lidar_rear_nodes.add_node(ptr_point_cloud_filter_transform_lidar_rear_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_lidar_rear_nodes.spin();
  }));
  std::cout << "point_cloud_filter_transform_lidar_rear_nodes" << std::endl;  

  //point_cloud_fusion_nodes
  using autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode;

  auto ptr_point_cloud_fusion_nodes = std::make_shared<PointCloudFusionNode>(
    "point_cloud_fusion_nodes", "lidars", configure_point_cloud_fusion_nodes());
  SingleThreadedExecutor exec_point_cloud_fusion_nodes;
  exec_point_cloud_fusion_nodes.add_node(ptr_point_cloud_fusion_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_fusion_nodes.spin();
  }));
  std::cout << "point_cloud_fusion_nodes" << std::endl;  
  
  //voxel_grid_nodes
  using autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode;

  auto ptr_voxel_grid_nodes = std::make_shared<VoxelCloudNode>(
    "voxel_grid_cloud_node", "lidars", configure_voxel_grid_nodes());
    //name = "euclidean_cluster_node" from euclidean_cluster_node.cpp
  SingleThreadedExecutor exec_voxel_grid_nodes;
  exec_voxel_grid_nodes.add_node(ptr_voxel_grid_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_voxel_grid_nodes.spin();
  }));
  std::cout << "voxel_grid_nodes" << std::endl;  

  //ray_ground_classifier_nodes
  using autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;

  auto ptr_ray_ground_classifier_nodes = std::make_shared<RayGroundClassifierCloudNode>(
    "ray_ground_classifier", "perception", configure_ray_ground_classifier_nodes());
  SingleThreadedExecutor exec_ray_ground_classifier_nodes;
  exec_ray_ground_classifier_nodes.add_node(ptr_ray_ground_classifier_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_ray_ground_classifier_nodes.spin();
  }));
  std::cout << "ray_ground_classifier_nodes" << std::endl;  

  //euclidean_cluster_nodes
  using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;

  auto ptr_euclidean_cluster_nodes = std::make_shared<EuclideanClusterNode>(
    "euclidean_cluster_node", "perception", configure_euclidean_cluster_nodes());
    //name = "euclidean_cluster_node" from euclidean_cluster_node.cpp
  SingleThreadedExecutor exec_euclidean_cluster_nodes;
  exec_euclidean_cluster_nodes.add_node(ptr_euclidean_cluster_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_euclidean_cluster_nodes.spin();
  }));
  std::cout << "euclidean_cluster_nodes" << std::endl;  

  //ndt_localizer_nodes
  using autoware::localization::ndt_nodes::P2DNDTLocalizerNode;

  auto ptr_ndt_localizer_nodes = std::make_shared<P2DNDTLocalizerNode<>>(
    "p2d_ndt_localizer_node", "localization", configure_ndt_localizer_nodes(),
    autoware::localization::ndt_nodes::PoseInitializer_{});
  // ptr_ndt_localizer = std::make_shared<P2DNDTLocalizerNode<>>("p2d_ndt_localizer_node", "localization",
  //   ndt_localizer_node_options, autoware::localization::ndt_nodes::PoseInitializer_{});
  SingleThreadedExecutor exec_ndt_localizer_nodes;
  exec_ndt_localizer_nodes.add_node(ptr_ndt_localizer_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_ndt_localizer_nodes.spin();
  }));
  std::cout << "ndt_localizer_nodes" << std::endl;  

  //rubis_drive
  using autoware::rubis_drive::RubisDriveNode;

  auto ptr_rubis_drive_nodes = std::make_shared<RubisDriveNode>(
    configure_rubis_drive_nodes());
  SingleThreadedExecutor exec_rubis_drive_nodes;
  exec_rubis_drive_nodes.add_node(ptr_rubis_drive_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_rubis_drive_nodes.spin();
  }));
  std::cout << "rubis_drive_nodes" << std::endl;

  //rubis_detect
  using autoware::rubis_detect::RubisDetectNode;

  auto ptr_rubis_detect_nodes = std::make_shared<RubisDetectNode>(
    configure_rubis_detect_nodes());
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