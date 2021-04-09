#include <cinttypes>
#include <cstdlib>
#include <ctime>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
//#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rcl/parser.h"

// #include "examples_rclcpp_cbg_executor/ping_node.hpp"
// #include "examples_rclcpp_cbg_executor/utilities.hpp"




#include "lane_planner_nodes/lane_planner_node.hpp"
#include "rubis_0/rubis_0_node.hpp"
#include "rubis_sched.hpp"
#include "ndt_nodes/map_publisher.hpp"
#include "mpc_controller_nodes/mpc_controller_node.hpp"
#include "lanelet2_map_provider/lanelet2_map_provider_node.hpp"
#include "voxel_grid_nodes/voxel_cloud_node.hpp"
#include "parking_planner_nodes/parking_planner_node.hpp"
#include "point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp"
#include "object_collision_estimator_nodes/object_collision_estimator_node.hpp"
#include "lanelet2_map_provider/lanelet2_map_visualizer.hpp"
#include "lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp"
#include "ndt_nodes/ndt_localizer_nodes.hpp"
#include "behavior_planner_nodes/behavior_planner_node.hpp"
#include "ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp"
#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using namespace std::chrono_literals;

// using examples_rclcpp_cbg_executor::PingNode;
// using examples_rclcpp_cbg_executor::configure_thread;
// using examples_rclcpp_cbg_executor::get_thread_time;
// using examples_rclcpp_cbg_executor::ThreadPriority;

using autoware::rubis_0::Rubis0Node;
using autoware::localization::ndt_nodes::NDTMapPublisherNode;
using motion::control::mpc_controller_nodes::MpcControllerNode;
using autoware::lanelet2_map_provider::Lanelet2MapProviderNode;
using autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode;
using autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode;
using autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode;
using motion::planning::object_collision_estimator_nodes::ObjectCollisionEstimatorNode;
using autoware::lanelet2_map_provider::Lanelet2MapVisualizer;
using autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode;
using autoware::localization::ndt_nodes::P2DNDTLocalizerNode;
using autoware::lane_planner_nodes::LanePlannerNode;
using autoware::behavior_planner_nodes::BehaviorPlannerNode;
using autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;


//std::unordered_map<std::thread> thrs;

//rubis0_node
//test
rclcpp::NodeOptions configure_rubis0_node(std::vector<rclcpp::Parameter> params) { 
  
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "rubis_0";

  node_options.parameter_overrides(params);
  std::shared_ptr<Rubis0Node> node_ptr;
  node_ptr = std::make_shared<Rubis0Node>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);
  
  
  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  ); 
  return node_options;
}

//map_publisher_node
std::string exec_map_publisher_node(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "ndt_map_publisher";

  params.emplace_back("map_pcd_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot_lgsvl.pcd");
  params.emplace_back("map_yaml_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot_lgsvl.yaml");
  params.emplace_back("map_frame", "map");
  params.emplace_back("map_config.capacity", 1000000);
  params.emplace_back("map_config.min_point.x", -1000.0);
  params.emplace_back("map_config.min_point.y", -1000.0);
  params.emplace_back("map_config.min_point.z", -3.0);
  params.emplace_back("map_config.max_point.x", 1000.0);
  params.emplace_back("map_config.max_point.y", 1000.0);
  params.emplace_back("map_config.max_point.z", 3.0);
  params.emplace_back("map_config.voxel_size.x", 3.5);
  params.emplace_back("map_config.voxel_size.y", 3.5);
  params.emplace_back("map_config.voxel_size.z", 3.5);
  params.emplace_back("viz_map", true);
   
  node_options.parameter_overrides(params);
  std::shared_ptr<NDTMapPublisherNode> node_ptr;
  node_ptr = std::make_shared<NDTMapPublisherNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//pointcloud_fusion_node
std::string exec_pointcloud_fusion_node(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "pointcloud_fusion_node";

  params.emplace_back("number_of_sources", 2);
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("cloud_size", 55000);
   
  node_options.parameter_overrides(params);
  std::shared_ptr<NDTMapPublisherNode> node_ptr;
  node_ptr = std::make_shared<NDTMapPublisherNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//mpc_controller_node
std::string exec_mpc_controller_node(void) {

  std::vector<rclcpp::Parameter> params;
  // rclcpp::NodeOptions node_options;
  std::string node_name = "mpc_controller_node";

  // node_options.parameter_overrides(params);
  std::shared_ptr<MpcControllerNode> node_ptr;
  node_ptr = std::make_shared<MpcControllerNode>("/home/rubis/AutowareAuto/src/rubis_0/param/mpc.param.yaml", "");
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//lanelet2_map_provider:map_provider.node
std::string exec_lanelet2_map_provider(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "lanelet2_map_provider";

  params.emplace_back("map_osm_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot.osm");

  node_options.parameter_overrides(params);
  std::shared_ptr<Lanelet2MapProviderNode> node_ptr;
  node_ptr = std::make_shared<Lanelet2MapProviderNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//voxel_grid_nodes scan_downsampler
std::string exec_voxel_grid_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "voxel_grid_nodes";

  params.emplace_back("is_approximate", false);
  params.emplace_back("config.capacity", 55000);
  params.emplace_back("config.min_point.x", -130.0);
  params.emplace_back("config.min_point.y", -130.0);
  params.emplace_back("config.min_point.z", -3.0);
  params.emplace_back("config.max_point.x", 130.0);
  params.emplace_back("config.max_point.y", 130.0);
  params.emplace_back("config.max_point.z", 3.0);
  params.emplace_back("config.voxel_size.x", true);
  params.emplace_back("config.voxel_size.y", true);
  params.emplace_back("config.voxel_size.z", true);

  node_options.parameter_overrides(params);
  std::shared_ptr<VoxelCloudNode> node_ptr;
  node_ptr = std::make_shared<VoxelCloudNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//parking_planner_nodes
std::string exec_parking_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "parking_planner_nodes";

  params.emplace_back("vehicle.cg_to_front_m", 2.74);
  params.emplace_back("vehicle.cg_to_rear_m", 0.1);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 1.83);
  params.emplace_back("vehicle.front_overhang_m", true);
  params.emplace_back("vehicle.rear_overhang_m", 1.03);
  params.emplace_back("optimization_weights.steering", true);
  params.emplace_back("optimization_weights.throttle", true);
  params.emplace_back("optimization_weights.goal", 0.5);
  params.emplace_back("state_bounds.lower.x_m", -300.0);
  params.emplace_back("state_bounds.lower.y_m", -300.0);
  params.emplace_back("state_bounds.lower.velocity_mps", -3.0);
  params.emplace_back("state_bounds.lower.heading_rad", -6.2832);
  params.emplace_back("state_bounds.lower.steering_rad", -0.53);
  params.emplace_back("state_bounds.upper.x_m", 300.0);
  params.emplace_back("state_bounds.upper.y_m", 300.0);
  params.emplace_back("state_bounds.upper.velocity_mps", 3.0);
  params.emplace_back("state_bounds.upper.heading_rad", 6.2832);
  params.emplace_back("state_bounds.upper.steering_rad", 0.53);
  params.emplace_back("command_bounds.lower.steering_rate_rps", -5.0);
  params.emplace_back("command_bounds.lower.throttle_mps2", -5.0);
  params.emplace_back("command_bounds.upper.steering_rate_rps", 5.0);
  params.emplace_back("command_bounds.upper.throttle_mps2", 5.0);

  node_options.parameter_overrides(params);
  std::shared_ptr<ParkingPlannerNode> node_ptr;
  node_ptr = std::make_shared<ParkingPlannerNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//point_cloud_filter_transform_nodes
std::string exec_point_cloud_filter_transform_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "point_cloud_filter_transform_nodes";

  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.timeout_ms", 110);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.pcl_size", 55000);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.input_frame_id", "lidar_front");
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.output_frame_id", "base_link");
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.init_timeout_ms", 5000);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.expected_num_subscribers", true);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.expected_num_publishers", true);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.start_angle", false);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.end_angle", 6.28);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.min_radius", 6.0);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.max_radius", 150.0);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.quaternion.x", false);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.quaternion.y", false);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.quaternion.z", false);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.quaternion.w", true);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.translation.x", 1.498);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.translation.y", -0.022);
  params.emplace_back("lidar_front.filter_transform_vlp16_front.ros__parameters.static_transformer.translation.z", 1.49);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.timeout_ms", 110);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.pcl_size", 55000);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.input_frame_id", "lidar_rear");
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.output_frame_id", "base_link");
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.init_timeout_ms", 5000);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.expected_num_subscribers", true);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.expected_num_publishers", true);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.start_angle", false);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.end_angle", 6.28);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.min_radius", 6.0);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.max_radius", 150.0);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.quaternion.x", false);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.quaternion.y", false);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.quaternion.z", false);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.quaternion.w", true);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.translation.x", 0.308);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.translation.y", -0.022);
  params.emplace_back("lidar_rear.filter_transform_vlp16_rear.ros__parameters.static_transformer.translation.z", 1.49);

  node_options.parameter_overrides(params);
  std::shared_ptr<PointCloud2FilterTransformNode> node_ptr;
  node_ptr = std::make_shared<PointCloud2FilterTransformNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//object_collision_estimator_nodes
std::string exec_object_collision_estimator_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "object_collision_estimator_nodes";

  params.emplace_back("vehicle.cg_to_front_m", 2.74);
  params.emplace_back("vehicle.cg_to_rear_m", 0.1);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 1.83);
  params.emplace_back("vehicle.front_overhang_m", true);
  params.emplace_back("vehicle.rear_overhang_m", 1.03);
  params.emplace_back("safety_factor", 2.0);
  params.emplace_back("stop_margin", 20.0);
  params.emplace_back("trajectory_smoother.kernel_std", 5.0);
  params.emplace_back("trajectory_smoother.kernel_size", 25);
  params.emplace_back("target_frame_id", "map");

  node_options.parameter_overrides(params);
  std::shared_ptr<ObjectCollisionEstimatorNode> node_ptr;
  node_ptr = std::make_shared<ObjectCollisionEstimatorNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//lanelet2_map_provider:map_visualizer
//same package with lanelet2_map_provider
std::string exec_lanelet2_map_visualizer(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "lanelet2_map_visualizer";

  params.emplace_back("map_osm_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot.osm");
  
  node_options.parameter_overrides(params);
  std::shared_ptr<Lanelet2MapVisualizer> node_ptr;
  node_ptr = std::make_shared<Lanelet2MapVisualizer>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//lanelet2_global_planner_nodes
std::string exec_lanelet2_global_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "lanelet2_global_planner_nodes";

  params.emplace_back("heading_weight", 0.1);
  params.emplace_back("lane_planner.trajectory_resolution", 0.5);
  params.emplace_back("vehicle.cg_to_front_m", 1.228);
  params.emplace_back("vehicle.cg_to_rear_m", 1.5618);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 2.0);
  params.emplace_back("vehicle.front_overhang_m", 1.05);
  params.emplace_back("vehicle.rear_overhang_m", 0.92);
  params.emplace_back("gaussian_smoother.standard_deviation", 5.0);
  params.emplace_back("gaussian_smoother.kernel_size", 3);
  
  node_options.parameter_overrides(params);
  std::shared_ptr<Lanelet2GlobalPlannerNode> node_ptr;
  node_ptr = std::make_shared<Lanelet2GlobalPlannerNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//p2d_ndt_localizer
std::string exec_ndt_localizer(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "ndt_localizer";

  params.emplace_back("observation_sub.history_depth", 10);
  params.emplace_back("map_sub.history_depth", true);
  params.emplace_back("pose_pub.history_depth", 10);
  params.emplace_back("publish_tf", true);
  params.emplace_back("predict_pose_threshold.translation", 50.0);
  params.emplace_back("predict_pose_threshold.rotation", 3.15);
  params.emplace_back("init_hack.translation.x", -57.6);
  params.emplace_back("init_hack.translation.y", -41.0);
  params.emplace_back("init_hack.translation.z", -2.01);
  params.emplace_back("init_hack.quaternion.x", false);
  params.emplace_back("init_hack.quaternion.y", false);
  params.emplace_back("init_hack.quaternion.z", false);
  params.emplace_back("init_hack.quaternion.w", true);
  params.emplace_back("init_hack.enabled", false);
  params.emplace_back("localizer.map.capacity", 1000000);
  params.emplace_back("localizer.map.min_point.x", -1000.0);
  params.emplace_back("localizer.map.min_point.y", -1000.0);
  params.emplace_back("localizer.map.min_point.z", -3.0);
  params.emplace_back("localizer.map.max_point.x", 1000.0);
  params.emplace_back("localizer.map.max_point.y", 1000.0);
  params.emplace_back("localizer.map.max_point.z", 3.0);
  params.emplace_back("localizer.map.voxel_size.x", 3.5);
  params.emplace_back("localizer.map.voxel_size.y", 3.5);
  params.emplace_back("localizer.map.voxel_size.z", 3.5);
  params.emplace_back("localizer.scan.capacity", 55000);
  params.emplace_back("localizer.optimization.outlier_ratio", 0.55);
  params.emplace_back("localizer.optimizer.max_iterations", 30);
  params.emplace_back("localizer.optimizer.score_tolerance", 0.0002);
  params.emplace_back("localizer.optimizer.parameter_tolerance", 0.0002);
  params.emplace_back("localizer.optimizer.gradient_tolerance", 0.0002);
  params.emplace_back("localizer.optimizer.line_search.step_max", 0.12);
  params.emplace_back("localizer.optimizer.line_search.step_min", 0.0001);
  params.emplace_back("localizer.guess_time_tolerance_ms", 10000);
  
  node_options.parameter_overrides(params);
  std::shared_ptr<P2DNDTLocalizerNode<>> node_ptr;
  node_ptr = std::make_shared<P2DNDTLocalizerNode<>>("p2d_ndt_localizer_node", node_options,
    autoware::localization::ndt_nodes::PoseInitializer_{});
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//lane_planner_nodes
std::string exec_lane_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "lane_planner_nodes";

  params.emplace_back("heading_weight", 0.1);
  params.emplace_back("lane_planner.trajectory_resolution", 0.5);
  params.emplace_back("vehicle.cg_to_front_m", 1.228);
  params.emplace_back("vehicle.cg_to_rear_m", 1.5618);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 2.0);
  params.emplace_back("vehicle.front_overhang_m", 1.05);
  params.emplace_back("vehicle.rear_overhang_m", 0.92);
  params.emplace_back("gaussian_smoother.standard_deviation", 5.0);
  params.emplace_back("gaussian_smoother.kernel_size", 3);

  node_options.parameter_overrides(params);
  std::shared_ptr<LanePlannerNode> node_ptr;
  node_ptr = std::make_shared<LanePlannerNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//exec_behavior_planner_nodes
std::string exec_behavior_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "behavior_planner_nodes";

  params.emplace_back("heading_weight", 0.1);
  params.emplace_back("goal_distance_thresh", 3.0);
  params.emplace_back("stop_velocity_thresh", 2.0);
  params.emplace_back("subroute_goal_offset_lane2parking", 7.5);
  params.emplace_back("subroute_goal_offset_parking2lane", 7.5);
  
  node_options.parameter_overrides(params);
  std::shared_ptr<BehaviorPlannerNode> node_ptr;
  node_ptr = std::make_shared<BehaviorPlannerNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//ray_ground_classifier_cloud_nodes
std::string exec_ray_ground_classifier_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "ray_ground_classifier_nodes";

  params.emplace_back("timeout_ms", 10);
  params.emplace_back("cloud_timeout_ms", 110);
  params.emplace_back("pcl_size", 55000);
  params.emplace_back("frame_id", "base_link");
  params.emplace_back("is_structured", true);
  params.emplace_back("classifier.sensor_height_m", 0.368);
  params.emplace_back("classifier.max_local_slope_deg", 20.0);
  params.emplace_back("classifier.max_global_slope_deg", 7.0);
  params.emplace_back("classifier.nonground_retro_thresh_deg", 70.0);
  params.emplace_back("classifier.min_height_thresh_m", 0.05);
  params.emplace_back("classifier.max_global_height_thresh_m", 0.3);
  params.emplace_back("classifier.max_last_local_ground_thresh_m", 0.6);
  params.emplace_back("classifier.max_provisional_ground_distance_m", 5.0);
  params.emplace_back("classifier.min_height_m", -0.5);
  params.emplace_back("classifier.max_height_m", 1.5);
  params.emplace_back("aggregator.min_ray_angle_rad", -3.14159);
  params.emplace_back("aggregator.max_ray_angle_rad", 3.14159);
  params.emplace_back("aggregator.ray_width_rad", 0.01);
  params.emplace_back("aggregator.max_ray_points", 512);
  
  node_options.parameter_overrides(params);
  std::shared_ptr<RayGroundClassifierCloudNode> node_ptr;
  node_ptr = std::make_shared<RayGroundClassifierCloudNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

//ray_ground_classifier_cloud_nodes
std::string exec_euclidean_cluster_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "ray_euclidean_cluster_nodes";

  params.emplace_back("use_cluster", true);
  params.emplace_back("use_box", true);
  params.emplace_back("max_cloud_size", 55000);
  params.emplace_back("downsample", false);
  params.emplace_back("use_lfit", true);
  params.emplace_back("use_z", true);
  params.emplace_back("cluster.frame_id", "base_link");
  params.emplace_back("cluster.min_cluster_size", 10);
  params.emplace_back("cluster.max_num_clusters", 256);
  params.emplace_back("hash.min_x", -130.0);
  params.emplace_back("hash.max_x", 130.0);
  params.emplace_back("hash.min_y", -130.0);
  params.emplace_back("hash.max_y", 130.0);
  params.emplace_back("hash.side_length", 0.7);
  params.emplace_back("voxel.min_point.x", -130.0);
  params.emplace_back("voxel.min_point.y", -130.0);
  params.emplace_back("voxel.min_point.z", -130.0);
  params.emplace_back("voxel.max_point.x", 130.0);
  params.emplace_back("voxel.max_point.y", 130.0);
  params.emplace_back("voxel.max_point.z", 130.0);
  params.emplace_back("voxel.voxel_size.x", 0.2);
  params.emplace_back("voxel.voxel_size.y", 0.2);
  params.emplace_back("voxel.voxel_size.z", 0.2);
  
  node_options.parameter_overrides(params);
  std::shared_ptr<EuclideanClusterNode> node_ptr;
  node_ptr = std::make_shared<EuclideanClusterNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor_node;
  executor_node.add_node(node_ptr);

  thrs.emplace_back(
    std::thread(
    [&]() {
      executor_node.spin();
    })
  );
  return node_name;
  //return thrs[thrs.size()-1];
}

/// The main function puts a Ping node in one OS process and runs the
/// experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::vector<std::thread> thrs;

  std::cout << "rubis0" << std::endl;
  std::cout << "rubis0" << std::endl;
  std::cout << "rubis0" << std::endl;
  std::cout << "rubis0" << std::endl;
  exec_rubis0_node();

  // std::cout << "map_publisher_node" << std::endl;
  // exec_map_publisher_node();
  // std::cout << "pointcloud_fusion_node" << std::endl;
  // exec_pointcloud_fusion_node();
  // std::cout << "lanelet2_map_provider" << std::endl;
  // exec_lanelet2_map_provider();
  // std::cout << "voxel_grid_nodes" << std::endl;
  // exec_voxel_grid_nodes();
  // std::cout << "parking_planner_nodes" << std::endl;
  // exec_parking_planner_nodes();
  // std::cout << "point_cloud_filter_transform_nodes" << std::endl;
  // exec_point_cloud_filter_transform_nodes();
  // std::cout << "object_collision_estimator_nodes" << std::endl;
  // exec_object_collision_estimator_nodes();
  // std::cout << "lanelet2_map_visualizer" << std::endl;
  // exec_lanelet2_map_visualizer();
  // std::cout << "lanelet2_global_planner_nodes" << std::endl;
  // exec_lanelet2_global_planner_nodes();
  // std::cout << "ndt_localizer" << std::endl;
  // exec_ndt_localizer();
  // std::cout << "lane_planner_nodes" << std::endl;
  // exec_lane_planner_nodes();
  // std::cout << "behavior_planner_nodes" << std::endl;
  // exec_behavior_planner_nodes();
  // std::cout << "ray_ground_classifier_nodes" << std::endl;
  // exec_ray_ground_classifier_nodes();
  // std::cout << "euclidean_cluster_nodes" << std:: endl;
  // exec_euclidean_cluster_nodes();

  const std::chrono::seconds EXPERIMENT_DURATION = 40s;
  std::cout << "Running experiment from now on for " << EXPERIMENT_DURATION.count() << "s ..." << std::endl;
  std::this_thread::sleep_for(EXPERIMENT_DURATION);
  // ... and stop the experiment.
  
  for(int i=0; i<thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();
  // ping_node->print_statistics(EXPERIMENT_DURATION);
  
  rclcpp::shutdown();
  return 0;
}






//   // Create Ping node instance and add it to high-prio executor.
//   auto ping_node = std::make_shared<PingNode>();
//   high_prio_executor.add_node(ping_node);

//   rclcpp::Logger logger = ping_node->get_logger();

  // Create a thread for the executor ...


//   // ... and configure it accordinly as high prio and pin it to the first CPU.
//   const int CPU_ZERO = 0;
//   bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
//   if (!ret) {
//     RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
//   }