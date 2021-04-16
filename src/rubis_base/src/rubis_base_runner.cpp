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
#include "point_cloud_fusion_nodes/point_cloud_fusion_node.hpp"
#include "mpc_controller_nodes/mpc_controller_node.hpp"
#include "mpc_controller/mpc_controller.hpp"
#include "mpc_controller/config.hpp"
#include "motion_common/motion_common.hpp"
#include "controller_common/controller_base.hpp"
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
using autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode;
using motion::control::mpc_controller_nodes::MpcControllerNode;
using motion::control::mpc_controller::Config;
using motion::control::mpc_controller::Interpolation;
using motion::control::controller_common::BehaviorConfig;
using motion::motion_common::LimitsConfig;
using motion::motion_common::VehicleConfig;
using motion::motion_common::StateWeight;
using motion::motion_common::OptimizationConfig;
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
rclcpp::NodeOptions configure_rubis0_node(void) { 
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  
  node_options.parameter_overrides(params);
  
  return node_options;
}

//map_publisher_node
rclcpp::NodeOptions configure_map_publisher_node(void) {
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("map_pcd_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot_lgsvl.pcd");
  params.emplace_back("map_yaml_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot_lgsvl.yaml");
  params.emplace_back("map_frame", "map");
  params.emplace_back("map_config.capacity", static_cast<int32_t>(1000000));
  params.emplace_back("map_config.min_point.x", static_cast<float32_t>(-1000.0));
  params.emplace_back("map_config.min_point.y", static_cast<float32_t>(-1000.0));
  params.emplace_back("map_config.min_point.z", static_cast<float32_t>(-3.0));
  params.emplace_back("map_config.max_point.x",static_cast<float32_t>(1000.0));
  params.emplace_back("map_config.max_point.y",static_cast<float32_t>(1000.0));
  params.emplace_back("map_config.max_point.z",static_cast<float32_t>(3.0));
  params.emplace_back("map_config.voxel_size.x",static_cast<float32_t>(3.5));
  params.emplace_back("map_config.voxel_size.y",static_cast<float32_t>(3.5));
  params.emplace_back("map_config.voxel_size.z",static_cast<float32_t>(3.5));
  params.emplace_back("viz_map", true);
   
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  
  return node_options;
}

//pointcloud_fusion_node
rclcpp::NodeOptions configure_pointcloud_fusion_node(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("number_of_sources", 2);
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("cloud_size", 55000);
   
  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//lanelet2_map_provider:map_provider.node
rclcpp::NodeOptions configure_lanelet2_map_provider(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("map_osm_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot.osm");

  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//voxel_grid_nodes scan_downsampler
rclcpp::NodeOptions configure_voxel_grid_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("is_approximate", false);
  params.emplace_back("config.capacity", 55000);
  params.emplace_back("config.min_point.x", -130.0);
  params.emplace_back("config.min_point.y", -130.0);
  params.emplace_back("config.min_point.z", -3.0);
  params.emplace_back("config.max_point.x", 130.0);
  params.emplace_back("config.max_point.y", 130.0);
  params.emplace_back("config.max_point.z", 3.0);
  params.emplace_back("config.voxel_size.x", 1.0);
  params.emplace_back("config.voxel_size.y", 1.0);
  params.emplace_back("config.voxel_size.z", 1.0);

  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//parking_planner_nodes
rclcpp::NodeOptions configure_parking_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("vehicle.cg_to_front_m", 2.74);
  params.emplace_back("vehicle.cg_to_rear_m", 0.1);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 1.83);
  params.emplace_back("vehicle.front_overhang_m", 1.0);
  params.emplace_back("vehicle.rear_overhang_m", 1.03);
  params.emplace_back("optimization_weights.steering", 1.0);
  params.emplace_back("optimization_weights.throttle", 1.0);
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
  params.emplace_back("command_bounds.upper.throttle_mps2", 5.0);;

  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//point_cloud_filter_transform_nodes_front
rclcpp::NodeOptions configure_point_cloud_filter_transform_nodes_front(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("timeout_ms", 110);
  params.emplace_back("pcl_size", 55000);
  params.emplace_back("input_frame_id", "lidar_front");
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("init_timeout_ms", 5000);
  params.emplace_back("expected_num_subscribers", 1);
  params.emplace_back("expected_num_publishers", 1);
  params.emplace_back("start_angle", 0.0);
  params.emplace_back("end_angle", 6.28);
  params.emplace_back("min_radius", 6.0);
  params.emplace_back("max_radius", 150.0);
  params.emplace_back("static_transformer.quaternion.x", 0.0);
  params.emplace_back("static_transformer.quaternion.y", 0.0);
  params.emplace_back("static_transformer.quaternion.z", 0.0);
  params.emplace_back("static_transformer.quaternion.w", 1.0);
  params.emplace_back("static_transformer.translation.x", 1.498);
  params.emplace_back("static_transformer.translation.y", -0.022);
  params.emplace_back("static_transformer.translation.z", 1.49);
  
  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//point_cloud_filter_transform_nodes_front
rclcpp::NodeOptions configure_point_cloud_filter_transform_nodes_rear(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("timeout_ms", 110);
  params.emplace_back("pcl_size", 55000);
  params.emplace_back("input_frame_id", "lidar_rear");
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("init_timeout_ms", 5000);
  params.emplace_back("expected_num_subscribers", 1);
  params.emplace_back("expected_num_publishers", 1);
  params.emplace_back("start_angle", 0.0);
  params.emplace_back("end_angle", 6.28);
  params.emplace_back("min_radius", 6.0);
  params.emplace_back("max_radius", 150.0);
  params.emplace_back("static_transformer.quaternion.x", 0.0);
  params.emplace_back("static_transformer.quaternion.y", 0.0);
  params.emplace_back("static_transformer.quaternion.z", 0.0);
  params.emplace_back("static_transformer.quaternion.w", 1.0);
  params.emplace_back("static_transformer.translation.x", 0.308);
  params.emplace_back("static_transformer.translation.y", -0.022);
  params.emplace_back("static_transformer.translation.z", 1.49);
  
  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//object_collision_estimator_nodes
rclcpp::NodeOptions configure_object_collision_estimator_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("vehicle.cg_to_front_m", 2.74);
  params.emplace_back("vehicle.cg_to_rear_m", 0.1);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 1.83);
  params.emplace_back("vehicle.front_overhang_m", 1.0);
  params.emplace_back("vehicle.rear_overhang_m", 1.03);
  params.emplace_back("safety_factor", 2.0);
  params.emplace_back("stop_margin", 20.0);
  params.emplace_back("trajectory_smoother.kernel_std", 5.0);
  params.emplace_back("trajectory_smoother.kernel_size", 25);
  params.emplace_back("staleness_threshold_ms", 500);
  params.emplace_back("target_frame_id", "map");

  node_options.parameter_overrides(params);

  return node_options;
}

Config configure_mpc_controller_node(void) {
  LimitsConfig limits(
    LimitsConfig::Extremum(-5.00F, 35.0F),  //longitudinal_velocity_mps
    LimitsConfig::Extremum(-3.0F, 3.0F),    //lateral_velocity_mps
    LimitsConfig::Extremum(-3.0F, 3.0F),    //acceleration_mps2
    LimitsConfig::Extremum(-3.0F, 3.0F),    //yaw_rate_rps
    LimitsConfig::Extremum(-10.0F, 10.0F),  //jerk_mps3
    LimitsConfig::Extremum(-0.6F, 0.6F),    //steer_angle_rad
    LimitsConfig::Extremum(-0.6F, 0.6)     //steer_angle_rate_rps
  );
  VehicleConfig vehicle_param(
    1.228F,     //length_cg_front_axel_m, cg_to_front_m
    1.5618F,    //length_cg_rear_axel_m, cg_to_rear_m
    17000.0F,   //front_cornering_stiffness_N, front_corner_stiffness
    20000.0F,   //rear_cornering_stiffness_N, rear_corner_stiffness
    1460.0F,    //mass_kg, mass_kg
    2170.0F,    //inertia_kgm2, yaw_inertia_kgm2
    2.0F,       //width_m, width_m
    0.5F,       //front_overhang_m, front_overhang_m
    0.5F       //rear_overhang_m, rear_overhang_m
  );
  StateWeight nominal(
    10.0F,      //pose
    10.0F,      //heading
    10.0F,      //longitudinal_velocity
    10.0F,      //lateral_velocity
    10.0F,      //yaw_rate
    10.0F,      //acceleration
    10.0F,      //jerk
    10.0F,      //steer_angle
    10.0F       //steer_angle_rate
  );
  StateWeight terminal(
    10000.0F,      //pose
    10000.0F,      //heading
    100.0F,        //longitudinal_velocity
    10.0F,         //lateral_velocity
    10.0F,         //yaw_rate
    0.00001F,      //acceleration
    0.00001F,      //jerk
    0.00001F,      //steer_angle
    0.00001F       //steer_angle_rate
  );
  BehaviorConfig behavior(
    3.0F,           //stop_rate_mps2
    100ms,          //time_step_ms
    motion::control::controller_common::ControlReference::TEMPORAL
    //is_temporal_reference, TEMPORAL, SPATIAL
  );
  OptimizationConfig optimization_param(
    nominal,
    terminal
  );
  std::chrono::nanoseconds sample_period_tolerance = 20ms;
  std::chrono::nanoseconds control_lookahead_duration = 100ms;
  Interpolation interpolation =
    motion::control::mpc_controller::Interpolation::YES;
  Config config(
    limits,
    vehicle_param,
    behavior,
    optimization_param,
    sample_period_tolerance,
    control_lookahead_duration,
    interpolation
  );

  return config;
}

//lanelet2_map_provider:map_visualizer
//same package with lanelet2_map_provider
rclcpp::NodeOptions configure_lanelet2_map_visualizer(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::string node_name = "lanelet2_map_visualizer";

  params.emplace_back("map_osm_file", "/home/rubis/AutowareAuto/install/rubis_0/share/rubis_0/data/autonomoustuff_parking_lot.osm");
  
  node_options.parameter_overrides(params);

  return node_options;
  //return thrs[thrs.size()-1];
}

//lanelet2_global_planner_nodes
rclcpp::NodeOptions configure_lanelet2_global_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

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

  return node_options;
}

//p2d_ndt_localizer
rclcpp::NodeOptions configure_ndt_localizer(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("observation_sub.history_depth", 10);
  params.emplace_back("map_sub.history_depth", 1);
  params.emplace_back("pose_pub.history_depth", 10);
  params.emplace_back("publish_tf", true);
  params.emplace_back("predict_pose_threshold.translation", 50.0);
  params.emplace_back("predict_pose_threshold.rotation", 3.15);
  params.emplace_back("init_hack.translation.x", -57.6);
  params.emplace_back("init_hack.translation.y", -41.0);
  params.emplace_back("init_hack.translation.z", -2.01);
  params.emplace_back("init_hack.quaternion.x", 0.0);
  params.emplace_back("init_hack.quaternion.y", 0.0);
  params.emplace_back("init_hack.quaternion.z", 0.0);
  params.emplace_back("init_hack.quaternion.w", 1.0);
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
  
  return node_options;
  //return thrs[thrs.size()-1];
}

//lane_planner_nodes
rclcpp::NodeOptions configure_lane_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

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
  
  return node_options;
}

//exec_behavior_planner_nodes
rclcpp::NodeOptions configure_behavior_planner_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

  params.emplace_back("heading_weight", 0.1);
  params.emplace_back("goal_distance_thresh", 3.0);
  params.emplace_back("stop_velocity_thresh", 2.0);
  params.emplace_back("subroute_goal_offset_lane2parking", 7.5);
  params.emplace_back("subroute_goal_offset_parking2lane", 7.5);
  
  node_options.parameter_overrides(params);
  
  return node_options;
}

//ray_ground_classifier_cloud_nodes
rclcpp::NodeOptions configure_ray_ground_classifier_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

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
  
  return node_options;
}

//euclidean_cluster_nodes
rclcpp::NodeOptions configure_euclidean_cluster_nodes(void) {

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

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
  
  return node_options;
}

/// The main function puts a Ping node in one OS process and runs the
/// experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);
  std::cout << "hi!" << std::endl;
  
  rclcpp::NodeOptions node_options;
  std::vector<std::thread> thrs;

  const std::chrono::seconds WAIT = 1s;

  //rubis0_node
  // node_options = configure_rubis0_node();
  // std::shared_ptr<Rubis0Node> ptr_rubis0_node;
  // ptr_rubis0_node = std::make_shared<Rubis0Node>(node_options);
  // rclcpp::executors::SingleThreadedExecutor exec_rubis_0_node;
  // exec_rubis_0_node.add_node(ptr_rubis0_node);
    
  // thrs.emplace_back(
  //   std::thread(
  //   [&]() {
  //     exec_rubis_0_node.spin();
  //   })
  // );
  // std::cout << "rubis0_node" << std::endl;

  //point_cloud_filter_transform_nodes_front
  rclcpp::NodeOptions point_cloud_filter_transform_nodes_front_node_options = configure_point_cloud_filter_transform_nodes_front();
  std::shared_ptr<PointCloud2FilterTransformNode> ptr_point_cloud_filter_transform_nodes_front;
  ptr_point_cloud_filter_transform_nodes_front = std::make_shared<PointCloud2FilterTransformNode>("filter_transform_vlp16_front", "lidar_front", point_cloud_filter_transform_nodes_front_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_point_cloud_filter_transform_nodes_front;
  exec_point_cloud_filter_transform_nodes_front.add_node(ptr_point_cloud_filter_transform_nodes_front);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_nodes_front.spin();
  }));
  std::cout << "point_cloud_filter_transform_nodes_front" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //point_cloud_filter_transform_nodes_rear
  rclcpp::NodeOptions point_cloud_filter_transform_nodes_rear_node_options = configure_point_cloud_filter_transform_nodes_rear();
  std::shared_ptr<PointCloud2FilterTransformNode> ptr_point_cloud_filter_transform_nodes_rear;
  ptr_point_cloud_filter_transform_nodes_rear = std::make_shared<PointCloud2FilterTransformNode>("filter_transform_vlp16_rear", "lidar_rear", point_cloud_filter_transform_nodes_rear_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_point_cloud_filter_transform_nodes_rear;
  exec_point_cloud_filter_transform_nodes_rear.add_node(ptr_point_cloud_filter_transform_nodes_rear);

  thrs.emplace_back(std::thread([&](){
    exec_point_cloud_filter_transform_nodes_rear.spin();
  }));
  std::cout << "point_cloud_filter_transform_nodes_rear" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //pointcloud_fusion_node
  rclcpp::NodeOptions pointcloud_fusion_node_node_options = configure_pointcloud_fusion_node();
  std::shared_ptr<PointCloudFusionNode> ptr_pointcloud_fusion_node;
  ptr_pointcloud_fusion_node = std::make_shared<PointCloudFusionNode>("point_cloud_fusion_node", "lidars", pointcloud_fusion_node_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_pointcloud_fusion_node;
  exec_pointcloud_fusion_node.add_node(ptr_pointcloud_fusion_node);

  thrs.emplace_back(std::thread([&](){
    exec_pointcloud_fusion_node.spin();
  }));
  std::cout << "pointcloud_fusion_node" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //ray_ground_classifier_nodes
  rclcpp::NodeOptions ray_ground_classifier_nodes_node_options = configure_ray_ground_classifier_nodes();
  std::shared_ptr<RayGroundClassifierCloudNode> ptr_ray_ground_classifier_nodes;
  ptr_ray_ground_classifier_nodes = std::make_shared<RayGroundClassifierCloudNode>("ray_ground_classifier", "perception", ray_ground_classifier_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_ray_ground_classifier_nodes;
  exec_ray_ground_classifier_nodes.add_node(ptr_ray_ground_classifier_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_ray_ground_classifier_nodes.spin();
  }));
  std::cout << "ray_ground_classifier_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //voxel_grid_nodes
  rclcpp::NodeOptions voxel_grid_nodes_node_options = configure_voxel_grid_nodes();
  std::shared_ptr<VoxelCloudNode> ptr_voxel_grid_nodes;
  ptr_voxel_grid_nodes = std::make_shared<VoxelCloudNode>("voxel_grid_cloud_node", "lidars", voxel_grid_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_voxel_grid_nodes;
  exec_voxel_grid_nodes.add_node(ptr_voxel_grid_nodes);

  thrs.emplace_back( std::thread([&](){
    exec_voxel_grid_nodes.spin();
  }));
  std::cout << "voxel_grid_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //euclidean_cluster_nodes
  rclcpp::NodeOptions euclidean_cluster_nodes_node_options = configure_euclidean_cluster_nodes();
  std::shared_ptr<EuclideanClusterNode> ptr_euclidean_cluster_nodes;
  ptr_euclidean_cluster_nodes = std::make_shared<EuclideanClusterNode>("euclidean_cluster_cloud_node", "perception", euclidean_cluster_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_euclidean_cluster_nodes;
  exec_euclidean_cluster_nodes.add_node(ptr_euclidean_cluster_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_euclidean_cluster_nodes.spin();
  }));
  std::cout << "euclidean_cluster_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //map_publisher_node
  rclcpp::NodeOptions map_publisher_node_node_options = configure_map_publisher_node();
  std::shared_ptr<NDTMapPublisherNode> ptr_map_publisher_node;
  ptr_map_publisher_node = std::make_shared<NDTMapPublisherNode>("ndt_map_publisher_node", "localization", map_publisher_node_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_map_publisher_node;
  exec_map_publisher_node.add_node(ptr_map_publisher_node);
    
  thrs.emplace_back(std::thread([&](){
    exec_map_publisher_node.spin();
  }));
  std::cout << "map_publisher_node" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //lanelet2_map_provider
  rclcpp::NodeOptions lanelet2_map_provider_node_options = configure_lanelet2_map_provider();
  std::shared_ptr<Lanelet2MapProviderNode> ptr_lanelet2_map_provider;
  ptr_lanelet2_map_provider = std::make_shared<Lanelet2MapProviderNode>("Lanelet2MapProvider", "had_maps", lanelet2_map_provider_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_lanelet2_map_provider;
  exec_lanelet2_map_provider.add_node(ptr_lanelet2_map_provider);

  thrs.emplace_back(std::thread([&](){
    exec_lanelet2_map_provider.spin();
  }));
  std::cout << "lanelet2_map_provider" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //lanelet2_map_visualizer
  rclcpp::NodeOptions lanelet2_map_visualizer_node_options = configure_lanelet2_map_visualizer();
  std::shared_ptr<Lanelet2MapVisualizer> ptr_lanelet2_map_visualizer;
  ptr_lanelet2_map_visualizer = std::make_shared<Lanelet2MapVisualizer>("lanelet2_map_visualizer", "had_maps", lanelet2_map_visualizer_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_lanelet2_map_visualizer;
  exec_lanelet2_map_visualizer.add_node(ptr_lanelet2_map_visualizer);

  thrs.emplace_back(std::thread([&](){
    exec_lanelet2_map_visualizer.spin();
  }));
  std::cout << "lanelet2_map_visualizer" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //ndt_localizer
  rclcpp::NodeOptions ndt_localizer_node_options = configure_ndt_localizer();
  std::shared_ptr<P2DNDTLocalizerNode<>> ptr_ndt_localizer;
  ptr_ndt_localizer = std::make_shared<P2DNDTLocalizerNode<>>("p2d_ndt_localizer_node", "localization",
    ndt_localizer_node_options, autoware::localization::ndt_nodes::PoseInitializer_{});
  rclcpp::executors::SingleThreadedExecutor exec_ndt_localizer;
  exec_ndt_localizer.add_node(ptr_ndt_localizer);

  thrs.emplace_back(std::thread([&](){
      exec_ndt_localizer.spin();
  }));
  std::cout << "ndt_localizer" << std::endl;   
  std::this_thread::sleep_for(WAIT);

  //lanelet2_global_planner_nodes
  rclcpp::NodeOptions lanelet2_global_planner_nodes_node_options = configure_lanelet2_global_planner_nodes();
  std::shared_ptr<Lanelet2GlobalPlannerNode> ptr_lanelet2_global_planner_nodes;
  ptr_lanelet2_global_planner_nodes = std::make_shared<Lanelet2GlobalPlannerNode>("lanelet2_global_planner_node", "planning", lanelet2_global_planner_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_lanelet2_global_planner_nodes;
  exec_lanelet2_global_planner_nodes.add_node(ptr_lanelet2_global_planner_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_lanelet2_global_planner_nodes.spin();
  }));
  std::cout << "lanelet2_global_planner_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //parking_planner_nodes
  rclcpp::NodeOptions parking_planner_nodes_node_options = configure_parking_planner_nodes();
  std::shared_ptr<ParkingPlannerNode> ptr_parking_planner_nodes;
  ptr_parking_planner_nodes = std::make_shared<ParkingPlannerNode>("parking_planner_node", "planning", parking_planner_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_parking_planner_nodes;
  exec_parking_planner_nodes.add_node(ptr_parking_planner_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_parking_planner_nodes.spin();
  }));
  std::cout << "parking_planner_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //lane_planner_nodes
  rclcpp::NodeOptions lane_planner_nodes_node_options = configure_lane_planner_nodes();
  std::shared_ptr<LanePlannerNode> ptr_lane_planner_nodes;
  ptr_lane_planner_nodes = std::make_shared<LanePlannerNode>("lane_planner_node", "planning", lane_planner_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_lane_planner_nodes;
  exec_lane_planner_nodes.add_node(ptr_lane_planner_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_lane_planner_nodes.spin();
  }));
  std::cout << "lane_planner_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //object_collision_estimator_nodes
  rclcpp::NodeOptions object_collision_estimator_nodes_node_options = configure_object_collision_estimator_nodes();
  std::shared_ptr<ObjectCollisionEstimatorNode> ptr_object_collision_estimator_nodes;
  ptr_object_collision_estimator_nodes = std::make_shared<ObjectCollisionEstimatorNode>("object_collision_estimator_node", "planning", object_collision_estimator_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_object_collision_estimator_nodes;
  exec_object_collision_estimator_nodes.add_node(ptr_object_collision_estimator_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_object_collision_estimator_nodes.spin();
  }));
  std::cout << "object_collision_estimator_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //behavior_planner_nodes
  rclcpp::NodeOptions behavior_planner_nodes_node_options = configure_behavior_planner_nodes();
  std::shared_ptr<BehaviorPlannerNode> ptr_behavior_planner_nodes;
  ptr_behavior_planner_nodes = std::make_shared<BehaviorPlannerNode>("behavior_planner_node", "planning", behavior_planner_nodes_node_options);
  rclcpp::executors::SingleThreadedExecutor exec_behavior_planner_nodes;
  exec_behavior_planner_nodes.add_node(ptr_behavior_planner_nodes);

  thrs.emplace_back(std::thread([&](){
    exec_behavior_planner_nodes.spin();
  }));
  std::cout << "behavior_planner_nodes" << std::endl;
  std::this_thread::sleep_for(WAIT);

  //mpc_controller_node
  Config config = configure_mpc_controller_node();
  std::shared_ptr<MpcControllerNode> ptr_mpc_controller_node;
  ptr_mpc_controller_node = std::make_shared<MpcControllerNode>(
    "mpc_controller",
    "control",
    "/vehicle/vehicle_command",
    "/vehicle/vehicle_kinematic_state",
    "/tf",
    "/planning/trajectory",
    "control_diagnostic",
    "/tf_static",
    config
  );
  rclcpp::executors::SingleThreadedExecutor exec_mpc_controller_node;
  exec_mpc_controller_node.add_node(ptr_mpc_controller_node);

  thrs.emplace_back(std::thread([&](){
    exec_mpc_controller_node.spin();
  })); 
  std::cout << "mpc_controller_node" << std::endl;
  std::this_thread::sleep_for(WAIT);

  std::cout << "thread size: " << thrs.size() <<std::endl;
  
  const std::chrono::seconds EXPERIMENT_DURATION = 5s;
  std::cout << "Running experiment from now on for " << EXPERIMENT_DURATION.count() << "s ..." << std::endl;
  std::this_thread::sleep_for(EXPERIMENT_DURATION);
  // ... and stop the experiment.
  
  for(int i=0; i<thrs.size(); i++) {
    thrs.at(i).join();
  }
  thrs.clear();
//   ping_node->print_statistics(EXPERIMENT_DURATION);
  
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