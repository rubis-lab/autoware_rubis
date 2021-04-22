
#ifndef RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
#define RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "point_cloud_fusion_nodes/point_cloud_fusion_node.hpp"
#include "voxel_grid_nodes/voxel_cloud_node.hpp"
#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"
#include "rubis_detect/rubis_detect_node.hpp"
#include "ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp"
#include "point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp"
#include "ndt_nodes/ndt_localizer_nodes.hpp"
#include "rubis_drive/rubis_drive_node.hpp"


rclcpp::NodeOptions configure_point_cloud_fusion_nodes(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("number_of_sources", 2);
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("cloud_size", 55000);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_voxel_grid_nodes(void) {
  std::vector<rclcpp::Parameter> params;

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

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_euclidean_cluster_nodes(void) {
  std::vector<rclcpp::Parameter> params;

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

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_rubis_detect_nodes(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("vehicle.cg_to_front_m", 1.0);
  params.emplace_back("vehicle.cg_to_rear_m", 1.0);
  params.emplace_back("vehicle.front_corner_stiffness", 0.1);
  params.emplace_back("vehicle.rear_corner_stiffness", 0.1);
  params.emplace_back("vehicle.mass_kg", 1500.0);
  params.emplace_back("vehicle.yaw_inertia_kgm2", 12.0);
  params.emplace_back("vehicle.width_m", 2.0);
  params.emplace_back("vehicle.front_overhang_m", 0.5);
  params.emplace_back("vehicle.rear_overhang_m", 0.5);
  params.emplace_back("safety_factor", 1.1);
  params.emplace_back("stop_margin", 12.5);
  params.emplace_back("trajectory_smoother.kernel_std", 5.0);
  params.emplace_back("trajectory_smoother.kernel_size", 25);
  params.emplace_back("staleness_threshold_ms", 500);
  params.emplace_back("target_frame_id", "map");
  params.emplace_back("lookahead_boxes", 30);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_ray_ground_classifier_nodes(void) {
  std::vector<rclcpp::Parameter> params;

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

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_point_cloud_filter_transform_lidar_front_nodes(void) {
  std::vector<rclcpp::Parameter> params;

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
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_point_cloud_filter_transform_lidar_rear_nodes(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("timeout_ms", 110);
  params.emplace_back("pcl_size", 55000);
  params.emplace_back("input_frame_id", "lidar_rear");
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("init_timeout_ms", 5000);
  params.emplace_back("expected_num_subscribers", 1);
  params.emplace_back("expected_num_publishers", 1);
  params.emplace_back("start_angle", 0.0);
  params.emplace_back("end_angle", 6.28);
  params.emplace_back("min_radius", 6.00001);
  params.emplace_back("max_radius", 150.0);
  params.emplace_back("static_transformer.quaternion.x", 0.0);
  params.emplace_back("static_transformer.quaternion.y", 0.0);
  params.emplace_back("static_transformer.quaternion.z", 0.0);
  params.emplace_back("static_transformer.quaternion.w", 1.0);
  params.emplace_back("static_transformer.translation.x", 0.308);
  params.emplace_back("static_transformer.translation.y", -0.022);
  params.emplace_back("static_transformer.translation.z", 1.49);
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_ndt_localizer_nodes(void) {
  std::vector<rclcpp::Parameter> params;

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

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_rubis_drive_nodes(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("target_vel", 10.0);
  params.emplace_back("cur2tar", 1.0);
  params.emplace_back("safe_dist", 30.0);
  params.emplace_back("danger_scale", 20);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

#endif  //RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
