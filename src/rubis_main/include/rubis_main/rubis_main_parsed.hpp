
#ifndef RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
#define RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rubis_detect/rubis_detect_node.hpp"
#include "rubis_drive/rubis_drive_node.hpp"


rclcpp::NodeOptions configure_rubis_detect(void) {
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

rclcpp::NodeOptions configure_rubis_drive(void) {
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
