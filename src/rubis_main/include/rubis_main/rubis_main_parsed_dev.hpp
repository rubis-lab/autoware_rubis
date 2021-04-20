
#ifndef RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
#define RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rubis_drive/rubis_drive_node.hpp"


rclcpp::NodeOptions configure_rubis_drive(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("target_vel", 10.0);
  params.emplace_back("cur2tar", true);
  params.emplace_back("safe_dist", 30.0);
  params.emplace_back("danger_scale", 20);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

rclcpp::NodeOptions configure_rubis_main_runner(void) {
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("target_vel", 10.0);
  params.emplace_back("cur2tar", true);
  params.emplace_back("safe_dist", 30.0);
  params.emplace_back("danger_scale", 20);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  return node_options;
}

#endif  //RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
