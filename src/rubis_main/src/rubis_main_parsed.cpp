
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
