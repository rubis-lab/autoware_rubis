// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rubis_drive/rubis_drive_node.hpp"

using namespace std::chrono_literals;

namespace autoware
{
namespace rubis_drive
{

RubisDriveNode::RubisDriveNode(const rclcpp::NodeOptions & options)
:  Node("rubis_drive", options),
  verbose(true)
{
  target_vel = static_cast<float32_t>(declare_parameter(
      "target_vel"
    ).get<float32_t>());

  // reach target velocity in {cur2tar}s later
  cur2tar = static_cast<float32_t>(declare_parameter(
      "cur2tar"
    ).get<float32_t>());; 

  publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_drive_topic", 10);
  timer_ = this->create_wall_timer(
    4000ms, std::bind(&RubisDriveNode::timer_callback, this));

  command_publisher_ = this->create_publisher<Command>("/vehicle/vehicle_command", 10);
//   command_timer_ = this->create_wall_timer(
//     1000ms, std::bind(&RubisDriveNode::command_timer_callback, this));

  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  state_subscriber_ = create_subscription<CBD>(
    "/lgsvl/state_report", 10,
    [this](const CBD::SharedPtr msg) {on_state(msg);}, SubAllocT{});
  
  danger_subscriber_ = create_subscription<std_msgs::msg::String>(
    "rubis_danger", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {on_danger(msg);}, SubAllocT{});

}

int32_t RubisDriveNode::print_hello() const
{
  return rubis_drive::print_hello();
}

void RubisDriveNode::timer_callback()
{
  RCLCPP_WARN(get_logger(), "Timer triggered.");
  auto message = std_msgs::msg::String();
  message.data = "Hello, RUBIS! " + std::to_string(100);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

// void RubisDriveNode::command_timer_callback()
// {
//   RCLCPP_WARN(get_logger(), "(Command) Timer triggered.");
// //   auto message = compute_command();
//   auto message = 1;
//   command_publisher_->publish(message);
// }

void RubisDriveNode::on_state(const CBD::SharedPtr & msg)
{
  std::cout<<"onstate callback"<<std::endl;
  // const auto cmd{compute_command(*msg)};
  auto cmd = compute_command_rubis(*msg);
  command_publisher_->publish(cmd);
}

void RubisDriveNode::on_danger(const std_msgs::msg::String::SharedPtr & msg)
{
  std::string collision_distance = msg->data;
  dist = std::stod(collision_distance, nullptr);
}

Command RubisDriveNode::compute_command_rubis(const CBD & state)
{
  Command ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  ret.stamp = state.header.stamp;
  // Steering angle "uses up" stopping power/grip capacity
  ret.front_wheel_angle_rad = Real{};  // zero initialization etc.
  ret.rear_wheel_angle_rad = Real{};
  // Compute stopping acceleration
  // validate input
//   const auto velocity = std::fabs(state.state.longitudinal_velocity_mps);
//   const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(std::chrono::milliseconds(100L));

//   const auto decel = std::min(
//     velocity / dt.count(),
//     3.0F);   // positive
//   ret.long_accel_mps2 = state.state.longitudinal_velocity_mps >= 0.0F ? -decel : decel;
  float32_t danger;
  float32_t safe_dist;
//   cur_vel = state.state.longitudinal_velocity_mps;
  cur_vel = state.speed_mps;
//   cur_acc = state.state.acceleration_mps2;
    
  // safe_dist = (target_vel >= 60/3.6) ? cur_vel * 3.6 : cur_vel * 3.6 - 15;
  safe_dist = 30;
  RCLCPP_WARN(get_logger(), "RubisDriveNode::compute_command_rubis: cur_vel: " + std::to_string(cur_vel));
  std::cout<<"current_velocity = " << cur_vel << std::endl;
  std::cout<<"dist = " << dist << std::endl;
  std::cout<<"safe_dist = " << safe_dist << std::endl;

  if(dist >= safe_dist)
    danger = 0;
  else if( (safe_dist - dist) * 20 < 100 )
    danger = (safe_dist - dist) * 20;
  else
    danger = 100;
  
  std::cout<<"danger: " << danger << std::endl;
  ret.long_accel_mps2 = static_cast<float32_t>((target_vel - cur_vel)/cur2tar - danger*danger/100);
  std::cout << "command: " << (target_vel - cur_vel)/cur2tar - danger*danger/100 << std::endl;
  std::cout << "compute_command_end" << std::endl;
  return ret;
}

}  // namespace rubis_drive
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_drive::RubisDriveNode)
