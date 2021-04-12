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
  publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_drive_topic", 10);
  timer_ = this->create_wall_timer(
    4000ms, std::bind(&RubisDriveNode::timer_callback, this));

  command_publisher_ = this->create_publisher<Command>("rubis_command_topic", 10);
//   command_timer_ = this->create_wall_timer(
//     1000ms, std::bind(&RubisDriveNode::command_timer_callback, this));

  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  state_subscriber_ = create_subscription<State>(
    "/vehicle/vehicle_kinematic_state", 10,
    [this](const State::SharedPtr msg) {on_state(msg);}, SubAllocT{});
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

void RubisDriveNode::on_state(const State::SharedPtr & msg)
{
  const auto cmd{compute_command(*msg)};
  command_publisher_->publish(cmd);
}

Command RubisDriveNode::compute_command(const State & state) const noexcept
{
  // dummy command
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

  ret.long_accel_mps2 = 3.0F;

  return ret;
}

}  // namespace rubis_drive
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_drive::RubisDriveNode)
