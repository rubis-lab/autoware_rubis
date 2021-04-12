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

}  // namespace rubis_drive
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_drive::RubisDriveNode)
