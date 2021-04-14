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

#include "rubis_detect/rubis_detect_node.hpp"

namespace autoware
{
namespace rubis_detect
{

RubisDetectNode::RubisDetectNode(const rclcpp::NodeOptions & options)
:  Node("rubis_detect", options),
  verbose(true)
{
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  state_subscriber_ = create_subscription<State>(
    "/vehicle/vehicle_kinematic_state", 10,
    [this](const State::SharedPtr msg) {on_state(msg);}, SubAllocT{});

  bounding_box_subscriber_ = create_subscription<BoundingBoxArray>(
    "/perception/lidar_bounding_boxes", 10,
    [this](const BoundingBoxArray::SharedPtr msg) {on_bounding_box(msg);}, SubAllocT{});

  danger_publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_danger", 10);
}

int32_t RubisDetectNode::print_hello() const
{
  return rubis_detect::print_hello();
}

void RubisDetectNode::on_state(const State::SharedPtr & msg)
{
  save_state(*msg);
  return;
}

void RubisDetectNode::save_state(const State & state)
{
  last_x = state.state.x;
  last_y = state.state.y;
  last_heading = state.state.heading;
//   RCLCPP_WARN(get_logger(), "RubisDetectNode::on_state: last_x" + std::to_string(last_x));
}

void RubisDetectNode::on_bounding_box(const BoundingBoxArray::SharedPtr & msg)
{
  const auto danger{compute_danger(*msg)};
  danger_publisher_->publish(danger);
}

std_msgs::msg::String RubisDetectNode::compute_danger(const BoundingBoxArray & msg)
{
  // compute heading
  auto angle = to_angle(last_heading);
  RCLCPP_WARN(get_logger(), "RubisDetectNode::compute_danger: angle" + std::to_string(angle));
  for(const auto &bbox : msg.boxes) {
    auto centroid = bbox.centroid;
  }
  auto message = std_msgs::msg::String();
  message.data = "Hello, RUBIS! " + std::to_string(200);
  return message;
}

}  // namespace rubis_detect
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_detect::RubisDetectNode)
