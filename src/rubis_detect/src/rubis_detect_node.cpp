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
  // config
  const auto vehicle_param = VehicleConfig{
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_front_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_rear_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.mass_kg"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.yaw_inertia_kgm2"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.width_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_overhang_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_overhang_m"
    ).get<float32_t>())
  };

  const auto safety_factor =
    static_cast<float32_t>(declare_parameter(
      "safety_factor"
    ).get<float32_t>());
  const auto stop_margin =
    static_cast<float32_t>(declare_parameter(
      "stop_margin"
    ).get<float32_t>());

  // init

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

  // create dummy collision box
//   auto box = point_to_box(last_x, last_y, last_heading);
  // find collision

  for(const auto &bbox : msg.boxes) {
    auto centroid = bbox.centroid;
  }
  auto message = std_msgs::msg::String();
  message.data = "Hello, RUBIS! " + std::to_string(200);
  return message;
}

// modified from object_collision_estimator.cpp
BoundingBox RubisDetectNode::point_to_box(const Real _x, const Real _y, const Complex32 _heading, const VehicleConfig & vehicle_param, const float32_t safety_factor)
{
  // Shorthands to keep the formulas sane
  float32_t lf = vehicle_param.length_cg_front_axel() + vehicle_param.front_overhang();
  float32_t lr = vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  float32_t wh = vehicle_param.width() * 0.5f;
  float32_t angle = to_angle(_heading);
  float32_t ch = std::cos(angle);
  float32_t sh = std::sin(angle);

  // inflate size of vehicle by safety factor
  lf *= safety_factor;
  lr *= safety_factor;
  wh *= safety_factor;

  // Create a list of corners for the vehicle
  std::list<Point32> corners;
  {     // Front left
    auto p = Point32{};
    p.x = _x + (lf * ch) - (wh * sh);
    p.y = _y + (lf * sh) + (wh * ch);
    corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = _x + (lf * ch) + (wh * sh);
    p.y = _y + (lf * sh) - (wh * ch);
    corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = _x - (lr * ch) + (wh * sh);
    p.y = _y - (lr * sh) - (wh * ch);
    corners.push_back(p);
  }
  {     // Rear left
    auto p = Point32{};
    p.x = _x - (lr * ch) - (wh * sh);
    p.y = _y - (lr * sh) + (wh * ch);
    corners.push_back(p);
  }
  return minimum_perimeter_bounding_box(corners);
}

}  // namespace rubis_detect
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_detect::RubisDetectNode)
