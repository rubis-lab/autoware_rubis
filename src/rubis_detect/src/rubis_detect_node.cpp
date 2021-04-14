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
  init_vehicle(vehicle_param);
  safety_factor =
    static_cast<float32_t>(declare_parameter(
      "safety_factor"
    ).get<float32_t>());
  stop_margin =
    static_cast<float32_t>(declare_parameter(
      "stop_margin"
    ).get<float32_t>());

  lookahead_boxes =
    static_cast<int32_t>(declare_parameter(
      "lookahead_boxes"
    ).get<int32_t>());

  // init
  last_p = Point32{};
  last_p.x = 0;
  last_p.y = 0;
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  state_subscriber_ = create_subscription<State>(
    "/vehicle/vehicle_kinematic_state", 10,
    [this](const State::SharedPtr msg) {on_state(msg);}, SubAllocT{});

  bounding_box_subscriber_ = create_subscription<BoundingBoxArray>(
    "/perception/lidar_bounding_boxes", 10,
    [this](const BoundingBoxArray::SharedPtr msg) {on_bounding_box(msg);}, SubAllocT{});

  danger_publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_danger", 10);
}

void RubisDetectNode::init_vehicle(const VehicleConfig & _vehicle_param)
{
  lf = _vehicle_param.length_cg_front_axel() + _vehicle_param.front_overhang();
  lr = _vehicle_param.length_cg_rear_axel() + _vehicle_param.rear_overhang();
  wh = _vehicle_param.width() * 0.5f;
  // inflate size of vehicle by safety factor
  lf *= safety_factor;
  lr *= safety_factor;
  wh *= safety_factor;

  // find the dimension of the ego vehicle.
  vehicle_length =
    _vehicle_param.front_overhang() + _vehicle_param.length_cg_front_axel() +
    _vehicle_param.length_cg_rear_axel() + _vehicle_param.rear_overhang();
  vehicle_width = _vehicle_param.width();
  vehicle_diagonal = sqrtf(
    (vehicle_width * vehicle_width) + (vehicle_length * vehicle_length));

  // define a distance threshold to filter obstacles that are too far away to cause any collision.
  distance_threshold = vehicle_diagonal * safety_factor;
  return;
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
  last_p.x = state.state.x;
  last_p.y = state.state.y;
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

  auto collision_index = detect_collision(last_p, last_heading, msg);
  auto message = std_msgs::msg::String();
  message.data = "Collision with " + std::to_string(collision_index);
  return message;
}

// modified from object_collision_estimator.cpp
BoundingBox RubisDetectNode::point_to_box(const Point32 _p, const Complex32 _heading)
{
  // Shorthands to keep the formulas sane
  float32_t angle = to_angle(_heading);
  float32_t ch = std::cos(angle);
  float32_t sh = std::sin(angle);

  // Create a list of corners for the vehicle
  std::list<Point32> corners;
  {     // Front left
    auto p = Point32{};
    p.x = _p.x + (lf * ch) - (wh * sh);
    p.y = _p.y + (lf * sh) + (wh * ch);
    corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = _p.x + (lf * ch) + (wh * sh);
    p.y = _p.y + (lf * sh) - (wh * ch);
    corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = _p.x - (lr * ch) + (wh * sh);
    p.y = _p.y - (lr * sh) - (wh * ch);
    corners.push_back(p);
  }
  {     // Rear left
    auto p = Point32{};
    p.x = _p.x - (lr * ch) - (wh * sh);
    p.y = _p.y - (lr * sh) + (wh * ch);
    corners.push_back(p);
  }
  return minimum_perimeter_bounding_box(corners);
}

std::list<Point32> RubisDetectNode::get_expected_trajectory(const Point32 _p, const Complex32 _heading)
{
  // Shorthands to keep the formulas sane
  float32_t angle = to_angle(_heading);
  float32_t ch = std::cos(angle);
  float32_t sh = std::sin(angle);

  std::list<Point32> expected_trajectory;
  for(int32_t i = 0; i < lookahead_boxes; i++) {
    auto p = Point32{};
    p.x = _p.x + 2 * (lf * ch);
    p.y = _p.y + 2 * (lf * sh);
    expected_trajectory.push_back(p);
  }
  return expected_trajectory;
}

int32_t RubisDetectNode::detect_collision(const Point32 _p, const Complex32 _heading, const BoundingBoxArray & obstacles)
{
  int32_t collision_index = -1;

  auto expected_trajectory = get_expected_trajectory(_p,_heading);

  int32_t t_idx = 0;
  for(auto const& p : expected_trajectory) {
    const auto p_box = point_to_box(p, _heading);
    for(const auto & obstacle_bbox : obstacles.boxes) {
      if(!is_too_far_away(p, obstacle_bbox, distance_threshold)) {
        if(autoware::common::geometry::intersect(
          p_box.corners.begin(), p_box.corners.end(),
          obstacle_bbox.corners.begin(), obstacle_bbox.corners.end())) {
          // Collision detected
          collision_index = t_idx;
          break;
        }
      }
    }
    t_idx++;
  }
  RCLCPP_WARN(get_logger(), "RubisDetectNode::detect_collision: collision" + std::to_string(collision_index));

  return collision_index;
}

bool8_t RubisDetectNode::is_too_far_away(const Point32 _p, const BoundingBox obstacle_bbox, const float32_t distance_threshold)
{
  bool is_too_far_away{true};
  auto distance_threshold_squared = distance_threshold * distance_threshold;

  for (auto corner : obstacle_bbox.corners) {
    auto dx = corner.x - _p.x;
    auto dy = corner.y - _p.y;
    auto distance_squared = (dx * dx) + (dy * dy);

    if (distance_threshold_squared > distance_squared) {
      is_too_far_away = false;
      break;
    }
  }

  return is_too_far_away;
}

}  // namespace rubis_detect
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_detect::RubisDetectNode)
