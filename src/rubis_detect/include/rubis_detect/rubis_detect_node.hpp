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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the rubis_detect_node class.

#ifndef RUBIS_DETECT__RUBIS_DETECT_NODE_HPP_
#define RUBIS_DETECT__RUBIS_DETECT_NODE_HPP_

#include <rubis_detect/rubis_detect.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <motion_common/motion_common.hpp>
#include <controller_common/controller_base.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace autoware
{
namespace rubis_detect
{

using motion::control::controller_common::State;
using motion::control::controller_common::Real;
using motion::motion_common::to_angle;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::Complex32;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

/// \class RubisDetectNode
/// \brief ROS 2 Node for hello world.
class RUBIS_DETECT_PUBLIC RubisDetectNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit RubisDetectNode(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.
  Real last_x;
  Real last_y;
  autoware_auto_msgs::msg::Complex32 last_heading;

  rclcpp::Subscription<State>::SharedPtr state_subscriber_{};
  void on_state(const State::SharedPtr & msg);
  void save_state(const State & state);

  rclcpp::Subscription<BoundingBoxArray>::SharedPtr bounding_box_subscriber_{};
  void on_bounding_box(const BoundingBoxArray::SharedPtr & msg);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr danger_publisher_;
  std_msgs::msg::String compute_danger(const BoundingBoxArray & msg);
};
}  // namespace rubis_detect
}  // namespace autoware

#endif  // RUBIS_DETECT__RUBIS_DETECT_NODE_HPP_
