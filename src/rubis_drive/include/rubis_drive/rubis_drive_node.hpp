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
/// \brief This file defines the rubis_drive_node class.

#ifndef RUBIS_DRIVE__RUBIS_DRIVE_NODE_HPP_
#define RUBIS_DRIVE__RUBIS_DRIVE_NODE_HPP_

#include <rubis_drive/rubis_drive.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <motion_common/motion_common.hpp>
#include <controller_common/controller_base.hpp>

#include <string>
#include <chrono>
#include <time_utils/time_utils.hpp>
#include <algorithm>
#include <limits>
#include <utility>

namespace autoware
{
namespace rubis_drive
{

using motion::control::controller_common::Command;
using motion::control::controller_common::State;
// using State = autoware_auto_msgs::msg::VehicleKinematicState;
using motion::control::controller_common::Real;
using std::placeholders::_1;


/// \class RubisDriveNode
/// \brief ROS 2 Node for hello world.
class RUBIS_DRIVE_PUBLIC RubisDriveNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit RubisDriveNode(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.

  double dist;
  double cur_vel;
  double cur_acc;
  double target_vel;
  double cur2tar;

  // Timer related
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::Publisher<Command>::SharedPtr command_publisher_;
  Command compute_command(const State & state) const noexcept;

  Command compute_command_rubis(const State & state);

  rclcpp::Subscription<State>::SharedPtr state_subscriber_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr danger_subscriber_{};
  void on_state(const State::SharedPtr & msg);
  void on_danger(const std_msgs::msg::String::SharedPtr & msg);
};
}  // namespace rubis_drive
}  // namespace autoware

#endif  // RUBIS_DRIVE__RUBIS_DRIVE_NODE_HPP_
