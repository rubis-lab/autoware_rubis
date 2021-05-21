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
#include <lgsvl_msgs/msg/can_bus_data.hpp>

#include <string>
#include <chrono>
#include <time_utils/time_utils.hpp>
#include <common/types.hpp>
#include <algorithm>
#include <limits>
#include <utility>
#include <math.h>
#include <vector>

#include "rubis_rt/sched_log.hpp"
#include "rubis_rt/sched.hpp"
#include <ctime>
#include <omp.h>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using rubis::sched_log::SchedLog;
using rubis::sched_log::sched_info;
using rubis::sched_log::sched_data;

namespace autoware
{
namespace rubis_drive
{

using motion::control::controller_common::Command;
using motion::control::controller_common::State;
// using State = autoware_auto_msgs::msg::VehicleKinematicState;
using motion::control::controller_common::Real;
// using VSD = lgsvl_msgs::msg::VehicleStateData;
using CBD = lgsvl_msgs::msg::CanBusData;
using autoware::common::types::float32_t;
using std::placeholders::_1;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

using rubis::sched_log::SchedLog;
using rubis::sched_log::sched_info;
using rubis::sched_log::sched_data;

/// \class RubisDriveNode
/// \brief ROS 2 Node for hello world.
class RUBIS_DRIVE_PUBLIC RubisDriveNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit RubisDriveNode(const rclcpp::NodeOptions & options);


private:
  SchedLog __slog;
  sched_info __si;
  std::vector<bool8_t> __rt_configured;
  int32_t __iter;

  bool verbose;  ///< whether to use verbose output or not.
  float32_t cur_vel;
  float32_t cur_acc;

  uint32_t lookahead;
  
  float32_t dngarr[10000];
  float32_t velarr[10000];
  float32_t accarr[10000];
  float32_t posarr[10000];
  bool checksafe[10000];

  bool check_safe(float32_t dist, float32_t vel, float32_t acc);

  // drive param
  float32_t target_vel;
  float32_t cur2tar;
  float32_t safe_dist0;
  float32_t safe_dist1;
  float32_t safe_dist2;
  
  float32_t danger_scale;

  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::Publisher<Command>::SharedPtr command_publisher_;
  // Command compute_command(const CBD & state);
  Command compute_command(float32_t dist);

  // state
  CBD last_cbd_msg;
  rclcpp::Subscription<CBD>::SharedPtr state_subscriber_{};
  void on_state(const CBD::SharedPtr & msg);

  // danger
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr danger_subscriber_{};
  void on_danger(const std_msgs::msg::String::SharedPtr & msg);
};
}  // namespace rubis_drive
}  // namespace autoware

#endif  // RUBIS_DRIVE__RUBIS_DRIVE_NODE_HPP_
