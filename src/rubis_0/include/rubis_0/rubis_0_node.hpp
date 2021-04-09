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
/// \brief This file defines the rubis_0_node class.

#ifndef RUBIS_0__RUBIS_0_NODE_HPP_
#define RUBIS_0__RUBIS_0_NODE_HPP_

#include <rubis_0/rubis_0.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <linux/kernel.h>
#include <linux/types.h>
#include <unistd.h>
#include <sys/syscall.h>

namespace autoware
{
namespace rubis_0
{

/// \class Rubis0Node
/// \brief ROS 2 Node for hello world.
class RUBIS_0_PUBLIC Rubis0Node : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit Rubis0Node(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.'

  // Timer related
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  int work_hard(unsigned int _msec);
};
}  // namespace rubis_0
}  // namespace autoware

#endif  // RUBIS_0__RUBIS_0_NODE_HPP_
