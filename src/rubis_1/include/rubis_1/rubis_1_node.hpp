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
/// \brief This file defines the rubis_1_node class.

#ifndef RUBIS_1__RUBIS_1_NODE_HPP_
#define RUBIS_1__RUBIS_1_NODE_HPP_

#include <rubis_1/rubis_1.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace autoware
{
namespace rubis_1
{

/// \class Rubis1Node
/// \brief ROS 2 Node for hello world.
class RUBIS_1_PUBLIC Rubis1Node : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit Rubis1Node(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;
  int work_hard(unsigned int _msec) const;

private:
  bool verbose;  ///< whether to use verbose output or not.
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // namespace rubis_1
}  // namespace autoware

#endif  // RUBIS_1__RUBIS_1_NODE_HPP_
