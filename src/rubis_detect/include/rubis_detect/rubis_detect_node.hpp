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

namespace autoware
{
namespace rubis_detect
{

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
};
}  // namespace rubis_detect
}  // namespace autoware

#endif  // RUBIS_DETECT__RUBIS_DETECT_NODE_HPP_
