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

#include "rubis_1/rubis_1_node.hpp"

namespace autoware
{
namespace rubis_1
{

Rubis1Node::Rubis1Node(const rclcpp::NodeOptions & options)
:  Node("rubis_1", options),
  verbose(true)
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "rubis_topic", 10, std::bind(&Rubis1Node::topic_callback, this, _1));
}

int32_t Rubis1Node::print_hello() const
{
  return rubis_1::print_hello();
}

void Rubis1Node::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(get_logger(), "I heard: '%s', Now I am going to work.", msg->data.c_str());
  int work_result = Rubis1Node::work_hard(1500); // 1.5s
  RCLCPP_INFO(get_logger(), "Work done" + std::to_string(work_result));
}

int Rubis1Node::work_hard(unsigned int _msec) const
{
  // 600000 addition takes about 1msec on an 1.2GHz CPU
  unsigned int iter = 600000 * _msec;
  int sum = 0;
  for(unsigned int i = 0; i < iter; i++) {
    sum++;
  }
  return sum;
}

}  // namespace rubis_1
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_1::Rubis1Node)
