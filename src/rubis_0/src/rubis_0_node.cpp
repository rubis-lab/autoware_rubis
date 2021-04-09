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

#include "rubis_0/rubis_0_node.hpp"

using namespace std::chrono_literals;

namespace autoware
{
namespace rubis_0
{

Rubis0Node::Rubis0Node(const rclcpp::NodeOptions & options)
:  Node("rubis_0", options),
  verbose(true)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_topic", 10);
  timer_ = this->create_wall_timer(
    4000ms, std::bind(&Rubis0Node::timer_callback, this));
}

int32_t Rubis0Node::print_hello() const
{
  RCLCPP_WARN(get_logger(), "RUBIS RUBIS RUBIS RUBIS RUBIS");
  return rubis_0::print_hello();
}

void Rubis0Node::timer_callback()
{
  RCLCPP_WARN(get_logger(), "Timer triggered.");
  int work_result = Rubis0Node::work_hard(2000); // 2s
  auto message = std_msgs::msg::String();
  message.data = "Hello, RUBIS! " + std::to_string(work_result);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  std::cout << "tid: " << syscall(__NR_gettid) << std::endl;
  publisher_->publish(message);
}

int Rubis0Node::work_hard(unsigned int _msec)
{
  // 600000 addition takes about 1msec on an 1.2GHz CPU
  unsigned int iter = 600000 * _msec;
  int sum = 0;
  for(unsigned int i = 0; i < iter; i++) {
    sum++;
  }
  return sum;
}

}  // namespace rubis_0
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_0::Rubis0Node)
