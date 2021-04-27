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

#include "rubis_drive/rubis_drive_node.hpp"

using namespace std::chrono_literals;

namespace autoware
{
namespace rubis_drive
{

RubisDriveNode::RubisDriveNode(const rclcpp::NodeOptions & options)
:  Node("rubis_drive", options),
  verbose(true)
{
  // params
  target_vel = static_cast<float32_t>(declare_parameter(
    "target_vel").get<float32_t>());

  // reach target velocity in {cur2tar}s later
  cur2tar = static_cast<float32_t>(declare_parameter(
    "cur2tar").get<float32_t>());

  safe_dist = static_cast<float32_t>(declare_parameter(
    "safe_dist").get<float32_t>());

  danger_scale = static_cast<int32_t>(declare_parameter(
    "danger_scale").get<int32_t>());

  command_publisher_ = this->create_publisher<Command>(
    "/vehicle/vehicle_command", 10);
//   command_timer_ = this->create_wall_timer(
//     1000ms, std::bind(&RubisDriveNode::command_timer_callback, this));

  state_subscriber_ = create_subscription<CBD>(
    "/lgsvl/state_report", 10,
    [this](const CBD::SharedPtr msg) {on_state(msg);}, SubAllocT{});
  
  danger_subscriber_ = create_subscription<std_msgs::msg::String>(
    "rubis_danger", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {on_danger(msg);}, SubAllocT{});
  
  // sched_log params
  auto timestamp = (int32_t) std::time(nullptr);
  auto f_timestamp = (timestamp + 50) / 100 * 100;
  __si = {
    static_cast<int32_t>(declare_parameter(
      "rubis.sched_info.task_id").get<int32_t>()), // task_id
    static_cast<int32_t>(declare_parameter(
      "rubis.sched_info.max_opt").get<int32_t>()), // max_opt
    static_cast<std::string>(declare_parameter(
      "rubis.sched_info.name").get<std::string>()), // name
    static_cast<std::string>(declare_parameter(
      "rubis.sched_info.log_dir").get<std::string>()) + std::to_string(f_timestamp) + ".log", // file
    static_cast<uint64_t>(declare_parameter(
      "rubis.sched_info.exec_time").get<uint64_t>()), // exec_time
    static_cast<uint64_t>(declare_parameter(
      "rubis.sched_info.deadline").get<uint64_t>()), // deadline
    static_cast<uint64_t>(declare_parameter(
      "rubis.sched_info.period").get<uint64_t>()) // period
  };
  __slog = SchedLog(__si);
  __iter = 0;

  for(int i = 0; i < __si.max_option; i++) {
    __rt_configured.push_back(false);
  }
  return;
}

// void RubisDriveNode::command_timer_callback()
// {
//   RCLCPP_WARN(get_logger(), "(Command) Timer triggered.");
// //   auto message = compute_command();
//   auto message = 1;
//   command_publisher_->publish(message);
// }

void RubisDriveNode::on_state(const CBD::SharedPtr & msg)
{
  last_cbd_msg = *msg;
}

void RubisDriveNode::on_danger(const std_msgs::msg::String::SharedPtr & msg)
{
  std::string collision_distance = msg->data;
  auto dist = static_cast<float32_t>(
    std::stod(collision_distance, nullptr));

  auto cmd = compute_command(dist);
  command_publisher_->publish(cmd);
}

Command RubisDriveNode::compute_command(float32_t dist)
{
  omp_set_dynamic(0);
  auto start_time = omp_get_wtime();
  cur_vel = last_cbd_msg.speed_mps;
  std::cout << "current_velocity = " << cur_vel << std::endl;
  std::cout << "dist/safe_dist = " << dist << "/" << safe_dist << std::endl;

  // compute danger
  float32_t danger;
  if(dist >= safe_dist) {
    danger = 0;
  } else if((safe_dist - dist) * danger_scale < 100) {
    danger = (safe_dist - dist) * danger_scale;
  } else {
    danger = 100;
  }
  std::cout << "danger: " << danger << std::endl;

  // determine accel
  float32_t accel = static_cast<float32_t>(
    (target_vel - cur_vel) / cur2tar - danger * danger / 100);
  if(accel > 0.0) {
    std::cout << "accel: " << accel << std::endl;
  } else {
    std::cout << "accel: " << accel << " (BRAKE)" << std::endl;
  }

  // construct steering command
  Command ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  ret.stamp = last_cbd_msg.header.stamp;
  ret.front_wheel_angle_rad = Real{};  // zero initialization etc.
  ret.rear_wheel_angle_rad = Real{};
  ret.long_accel_mps2 = accel;

  // log
  auto end_time = omp_get_wtime();
  auto response_time = (end_time - start_time) * 1e3;
  sched_data sd {
    ++__iter,  // iter
    response_time,  // response_time
    start_time,  // start_time
    end_time  // end_time
  };
  __slog.add_entry(sd);
  return ret;
}

}  // namespace rubis_drive
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_drive::RubisDriveNode)
