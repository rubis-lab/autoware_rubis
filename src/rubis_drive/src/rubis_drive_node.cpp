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

  safe_dist0 = static_cast<float32_t>(declare_parameter(
    "safe_dist0").get<float32_t>());
  safe_dist1 = static_cast<float32_t>(declare_parameter(
    "safe_dist1").get<float32_t>());
  safe_dist2 = static_cast<float32_t>(declare_parameter(
    "safe_dist2").get<float32_t>());
  danger_scale = static_cast<float32_t>(declare_parameter(
    "danger_scale").get<float32_t>());

  lookahead = static_cast<uint32_t>(declare_parameter(
    "lookahead").get<uint32_t>());

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

  cur_vel = 0;
  cur_acc = 0;

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

bool RubisDriveNode::check_safe(float32_t dist, float32_t vel, float32_t acc) {
  if(acc >= 0)
    return false;
  else{
    if(dist > -1/2*vel*vel/acc)
      return true;
    else
      return false;
  }
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

  // configure rt

  auto thr_id = 0;
  auto start_time = omp_get_wtime();
  cur_vel = last_cbd_msg.speed_mps;
  std::cout << "cur_vel/dist/safe_dist0 = " << cur_vel << "/" << dist << "/" << safe_dist0 << std::endl;
  #pragma omp parallel num_threads(__si.max_option)
  {
    auto thr_id = omp_get_thread_num();
    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[RubisDriveNode] (" << tid << "): __rt_configured (" << __si.exec_time << ", " << __si.deadline << ", " << __si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    auto start_time = omp_get_wtime();

    #pragma omp for schedule(dynamic) nowait
    for(uint32_t i=0; i<lookahead; i++) {
      float32_t danger;
      // float32_t time = static_cast<float32_t>(
      //   static_cast<float32_t>(i)/lookahead*10);
      float32_t time = 10.0/lookahead;
      float32_t tempvel = cur_vel;
      float32_t temppos = 0;
      float32_t tempacc = 0;
      float32_t tempsafe = 100;

      if(tempvel < target_vel/3) {
        tempsafe = safe_dist0;
      } else if(tempvel < target_vel) {
        tempsafe = safe_dist1;
      } else {
        tempsafe = safe_dist2;
      }

      float32_t local_dist = dist - temppos;
      
      if(local_dist >= tempsafe) {
        danger = 0.0;
      } else if( (tempsafe - local_dist) * danger_scale < 100.0) {
        danger = (tempsafe - local_dist) * danger_scale;
      } else {
        danger = 100.0;
      }
      
      tempacc = static_cast<float32_t>(
        (target_vel - tempvel) / cur2tar - danger * danger / 100);
      

      for(uint32_t j=1; j<i; j++) {
        temppos = temppos + tempvel * time;
        tempvel = tempvel + tempacc * time;
        
        local_dist = dist - temppos;
        if(local_dist >= tempsafe) {
          danger = 0.0;
        } else if( (tempsafe - local_dist) * danger_scale < 100.0) {
          danger = (tempsafe - local_dist) * danger_scale;
        } else {
          danger = 100.0;
        }
        
        tempacc = static_cast<float32_t>(
          (target_vel - tempvel) / cur2tar - danger * danger / 100);
      }
      
      
      dngarr[i] = danger;
      posarr[i] = temppos;
      velarr[i] = tempvel;
      accarr[i] = tempacc;
      // checksafe[i] = check_safe(local_dist, tempvel, tempacc);
      //Xarr[i] X after i/lookahead second
    }

        // log
    auto end_time = omp_get_wtime();
    auto response_time = (end_time - start_time) * 1e3;
    sched_data sd {
      thr_id,
      __iter,  // iter
      start_time,  // start_time
      end_time,  // end_time
      response_time   //response_time
    };

    std::string dist_report;
    dist_report = "distance$$$$" + std::to_string(dist);
    #pragma omp critical (log_lock)
    {
      __slog.add_entry(sd, dist_report);
    }
    sched_yield();
  }

  
  cur_acc = accarr[0];
  std::cout << "danger/acc = " << static_cast<float32_t>(dngarr[0]) << "/" << static_cast<float32_t>(accarr[0]) << std::endl;

  // construct steering command
  Command ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  ret.stamp = last_cbd_msg.header.stamp;
  ret.front_wheel_angle_rad = Real{};  // zero initialization etc.
  ret.rear_wheel_angle_rad = Real{};
  
  ret.long_accel_mps2 = cur_acc;

  __iter++;
  return ret;
  // // compute danger
  // float32_t danger;
  // if(dist >= safe_dist) {
  //   danger = 0;
  // } else if((safe_dist - dist) * danger_scale < 100) {
  //   danger = (safe_dist - dist) * danger_scale;
  // } else {
  //   danger = 100;
  // }
  // std::cout << "danger: " << danger << std::endl;

  // // determine accel
  // float32_t accel = static_cast<float32_t>(
  //   (target_vel - cur_vel) / cur2tar - danger * danger / 100);
  // if(accel > 0.0) {
  //   std::cout << "accel: " << accel << std::endl;
  // } else {
  //   std::cout << "accel: " << accel << " (BRAKE)" << std::endl;
  // }
}
}  // namespace rubis_drive
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_drive::RubisDriveNode)


      // posarr[i] = cur_vel * time;
      // velarr[i] = cur_vel + cur_acc * time;

      // float32_t local_dist = dist - posarr[i]; //dist to object
      // float32_t local_vel = velarr[i];

      // if(local_dist >= safe_dist) {
      //   danger = 0;
      // } else if((safe_dist-local_dist)*danger_scale < 100) {
      //   danger = (safe_dist-local_dist) * danger_scale;
      // } else {
      //   danger = 100;
      // }
