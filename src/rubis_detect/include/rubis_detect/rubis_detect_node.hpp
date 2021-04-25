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

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry/intersection.hpp>
#include <geometry/bounding_box/rotating_calipers.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>
#include <controller_common/controller_base.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <common/types.hpp>
#include <list>
#include <chrono>
#include <time_utils/time_utils.hpp>
#include <ctime>
#include <omp.h>

#include <rubis_detect/rubis_detect.hpp>
#include "rubis_rt/sched.hpp"
#include "rubis_rt/sched_log.hpp"


namespace autoware
{
namespace rubis_detect
{

using motion::control::controller_common::State;
using motion::control::controller_common::Real;
using motion::motion_common::VehicleConfig;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using motion::motion_common::from_angle;
using motion::motion_common::to_angle;
using autoware_auto_msgs::msg::BoundingBox;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::Complex32;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;
using geometry_msgs::msg::Point32;
using time_utils::to_message;
using time_utils::from_message;
using TimeStamp = builtin_interfaces::msg::Time;
using rubis::sched_log::SchedLog;
using rubis::sched_log::sched_info;
using rubis::sched_log::sched_data;
using namespace std::chrono_literals;

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
  SchedLog __slog;
  sched_info __si;
  int32_t __iter;

  // rt
  std::vector<bool8_t> __rt_configured;

  float32_t safety_factor;
  float32_t stop_margin;
  float32_t lf;
  float32_t lr;
  float32_t wh;
  float32_t vehicle_length;
  float32_t vehicle_width;
  float32_t vehicle_diagonal;
  float32_t distance_threshold;
  int32_t lookahead_boxes;
  void init_vehicle(const VehicleConfig & _vehicle_param);
  bool verbose;  ///< whether to use verbose output or not.
  Point32 last_p;
  Complex32 last_heading;
  TimeStamp last_timestamp;
  std::string last_frame_id;

  rclcpp::Subscription<State>::SharedPtr state_subscriber_{};
  bool8_t has_received_state = false;
  void on_state(const State::SharedPtr & msg);
  void save_state(const State & state);

  rclcpp::Subscription<BoundingBoxArray>::SharedPtr bounding_box_subscriber_{};
  void on_bounding_box(const BoundingBoxArray::SharedPtr & msg);

  bool8_t has_received_bounding_box = false;
  BoundingBoxArray::SharedPtr last_bboxes;
  rclcpp::TimerBase::SharedPtr danger_timer_;
  void danger_timer_callback();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr danger_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr danger_publisher_debug_;
  std_msgs::msg::String compute_danger(const BoundingBoxArray & msg);

  BoundingBox point_to_box(const Point32 _p, const Complex32 _heading);
  BoundingBox point_to_box_alt(const Point32 _p, const Complex32 _heading);
  std::vector<Point32> get_expected_trajectory();
  std::vector<Point32> get_expected_trajectory_alt(const Point32 _p, const Complex32 _heading);
  int32_t detect_collision(const Point32 _p, const Complex32 _heading, const BoundingBoxArray & obstacles);
  bool8_t is_too_far_away(const Point32 _p, const BoundingBox obstacle_bbox, const float32_t distance_threshold);
  float32_t calc_collision_distance(int32_t collision_index);
  MarkerArray to_visualization_marker_array(const BoundingBoxArray bboxes, const int32_t collision_idx);

//   std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
//   std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};
}  // namespace rubis_detect
}  // namespace autoware

#endif  // RUBIS_DETECT__RUBIS_DETECT_NODE_HPP_
