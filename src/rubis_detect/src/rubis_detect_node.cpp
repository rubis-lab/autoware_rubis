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

#include "rubis_detect/rubis_detect_node.hpp"

namespace autoware
{
namespace rubis_detect
{

RubisDetectNode::RubisDetectNode(const rclcpp::NodeOptions & options)
:  Node("rubis_detect", options),
  verbose(true)
{
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

  __use_timer = static_cast<bool8_t>(declare_parameter(
      "rubis.use_timer").get<bool8_t>());

  // config
  const auto vehicle_param = VehicleConfig{
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_front_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_rear_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.mass_kg"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.yaw_inertia_kgm2"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.width_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_overhang_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_overhang_m"
    ).get<float32_t>())
  };
  safety_factor =
    static_cast<float32_t>(declare_parameter(
      "safety_factor"
    ).get<float32_t>());

  stop_margin =
    static_cast<float32_t>(declare_parameter(
      "stop_margin"
    ).get<float32_t>());

  lookahead_boxes =
    static_cast<int32_t>(declare_parameter(
      "lookahead_boxes"
    ).get<int32_t>());
  box_ratio =
    static_cast<float32_t>(declare_parameter(
      "box_ratio"
    ).get<float32_t>());
  init_vehicle(vehicle_param);

  // init
  last_p = Point32{};
  last_p.x = 0;
  last_p.y = 0;
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  state_subscriber_ = create_subscription<State>(
    "/vehicle/vehicle_kinematic_state", 10,
    [this](const State::SharedPtr msg) {on_state(msg);}, SubAllocT{});

  bounding_box_subscriber_ = create_subscription<BoundingBoxArray>(
    "/perception/lidar_bounding_boxes", 10,
    [this](const BoundingBoxArray::SharedPtr msg) {on_bounding_box(msg);}, SubAllocT{});

  danger_publisher_ = this->create_publisher<std_msgs::msg::String>("rubis_danger", 10);
  danger_publisher_debug_ = this->create_publisher<MarkerArray>("rubis_danger_debug", 10);

  // timer
  if(__use_timer) {
    auto period = std::chrono::milliseconds{
      static_cast<uint32_t>(__si.period / 1000000)
    };
    // auto period = static_cast<std::chrono::milliseconds>(__si.period/1000000)
    danger_timer_ = this->create_wall_timer(
      period, std::bind(&RubisDetectNode::danger_timer_callback, this));
  }
}

void RubisDetectNode::init_vehicle(const VehicleConfig & _vehicle_param)
{
  lf = _vehicle_param.length_cg_front_axel() + _vehicle_param.front_overhang();
  lr = _vehicle_param.length_cg_rear_axel() + _vehicle_param.rear_overhang();
  wh = _vehicle_param.width() * 0.5f;
  // inflate size of vehicle by safety factor
  lf *= safety_factor;
  lr *= safety_factor;
  wh *= safety_factor;

  // find the dimension of the ego vehicle.
  vehicle_length =
    _vehicle_param.front_overhang() + _vehicle_param.length_cg_front_axel() +
    _vehicle_param.length_cg_rear_axel() + _vehicle_param.rear_overhang();
  vehicle_width = _vehicle_param.width();
  vehicle_diagonal = sqrtf(
    (vehicle_width * vehicle_width) + (vehicle_length * vehicle_length));

  // define a distance threshold to filter obstacles that are too far away to cause any collision.
  distance_threshold = vehicle_diagonal * safety_factor;
  return;
}

int32_t RubisDetectNode::print_hello() const
{
  return rubis_detect::print_hello();
}

void RubisDetectNode::on_state(const State::SharedPtr & msg)
{
  has_received_state = true;
  save_state(*msg);
  if(!__use_timer) {
    danger_timer_callback();
  }
  return;
}

void RubisDetectNode::save_state(const State & state)
{
  last_p.x = state.state.x;
  last_p.y = state.state.y;
  last_heading = state.state.heading;
  last_timestamp = state.header.stamp;
  last_frame_id = state.header.frame_id; // odom
//   RCLCPP_WARN(get_logger(), "RubisDetectNode::on_state: frame_id" + state.header.frame_id);
  return;
}

void RubisDetectNode::on_bounding_box(const BoundingBoxArray::SharedPtr & msg)
{
  if(!has_received_state) {
    // RCLCPP_WARN(get_logger(), "RubisDetectNode::on_bounding_box: did not receive state yet.");
    return;
  }
  has_received_bounding_box = true;
  last_bboxes = msg;
  return;
}

void RubisDetectNode::danger_timer_callback()
{
  // if(__use_timer) {
  //   RCLCPP_WARN(get_logger(), "RubisDetectNode::timer callback called!");
  // } else {
  //   RCLCPP_WARN(get_logger(), "RubisDetectNode::data callback called!");
  // }
  if(!has_received_bounding_box) {
    // RCLCPP_WARN(get_logger(), "RubisDetectNode::danger_timer_callback: did not receive bbox yet.");
    return;
  }

  // has_received_bounding_box = false;
  const auto danger{compute_danger(*last_bboxes)};
  danger_publisher_->publish(danger);
  return;
}

std_msgs::msg::String RubisDetectNode::compute_danger(const BoundingBoxArray & msg)
{
  // rubis_danger
  // data : {distance}
  auto collision_index = detect_collision(last_p, last_heading, msg);
  auto collision_distance = calc_collision_distance(collision_index);

  auto message = std_msgs::msg::String();
  message.data = std::to_string(collision_distance);
  return message;
}

// modified from object_collision_estimator.cpp
BoundingBox RubisDetectNode::point_to_box(const Point32 _p, const Complex32 _heading)
{
  // Shorthands to keep the formulas sane
  float32_t angle = to_angle(_heading);
  float32_t ch = std::cos(angle);
  float32_t sh = std::sin(angle);

  // Create a list of corners for the vehicle
  std::list<Point32> corners;
  {     // Front left
    auto p = Point32{};
    p.x = _p.x + (box_ratio * 0.5 * ch) - (1 * sh);
    p.y = _p.y + (box_ratio * 0.5 * sh) + (1 * ch);
    corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = _p.x + (box_ratio * 0.5 * ch) + (1 * sh);
    p.y = _p.y + (box_ratio * 0.5 * sh) - (1 * ch);
    corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = _p.x - (box_ratio * 0.5 * ch) + (1 * sh);
    p.y = _p.y - (box_ratio * 0.5 * sh) - (1 * ch);
    corners.push_back(p);
  }
  {     // Rear left
    auto p = Point32{};
    p.x = _p.x - (box_ratio * 0.5 * ch) - (1 * sh);
    p.y = _p.y - (box_ratio * 0.5 * sh) + (1 * ch);
    corners.push_back(p);
  }

  return minimum_perimeter_bounding_box(corners);
}

std::vector<Point32> RubisDetectNode::get_expected_trajectory()
{
  auto origin = Point32{};
  origin.x = 0;
  origin.y = 0;

  std::vector<Point32> expected_trajectory;
  for(int32_t i = 0; i < lookahead_boxes; i++) {
    auto p = Point32{};
    // p.x = origin.x + i * (lf + lr);
    p.x = origin.x + box_ratio * (lf+lr) * i;
    p.y = origin.y;
    expected_trajectory.push_back(p);
  }
  return expected_trajectory;
}

int32_t RubisDetectNode::detect_collision(const Point32 _p, const Complex32 _heading, const BoundingBoxArray & obstacles)
{
  int32_t collision_index = -1;

  auto expected_trajectory = get_expected_trajectory();
  // RCLCPP_WARN(get_logger(), "RubisDetectNode::get expected trajectory");
  // RCLCPP_WARN(get_logger(), "RubisDetectNode::expected trajectory_size: " + std::to_string(expected_trajectory.size()));

  BoundingBoxArray bboxes_debug;  // for visualization
  auto t_true = from_message(last_timestamp);
  auto t_buff = t_true - std::chrono::milliseconds(100);
  auto new_timestamp = to_message(t_buff);

  bboxes_debug.header.stamp = new_timestamp;
  bboxes_debug.header.frame_id = "base_link";
  // int32_t t_idx = 0;
  omp_set_dynamic(0);
  
  
  #pragma omp parallel num_threads(__si.max_option)
  {
    // configure rt
    auto thr_id = omp_get_thread_num();

    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[RubisDetectNode] (" << tid << "): __rt_configured (" << __si.exec_time << ", " << __si.deadline << ", " << __si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();
    size_t traj_size = expected_trajectory.size();
    size_t debug_interval = static_cast<size_t>(traj_size / 256) + 1;

    #pragma omp for schedule(dynamic) nowait
    // #pragma omp for schedule(static) nowait
    for(size_t i = 0; i < traj_size; i++) {
    // for(auto const& p : expected_trajectory) {
        auto heading_straight = from_angle(0.0F);
        // const auto p_box = point_to_box(p, _heading);
        // RCLCPP_WARN(get_logger(), "RubisDetectNode::iter" + std::to_string(i));
        const auto p_box = point_to_box(expected_trajectory[i], heading_straight);
        

        #pragma omp critical (bbox_lock)
        {
          if( (i % debug_interval) == 0) {
            bboxes_debug.boxes.push_back(p_box);
          } 
        }

        for(const auto & obstacle_bbox : obstacles.boxes) {
          if(!is_too_far_away(expected_trajectory[i], obstacle_bbox, distance_threshold)) {
            if(autoware::common::geometry::intersect(
              p_box.corners.begin(), p_box.corners.end(),
              obstacle_bbox.corners.begin(), obstacle_bbox.corners.end())) {

              // Collision detected
              if(collision_index == -1) {
                // collision_index = t_idx;
                collision_index = static_cast<int32_t>(i);
                // RCLCPP_WARN(get_logger(), "RubisDetectNode::detect_collision: collision: " + std::to_string(collision_index));
              }
            }
          }
        }
        // t_idx++;
    }

    // workload end
    auto end_time = omp_get_wtime();
    auto response_time = (end_time - start_time) * 1e3;
    
    
    // log
    sched_data sd {
      thr_id, // thr_id
      __iter,  // iter
      start_time,  // start_time
      end_time,  // end_time
      response_time  // response_time
    };


    #pragma omp critical (log_lock)
    {
      __slog.add_entry(sd);
    }
    sched_yield();
  }  // prama omp parallel

  ++__iter;
  if(collision_index == -1) {
    // RCLCPP_WARN(get_logger(), "RubisDetectNode::detect_collision: No collision");
    collision_index = lookahead_boxes;
  }
  // debug publisher
//   auto marker = to_visualization_marker_array(bboxes_debug, collision_index);
  auto marker = to_visualization_marker_array(bboxes_debug, collision_index);
  danger_publisher_debug_->publish(marker);

  return collision_index;
}

bool8_t RubisDetectNode::is_too_far_away(const Point32 _p, const BoundingBox obstacle_bbox, const float32_t distance_threshold)
{
  bool is_too_far_away{true};
  auto distance_threshold_squared = distance_threshold * distance_threshold;

  for (auto corner : obstacle_bbox.corners) {
    auto dx = corner.x - _p.x;
    auto dy = corner.y - _p.y;
    auto distance_squared = (dx * dx) + (dy * dy);

    if (distance_threshold_squared > distance_squared) {
      is_too_far_away = false;
      break;
    }
  }

  return is_too_far_away;
}

float32_t RubisDetectNode::calc_collision_distance(int32_t collision_index)
{
  // float32_t distance_increment = (lf + lr);
  float32_t distance_increment = box_ratio * (lf+lr);
  float32_t max_distance = lookahead_boxes * distance_increment;
  if(collision_index == -1) {
    return max_distance;
  }
  return distance_increment * collision_index;
}

MarkerArray RubisDetectNode::to_visualization_marker_array(const BoundingBoxArray bboxes, const int32_t collision_idx)
{
  MarkerArray marker_array{};

  // delete previous markers
  // TODO(mitsudome-r): remove delete_marker once lifetime is supported by rviz
  Marker delete_marker{};
  delete_marker.header = bboxes.header;
  delete_marker.ns = "bounding_box";
  delete_marker.action = Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // create markers for bounding boxes
  Marker marker{};
  marker.header = bboxes.header;
  marker.ns = "bounding_box";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.lifetime = time_utils::to_message(std::chrono::nanoseconds(100000));

  for (std::size_t i = 0; i < bboxes.boxes.size(); ++i) {
    marker.id = static_cast<int32_t>(i);
    marker.pose.orientation.w = 1.0;
    if (i < collision_idx) {
      marker.scale.x = 0.2;
      marker.color.a = 0.8F;
      marker.color.r = 0.0F;
      marker.color.g = 1.0F;
      marker.color.b = 0.0F;
    } else if (i == collision_idx) {
      marker.scale.x = 0.2;
      marker.color.a = 1.0F;
      marker.color.r = 1.0F;
      marker.color.g = 0.0F;
      marker.color.b = 0.0F;
    } else {
      marker.scale.x = 0.2;
      marker.color.a = 0.8F;
      marker.color.r = 0.7F;
      marker.color.g = 0.7F;
      marker.color.b = 0.7F;
    }
    marker.points.clear();
    const auto box = bboxes.boxes.at(i);
    for (std::size_t j = 0; j < 4; ++j) {
      geometry_msgs::msg::Point point;
      point.x = static_cast<float64_t>(box.corners.at(j).x);
      point.y = static_cast<float64_t>(box.corners.at(j).y);
      point.z = static_cast<float64_t>(box.corners.at(j).z);
      marker.points.push_back(point);
    }
    geometry_msgs::msg::Point point;
    point.x = static_cast<float64_t>(box.corners.at(0).x);
    point.y = static_cast<float64_t>(box.corners.at(0).y);
    point.z = static_cast<float64_t>(box.corners.at(0).z);
    marker.points.push_back(point);

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace rubis_detect
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rubis_detect::RubisDetectNode)



// BoundingBox RubisDetectNode::point_to_box_alt(const Point32 _p, const Complex32 _heading)
// {
//   // Shorthands to keep the formulas sane
//   float32_t angle = to_angle(_heading);
//   float32_t ch = std::cos(angle);
//   float32_t sh = std::sin(angle);

//   // Create a list of corners for the vehicle
//   std::list<Point32> corners;
//   {     // Front left
//     auto p = Point32{};
//     p.x = _p.x + (lf * ch) - (wh * sh);
//     p.y = _p.y + (lf * sh) + (wh * ch);
//     corners.push_back(p);
//   }
//   {     // Front right
//     auto p = Point32{};
//     p.x = _p.x + (lf * ch) + (wh * sh);
//     p.y = _p.y + (lf * sh) - (wh * ch);
//     corners.push_back(p);
//   }
//   {     // Rear right
//     auto p = Point32{};
//     p.x = _p.x - (lr * ch) + (wh * sh);
//     p.y = _p.y - (lr * sh) - (wh * ch);
//     corners.push_back(p);
//   }
//   {     // Rear left
//     auto p = Point32{};
//     p.x = _p.x - (lr * ch) - (wh * sh);
//     p.y = _p.y - (lr * sh) + (wh * ch);
//     corners.push_back(p);
//   }
//   return minimum_perimeter_bounding_box(corners);
// }

// std::vector<Point32> RubisDetectNode::get_expected_trajectory_alt(const Point32 _p, const Complex32 _heading)
// {
//   // Shorthands to keep the formulas sane
//   float32_t angle = to_angle(_heading);
//   float32_t ch = std::cos(angle);
//   float32_t sh = std::sin(angle);

//   std::vector<Point32> expected_trajectory;
//   for(int32_t i = 0; i < lookahead_boxes; i++) {
//     auto p = Point32{};
//     p.x = _p.x + i * ((lf + lr) * ch);
//     p.y = _p.y + i * ((lf + lr) * sh);
//     expected_trajectory.push_back(p);
//     // RCLCPP_WARN(get_logger(), "RubisDetectNode::get_expected_trajectory: " + std::to_string(i) + ": (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")");
//   }
//   return expected_trajectory;
// }
