// Copyright 2017-2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Boilerplate Apex.OS nodes around point_cloud_filter_transform_nodes
namespace point_cloud_filter_transform_nodes
{
using autoware::common::lidar_utils::add_point_to_cloud;
using autoware::common::lidar_utils::add_point_to_cloud_raw;
using autoware::common::lidar_utils::add_point_to_cloud_parallel;
using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::reset_pcl_msg;
using autoware::common::lidar_utils::resize_pcl_msg;
using autoware::common::lidar_utils::sanitize_point_cloud;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZIF;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::PointCloud2;

TransformStamped get_transform(
  const std::string & input_frame_id,
  const std::string & output_frame_id,
  float64_t r_x, float64_t r_y, float64_t r_z, float64_t r_w, float64_t t_x,
  float64_t t_y, float64_t t_z)
{
  TransformStamped ret;
  ret.header.frame_id = input_frame_id;
  ret.child_frame_id = output_frame_id;
  ret.transform.rotation.x = r_x;
  ret.transform.rotation.y = r_y;
  ret.transform.rotation.z = r_z;
  ret.transform.rotation.w = r_w;
  ret.transform.translation.x = t_x;
  ret.transform.translation.y = t_y;
  ret.transform.translation.z = t_z;
  return ret;
}

PointCloud2FilterTransformNode::PointCloud2FilterTransformNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_filter_transform_node", node_options),
  m_angle_filter{
    static_cast<float32_t>(declare_parameter("start_angle").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("end_angle").get<float64_t>())},
  m_distance_filter{
    static_cast<float32_t>(declare_parameter("min_radius").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("max_radius").get<float64_t>())},
  m_input_frame_id{declare_parameter("input_frame_id").get<std::string>()},
  m_output_frame_id{declare_parameter("output_frame_id").get<std::string>()},
  m_init_timeout{std::chrono::milliseconds{declare_parameter("init_timeout_ms").get<int32_t>()}},
  m_timeout{std::chrono::milliseconds{declare_parameter("timeout_ms").get<int32_t>()}},
  m_sub_ptr{create_subscription<PointCloud2>(
      "points_in", rclcpp::QoS{10},
      std::bind(
        &PointCloud2FilterTransformNode::process_filtered_transformed_message, this, _1))},
  m_pub_ptr{create_publisher<PointCloud2>("points_filtered", rclcpp::QoS{10})},
  m_expected_num_publishers{
    static_cast<size_t>(declare_parameter("expected_num_publishers").get<int32_t>())},
  m_expected_num_subscribers{
    static_cast<size_t>(declare_parameter("expected_num_subscribers").get<int32_t>())},
  m_pcl_size{static_cast<size_t>(declare_parameter("pcl_size").get<int32_t>())}
{  /// Declare transform parameters with the namespace

  // sched_log params
  auto timestamp = static_cast<int32_t>( std::time(nullptr));
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

  this->declare_parameter("static_transformer.quaternion.x");
  this->declare_parameter("static_transformer.quaternion.y");
  this->declare_parameter("static_transformer.quaternion.z");
  this->declare_parameter("static_transformer.quaternion.w");
  this->declare_parameter("static_transformer.translation.x");
  this->declare_parameter("static_transformer.translation.y");
  this->declare_parameter("static_transformer.translation.z");

  /// Declare objects to hold transform parameters
  rclcpp::Parameter quat_x_param;
  rclcpp::Parameter quat_y_param;
  rclcpp::Parameter quat_z_param;
  rclcpp::Parameter quat_w_param;
  rclcpp::Parameter trans_x_param;
  rclcpp::Parameter trans_y_param;
  rclcpp::Parameter trans_z_param;


  /// If transform parameters exist in the param file use them
  if (this->get_parameter("static_transformer.quaternion.x", quat_x_param) &&
    this->get_parameter("static_transformer.quaternion.y", quat_y_param) &&
    this->get_parameter("static_transformer.quaternion.z", quat_z_param) &&
    this->get_parameter("static_transformer.quaternion.w", quat_w_param) &&
    this->get_parameter("static_transformer.translation.x", trans_x_param) &&
    this->get_parameter("static_transformer.translation.y", trans_y_param) &&
    this->get_parameter("static_transformer.translation.z", trans_z_param))
  {
    RCLCPP_WARN(get_logger(), "Using transform from file.");
    m_static_transformer = std::make_unique<StaticTransformer>(
      get_transform(
        m_input_frame_id, m_output_frame_id,
        quat_x_param.as_double(),
        quat_y_param.as_double(),
        quat_z_param.as_double(),
        quat_w_param.as_double(),
        trans_x_param.as_double(),
        trans_y_param.as_double(),
        trans_z_param.as_double()).transform);
  } else {  /// Else lookup transform being published on /tf or /static_tf topics
    /// TF buffer
    tf2_ros::Buffer tf2_buffer(this->get_clock());
    /// TF listener
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    while (rclcpp::ok()) {
      try {
        RCLCPP_INFO(get_logger(), "Looking up the transform.");
        m_static_transformer = std::make_unique<StaticTransformer>(
          tf2_buffer.lookupTransform(
            m_output_frame_id, m_input_frame_id,
            tf2::TimePointZero).transform);
        break;
      } catch (const std::exception & transform_exception) {
        RCLCPP_INFO(get_logger(), "No transform was available. Retrying after 100 ms.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }
  }
  common::lidar_utils::init_pcl_msg(
    m_filtered_transformed_msg,
    m_output_frame_id.c_str(), m_pcl_size);
}

const PointCloud2 & PointCloud2FilterTransformNode::filter_and_transform(const PointCloud2 & msg)
{
  // Verify frame_id
  if (msg.header.frame_id != m_input_frame_id) {
    throw std::runtime_error(
            "Raw topic from unexpected frame. Expected: " +
            m_input_frame_id + ", got: " + msg.header.frame_id);
  }

  auto && intensity_temp = autoware::common::lidar_utils::IntensityIteratorWrapper(msg);
  size_t num_it = 0;
  while(!intensity_temp.eof()) {
    intensity_temp.next();
    num_it++;
  }

  auto point_cloud_idx = 0U;
  reset_pcl_msg(m_filtered_transformed_msg, m_pcl_size, point_cloud_idx);
  m_filtered_transformed_msg.header.stamp = msg.header.stamp;

  omp_set_dynamic(0);
  #pragma omp parallel num_threads(__si.max_option)
  {
    // configure rt
    auto thr_id = omp_get_thread_num();

    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      RCLCPP_INFO(get_logger(), "(" + std::to_string(tid) + "): __rt_configured (" + std::to_string(__si.exec_time) + ", " + std::to_string(__si.deadline) + ", " + std::to_string(__si.period) + ")");
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    // RCLCPP_INFO(get_logger(), "omp start");

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();

    size_t forloopiteration = 0;
    if(msg.data.size()/16 > num_it) {
      forloopiteration = num_it;
    } else {
      forloopiteration = msg.data.size()/16;
    }

    auto && intensity_it = autoware::common::lidar_utils::IntensityIteratorWrapper(msg);
    sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

    #pragma omp for schedule(dynamic) nowait
    for (size_t it = 0; it < forloopiteration; it++) {
      PointXYZIF pt;     
      uint32_t i = static_cast<uint32_t>(it);

      intensity_it.get_nth_value(pt.intensity, i);
      // implemented direct access
      pt.x = *(x_it+i);
      pt.y = *(y_it+i);
      pt.z = *(z_it+i);

      // RCLCPP_INFO(get_logger(), "it: " + it);
      // RCLCPP_INFO(get_logger(), "size of intensity_it: " + sizeof(intensity_it));
      // RCLCPP_INFO(get_logger(), "size of x_it: " + sizeof(x_it));

      if (point_not_filtered(pt)) {
        auto transformed_point = transform_point(pt);
        transformed_point.intensity = pt.intensity; 
        
        uint32_t local_idx;
        #pragma omp critical (idx_lock)
        {
          local_idx = point_cloud_idx;
          point_cloud_idx += 1;
        }
        //  RCLCPP_INFO(get_logger(), "for");
        // if (!add_point_to_cloud_raw(m_filtered_transformed_msg, transformed_point, static_cast<uint32_t>(it) )) {
        if (!add_point_to_cloud_parallel(m_filtered_transformed_msg, transformed_point, local_idx)) {
          throw std::runtime_error("Overran cloud msg point capacity");
        }
      }
    }
    // for (size_t it = 0; it < (msg.data.size() / 16); it++) {
    //   if(!intensity_it.eof()) {
    //     PointXYZIF pt;
    //     pt.x = *x_it;
    //     pt.y = *y_it;
    //     pt.z = *z_it;
    //     intensity_it.get_current_value(pt.intensity);

    //     if (point_not_filtered(pt)) {
    //       auto transformed_point = transform_point(pt);
    //       transformed_point.intensity = pt.intensity;
    //         #pragma omp critical
    //         {
    //           if (!add_point_to_cloud(m_filtered_transformed_msg, transformed_point, point_cloud_idx))
    //           {
    //             throw std::runtime_error("Overran cloud msg point capacity");
    //           }
    //         }
    //     }

    //     ++x_it;
    //     ++y_it;
    //     ++z_it;
    //     intensity_it.next();
    //   }
    // }

    // RCLCPP_INFO(get_logger(), "for end");
    // #pragma omp flush (abort)
    // workload end
    auto end_time = omp_get_wtime();
    auto response_time = (end_time - start_time) * 1e3;
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
    // RCLCPP_INFO(get_logger(), "omp end");
  }  // pragma omp parallel
  ++__iter;

  resize_pcl_msg(m_filtered_transformed_msg, point_cloud_idx);
  return m_filtered_transformed_msg;
}

void
PointCloud2FilterTransformNode::process_filtered_transformed_message(
  const PointCloud2::SharedPtr msg)
{
  const auto filtered_transformed_msg = filter_and_transform(*msg);
  m_pub_ptr->publish(filtered_transformed_msg);
}


}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode)
