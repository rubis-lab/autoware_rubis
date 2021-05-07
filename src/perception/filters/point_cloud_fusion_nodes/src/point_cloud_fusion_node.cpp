// Copyright 2019-2021 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <point_cloud_fusion_nodes/point_cloud_fusion_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion_nodes
{

PointCloudFusionNode::PointCloudFusionNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_fusion_nodes", node_options),
  m_cloud_publisher(create_publisher<PointCloudMsgT>("output_topic", rclcpp::QoS(10))),
  m_input_topics(declare_parameter("number_of_sources").get<std::size_t>()),
  m_output_frame_id(declare_parameter("output_frame_id").get<std::string>()),
  m_cloud_capacity(declare_parameter("cloud_size").get<uint32_t>())
{
  // sched_log params
  auto timestamp = (int32_t) std::time(nullptr);
  auto f_timestamp = (timestamp + 50) / 100 * 100;

  sched_info si = {
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

  __use_timer = static_cast<bool8_t>(declare_parameter(
      "rubis.use_timer").get<bool8_t>());

  // std::chrono::nanoseconds{
  //       static_cast<std::uint64_t>(std::floor(nanoseconds_in_second / publish_frequency))};

  // timer
  if(__use_timer) {
    auto period = std::chrono::milliseconds{
      static_cast<uint32_t>(si.period / 1000000)
    };
    __tmr = this->create_wall_timer(
      period, std::bind(&PointCloudFusionNode::handle_timer_callback, this));
  }

  

  for (size_t i = 0; i < m_input_topics.size(); ++i) {
    m_input_topics[i] = "input_topic" + std::to_string(i);
  }
  init();

  m_core->init_rubis(si);
}

void PointCloudFusionNode::init()
{
  m_core = std::make_unique<point_cloud_fusion::PointCloudFusion>(
    m_cloud_capacity,
    m_input_topics.size());

  common::lidar_utils::init_pcl_msg(
    m_cloud_concatenated, m_output_frame_id,
    m_cloud_capacity);

  if (m_input_topics.size() > 8 || m_input_topics.size() < 2) {
    throw std::domain_error(
            "Number of sources for point cloud fusion must be between 2 and 8."
            " Found: " + std::to_string(m_input_topics.size()));
  }

  for (size_t i = 0; i < 8; ++i) {
    if (i < m_input_topics.size()) {
      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[i]);
    } else {
      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[0]);
    }
  }
  m_cloud_synchronizer = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
    SyncPolicyT(10), *m_cloud_subscribers[0], *m_cloud_subscribers[1], *m_cloud_subscribers[2],
    *m_cloud_subscribers[3], *m_cloud_subscribers[4], *m_cloud_subscribers[5],
    *m_cloud_subscribers[6], *m_cloud_subscribers[7]);

  m_cloud_synchronizer->registerCallback(
    std::bind(
      &PointCloudFusionNode::pointcloud_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,
      std::placeholders::_6, std::placeholders::_7, std::placeholders::_8));
}

std::chrono::nanoseconds PointCloudFusionNode::convert_msg_time(builtin_interfaces::msg::Time stamp)
{
  return std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
}

void
PointCloudFusionNode::pointcloud_callback(
  const PointCloudMsgT::ConstSharedPtr & msg1, const PointCloudMsgT::ConstSharedPtr & msg2,
  const PointCloudMsgT::ConstSharedPtr & msg3, const PointCloudMsgT::ConstSharedPtr & msg4,
  const PointCloudMsgT::ConstSharedPtr & msg5, const PointCloudMsgT::ConstSharedPtr & msg6,
  const PointCloudMsgT::ConstSharedPtr & msg7, const PointCloudMsgT::ConstSharedPtr & msg8)
{
  std::cout << "pointcloud!!" << std::endl;
  has_received_point_cloud = true;
  last_point_cloud_1 = msg1;
  last_point_cloud_2 = msg2;
  last_point_cloud_3 = msg3;
  last_point_cloud_4 = msg4;
  last_point_cloud_5 = msg5;
  last_point_cloud_6 = msg6;
  last_point_cloud_7 = msg7;
  last_point_cloud_8 = msg8;

  if(!__use_timer) {
    handle_timer_callback();
  }
  return;
}

void PointCloudFusionNode::handle_timer_callback()
{
  // if(__use_timer) {
  //   RCLCPP_WARN(get_logger(), "PointCloudFusionNode::timer callback called!");
  // } else {
  //   RCLCPP_WARN(get_logger(), "PointCloudFusionNode::data callback called!");
  // }  
  if(!has_received_point_cloud) {
    // RCLCPP_WARN(get_logger(), "PointCloudFusionNode::handle_timer_callback: did not receive point_cloud yet.");
    return;
  }
  handle_periodic(last_point_cloud_1, last_point_cloud_2, last_point_cloud_3, last_point_cloud_4, last_point_cloud_5, last_point_cloud_6, last_point_cloud_7, last_point_cloud_8);
  return;
}

void PointCloudFusionNode::handle_periodic(
  const PointCloudMsgT::ConstSharedPtr & msg1, const PointCloudMsgT::ConstSharedPtr & msg2,
  const PointCloudMsgT::ConstSharedPtr & msg3, const PointCloudMsgT::ConstSharedPtr & msg4,
  const PointCloudMsgT::ConstSharedPtr & msg5, const PointCloudMsgT::ConstSharedPtr & msg6,
  const PointCloudMsgT::ConstSharedPtr & msg7, const PointCloudMsgT::ConstSharedPtr & msg8
)
{
  std::array<PointCloudMsgT::ConstSharedPtr, 8> msgs{msg1, msg2, msg3, msg4, msg5, msg6, msg7,
    msg8};

  uint32_t pc_concat_idx = 0;
  // reset pointcloud before using
  common::lidar_utils::reset_pcl_msg(
    m_cloud_concatenated, m_cloud_capacity,
    pc_concat_idx);

  auto latest_stamp = msgs[0]->header.stamp;
  auto total_size = 0U;

  // Get the latest time stamp of the point clouds and find the total size after concatenation
  for (uint32_t msg_idx = 0; msg_idx < m_input_topics.size(); ++msg_idx) {
    const auto & stamp = msgs[msg_idx]->header.stamp;
    if (convert_msg_time(stamp) > convert_msg_time(latest_stamp)) {
      latest_stamp = stamp;
    }
    total_size += msgs[msg_idx]->width;
  }

  if (total_size > m_cloud_capacity) {
    RCLCPP_WARN(
      get_logger(), "pointclouds that are trying to be fused exceed the cloud capacity. "
      "The exceeded clouds will be ignored.");
  }

  // Go through all the messages and fuse them.
  uint32_t fused_cloud_size = 0;
  // rubis: actual workload (fuse_pc_msgs)
  try {
    fused_cloud_size = m_core->fuse_pc_msgs(msgs, m_cloud_concatenated);
  } catch (point_cloud_fusion::PointCloudFusion::Error fuse_error) {
    if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::TOO_LARGE) {
      RCLCPP_WARN(get_logger(), "Pointcloud is too large to be fused and will be ignored.");
    } else if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::INSERT_FAILED) {
      RCLCPP_ERROR(get_logger(), "Points could not be added correctly to the fused cloud.");
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown error.");
    }
  }

  if (fused_cloud_size > 0) {
    // Resize and publish.
    common::lidar_utils::resize_pcl_msg(m_cloud_concatenated, fused_cloud_size);

    m_cloud_concatenated.header.stamp = latest_stamp;
    m_cloud_publisher->publish(m_cloud_concatenated);
  }
}
}  // namespace point_cloud_fusion_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode)
