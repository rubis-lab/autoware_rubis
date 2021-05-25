// Copyright 2019-2020 the Autoware Foundation
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
/// \file
/// \brief This file implements a clustering node that published colored point clouds and convex
///        hulls

#include <common/types.hpp>
#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <iostream>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
////////////////////////////////////////////////////////////////////////////////
EuclideanClusterNode::EuclideanClusterNode(
  const rclcpp::NodeOptions & node_options)
: Node("euclidean_cluster_cloud_node", node_options),
  m_cloud_sub_ptr{create_subscription<PointCloud2>(
      "points_in",
      rclcpp::QoS(10),
      [this](const PointCloud2::SharedPtr msg) {handle(msg);})},
  m_cluster_pub_ptr{declare_parameter("use_cluster").get<bool8_t>() ?
  create_publisher<Clusters>(
    "points_clustered",
    rclcpp::QoS(10)) : nullptr},
m_box_pub_ptr{declare_parameter("use_box").get<bool8_t>() ?
  create_publisher<BoundingBoxArray>(
    "lidar_bounding_boxes", rclcpp::QoS{10}) :
  nullptr},
m_marker_pub_ptr{get_parameter("use_box").as_bool() ?
  create_publisher<MarkerArray>(
    "lidar_bounding_boxes_viz", rclcpp::QoS{10}) :
  nullptr},
m_cluster_alg{
  euclidean_cluster::Config{
    declare_parameter("cluster.frame_id").get<std::string>().c_str(),
    static_cast<std::size_t>(declare_parameter("cluster.min_cluster_size").get<std::size_t>()),
    static_cast<std::size_t>(declare_parameter("cluster.max_num_clusters").get<std::size_t>())
  },
  euclidean_cluster::HashConfig{
    static_cast<float32_t>(declare_parameter("hash.min_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.min_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.side_length").get<float32_t>()),
    static_cast<std::size_t>(declare_parameter("max_cloud_size").get<std::size_t>())
  }
},
m_clusters{},
m_boxes{},
m_voxel_ptr{nullptr},  // Because voxel config's Point types don't accept positional arguments
m_use_lfit{declare_parameter("use_lfit").get<bool8_t>()},
m_use_z{declare_parameter("use_z").get<bool8_t>()}
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

  // timer
  if(__use_timer) {
    auto period = std::chrono::milliseconds{
      static_cast<uint32_t>(__si.period / 1000000)
    };
    __tmr = this->create_wall_timer(
      period, std::bind(&EuclideanClusterNode::handle_timer_callback, this));
  }

  init_rubis(m_cluster_alg.get_config());
  // Initialize voxel grid
  if (declare_parameter("downsample").get<bool8_t>()) {
    filters::voxel_grid::PointXYZ min_point;
    filters::voxel_grid::PointXYZ max_point;
    filters::voxel_grid::PointXYZ voxel_size;
    min_point.x = static_cast<float32_t>(declare_parameter("voxel.min_point.x").get<float32_t>());
    min_point.y = static_cast<float32_t>(declare_parameter("voxel.min_point.y").get<float32_t>());
    min_point.z = static_cast<float32_t>(declare_parameter("voxel.min_point.z").get<float32_t>());
    max_point.x = static_cast<float32_t>(declare_parameter("voxel.max_point.x").get<float32_t>());
    max_point.y = static_cast<float32_t>(declare_parameter("voxel.max_point.y").get<float32_t>());
    max_point.z = static_cast<float32_t>(declare_parameter("voxel.max_point.z").get<float32_t>());
    voxel_size.x = static_cast<float32_t>(declare_parameter("voxel.voxel_size.x").get<float32_t>());
    voxel_size.y = static_cast<float32_t>(declare_parameter("voxel.voxel_size.y").get<float32_t>());
    voxel_size.z = static_cast<float32_t>(declare_parameter("voxel.voxel_size.z").get<float32_t>());
    // Aggressive downsampling if not using z
    if (!m_use_z) {
      voxel_size.z = (max_point.z - min_point.z) + 1.0F;
      // Info
      RCLCPP_INFO(get_logger(), "z is not used, height aspect is fully downsampled away");
    }
    m_voxel_ptr = std::make_unique<VoxelAlgorithm>(
      filters::voxel_grid::Config{
              min_point,
              max_point,
              voxel_size,
              static_cast<std::size_t>(get_parameter("max_cloud_size").as_int())
            });
  }
}

////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::init(const euclidean_cluster::Config & cfg)
{
  // Sanity check
  if ((!m_box_pub_ptr) && (!m_cluster_pub_ptr)) {
    throw std::domain_error{"EuclideanClusterNode: No publisher topics provided"};
  }
  // Reserve
  m_clusters.clusters.reserve(cfg.max_num_clusters());
  m_boxes.header.frame_id.reserve(256U);
  m_boxes.header.frame_id = cfg.frame_id().c_str();
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::init_rubis(const euclidean_cluster::Config & cfg)
{
  // Sanity check
  if ((!m_box_pub_ptr) && (!m_cluster_pub_ptr)) {
    throw std::domain_error{"EuclideanClusterNode: No publisher topics provided"};
  }
  // Reserve
  m_clusters.clusters.reserve(cfg.max_num_clusters());
  m_boxes.header.frame_id.reserve(256U);
  m_boxes.header.frame_id = cfg.frame_id().c_str();
  m_cluster_alg.init_rubis(__si);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_plain(const PointCloud2 & cloud)
{
  using euclidean_cluster::PointXYZI;
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto begin = reinterpret_cast<const PointXYZI *>(&cloud.data[0U]);
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto end = reinterpret_cast<const PointXYZI *>(&cloud.data[cloud.row_step]);
  m_cluster_alg.insert(begin, end);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_voxel(const PointCloud2 & cloud)
{
  m_voxel_ptr->insert(cloud);
  insert_plain(m_voxel_ptr->get());
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert(const PointCloud2 & cloud)
{
  if (m_voxel_ptr) {
    insert_voxel(cloud);
  } else {
    insert_plain(cloud);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::publish_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  for (auto & cls : clusters.clusters) {
    cls.header.stamp = header.stamp;
    // frame id was reserved
    cls.header.frame_id = header.frame_id;
  }
  m_cluster_pub_ptr->publish(clusters);
}


// rubis: copied from euclidean_cluster.cpp
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::compute_eigenboxes_with_z_rubis(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();

  omp_set_dynamic(0);
  #pragma omp parallel num_threads(1)
  {
    // configure rt
    auto thr_id = omp_get_thread_num();

    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[EuclideanClusterNode] (" << tid << "): __rt_configured (" << __si.exec_time << ", " << __si.deadline << ", " << __si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();

    #pragma omp for schedule(dynamic) nowait
    for(size_t c_idx = 0; c_idx < clusters.clusters.size(); ++c_idx) {
      // for (auto & cls : clusters.clusters) {
      try {
        const auto iterators = euclidean_cluster::details::point_struct_iterators(clusters.clusters[c_idx]);
        auto box = autoware::common::geometry::bounding_box::eigenbox_2d(iterators.first, iterators.second);
        autoware::common::geometry::bounding_box::compute_height(iterators.first, iterators.second, box);
        #pragma omp critical
        {
          boxes.boxes.push_back(box);
        }
      } catch (const std::exception & e) {
        std::cerr << e.what() << "\n";
      }
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
    // #pragma omp critical
    // {
    //   __slog.add_entry(sd);
    // }
    sched_yield();

  }  // prama omp parallel
  __iter++;
}


////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  if (m_cluster_pub_ptr) {
    publish_clusters(clusters, header);
  }
  if (m_box_pub_ptr) {
    if (m_use_lfit) {
      if (m_use_z) {
        // euclidean_cluster::details::compute_eigenboxes_with_z(clusters, m_boxes);
        compute_eigenboxes_with_z_rubis(clusters, m_boxes);
      } else {
        euclidean_cluster::details::compute_eigenboxes(clusters, m_boxes);
      }
    } else {
      if (m_use_z) {
        euclidean_cluster::details::compute_lfit_bounding_boxes_with_z(clusters, m_boxes);
      } else {
        euclidean_cluster::details::compute_lfit_bounding_boxes(clusters, m_boxes);
      }
    }
    m_boxes.header.stamp = header.stamp;
    // Frame id was reserved
    m_boxes.header.frame_id = header.frame_id;
    m_box_pub_ptr->publish(m_boxes);

    // Also publish boxes for visualization
    uint32_t id_counter = 0;
    MarkerArray marker_array;
    for (const auto & box : m_boxes.boxes) {
      Marker m{};
      m.header.stamp = rclcpp::Time(0);
      m.header.frame_id = header.frame_id;
      m.ns = "bbox";
      m.id = id_counter;
      m.type = Marker::CUBE;
      m.action = Marker::ADD;
      m.pose.position.x = static_cast<float64_t>(box.centroid.x);
      m.pose.position.y = static_cast<float64_t>(box.centroid.y);
      m.pose.position.z = static_cast<float64_t>(box.centroid.z);
      m.pose.orientation.x = static_cast<float64_t>(box.orientation.x);
      m.pose.orientation.y = static_cast<float64_t>(box.orientation.y);
      m.pose.orientation.z = static_cast<float64_t>(box.orientation.z);
      m.pose.orientation.w = static_cast<float64_t>(box.orientation.w);
      // X and Y scale are swapped between these two message types
      m.scale.x = static_cast<float64_t>(box.size.y);
      m.scale.y = static_cast<float64_t>(box.size.x);
      m.scale.z = static_cast<float64_t>(box.size.z);
      m.color.r = 1.0;
      m.color.g = 0.5;
      m.color.b = 0.0;
      m.color.a = 0.75;
      m.lifetime.sec = 0;
      m.lifetime.nanosec = 500000000;
      marker_array.markers.push_back(m);
      id_counter++;
    }
    m_marker_pub_ptr->publish(marker_array);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle(const PointCloud2::SharedPtr msg_ptr)
{
  has_received_point_cloud = true;
  last_point_cloud = msg_ptr;
  if(!__use_timer) {
    handle_timer_callback();
  }
  return;
}

void EuclideanClusterNode::handle_timer_callback()
{
  // if(__use_timer) {
  //   RCLCPP_WARN(get_logger(), "EuclideanClusterNode::timer callback called!");
  // } else {
  //   RCLCPP_WARN(get_logger(), "EuclideanClusterNode::data callback called!");
  // }
  if(!has_received_point_cloud) {
    // RCLCPP_WARN(get_logger(), "EuclideanClusterNode::handle_timer_callback: did not receive point_cloud yet.");
    return;
  }
  handle_periodic(last_point_cloud);
  return;
}

void EuclideanClusterNode::handle_periodic(const PointCloud2::SharedPtr msg_ptr)
{
  auto start_time = omp_get_wtime();
  try {
    try {
      insert(*msg_ptr);
    } catch (const std::length_error & e) {
      // Hit limits of inserting, can still cluster, but in bad state
      RCLCPP_WARN(get_logger(), e.what());
    }
    std::cerr << "fuck that shit" << std::endl;
    // m_cluster_alg.cluster(m_clusters);
    m_cluster_alg.cluster_parallel(m_clusters);
    std::cerr << "Jonna ssibal gaessibal" << std::endl;
    //lint -e{523} NOLINT empty functions to make this modular
    // handle_clusters(m_clusters, msg_ptr->header);
    handle_clusters(m_clusters, msg_ptr->header);
    m_cluster_alg.cleanup(m_clusters);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "EuclideanClusterNode: Unexpected error occurred!");
    throw;
  }
  // rubis t1
    auto end_time = omp_get_wtime();
    auto response_time = (end_time - start_time) * 1e3;
    std::cerr << "euc: " << response_time << std::endl;

}
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode)
