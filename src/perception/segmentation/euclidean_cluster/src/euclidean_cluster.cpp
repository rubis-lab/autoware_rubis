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
#include <lidar_utils/point_cloud_utils.hpp>
#include <cstring>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <algorithm>
#include <string>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <utility>
#include "euclidean_cluster/euclidean_cluster.hpp"
#include "geometry/bounding_box_2d.hpp"

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{

void EuclideanCluster::init_rubis(sched_info _si)
{
  __si = _si;
  __slog = SchedLog(__si);
  __iter = 0;
  for(int i = 0; i < __si.max_option; i++) {
    __rt_configured.push_back(false);
  }
  __iter = 0;
  return;
}  
////////////////////////////////////////////////////////////////////////////////
PointXYZII::PointXYZII(const PointXYZI & pt, const uint32_t id)
: m_point{pt},
  m_id{id}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZII::PointXYZII(
  const float32_t x,
  const float32_t y,
  const float32_t z,
  const float32_t intensity,
  const uint32_t id)
: m_point{x, y, z, intensity},
  m_id{id}
{
}
////////////////////////////////////////////////////////////////////////////////
uint32_t PointXYZII::get_id() const
{
  return m_id;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZI & PointXYZII::get_point() const
{
  return m_point;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const std::string & frame_id,
  const std::size_t min_cluster_size,
  const std::size_t max_num_clusters)
: m_frame_id(frame_id),
  m_min_cluster_size(min_cluster_size),
  m_max_num_clusters(max_num_clusters)
{
  // TODO(c.ho) sanity checking
}

////////////////////////////////////////////////////////////////////////////////
std::size_t Config::min_cluster_size() const
{
  return m_min_cluster_size;
}
////////////////////////////////////////////////////////////////////////////////
std::size_t Config::max_num_clusters() const
{
  return m_max_num_clusters;
}
////////////////////////////////////////////////////////////////////////////////
const std::string & Config::frame_id() const
{
  return m_frame_id;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::EuclideanCluster(const Config & cfg, const HashConfig & hash_cfg)
: m_config(cfg),
  m_hash(hash_cfg),
  m_clusters(),
  m_cluster_pool(),
  m_last_error(Error::NONE),
  m_seen{}
{
  // Reservation
  m_clusters.clusters.reserve(m_config.max_num_clusters());
  m_cluster_pool.resize(m_config.max_num_clusters());
  m_seen.reserve(hash_cfg.get_capacity());
  // initialize clusters
  for (auto & cls : m_cluster_pool) {
    common::lidar_utils::init_pcl_msg(cls, cfg.frame_id(), hash_cfg.get_capacity());
    cls.width = 0U;
    // check pointstep vs sizeof(PointXY) and sizeof(PointXYZIF)
    if (cls.point_step < sizeof(PointXY)) {
      throw std::domain_error{"Cluster initialized with point size smaller than PointXY"};
    }
    if (cls.point_step != sizeof(PointXYZI)) {
      throw std::domain_error{"Cluster initialized with point size != PointXYZI"};
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters)
{
  if (clusters.clusters.capacity() < m_config.max_num_clusters()) {
    throw std::domain_error{"EuclideanCluster: Provided clusters must have sufficient capacity"};
  }
  cluster_impl(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::return_clusters(Clusters & clusters)
{
  for (std::size_t idx = 0U; idx < clusters.clusters.size(); ++idx) {
    m_cluster_pool[idx] = std::move(clusters.clusters[idx]);
    m_cluster_pool[idx].width = 0U;
  }
  clusters.clusters.resize(0U);
  m_seen.clear();
}
////////////////////////////////////////////////////////////////////////////////
const Clusters & EuclideanCluster::cluster(const builtin_interfaces::msg::Time stamp)
{
  // Reset clusters to pool
  return_clusters(m_clusters);
  // Actual clustering process
  cluster_impl(m_clusters);
  // Assign time stamp
  for (auto & cls : m_clusters.clusters) {
    cls.header.stamp = stamp;
  }
  return m_clusters;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cleanup(Clusters & clusters)
{
  // Return data to algorithm
  return_clusters(clusters);
  // Error handling after publishing
  switch (get_error()) {
    case Error::TOO_MANY_CLUSTERS:
      throw std::runtime_error{"EuclideanCluster: Too many clusters"};
    case Error::NONE:
    default:
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::Error EuclideanCluster::get_error() const
{
  return m_last_error;
}
////////////////////////////////////////////////////////////////////////////////
const Config & EuclideanCluster::get_config() const
{
  return m_config;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster_impl(Clusters & clusters)
{
  m_last_error = Error::NONE;
  // rubis: parallelize here
  for (const auto & kv : m_hash) {
    const auto & pt = kv.second;
    if (!m_seen[pt.get_id()]) {
      cluster(clusters, pt);
    }
  }
  m_hash.clear();
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters, const PointXYZII & pt)
{
  // init new cluster
  const auto num_clusters = clusters.clusters.size();
  if (num_clusters >= m_config.max_num_clusters()) {
    m_last_error = Error::TOO_MANY_CLUSTERS;
  } else {
    //rubis critical
    clusters.clusters.emplace_back(std::move(m_cluster_pool[num_clusters]));
    // Seed cluster with new point
    auto & cluster = clusters.clusters.back();
    add_point(cluster, pt);
    // rubis critical
    m_seen[pt.get_id()] = true;
    // Start clustering process
    std::size_t last_seed_idx = 0U;
    while (last_seed_idx < cluster.width) {
      const auto pt = get_point(cluster, last_seed_idx);
      add_neighbors(cluster, pt);
      // Increment seed point
      ++last_seed_idx;
    }
    // check if cluster is large enough: roll back pointer if so
    if (last_seed_idx < m_config.min_cluster_size()) {
      // return cluster to pool
      m_cluster_pool[num_clusters] = std::move(clusters.clusters[num_clusters]);
      m_cluster_pool[num_clusters].width = 0U;
      clusters.clusters.resize(num_clusters);
    } else {
      // finalize cluster
      cluster.row_step = cluster.point_step * cluster.width;
    }
  }
}
void EuclideanCluster::cluster_parallel(Clusters & clusters)
{
  if (clusters.clusters.capacity() < m_config.max_num_clusters()) {
    throw std::domain_error{"EuclideanCluster: Provided clusters must have sufficient capacity"};
  }
  m_last_error = Error::NONE;
  // rubis: parallelize here
  omp_set_dynamic(0);
  #pragma omp parallel shared(clusters, m_hash, m_seen) num_threads(__si.max_option)
  {
    auto thr_id = omp_get_thread_num();
    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[EuclideanCluster] (" << tid << "): __rt_configured (" << __si.exec_time << ", " << __si.deadline << ", " << __si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();

    // #pragma omp for schedule(dynamic) nowait
    for (const auto & kv : m_hash) {
      const auto & pt = kv.second;
      if (!m_seen[pt.get_id()]) {
        cluster(clusters, pt);
      }
      // const auto & pt = kv.second;
      
      // // #pragma omp critical(temp)
      // if (!m_seen[pt.get_id()]) {
      //   // init new cluster
      //   const auto num_clusters = clusters.clusters.size();
      //   if (num_clusters >= m_config.max_num_clusters()) {
      //     m_last_error = Error::TOO_MANY_CLUSTERS;
      //   } else {
      //     //rubis critical
      //     // #pragma omp critical(initialize)
      //     clusters.clusters.emplace_back(std::move(m_cluster_pool[num_clusters]));
      //     // Seed cluster with new point
      //     auto & cluster = clusters.clusters.back();
          
      //     // #pragma omp critical(add)
      //     add_point(cluster, pt);
      //     // rubis critical
      //     m_seen[pt.get_id()] = true;
      //     // Start clustering process
          
      //     // while (last_seed_idx < cluster.width) {
      //     //   const auto pt = get_point(cluster, last_seed_idx);
      //     //   add_neighbors(cluster, pt);
      //     //   // Increment seed point
      //     //   ++last_seed_idx;
      //     // }


      //     std::size_t last_seed_idx = 0U;
      //     while( last_seed_idx < cluster.width) {
      //       const auto pt = get_point(cluster, last_seed_idx);
      //       // add_neighbors(cluster, pt);
      //       // std::vector<Output> & nbrs;
      //       // nbrs.clear();

      //       const auto & nbrs = m_hash.near(pt.x, pt.y);
      //       // For each point within a fixed radius, check if already seen
      //       for (const auto itd : nbrs) {
      //         const auto & qt = itd.get_point();
      //         const auto id = qt.get_id();
      //         // rubis critical
      //         if (!m_seen[id]) {
      //           // Add to cluster
      //           // #pragma omp critical(add)
      //           add_point(cluster, qt);
      //           // Mark point as seen
      //           m_seen[id] = true;
      //         }
      //       }
      //       last_seed_idx += 1;
      //     }
      //     // check if cluster is large enough: roll back pointer if so
      //     if (last_seed_idx < m_config.min_cluster_size()) {
      //       // return cluster to pool
      //       m_cluster_pool[num_clusters] = std::move(clusters.clusters[num_clusters]);
      //       m_cluster_pool[num_clusters].width = 0U;
      //       clusters.clusters.resize(num_clusters);
      //     } else {
      //       // finalize cluster
      //       cluster.row_step = cluster.point_step * cluster.width;
      //     }
      //   }
      // // cluster(clusters, pt);
      // iteration++;
      // std::cerr << "iteration: " << iteration << std::endl;
      // }
    }
    //workload end

    auto end_time = omp_get_wtime();
    auto response_time = (end_time - start_time) * 1e3;

    sched_data sd {
      thr_id, // thr_id
      __iter,  // iter
      start_time,  // start_time
      end_time,  // end_time
      response_time  // response_time
    };

    #pragma omp critical (add_entry)
    {
      __slog.add_entry(sd);
    }
    sched_yield();
  }
  ++__iter;
  
  m_hash.clear();
  return;
}

////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_neighbors(Cluster & cls, const EuclideanCluster::PointXY pt)
{
  // z is not needed since it's a 2d hash
  const auto & nbrs = m_hash.near(pt.x, pt.y);
  // For each point within a fixed radius, check if already seen
  for (const auto itd : nbrs) {
    const auto & qt = itd.get_point();
    const auto id = qt.get_id();
    // rubis critical
    if (!m_seen[id]) {
      // Add to cluster
      add_point(cls, qt);
      // Mark point as seen
      m_seen[id] = true;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_point(Cluster & cls, const PointXYZII & pt)
{
  // Clustering cannot overrun cluster capacity since each cluster is preallocated with the
  // max capacity of the hash, so the data structure would throw before you overrun the cluster
  using Size = decltype(Cluster::data)::size_type;
  const auto idx = static_cast<Size>(cls.width) * static_cast<Size>(cls.point_step);
  cls.data.resize(idx + static_cast<Size>(cls.point_step));
  // Placement new to ensure dynamic type is accurate (allowing for reinterpret_cast to not be UB)
  (void)new(&cls.data[idx]) PointXYZI(pt.get_point());
  ++cls.width;
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::PointXY EuclideanCluster::get_point(const Cluster & cls, const std::size_t idx)
{
  PointXY ret{};
  //lint -e{586, 925} NOLINT guaranteed not to have overlap, so it's fine; no other way to do
  (void)memcpy(
    static_cast<void *>(&ret),
    static_cast<const void *>(&cls.data[idx * cls.point_step]),
    sizeof(ret));
  return ret;
}

namespace details
{

// namespace
// {


std::pair<const PointXYZI *, const PointXYZI *> point_struct_iterators(
  const euclidean_cluster::Cluster & cls)
{
  using euclidean_cluster::PointXYZI;
  if (cls.data.empty()) {
    throw std::runtime_error("PointCloud2 data is empty");
  }
  if (cls.data.size() != cls.row_step * cls.height) {
    throw std::runtime_error("PointCloud2 data has invalid size");
  }
  if (cls.point_step != sizeof(PointXYZI)) {
    throw std::runtime_error("PointCloud2 data has unexpected point_step");
  }
  if (reinterpret_cast<std::uintptr_t>(cls.data.data()) % alignof(PointXYZI) != 0) {
    throw std::runtime_error("PointCloud2 data is not aligned like required by PointXYZI");
  }
  if ((__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__) != cls.is_bigendian) {
    throw std::runtime_error("PointCloud2 does not have native endianness");
  }
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto begin = reinterpret_cast<const PointXYZI *>(&cls.data[0]);
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto end = reinterpret_cast<const PointXYZI *>(&cls.data[0] + cls.data.size());
  return std::make_pair(begin, end);
}

std::pair<PointXYZI *, PointXYZI *> point_struct_iterators(euclidean_cluster::Cluster & cls)
{
  using euclidean_cluster::PointXYZI;
  auto iterators = point_struct_iterators(const_cast<const euclidean_cluster::Cluster &>(cls));
  return std::make_pair(
    const_cast<PointXYZI *>(iterators.first),
    const_cast<PointXYZI *>(iterators.second));
}

// }  // namespace

////////////////////////////////////////////////////////////////////////////////
BoundingBox compute_eigenbox(const euclidean_cluster::Cluster & cls)
{
  const auto iterators = point_struct_iterators(cls);
  return common::geometry::bounding_box::eigenbox_2d(iterators.first, iterators.second);
}
////////////////////////////////////////////////////////////////////////////////
BoundingBox compute_lfit_bounding_box(Cluster & cls)
{
  const auto iterators = point_struct_iterators(cls);
  return common::geometry::bounding_box::lfit_bounding_box_2d(iterators.first, iterators.second);
}
////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      boxes.boxes.push_back(compute_eigenbox(cls));
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      const auto iterators = point_struct_iterators(cls);
      auto box = common::geometry::bounding_box::eigenbox_2d(iterators.first, iterators.second);
      common::geometry::bounding_box::compute_height(iterators.first, iterators.second, box);
      boxes.boxes.push_back(box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      boxes.boxes.push_back(compute_lfit_bounding_box(cls));
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.boxes.clear();
  for (auto & cls : clusters.clusters) {
    try {
      boxes.boxes.push_back(autoware_auto_msgs::msg::BoundingBox{});
      auto & box = boxes.boxes[boxes.boxes.size()];
      const auto iterators = point_struct_iterators(cls);
      box = common::geometry::bounding_box::lfit_bounding_box_2d(iterators.first, iterators.second);
      common::geometry::bounding_box::compute_height(iterators.first, iterators.second, box);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
}  // namespace details
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
