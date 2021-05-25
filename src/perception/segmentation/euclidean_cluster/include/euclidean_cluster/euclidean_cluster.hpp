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
/// \brief This file defines the euclidean cluster algorithm for object detection

#ifndef EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_
#define EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_

#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/point_clusters.hpp>
#include <geometry/spatial_hash.hpp>
#include <euclidean_cluster/visibility_control.hpp>
#include <common/types.hpp>
#include <string>
#include <vector>
#include <utility>

#include <ctime>
#include <omp.h>
#include <chrono>
#include "rubis_rt/sched.hpp"
#include "rubis_rt/sched_log.hpp"

namespace autoware
{
namespace perception
{
namespace segmentation
{
/// \brief Supporting classes for euclidean clustering, an object detection algorithm
namespace euclidean_cluster
{
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;
using rubis::sched_log::SchedLog;
using rubis::sched_log::sched_info;
using rubis::sched_log::sched_data;
// using autoware::common::geometry::spatial_hash::Output;

/// \brief Simple point struct for memory mapping to and from PointCloud2 type
struct PointXYZI
{
  float32_t x = 0.0f;
  float32_t y = 0.0f;
  float32_t z = 0.0f;
  float32_t intensity = 0.0f;
};  // struct PointXYZI

/// \brief Helper point for which euclidean distance is computed only once
class EUCLIDEAN_CLUSTER_PUBLIC PointXYZII
{
public:
  PointXYZII() = default;
  /// \brief Conversion constructor
  /// \param[in] pt The point to convert
  /// \param[in] id The unique identifier for this point within a frame
  PointXYZII(const PointXYZI & pt, const uint32_t id);
  /// \brief Constructor
  /// \param[in] x The x position of the point
  /// \param[in] y The y position of the point
  /// \param[in] z The z position of the point
  /// \param[in] intensity The intensity value of the point
  /// \param[in] id The unique identifier for this point within a frame
  PointXYZII(
    const float32_t x,
    const float32_t y,
    const float32_t z,
    const float32_t intensity,
    const uint32_t id);
  /// \brief Getter for id, for "seen" bookkeeping
  /// \return Unique integer id
  uint32_t get_id() const;
  /// \brief Get core point
  /// \return Reference to internally stored point
  const PointXYZI & get_point() const;

private:
  // This could instead be a pointer; I'm pretty sure ownership would work out, but I'm
  // uncomfortable doing it that way (12 vs 20 bytes)
  PointXYZI m_point;
  uint32_t m_id = 0;
};  // class PointXYZII

using HashConfig = autoware::common::geometry::spatial_hash::Config2d;
using Hash = autoware::common::geometry::spatial_hash::SpatialHash2d<PointXYZII>;
using Clusters = autoware_auto_msgs::msg::PointClusters;
using Cluster = decltype(Clusters::clusters)::value_type;

/// \brief Configuration class for euclidean cluster
/// In the future this can become a base class with subclasses defining different
/// threshold functions. This configuration's threshold function currently assumes isotropy, and
/// minor details in the clustering implementation also assume this property.
class EUCLIDEAN_CLUSTER_PUBLIC Config
{
public:
  /// \brief Constructor
  /// \param[in] frame_id The frame id for which all clusters are initialized with
  /// \param[in] min_cluster_size The number of points that must be in a cluster before it is not
  ///                             considered noise
  /// \param[in] max_num_clusters The maximum preallocated number of clusters in a scene
  Config(
    const std::string & frame_id,
    const std::size_t min_cluster_size,
    const std::size_t max_num_clusters);
  /// \brief Gets minimum number of points needed for a cluster to not be considered noise
  /// \return Minimum cluster size
  std::size_t min_cluster_size() const;
  /// \brief Gets maximum preallocated number of clusters
  /// \return Maximum number of clusters
  std::size_t max_num_clusters() const;
  /// \brief Get frame id
  /// \return The frame id
  const std::string & frame_id() const;

private:
  const std::string m_frame_id;
  const std::size_t m_min_cluster_size;
  const std::size_t m_max_num_clusters;
};  // class Config

/// \brief implementation of euclidean clustering for point cloud segmentation
/// This clas implicitly projects points onto a 2D (x-y) plane, and segments
/// according to euclidean distance. This can be thought of as a graph-based
/// approach where points are vertices and edges are defined by euclidean distance
/// The input to this should be nonground points pased through a voxel grid.
class EUCLIDEAN_CLUSTER_PUBLIC EuclideanCluster
{
public:
  enum class Error : uint8_t
  {
    NONE = 0U,
    TOO_MANY_CLUSTERS
  };  // enum class Error
  /// \brief Constructor
  /// \param[in] cfg The configuration of the clustering algorithm, contains threshold function
  /// \param[in] hash_cfg The configuration of the underlying spatial hash, controls the maximum
  ///                     number of points in a scene
  EuclideanCluster(const Config & cfg, const HashConfig & hash_cfg);
  /// \brief Insert an individual point
  /// \param[in] args Parameters forwarded to PointXYZII constructor (except for ID)
  /// \throw std::length_error If the underlying spatial hash is full
  template<typename ... Args>
  void insert(Args && ... args)
  {
    // can't do anything with return values
    (void)m_hash.insert(
      PointXYZII{std::forward<Args>(args)..., static_cast<uint32_t>(m_seen.size())});
    m_seen.push_back(false);
  }

  void init_rubis(sched_info _si);

  /// \brief Multi-insert
  /// \param[in] begin Iterator pointing to to the first point to insert
  /// \param[in] end Iterator pointing to one past the last point to insert
  /// \throw std::length_error If the underlying spatial hash is full
  /// \tparam IT The type of the iterator
  template<typename IT>
  void insert(const IT begin, const IT end)
  {
    if ((static_cast<std::size_t>(std::distance(begin, end)) + m_hash.size()) > m_hash.capacity()) {
      throw std::length_error{"EuclideanCluster: Multi insert would overrun capacity"};
    }
    for (auto it = begin; it != end; ++it) {
      insert(*it);
    }
  }

  /// \brief Compute the clusters from the inserted points
  /// It should in theory be ok to reinterpret_cast the points into a PointXYZI. Internally, they
  /// were constructed in place using placement new, so the dynamic type should be correct.
  /// \return A reference to the resulting clusters
  const Clusters & cluster(const builtin_interfaces::msg::Time stamp);

  /// \brief Compute the clusters from the inserted points, where the final clusters object lives in
  ///        another scope. The final clusters object should return_clusters after being used
  /// It should in theory be ok to reinterpret_cast the points into a PointXYZI. Internally, they
  /// were constructed in place using placement new, so the dynamic type should be correct.
  /// \param[inout] clusters The clusters object
  void cluster(Clusters & clusters);
  void cluster_parallel(Clusters & clusters);

  /// \brief Gets last error, intended to be used with clustering with internal cluster result
  /// This is a separate function rather than using an exception because the main error mode is
  /// exceeding preallocated cluster capacity. However, throwing an exception would throw away
  /// perfectly valid information that is still usable in an error state.
  Error get_error() const;

  /// \brief Returns the preallocated clusters to the internal pool so the cluster object can safely
  ///        be resized without memory allocation due to default/copy construction. Additionally
  ///        throws an error based on the result of get_error. Intended to be used with a cluster
  ///        result that lives in an external scope
  /// \param[inout] clusters The vector of clusters for which all clusters will be moved away
  /// \throw std::runtime_error If the maximum number of clusters may have been exceeded
  void cleanup(Clusters & clusters);

  /// \brief Gets the internal configuration class, for use when it was inline generated
  /// \return Internal configuration class
  const Config & get_config() const;

private:
  SchedLog __slog;
  sched_info __si;
  std::vector<bool8_t> __rt_configured;
  int32_t __iter;
  bool8_t __use_timer = false;
  /// \brief Internal struct instead of pair since I can guarantee some memory stuff
  struct PointXY
  {
    float32_t x = 0.0f;
    float32_t y = 0.0f;
  };  // struct PointXYZ
  /// \brief Do the clustering process, with no error checking
  EUCLIDEAN_CLUSTER_LOCAL void cluster_impl(Clusters & clusters);
  /// \brief Compute the next cluster, seeded by the given point, and grown using the remaining
  ///         unseen points
  EUCLIDEAN_CLUSTER_LOCAL void cluster(Clusters & clusters, const PointXYZII & pt);
  /// \brief Add all near neighbors of a point to a given cluster
  EUCLIDEAN_CLUSTER_LOCAL void add_neighbors(Cluster & cls, const PointXY pt);
  /// \brief Adds a point to the cluster, internal version since no error checking is needed
  EUCLIDEAN_CLUSTER_LOCAL static void add_point(Cluster & cls, const PointXYZII & pt);

  // EUCLIDEAN_CLUSTER_LOCAL void add_neighbors_parallel(Cluster & cls, const PointXY pt);
  // EUCLIDEAN_CLUSTER_LOCAL static void add_point_parallel(Cluster & cls, const PointXYZII & pt);

  /// \brief Get a specified point from the cluster
  EUCLIDEAN_CLUSTER_LOCAL static PointXY get_point(const Cluster & cls, const std::size_t idx);
  /// \brief Returns the preallocated clusters to the internal pool so the cluster object can safely
  ///        be resized without memory allocation due to default/copy construction
  /// \param[inout] clusters The vector of clusters for which all clusters will be moved away
  EUCLIDEAN_CLUSTER_LOCAL void return_clusters(Clusters & clusters);

  const Config m_config;
  Hash m_hash;
  Clusters m_clusters;
  decltype(Clusters::clusters) m_cluster_pool;
  Error m_last_error;
  std::vector<bool8_t> m_seen;
};  // class EuclideanCluster

/// \brief Common euclidean cluster functions not intended for external use
namespace details
{
EUCLIDEAN_CLUSTER_PUBLIC std::pair<const PointXYZI *, const PointXYZI *> point_struct_iterators(const euclidean_cluster::Cluster & cls);

EUCLIDEAN_CLUSTER_PUBLIC  std::pair<PointXYZI *, PointXYZI *> point_struct_iterators(euclidean_cluster::Cluster & cls);

using BoundingBox = autoware_auto_msgs::msg::BoundingBox;
using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
/// \brief Compute lfit bounding box from individual cluster
/// \param[inout] cls The cluster for which to compute the bounding box, gets shuffled
/// \return Lfit bounding box
EUCLIDEAN_CLUSTER_PUBLIC BoundingBox compute_lfit_bounding_box(Cluster & cls);
/// \brief Compute eigenbox from individual cluster
/// \param[in] cls The cluster for which to compute the bounding box
/// \return Best fit eigenbox
EUCLIDEAN_CLUSTER_PUBLIC BoundingBox compute_eigenbox(const Cluster & cls);
/// \brief Compute lfit bounding boxes from clusters
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
///                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_PUBLIC
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute lfit bounding boxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
///                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_PUBLIC
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_PUBLIC
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_PUBLIC
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes);
}  // namespace details
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
namespace common
{
namespace geometry
{
namespace point_adapter
{
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto x_(
  const perception::segmentation::euclidean_cluster::PointXYZII & pt)
{
  return pt.get_point().x;
}
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto y_(
  const perception::segmentation::euclidean_cluster::PointXYZII & pt)
{
  return pt.get_point().y;
}
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto z_(
  const perception::segmentation::euclidean_cluster::PointXYZII & pt)
{
  return pt.get_point().z;
}
}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_
