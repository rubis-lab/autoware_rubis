// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
/// \brief This class defines common functions and classes to work with pointclouds

#ifndef LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
#define LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_

#include <lidar_utils/visibility_control.hpp>

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry/common_3d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace autoware
{
namespace common
{
namespace comp = helper_functions::comparisons;
namespace lidar_utils
{
using sensor_msgs::msg::PointCloud2;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

/// max number of points in a scan for VLP16s, assuming 300 rpm = 5hz: 57870.3703 points per full
/// rotation
static const uint32_t MAX_SCAN_POINTS = 57872U;

/// Point cloud iterator wrapper, that allows to reuse iterators. Reinitializing
/// of iterators is quite costly due to the implementation of the
/// PointCloud2IteratorBase<...>::set_field method.
class LIDAR_UTILS_PUBLIC PointCloudIts
{
public:
  /// \brief Creates new iterator wrapper with space reserved for 4 iterators,
  /// namely x, y, z and intensity
  PointCloudIts();

  /// \brief Resets the iterators for the given cloud to the given index
  /// \param[in] cloud the point cloud to reset the iterators to
  /// \param[in] idx the index/offset to advance the iterators to
  void reset(sensor_msgs::msg::PointCloud2 & cloud, uint32_t idx = 0);

  /// \brief Returns iterator for the "x" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & x_it()
  {
    return m_its[0];
  }

  /// \brief Returns iterator for the "y" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & y_it()
  {
    return m_its[1];
  }

  /// \brief Returns iterator for the "z" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & z_it()
  {
    return m_its[2];
  }

  /// \brief Returns iterator for the "intensity" field
  sensor_msgs::PointCloud2Iterator<float32_t> & intensity_it()
  {
    return m_its[3];
  }

private:
  /// Internal storage of the iterators
  ::std::vector<sensor_msgs::PointCloud2Iterator<float32_t>> m_its;
};

/// \brief Compute minimum safe length of point cloud access
/// \param[in] msg The point cloud message to validate
/// \return Byte index of one past the end of the last point ok to access
LIDAR_UTILS_PUBLIC
std::size_t index_after_last_safe_byte_index(const sensor_msgs::msg::PointCloud2 & msg) noexcept;

struct SafeCloudIndices
{
  std::size_t point_step;
  std::size_t data_length;
};

/// Compute the safe stride and max length for given point cloud
/// \return A pair size of data ok to read per point, and last index ok to read, for use with the
/// form `for (auto idx = 0U; idx < ret.data_length; idx += msg.point_step)
/// {memcpy(&pt, msg.data[idx], ret.point_step);}`
LIDAR_UTILS_PUBLIC SafeCloudIndices sanitize_point_cloud(const sensor_msgs::msg::PointCloud2 & msg);

/// \brief initializes header information for point cloud for x, y, z and intensity
/// \param[out] msg a point cloud message to initialize
/// \param[in] frame_id the name of the frame for the point cloud (assumed fixed)
/// \param[in] size number of points to preallocate for underyling data array
LIDAR_UTILS_PUBLIC void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size = static_cast<std::size_t>(MAX_SCAN_POINTS));

/// initializes header information for point cloud given frame id, size, number of frames and
///  a parameter pack of fields.
/// \tparam Fields Template paramater pack containing field types.
/// \param msg Point cloud message.
/// \param frame_id Frame ID of the point cloud.
/// \param size Size of the initialized point cloud.
/// \param num_fields Number of fields.
/// \param fields Set of parameters defining the fields. Each field must contain the following
/// parameters in strict order: `field_name, count, data_type`. These parameters should
/// be provided for each field
template<typename ... Fields>
LIDAR_UTILS_PUBLIC void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size,
  const uint32_t num_fields,
  Fields const & ... fields
)
{
  msg.height = 1U;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.header.frame_id = frame_id;
  // set the fields
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(num_fields, fields ...);
  // allocate memory so that iterators can be used
  modifier.resize(size);
}

/// \brief add a point in the cloud by memcpy instead of using iterators
/// This version prioritize speed and ease of parallelisation
/// it assumes : - PointXYZIF is a POD object equivalent to a point stored in the cloud,
///                which means same fields and same endianness.
///              - The caller is responsible for incrementing point_cloud_idx between two calls.
///                This differs from add_point_to_cloud who increment point_cloud_idx by itself

//rubis parallel
LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud_parallel(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t point_cloud_idx);

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud_raw(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t point_cloud_idx);

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud(
  PointCloudIts & cloud_its,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void reset_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t size,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void resize_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t new_size);

/// Check that the pointcloud msg has x, y, z of type float32_t as the first three fields,
/// otherwise throw an exception; check that the pointcloud msg has intensity field as
/// the fourth field, otherwise return false
LIDAR_UTILS_PUBLIC bool8_t
has_intensity_and_throw_if_no_xyz(const PointCloud2::SharedPtr & cloud);

/// Check that the pointcloud msg has x, y, z of type float32_t as the first three fields,
/// otherwise throw an exception; check that the pointcloud msg has intensity field as
/// the fourth field, otherwise return false
LIDAR_UTILS_PUBLIC bool8_t
has_intensity_and_throw_if_no_xyz(const PointCloud2 & cloud);

template<typename T>
struct _create_custom_pcl_datatype;

template<>
struct _create_custom_pcl_datatype<int8_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT8;
};

template<>
struct _create_custom_pcl_datatype<uint8_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT8;
};

template<>
struct _create_custom_pcl_datatype<int16_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT16;
};

template<>
struct _create_custom_pcl_datatype<uint16_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT16;
};

template<>
struct _create_custom_pcl_datatype<int32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT32;
};

template<>
struct _create_custom_pcl_datatype<uint32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT32;
};

template<>
struct _create_custom_pcl_datatype<float32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::FLOAT32;
};

template<>
struct _create_custom_pcl_datatype<float64_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::FLOAT64;
};

template<typename T>
LIDAR_UTILS_PUBLIC
sensor_msgs::msg::PointCloud2::SharedPtr create_custom_pcl(
  const std::vector<std::string> & field_names,
  const uint32_t cloud_size)
{
  using sensor_msgs::msg::PointCloud2;
  PointCloud2::SharedPtr msg = std::make_shared<PointCloud2>();
  const auto field_size = field_names.size();
  msg->height = 1U;
  msg->width = cloud_size;
  msg->fields.resize(field_size);
  for (uint32_t i = 0U; i < field_size; i++) {
    msg->fields[i].name = field_names[i];
  }
  msg->point_step = 0U;
  for (uint32_t idx = 0U; idx < field_size; ++idx) {
    msg->fields[idx].offset = static_cast<uint32_t>(idx * sizeof(T));
    msg->fields[idx].datatype = _create_custom_pcl_datatype<T>::DATATYPE;
    msg->fields[idx].count = 1U;
    msg->point_step += static_cast<uint32_t>(sizeof(T));
  }
  const std::size_t capacity = msg->point_step * cloud_size;
  msg->data.clear();
  msg->data.reserve(capacity);
  for (std::size_t i = 0; i < capacity; ++i) {
    msg->data.emplace_back(0U);  // initialize all values equal to 0
  }
  msg->row_step = msg->point_step * msg->width;
  msg->is_bigendian = false;
  msg->is_dense = false;
  msg->header.frame_id = "base_link";
  return msg;
}

/// \brief Filter class to check if a point lies within a range defined by a min and max radius.
class LIDAR_UTILS_PUBLIC DistanceFilter
{
public:
  /// \brief Cosntructor
  /// \param min_radius The radius the point's radius should be greater than
  /// \param max_radius The radius the point's radius should be lesser than
  DistanceFilter(float32_t min_radius, float32_t max_radius);
  static constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon();

  /// \brief Check if the point is within the allowed range of the filter. Check is done in
  /// square form to avoid `sqrt`
  /// \tparam T Point type
  /// \param pt Point to be filtered
  /// \return True if point is within the filter's range.
  template<typename T>
  bool8_t operator()(const T & pt) const
  {
    using common::geometry::dot_3d;
    auto pt_radius = dot_3d(pt, pt);
    return comp::abs_gte(pt_radius, m_min_r2, FEPS) &&
           comp::abs_lte(pt_radius, m_max_r2, FEPS);
  }

private:
  float32_t m_min_r2;
  float32_t m_max_r2;
};

/// \brief Transform to apply a constant transform to given points.
class LIDAR_UTILS_PUBLIC StaticTransformer
{
public:
  /// \brief Constructor. Pre-computes the rotation and translation matrices from the transform
  /// msg.
  /// \param tf Transform msg to be applied to points.
  explicit StaticTransformer(const geometry_msgs::msg::Transform & tf);

  /// \brief Apply the transform to a point that has the proper point adapters defined.
  /// \tparam T Point type
  /// \param ref Input point
  /// \param out Reference to output point
  template<typename T>
  void transform(const T & ref, T & out) const //NOLINT (false positive: this is not std::transform)
  {
    using common::geometry::point_adapter::x_;
    using common::geometry::point_adapter::y_;
    using common::geometry::point_adapter::z_;
    using common::geometry::point_adapter::xr_;
    using common::geometry::point_adapter::yr_;
    using common::geometry::point_adapter::zr_;
    Eigen::Matrix<float32_t, 3U, 1U> ref_mat({x_(ref), y_(ref), z_(ref)});
    Eigen::Vector3f out_mat = m_tf * ref_mat;
    xr_(out) = out_mat(0U, 0U);
    yr_(out) = out_mat(1U, 0U);
    zr_(out) = out_mat(2U, 0U);
  }

private:
  Eigen::Transform<float32_t, 3U, Eigen::Affine, Eigen::ColMajor> m_tf;
};

/// \brief Filter class to check if a point's azimuth lies within a range defined by a start and
/// end angles. The range is defined from start to the end angles in counter-clock-wise direction.
class LIDAR_UTILS_PUBLIC AngleFilter
{
public:
  /// \brief Constructor
  /// \param start_angle Minimum angle in radians
  /// \param end_angle Maximum angle in radians
  AngleFilter(float32_t start_angle, float32_t end_angle);

  using VectorT = autoware::common::types::PointXYZIF;
  static constexpr float32_t PI = 3.14159265359F;
  static constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon() * 1e2F;

  /// \brief Check if a point's azimuth lies in the range [start, end] in
  /// counter-clock-wise-direction. The point is treated as a 2D vector whose projection on the
  /// range normal is compared to a threshold.
  /// \tparam T Point type
  /// \param pt point to check
  /// \return return true if the point is contained within the range.
  template<typename T>
  bool8_t operator()(const T & pt) const
  {
    using common::geometry::dot_2d;
    bool8_t ret = false;

    // Squared magnitude of the vector
    const auto pt_len2 = dot_2d(pt, pt);
    const auto proj_on_normal = dot_2d(pt, m_range_normal);
    const auto proj_on_normal2 = proj_on_normal * proj_on_normal;
    const auto is_proj_negative = (proj_on_normal + FEPS) < 0.0F;

    // Since the input vector's projection is scaled by the length of itself, the
    // threshold is also scaled by the length of the input vector to make the comparison possible.

    // To avoid computing the length using sqrt, the expressions are kept in square form, hence
    // the following sign checks are made to ensure the correctness of the comparisons in
    // squared form.
    if ((!m_threshold_negative) && (!is_proj_negative)) {
      ret = (proj_on_normal2) >= (pt_len2 * (m_threshold2 - FEPS));
    } else if (m_threshold_negative && (!is_proj_negative)) {
      ret = true;
    } else if ((!m_threshold_negative) && is_proj_negative) {
      ret = false;
    } else {
      ret = (proj_on_normal2) <= (pt_len2 * (m_threshold2 + FEPS));
    }
    return ret;
  }

private:
  VectorT m_range_normal;
  bool8_t m_threshold_negative;
  float32_t m_threshold2;
};

class LIDAR_UTILS_PUBLIC IntensityIteratorWrapper
{
private:
  sensor_msgs::PointCloud2ConstIterator<uint8_t> m_intensity_it_uint8;
  sensor_msgs::PointCloud2ConstIterator<float32_t> m_intensity_it_float32;
  decltype(sensor_msgs::msg::PointField::datatype) m_intensity_datatype;

public:
  explicit IntensityIteratorWrapper(const PointCloud2 & msg);

  void next();
  
  bool8_t eof();

  template<typename PointFieldValueT>
  void get_current_value(PointFieldValueT & point_field_value)
  {
    switch (m_intensity_datatype) {
      case sensor_msgs::msg::PointField::UINT8:
        point_field_value = *m_intensity_it_uint8;
        break;
      case sensor_msgs::msg::PointField::FLOAT32:
        point_field_value = *m_intensity_it_float32;
        break;
      default:
        throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
    }
  }

  template<typename PointFieldValueT>
  void get_nth_value(PointFieldValueT & point_field_value, uint32_t n)
  {
    switch (m_intensity_datatype) {
      case sensor_msgs::msg::PointField::UINT8:
        point_field_value = *(m_intensity_it_uint8 + n);
        break;
      case sensor_msgs::msg::PointField::FLOAT32:
        point_field_value = *(m_intensity_it_float32 + n);
        break;
      default:
        throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
    }
  }
};

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
