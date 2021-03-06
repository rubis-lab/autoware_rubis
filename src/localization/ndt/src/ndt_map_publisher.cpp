// Copyright 2020 the Autoware Foundation
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

#include <GeographicLib/Geocentric.hpp>
#include <ndt/ndt_map_publisher.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{

void read_from_yaml(
  const std::string & yaml_file_name,
  geodetic_pose_t * geo_pose)
{
  try {
    YAML::Node map_info = YAML::LoadFile(yaml_file_name);
    if (map_info["map_config"]) {
      if (map_info["map_config"]["latitude"] &&
        map_info["map_config"]["longitude"] &&
        map_info["map_config"]["elevation"])
      {
        geo_pose->latitude = map_info["map_config"]["latitude"].as<double>();
        geo_pose->longitude = map_info["map_config"]["longitude"].as<double>();
        geo_pose->elevation = map_info["map_config"]["elevation"].as<double>();
      } else {
        throw std::runtime_error("Yaml file: map origin not found\n");
      }
      if (map_info["map_config"]["roll"]) {
        geo_pose->roll = map_info["map_config"]["roll"].as<double>();
      }
      if (map_info["map_config"]["pitch"]) {
        geo_pose->pitch = map_info["map_config"]["pitch"].as<double>();
      }
      if (map_info["map_config"]["yaw"]) {
        geo_pose->yaw = map_info["map_config"]["yaw"].as<double>();
      }
    } else {
      throw std::runtime_error("Yaml file: map config not found\n");
    }
  } catch (const YAML::BadFile & ex) {
    throw std::runtime_error("Yaml file not found\n");
  } catch (const YAML::ParserException & ex) {
    throw std::runtime_error("Yaml syntax error\n");
  }
}

void read_from_pcd(const std::string & file_name, sensor_msgs::msg::PointCloud2 * msg)
{
  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(file_name, pcl_cloud) == -1) {  // load the file
    throw std::runtime_error(std::string("PCD file ") + file_name + " could not be loaded.");
  }
  if (pcl_cloud.data.size() == 0) {
    throw std::runtime_error("PCD cloud empty\n");
  }

  // Convert to sensor_msgs in order to check the available fields
  sensor_msgs::msg::PointCloud2 cloud;
  pcl_conversions::moveFromPCL(pcl_cloud, cloud);

  // Ensure that we have at least the x, y, z fields and check whether we have intensity
  const auto has_intensity = common::lidar_utils::has_intensity_and_throw_if_no_xyz(cloud);
  if (has_intensity && msg->fields.size() == 4U &&
    msg->fields[3U].datatype == sensor_msgs::msg::PointField::FLOAT32)
  {
    // Quick path: the data already has the desired format
    *msg = std::move(cloud);
    return;
  }

  // We don't have intensity of the correct format
  // Set up a new point cloud with the correct fields
  sensor_msgs::msg::PointCloud2 adjusted_cloud;
  const size_t num_points = cloud.data.size() / cloud.point_step;
  common::lidar_utils::init_pcl_msg(adjusted_cloud, msg->header.frame_id, num_points);

  // Copy x, y, z into it
  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t * src = cloud.data.data() + cloud.point_step * i;
    uint8_t * dest = adjusted_cloud.data.data() + adjusted_cloud.point_step * i;
    std::memcpy(dest, src, 3 * sizeof(float32_t));
    // If intensity exists, copy it into the new cloud, otherwise, set it to 0.0
    // This would be faster with separate loops, but this function isn't a hotspot
    float intensity = 0.0f;
    if (has_intensity) {
      const size_t intensity_offset = cloud.point_step * i + 3 * sizeof(float32_t);
      if (msg->fields[3U].datatype == sensor_msgs::msg::PointField::FLOAT32) {
        std::memcpy(&intensity, cloud.data.data() + intensity_offset, sizeof(float32_t));
      } else if (msg->fields[3U].datatype == sensor_msgs::msg::PointField::UINT8) {
        intensity = static_cast<float32_t>(cloud.data[intensity_offset]);
      } else {
        throw std::runtime_error("intensity datatype is not float or uint8_t");
      }
    }
    uint8_t * dest_intensity = adjusted_cloud.data.data() + adjusted_cloud.point_step * i + 3 *
      sizeof(float32_t);
    std::memcpy(dest_intensity, &intensity, sizeof(float32_t));
  }

  *msg = std::move(adjusted_cloud);
}

NDTMapPublisher::NDTMapPublisher(
  const MapConfig & map_config,
  sensor_msgs::msg::PointCloud2 & map_pc,
  sensor_msgs::msg::PointCloud2 & source_pc
)
: m_map_config(map_config),
  m_map_pc(map_pc),
  m_source_pc(source_pc)
{
}

geocentric_pose_t NDTMapPublisher::load_map(
  const std::string & yaml_file_name,
  const std::string & pcl_file_name)
{
  reset_pc_msg(m_map_pc);  // TODO(yunus.caliskan): Change in #102
  reset_pc_msg(m_source_pc);  // TODO(yunus.caliskan): Change in #102

  geodetic_pose_t geodetic_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!yaml_file_name.empty()) {
    read_from_yaml(yaml_file_name, &geodetic_pose);
  } else {
    throw std::runtime_error("YAML file name empty\n");
  }

  if (!pcl_file_name.empty()) {
    read_from_pcd(pcl_file_name, &m_source_pc);
  } else {
    throw std::runtime_error("PCD file name empty\n");
  }

  float64_t x(0.0), y(0.0), z(0.0);

  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Forward(
    geodetic_pose.latitude,
    geodetic_pose.longitude,
    geodetic_pose.elevation,
    x, y, z);

  return {x, y, z, geodetic_pose.roll, geodetic_pose.pitch, geodetic_pose.yaw};
}

void NDTMapPublisher::map_to_pc(const ndt::DynamicNDTMap & ndt_map)
{
  reset_pc_msg(m_map_pc);
  common::lidar_utils::resize_pcl_msg(m_map_pc, ndt_map.size());

  // TODO(yunus.caliskan): Make prettier -> #102
  sensor_msgs::PointCloud2Iterator<ndt::Real> x_it(m_map_pc, "x");
  sensor_msgs::PointCloud2Iterator<ndt::Real> y_it(m_map_pc, "y");
  sensor_msgs::PointCloud2Iterator<ndt::Real> z_it(m_map_pc, "z");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xx_it(m_map_pc, "icov_xx");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xy_it(m_map_pc, "icov_xy");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xz_it(m_map_pc, "icov_xz");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_yy_it(m_map_pc, "icov_yy");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_yz_it(m_map_pc, "icov_yz");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_zz_it(m_map_pc, "icov_zz");
  sensor_msgs::PointCloud2Iterator<uint32_t> cell_id_it(m_map_pc, "cell_id");

  auto num_used_cells = 0U;
  for (const auto & vx_it : ndt_map) {
    if (!  // No `==` operator defined for PointCloud2Iterators
      (y_it != y_it.end() &&
      z_it != z_it.end() &&
      icov_xx_it != icov_xx_it.end() &&
      icov_xy_it != icov_xy_it.end() &&
      icov_xz_it != icov_xz_it.end() &&
      icov_yy_it != icov_yy_it.end() &&
      icov_yz_it != icov_yz_it.end() &&
      icov_zz_it != icov_zz_it.end() &&
      cell_id_it != cell_id_it.end()))
    {
      // This should not occur as the cloud is resized to the map's size.
      throw std::length_error("NDTMapPublisherNode: NDT map is larger than the map point cloud.");
    }
    const auto & vx = vx_it.second;
    if (!vx.usable()) {
      // Voxel doesn't have enough points to be used in NDT
      continue;
    }

    const auto inv_covariance_opt = vx.inverse_covariance();
    if (!inv_covariance_opt) {
      // Voxel covariance is not invertible
      continue;
    }

    const auto & centroid = vx.centroid();
    const auto & inv_covariance = inv_covariance_opt.value();
    *(x_it) = centroid(0U);
    *(y_it) = centroid(1U);
    *(z_it) = centroid(2U);
    *(icov_xx_it) = inv_covariance(0U, 0U);
    *(icov_xy_it) = inv_covariance(0U, 1U);
    *(icov_xz_it) = inv_covariance(0U, 2U);
    *(icov_yy_it) = inv_covariance(1U, 1U);
    *(icov_yz_it) = inv_covariance(1U, 2U);
    *(icov_zz_it) = inv_covariance(2U, 2U);

    // There are cases where the centroid of a voxel does get indexed to another voxel. To prevent
    // ID mismatches while transferring the map. The index from the voxel grid config is used.
    const auto correct_idx = m_map_config.index(centroid);

    std::memcpy(&cell_id_it[0U], &(correct_idx), sizeof(correct_idx));
    ++x_it;
    ++y_it;
    ++z_it;
    ++icov_xx_it;
    ++icov_xy_it;
    ++icov_xz_it;
    ++icov_yy_it;
    ++icov_yz_it;
    ++icov_zz_it;
    ++cell_id_it;
    ++num_used_cells;
  }

  // Resize to throw out unused cells.
  common::lidar_utils::resize_pcl_msg(m_map_pc, num_used_cells);
}

void NDTMapPublisher::reset_pc_msg(sensor_msgs::msg::PointCloud2 & msg)
{
  auto dummy_idx = 0U;  // TODO(yunus.caliskan): Change in #102
  common::lidar_utils::reset_pcl_msg(msg, 0U, dummy_idx);
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
