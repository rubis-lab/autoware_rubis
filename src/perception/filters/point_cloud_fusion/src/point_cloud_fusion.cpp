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

#include <point_cloud_fusion/point_cloud_fusion.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion
{

PointCloudFusion::PointCloudFusion(
  uint32_t cloud_capacity,
  size_t input_topics_size)
: m_cloud_capacity(cloud_capacity),
  m_input_topics_size(input_topics_size)
{
}

void PointCloudFusion::init_rubis(sched_info _si)
{
  __si = _si;
  __slog = SchedLog(__si);
  __iter = 0;

  for(int i = 0; i < __si.max_option; i++) {
    __rt_configured.push_back(false);
  }
  return;
}

uint32_t PointCloudFusion::fuse_pc_msgs(
  const std::array<PointCloudMsgT::ConstSharedPtr, 8> & msgs,
  PointCloudMsgT & cloud_concatenated)
{
  uint32_t pc_concat_idx = 0;

  omp_set_dynamic(0);

  #pragma omp parallel num_threads(__si.max_option)
  {
    // configure rt
    auto thr_id = omp_get_thread_num();

    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[PointCloudFusion] (" << tid << "): __rt_configured (" << __si.exec_time << ", " << __si.deadline << ", " << __si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, __si.exec_time, __si.deadline, __si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();



    for (size_t i = 0; i < m_input_topics_size; ++i) {
        // uint32_t local_pc_concat_idx = static_cast<int32_t>(i);
        // concatenate_pointcloud(*msgs[i], cloud_concatenated, local_pc_concat_idx);
        if (((*msgs[i]).width + pc_concat_idx) > m_cloud_capacity) {
          throw Error::TOO_LARGE;
        }

        sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_in(*msgs[i], "x");
        sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_in(*msgs[i], "y");
        sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_in(*msgs[i], "z");
        sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_in(*msgs[i], "intensity");
        sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_temp(*msgs[i], "intensity");

        size_t num_it = 0;
        while(intensity_it_temp != intensity_it_temp.end()) {
          ++intensity_it_temp;  //++ pre define operator
          num_it++;
        }

        #pragma omp for schedule(dynamic) nowait
        for (size_t it = 0; it < num_it; it++) {
          common::types::PointXYZIF pt;
          uint32_t i = static_cast<uint32_t>(it);

          pt.intensity = *(intensity_it_in+i);
          pt.x = *(x_it_in+i);
          pt.y = *(y_it_in+i);
          pt.z = *(z_it_in+i);

          uint32_t local_idx;
          #pragma omp critical (idx_lock)
          {
            local_idx = pc_concat_idx;
            pc_concat_idx += 1;
          }
          if(!common::lidar_utils::add_point_to_cloud_parallel(cloud_concatenated, pt, local_idx)) {
            throw Error::INSERT_FAILED;
          }
        }
    }

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
    //   std::cerr << "point_cloud_fusion::fuse_pc_msgs log added" << std::endl;
    }
    sched_yield();

  }  // prama omp parallel
  ++__iter;

  return pc_concat_idx;
}

void PointCloudFusion::concatenate_pointcloud(
  const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out,
  uint32_t & concat_idx) const
{
  if ((pc_in.width + concat_idx) > m_cloud_capacity) {
    throw Error::TOO_LARGE;
  }

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_in(pc_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_in(pc_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_in(pc_in, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_in(pc_in, "intensity");

  while (x_it_in != x_it_in.end() &&
    y_it_in != y_it_in.end() &&
    z_it_in != z_it_in.end() &&
    intensity_it_in != intensity_it_in.end())
  {
    common::types::PointXYZIF pt;
    pt.x = *x_it_in;
    pt.y = *y_it_in;
    pt.z = *z_it_in;
    pt.intensity = *intensity_it_in;

    if (common::lidar_utils::add_point_to_cloud(pc_out, pt, concat_idx)) {
      ++x_it_in;
      ++y_it_in;
      ++z_it_in;
      ++intensity_it_in;
    } else {
      // Somehow the point could be inserted to the concatenated cloud. Something regarding
      // the cloud sizes must be off.
      throw Error::INSERT_FAILED;
    }
  }
}

void PointCloudFusion::concatenate_pointcloud_raw(
  const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out,
  uint32_t & concat_idx) const
{
  if ((pc_in.width + concat_idx) > m_cloud_capacity) {
    throw Error::TOO_LARGE;
  }

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_in(pc_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_in(pc_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_in(pc_in, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_in(pc_in, "intensity");

  while (x_it_in != x_it_in.end() &&
    y_it_in != y_it_in.end() &&
    z_it_in != z_it_in.end() &&
    intensity_it_in != intensity_it_in.end())
  {
    common::types::PointXYZIF pt;
    pt.x = *x_it_in;
    pt.y = *y_it_in;
    pt.z = *z_it_in;
    pt.intensity = *intensity_it_in;

    if (common::lidar_utils::add_point_to_cloud_raw(pc_out, pt, concat_idx)) {
      ++x_it_in;
      ++y_it_in;
      ++z_it_in;
      ++intensity_it_in;
    } else {
      // Somehow the point could be inserted to the concatenated cloud. Something regarding
      // the cloud sizes must be off.
      throw Error::INSERT_FAILED;
    }
  }
}

}  // namespace point_cloud_fusion
}  // namespace filters
}  // namespace perception
}  // namespace autoware
