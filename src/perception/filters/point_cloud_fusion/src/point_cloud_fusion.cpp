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

  for (size_t i = 0; i < m_input_topics_size; ++i) {
    concatenate_pointcloud(*msgs[i], cloud_concatenated, pc_concat_idx);
  }
  return pc_concat_idx;
}

uint32_t PointCloudFusion::fuse_pc_msgs_rubis(
  const std::array<PointCloudMsgT::ConstSharedPtr, 8> & msgs,
  PointCloudMsgT & cloud_concatenated, sched_info _si)
{
  uint32_t pc_concat_idx = 0;

  omp_set_dynamic(0);

  #pragma omp parallel num_threads(_si.max_option)
  {
    // configure rt
    auto thr_id = omp_get_thread_num();

    if(!__rt_configured[thr_id]) {
      auto tid = gettid();
      std::cout << "[PointCloudFusion] (" << tid << "): __rt_configured (" << _si.exec_time << ", " << _si.deadline << ", " << _si.period << ")" << std::endl;
      rubis::sched::set_sched_deadline(tid, _si.exec_time, _si.deadline, _si.period);
      __rt_configured[thr_id] = true;
    }

    #pragma omp barrier

    // workload start
    auto start_time = omp_get_wtime();

    #pragma omp for schedule(dynamic) nowait
    for (size_t i = 0; i < m_input_topics_size; ++i) {
        concatenate_pointcloud(*msgs[i], cloud_concatenated, pc_concat_idx);
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
    #pragma omp critical
    {
      __slog.add_entry(sd);
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

}  // namespace point_cloud_fusion
}  // namespace filters
}  // namespace perception
}  // namespace autoware
