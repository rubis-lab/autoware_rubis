# Copyright 2017-2018 the Autoware Foundation
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

### Export headers
project(voxel_grid_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

### build node as library
set(VOXEL_GRID_NODE_LIB voxel_grid_node)
ament_auto_add_library(${VOXEL_GRID_NODE_LIB} SHARED
  include/voxel_grid_nodes/algorithm/voxel_cloud_base.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp
  include/voxel_grid_nodes/visibility_control.hpp
  src/algorithm/voxel_cloud_base.cpp
  src/algorithm/voxel_cloud_approximate.cpp
  src/algorithm/voxel_cloud_centroid.cpp
  include/voxel_grid_nodes/voxel_cloud_node.hpp
  src/voxel_cloud_node.cpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched.hpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp
)
autoware_set_compile_options(${VOXEL_GRID_NODE_LIB} -fopenmp)

rclcpp_components_register_node(${VOXEL_GRID_NODE_LIB}
  PLUGIN "autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode"
  EXECUTABLE ${VOXEL_GRID_NODE_LIB}_exe
)

target_include_directories(${VOXEL_GRID_NODE_LIB} PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

target_link_libraries(${VOXEL_GRID_NODE_LIB}
  ${OpenMP_LIBS}
)

target_compile_options(${VOXEL_GRID_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion "-fopenmp")

# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE
  param)
