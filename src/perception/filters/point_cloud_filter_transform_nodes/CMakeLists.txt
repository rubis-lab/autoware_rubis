# Copyright 2017-2020 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(point_cloud_filter_transform_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

### build node as library
set(FILTER_NODE_LIB point_cloud_filter_transform_node)
ament_auto_add_library(${FILTER_NODE_LIB} SHARED
    include/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp
    include/point_cloud_filter_transform_nodes/visibility_control.hpp
    src/point_cloud_filter_transform_node.cpp
    /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched.hpp
    /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp
    )
autoware_set_compile_options(${FILTER_NODE_LIB} -fopenmp)
target_compile_options(${FILTER_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion "-fopenmp")

rclcpp_components_register_node(${FILTER_NODE_LIB}
  PLUGIN "autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode"
  EXECUTABLE ${FILTER_NODE_LIB}_exe
)

target_link_libraries(${FILTER_NODE_LIB}
  ${OpenMP_LIBS}
)
target_include_directories(${FILTER_NODE_LIB} PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

ament_auto_package(
    INSTALL_TO_SHARE
    param
    test/param
)
