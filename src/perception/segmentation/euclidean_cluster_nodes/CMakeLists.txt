# Copyright 2017-2020 the Autoware Foundation
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

### Build the nodes
project(euclidean_cluster_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

# Library
set(CLUSTER_NODE_LIB euclidean_cluster_node)
ament_auto_add_library(${CLUSTER_NODE_LIB} SHARED
  include/euclidean_cluster_nodes/euclidean_cluster_node.hpp
  src/euclidean_cluster_node.cpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp)
autoware_set_compile_options(${CLUSTER_NODE_LIB} -fopenmp)
target_compile_options(
  ${CLUSTER_NODE_LIB} PUBLIC "-Wno-sign-conversion" "-Wno-conversion" "-fopenmp")

rclcpp_components_register_node(${CLUSTER_NODE_LIB}
  PLUGIN "autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode"
  EXECUTABLE ${CLUSTER_NODE_LIB}_exe
)

target_include_directories(${CLUSTER_NODE_LIB} PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

target_link_libraries(${CLUSTER_NODE_LIB}
  ${OpenMP_LIBS}
)

# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE
  launch
  param)
