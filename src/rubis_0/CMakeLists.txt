# Copyright 2021 The Autoware Foundation
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

project(rubis_0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# rubis
set(CMAKE_CXX_FLAGS "-O0")

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(RUBIS_0_LIB_SRC
  src/rubis_0.cpp
)

set(RUBIS_0_LIB_HEADERS
  include/rubis_0/rubis_0.hpp
  include/rubis_0/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RUBIS_0_LIB_SRC}
  ${RUBIS_0_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(RUBIS_0_NODE_SRC
  src/rubis_0_node.cpp
)

set(RUBIS_0_NODE_HEADERS
  include/rubis_0/rubis_0_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${RUBIS_0_NODE_SRC}
  ${RUBIS_0_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::rubis_0::Rubis0Node"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_rubis_0.cpp)
  set(TEST_RUBIS_0_EXE test_rubis_0)
  ament_add_gtest(${TEST_RUBIS_0_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_RUBIS_0_EXE})
  target_link_libraries(${TEST_RUBIS_0_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  param data launch
)

##### rubis

find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(ndt_nodes REQUIRED)
# find_package(point_cloud_fusion_nodes REQUIRED)
# find_package(mpc_controller_nodes REQUIRED)
# # find_package(lanelet2_core REQURED)
# find_package(lanelet2_map_provider REQUIRED)
# find_package(voxel_grid_nodes REQUIRED)
# # find_package(casadi_vendor REQUIRED)
# find_package(parking_planner_nodes REQUIRED)
# find_package(point_cloud_filter_transform_nodes REQUIRED)
# find_package(object_collision_estimator_nodes REQUIRED)
# find_package(lanelet2_global_planner_nodes REQUIRED)
# find_package(lane_planner_nodes REQUIRED)
# find_package(behavior_planner_nodes REQUIRED)
# find_package(ray_ground_classifier_nodes REQUIRED)
# find_package(euclidean_cluster_nodes REQUIRED)

ament_auto_find_build_dependencies()

# add_executable(
#   rubis_0_runner
#   src/rubis_0_runner.cpp
#   src/rubis_0.cpp
#   src/rubis_0_node.cpp
#   /home/rubis/AutowareAuto/src/rt/include/rubis_sched.hpp
# )

# target_include_directories(rubis_0_runner PUBLIC include /home/rubis/AutowareAuto/src/rt/include/)
# ament_target_dependencies(
#   rubis_0_runner
#   rclcpp
#   rclcpp_components
#   std_msgs
#   ndt_nodes
#   mpc_controller_nodes
#   lanelet2_map_provider
#   voxel_grid_nodes
#   parking_planner_nodes
#   point_cloud_filter_transform_nodes
#   object_collision_estimator_nodes
#   lanelet2_global_planner_nodes
#   lane_planner_nodes
#   behavior_planner_nodes
#   ray_ground_classifier_nodes
#   euclidean_cluster_nodes
#   point_cloud_fusion_nodes
# )


# install(
#   TARGETS rubis_0_runner
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

### end rubis