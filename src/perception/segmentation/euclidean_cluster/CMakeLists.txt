# Copyright 2017-2019 the Autoware Foundation
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
project(euclidean_cluster)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# includes
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/euclidean_cluster/euclidean_cluster.hpp
  include/euclidean_cluster/visibility_control.hpp
  src/euclidean_cluster.cpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched.hpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp
)
autoware_set_compile_options(${PROJECT_NAME} -fopenmp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion "-fopenmp")

target_include_directories(${PROJECT_NAME} PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(EUCLIDEAN_CLUSTER_GTEST euclidean_cluster_gtest)
  ament_add_gtest(${EUCLIDEAN_CLUSTER_GTEST} test/src/test_euclidean_cluster.cpp)
  autoware_set_compile_options(${EUCLIDEAN_CLUSTER_GTEST})
  target_compile_options(${EUCLIDEAN_CLUSTER_GTEST} PRIVATE -Wno-conversion)
  target_include_directories(${EUCLIDEAN_CLUSTER_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${EUCLIDEAN_CLUSTER_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${EUCLIDEAN_CLUSTER_GTEST} "autoware_auto_geometry")
endif()

ament_auto_package()
