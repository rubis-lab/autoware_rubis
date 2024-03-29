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

### Build the nodes
project(lidar_integration)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Task is deprecated
add_compile_options(-Wno-deprecated-declarations)

# spoofers
set(SPOOFER_LIB lidar_integration_spoofer)
ament_auto_add_library(${SPOOFER_LIB} SHARED
  src/udp_sender.cpp
  src/vlp16_integration_spoofer.cpp
  src/point_cloud_mutation_spoofer.cpp)
autoware_set_compile_options(${SPOOFER_LIB})
# FIXME "htons" used in src/udp_sender.cpp produces sign conversion for gcc 7.5
# on ARM 64 Linux when compiling with optimizations, thus disable warning for
# now until UdpSender is ported to cross platform implementation.
# See:
#   https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/merge_requests/116#note_313794141
target_compile_options(${SPOOFER_LIB} PRIVATE -Wno-sign-conversion)

set(VLP16_INTEGRATION_SPOOFER vlp16_integration_spoofer_exe)
ament_auto_add_executable(${VLP16_INTEGRATION_SPOOFER}
  src/vlp16_integration_spoofer_main.cpp)
autoware_set_compile_options(${VLP16_INTEGRATION_SPOOFER})
add_dependencies(${VLP16_INTEGRATION_SPOOFER} ${SPOOFER_LIB})

set(POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER point_cloud_mutation_spoofer_exe)
ament_auto_add_executable(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER}
  src/point_cloud_mutation_spoofer_main.cpp
  include/lidar_integration/point_cloud_mutation_spoofer.hpp)
autoware_set_compile_options(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER})
add_dependencies(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER} ${SPOOFER_LIB})

# # LIDAR_INTEGRATION_LISTENER
set(LIDAR_LISTENER lidar_integration_listener)
ament_auto_add_library(${LIDAR_LISTENER} SHARED
  src/lidar_integration_listener.cpp)
autoware_set_compile_options(${LIDAR_LISTENER})

set(LIDAR_INTEGRATION_LISTENER lidar_integration_listener_exe)
ament_auto_add_executable(${LIDAR_INTEGRATION_LISTENER}
  src/lidar_lc_integration_listener.hpp
  src/lidar_integration_listener_main.cpp)
autoware_set_compile_options(${LIDAR_INTEGRATION_LISTENER})
add_dependencies(${LIDAR_INTEGRATION_LISTENER} ${LIDAR_LISTENER})

# "Unit test" library
ament_auto_add_library(${PROJECT_NAME} SHARED src/lidar_integration.cpp)
autoware_set_compile_options(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${SPOOFER_LIB} ${LIDAR_LISTENER})

## Testing
if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Python helpers for launch_testing/ros_testing
ament_python_install_package(lidar_integration
    PACKAGE_DIR lidar_integration
)

## install stuff
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "lidar_integration-extras.cmake"
)
ament_auto_package(INSTALL_TO_SHARE
  cmake
)
