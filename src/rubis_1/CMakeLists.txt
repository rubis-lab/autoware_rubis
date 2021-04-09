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

project(rubis_1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(RUBIS_1_LIB_SRC
  src/rubis_1.cpp
)

set(RUBIS_1_LIB_HEADERS
  include/rubis_1/rubis_1.hpp
  include/rubis_1/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RUBIS_1_LIB_SRC}
  ${RUBIS_1_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(RUBIS_1_NODE_SRC
  src/rubis_1_node.cpp
)

set(RUBIS_1_NODE_HEADERS
  include/rubis_1/rubis_1_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${RUBIS_1_NODE_SRC}
  ${RUBIS_1_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::rubis_1::Rubis1Node"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_rubis_1.cpp)
  set(TEST_RUBIS_1_EXE test_rubis_1)
  ament_add_gtest(${TEST_RUBIS_1_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_RUBIS_1_EXE})
  target_link_libraries(${TEST_RUBIS_1_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)

##### rubis

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  rubis_1_runner
  src/rubis_1_runner.cpp
  src/rubis_1.cpp
  src/rubis_1_node.cpp
)

target_include_directories(rubis_1_runner PUBLIC include )
ament_target_dependencies(rubis_1_runner rclcpp rclcpp_components std_msgs)

install(
  TARGETS rubis_1_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

### end rubis