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

project(rubis_drive)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

set(RUBIS_DRIVE_LIB_SRC
  src/rubis_drive.cpp
)

set(RUBIS_DRIVE_LIB_HEADERS
  include/rubis_drive/rubis_drive.hpp
  include/rubis_drive/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RUBIS_DRIVE_LIB_SRC}
  ${RUBIS_DRIVE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(RUBIS_DRIVE_NODE_SRC
  src/rubis_drive_node.cpp
  /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp
)

set(RUBIS_DRIVE_NODE_HEADERS
  include/rubis_drive/rubis_drive_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${RUBIS_DRIVE_NODE_SRC}
  ${RUBIS_DRIVE_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::rubis_drive::RubisDriveNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenMP_LIBS}
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch param
)
