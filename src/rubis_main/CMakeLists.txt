# Copyright 2021 RUBIS
cmake_minimum_required(VERSION 3.5)

project(rubis_main)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(RUBIS_MAIN_LIB_SRC
  src/rubis_main.cpp
)

set(RUBIS_MAIN_LIB_HEADERS
  include/rubis_main/rubis_main.hpp
  include/rubis_main/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RUBIS_MAIN_LIB_SRC}
  ${RUBIS_MAIN_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

add_executable(rubis_main_runner
  include/rubis_main/rubis_main_parsed.hpp
  src/rubis_main_runner.cpp
)

target_include_directories(rubis_main_runner PUBLIC include
  /home/rubis/AutowareAuto/src/rubis_main/include/
)

# add package.xml dependencies here.
ament_target_dependencies(
  rubis_main_runner
  rclcpp

  point_cloud_filter_transform_nodes
  point_cloud_fusion_nodes
  ray_ground_classifier_nodes
  euclidean_cluster_nodes
  ndt_nodes

  rubis_drive
  rubis_detect
)

install(
  TARGETS rubis_main_runner
  DESTINATION lib/${PROJECT_NAME}
)

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch param
)
