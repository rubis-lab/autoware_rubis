# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_nodes)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)
ament_auto_find_build_dependencies()

# includes
set(LOCALIZATION_NODES_LIB_SRC
    src/localization_node.cpp
    /home/rubis/AutowareAuto/src/rubis_rt/include/rubis_rt/sched_log.hpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_NODES_LIB_SRC}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
  -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_link_libraries(${PROJECT_NAME}
  ${OpenMP_LIBS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  /home/rubis/AutowareAuto/src/rubis_rt/include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(LOCALIZATION_NODE_TEST localization_node_gtest)

  ament_add_gtest(${LOCALIZATION_NODE_TEST}
          test/test_relative_localizer_node.hpp
          test/test_relative_localizer_node.cpp)
  autoware_set_compile_options(${LOCALIZATION_NODE_TEST})
  target_compile_options(${LOCALIZATION_NODE_TEST} PRIVATE -Wno-sign-conversion -Wno-conversion)
  target_link_libraries(${LOCALIZATION_NODE_TEST} ${PROJECT_NAME})
endif()

ament_auto_package()
