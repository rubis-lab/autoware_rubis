# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(optimization)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)
ament_auto_find_build_dependencies()

# includes
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


set(OPTIMIZATION_LIB_SRC
        src/newtons_method_optimizer.cpp
        src/optimizer_options.cpp
        src/utils.cpp)

set(OPTIMIZATION_LIB_HEADERS
        include/optimization/visibility_control.hpp
        include/optimization/utils.hpp
        include/optimization/optimization_problem.hpp
        include/optimization/optimizer_options.hpp
        include/optimization/optimizer.hpp
        include/optimization/newtons_method_optimizer.hpp
        include/optimization/line_search/line_search.hpp
        include/optimization/line_search/fixed_line_search.hpp
        include/optimization/line_search/more_thuente_line_search.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${OPTIMIZATION_LIB_HEADERS}
        ${OPTIMIZATION_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME} -fopenmp)
target_compile_options(${PROJECT_NAME} PRIVATE
  "-fopenmp"
)

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "optimization-extras.cmake")
ament_auto_package()
