project(navigation)
cmake_minimum_required(VERSION 3.18.2)

add_subdirectory (astar)
add_subdirectory (quintic_polynomial_planner)
add_subdirectory (cubic_spline_planner)

option(BUILD_TESTS "Build tests" OFF)
if (${BUILD_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()