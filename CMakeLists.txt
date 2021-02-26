project(navigation)
cmake_minimum_required(VERSION 3.18.2)

add_subdirectory (planning)
add_subdirectory (control)

option(BUILD_TESTS "Build tests" OFF)
if (${BUILD_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()