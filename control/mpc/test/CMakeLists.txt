set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_executable(example_mpc example_mpc.cpp)
target_link_libraries(example_mpc
  glog
  gflags
  cubic_spline_planner
  mpc
  Python3::Python
  Python3::Module
)