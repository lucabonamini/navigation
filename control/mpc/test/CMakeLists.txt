set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

# MPC
add_executable(example_mpc example_mpc.cpp)
target_link_libraries(example_mpc
  glog
  gflags
  cubic_spline_planner
  mpc
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_mpc PUBLIC ../../../)
)