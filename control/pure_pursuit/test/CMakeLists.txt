set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

add_executable(example_pure_pursuit example_pure_pursuit.cpp)
target_link_libraries(example_pure_pursuit
  glog
  gflags
  pure_pursuit
  cubic_spline_planner
  models
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_pure_pursuit PUBLIC ../../../)