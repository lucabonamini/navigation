set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

add_executable(example_quintic_planner example_quintic_planner.cpp)
target_link_libraries(example_quintic_planner
  glog
  gflags
  quintic_polynomial_planner
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_quintic_planner PUBLIC ../../../)