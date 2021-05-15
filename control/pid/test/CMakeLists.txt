set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

add_executable(example_pid example_pid.cpp)
target_link_libraries(example_pid
  glog
  gflags
  pid
  stanley_steer_control
  cubic_spline_planner
  models
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_pid PUBLIC ../../../)

##### TODO fix
# # GA
# add_executable(ga ga.cpp)
# target_link_libraries(ga
#   glog
#   gflags
#   cubic_spline_planner
#   pid
#   Python2::Python
#   Python2::NumPy
# )