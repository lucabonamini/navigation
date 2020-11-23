find_package(PythonLibs 3.0)
find_package(OpenCV REQUIRED)

# AStar
add_executable(example_astar
  src/example_astar.cpp ${SRCS}
)
target_link_libraries(example_astar
  ${OpenCV_LIBS}
  glog
  gflags
)

# Cubic
add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_include_directories(example_cubic_spline_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_cubic_spline_planner
  cubic_spline_planner
  ${PYTHON_LIBRARIES}
)

# Quintic
add_executable(example_quintic_planner example_quintic_planner.cpp)
target_include_directories(example_quintic_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_quintic_planner
  glog
  gflags
  quintic_polynomial_planner
  ${PYTHON_LIBRARIES}
)

#### TESTS ####
add_executable(test_quintic_planner test_quintic_planner.cpp)
target_link_libraries(test_quintic_planner gtest_main quintic_polynomial_planner)

add_test(NAME test_quintic_planner COMMAND test_quintic_planner)
########
