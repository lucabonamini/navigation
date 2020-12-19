set(CMAKE_CXX_STANDARD 20)

#### Setup GoogleTest ####
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)
########

# find python libraries
find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(PythonLibs 3.0 REQUIRED)
# include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
# find_package(PythonLibs 2.7)
find_package(OpenCV REQUIRED)

# AStar
add_executable(example_astar example_astar.cpp)
target_link_libraries(example_astar
  ${OpenCV_LIBS}
  glog
  gflags
  astar
)

# Cubic
add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_include_directories(example_cubic_spline_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_cubic_spline_planner
  cubic_spline_planner
  Python2::Python
  Python2::NumPy
  # ${PYTHON_LIBRARIES}
)

# Quintic
add_executable(example_quintic_planner example_quintic_planner.cpp)
target_link_libraries(example_quintic_planner
  glog
  gflags
  quintic_polynomial_planner
  Python2::Python
  Python2::NumPy
  # ${PYTHON_LIBRARIES}
)

# Lqr
add_executable(example_lqr example_lqr.cpp)
target_link_libraries(example_lqr
  glog
  gflags
  cubic_spline_planner
  lqr
  Python2::Python
  Python2::NumPy
  # ${PYTHON_LIBRARIES}
)

# Pid
add_executable(example_pid example_pid.cpp)
target_link_libraries(example_pid
  glog
  gflags
  cubic_spline_planner
  pid
  Python2::Python
  Python2::NumPy
  # ${PYTHON_LIBRARIES}
)

# GA
add_executable(ga ga.cpp)
target_link_libraries(ga
  glog
  gflags
  cubic_spline_planner
  pid
  Python2::Python
  Python2::NumPy
  # ${PYTHON_LIBRARIES}
)

#### TESTS ####
add_executable(test_quintic_planner test_quintic_planner.cpp)
target_link_libraries(test_quintic_planner gtest_main quintic_polynomial_planner)

add_test(NAME test_quintic_planner COMMAND test_quintic_planner)
########
