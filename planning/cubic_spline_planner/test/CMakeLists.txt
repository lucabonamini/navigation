set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

# Cubic
add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_include_directories(example_cubic_spline_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_cubic_spline_planner
  cubic_spline_planner
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_cubic_spline_planner PUBLIC ../../../)