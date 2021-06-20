set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_executable(example_cubic_spline_planner example_cubic_spline_planner.cpp)
target_link_libraries(example_cubic_spline_planner
  cubic_spline_planner
  Python3::Python
  Python3::Module
)