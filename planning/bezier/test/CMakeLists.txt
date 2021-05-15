set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)

add_executable(example_bezier example_bezier.cpp)
target_link_libraries(example_bezier
  glog
  gflags
  bezier
  Python2::Python
  Python2::NumPy
)
target_include_directories(example_bezier PUBLIC ../../../)