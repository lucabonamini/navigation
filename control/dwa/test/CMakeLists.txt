set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_executable(example_dwa example_dwa.cpp)
target_link_libraries(example_dwa
  dwa
  gflags
  Python3::Python
  Python3::Module
)
