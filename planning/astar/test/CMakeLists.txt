set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

add_executable(example_astar example_astar.cpp)
target_link_libraries(example_astar
  ${OpenCV_LIBS}
  glog
  gflags
  astar
)