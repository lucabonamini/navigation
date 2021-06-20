set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

add_executable(example_rrt example_rrt.cpp)
target_link_libraries(example_rrt
  rrt
  ${OpenCV_LIBS}
)