set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(example_frenet example_frenet.cpp)
target_link_libraries(example_frenet
  frenet
  ${OpenCV_LIBS}
  Python3::Python
  Python3::Module
)
