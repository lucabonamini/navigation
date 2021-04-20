set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(OpenCV REQUIRED)

# Frenet
add_executable(example_frenet example_frenet.cpp)
target_include_directories(example_frenet PRIVATE ${PYTHON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(example_frenet
  frenet
  Python2::Python
  Python2::NumPy
  ${OpenCV_LIBS}
)