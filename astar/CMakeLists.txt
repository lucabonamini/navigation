project(astar)
cmake_minimum_required(VERSION 3.18.2)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
  -pedantic -Wall -Wextra -Wcast-align -Wcast-qual \
  -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept \
  -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow \
  -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 \
  -Wswitch-default -Wundef -Werror -Wunused")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "9.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpessimizing-move -Wredundant-move")
  endif()
endif()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include/
  ${EIGEN_INCLUDE_DIRS}
)

list(APPEND SRCS
  src/astar.cpp
)

add_executable(${CMAKE_PROJECT_NAME} 
  src/main.cpp ${SRCS}
)
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${OpenCV_LIBS}
  glog
  gflags
)


install(TARGETS ${CMAKE_PROJECT_NAME} 
  ARCHIVE DESTINATION /usr/local/lib 
  LIBRARY DESTINATION /usr/local/lib
)
install(DIRECTORY include/ 
  DESTINATION /usr/local/include
)
