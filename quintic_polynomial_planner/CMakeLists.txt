project(quintic_polynomial_planner)
cmake_minimum_required(VERSION 3.18.2)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
  -pedantic -Wall -Wextra -Wcast-align -Wcast-qual \
  -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept \
  -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow \
  -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 \
  -Werror -Wunused")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "9.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpessimizing-move -Wredundant-move")
  endif()
endif()
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
)

list(APPEND SRCS
  src/quintic_polynomial_planner.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SRCS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION /usr/local/lib
  LIBRARY DESTINATION /usr/local/lib
)
install(DIRECTORY include/
  DESTINATION /usr/local/include
)
