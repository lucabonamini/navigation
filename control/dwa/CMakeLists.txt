project(dwa)
cmake_minimum_required(VERSION 3.18.2)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
  -pedantic -Wall -Wextra -Wcast-align \
  -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept \
  -Woverloaded-virtual -Wredundant-decls -Wshadow \
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
  src/dwa.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SRCS}
)
target_link_libraries(${PROJECT_NAME}
  models
)

option(BUILD_TESTS "Build tests" OFF)
if (${BUILD_TESTS})
  enable_testing()
  add_subdirectory(test)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION /usr/local/lib
  LIBRARY DESTINATION /usr/local/lib
)
install(DIRECTORY include/
  DESTINATION /usr/local/include
)
