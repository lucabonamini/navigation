project(models)
cmake_minimum_required(VERSION 3.18.2)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
  -pedantic -Wall -Wextra -Wcast-align -Wcast-qual \
  -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept \
  -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow \
  -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wunused")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "9.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpessimizing-move -Wredundant-move")
  endif()
endif()
# set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

include_directories(
  include/
)

list(APPEND SRCS
  src/unicycle.cpp
  src/bicycle.cpp
  src/dynamic_unicycle.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SRCS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PUBLIC ../../)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION /usr/local/lib
  LIBRARY DESTINATION /usr/local/lib
)
install(DIRECTORY include/
  DESTINATION /usr/local/include
)
