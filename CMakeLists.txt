cmake_minimum_required(VERSION 3.15)

project(graph)

set(GRAPH_DIR ${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_STANDARD 23)

add_executable(graph ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

set_target_properties(
  graph PROPERTIES
    COMPILE_FLAGS " -Wall -Wextra "
)

target_include_directories(graph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(tests)
