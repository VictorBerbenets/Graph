cmake_minimum_required(VERSION 3.15)

project("testing")

aux_source_directory(./end2end/src END2END_TESTS)

add_executable(end2end ${END2END_TESTS})
add_executable(checker end2end/checker.cpp)

target_include_directories(end2end PRIVATE end2end/include ${INCLUDE_DIR})
target_include_directories(checker PRIVATE ${GRAPH_DIR})

