cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

file(GLOB SRCS *.cc)

set(target_name test_array_update)
add_executable(${target_name} ${SRCS})
target_link_libraries(${target_name} TBB::tbb)
