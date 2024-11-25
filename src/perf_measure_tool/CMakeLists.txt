cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

file(GLOB SRCS *.cc)

set(target_name perf_measure_tool)
add_executable(${target_name} ${SRCS})
target_link_libraries(${target_name} TBB::tbb)
