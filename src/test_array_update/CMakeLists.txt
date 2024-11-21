cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

file(GLOB SRCS *.cc)

set(target_name test_array_update)
add_executable(${target_name} ${SRCS})
target_link_libraries(${target_name} TBB::tbb)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # target_link_libraries(${target_name} PRIVATE "-Wl,--no-as-needed" ${PROFILER_LIBS} "-Wl,--as-needed")
    target_compile_options(${target_name} PRIVATE -fno-omit-frame-pointer -g)
    target_link_options(${target_name} PRIVATE -fno-omit-frame-pointer)
endif()