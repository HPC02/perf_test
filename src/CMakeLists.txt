cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(test_array_update)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_subdirectory(perf_measure_tool)
endif()
