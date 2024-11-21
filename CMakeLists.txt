cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(perf_tests CXX)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ standard to be used")
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE STRING "C++ standard to be used")

if(NOT CMAKE_CONFIGURATION_TYPES)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
    Release
    CACHE STRING "Release" FORCE)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  add_definitions(-DSPDLOG_WCHAR_TO_UTF8_SUPPORT) # VS 输出窗口支持 UTF8
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN)
  add_compile_definitions(-DNDEBUG)
else()
  add_compile_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(CMAKE_POLICY_DEFAULT_CMP0079 NEW)
  cmake_policy(SET CMP0074 NEW) # PCL_ROOT not set warning
  cmake_policy(SET CMP0144 NEW)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # https://cmake.org/cmake/help/latest/variable/CMAKE_MSVC_RUNTIME_LIBRARY.html#variable:CMAKE_MSVC_RUNTIME_LIBRARY
  cmake_policy(SET CMP0091 NEW)
  cmake_policy(SET CMP0167 NEW)
endif()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive- /GS /TP /W3 /Zc:wchar_t
# /DAPL_CORE_UWP  /Zc:inline /fp:precise /DWIN32_LEAN_AND_MEAN /DNOGDI /DWIN32
# /D_WINDOWS /D_USE_MATH_DEFINES /utf-8")
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/release")
  else()
    list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/debug")
  endif()

  # list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/pcl_1.14.1")
  # list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/pcl_1.14.1/3rdParty/Boost")
  list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/opencv/x64/vc16/lib")
  list(APPEND CMAKE_PREFIX_PATH "D:/dev_libs/oneAPI/tbb/latest/lib")
endif()

if(MSVC)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O3 /Ox /Ot /Qpar /Gy /Gw /GF")
  endif()

  string(REGEX REPLACE "/W[1-3]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
else()
  add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}\n")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")

# package boost
find_package(Boost REQUIRED COMPONENTS filesystem regex)
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libs: ${Boost_LIBRARIES}")

find_package(spdlog REQUIRED)

# option(ENABLE_TBB "Enable intel TBB" OFF)
find_package(TBB CONFIG REQUIRED)
message(STATUS "TBB version: ${TBB_VERSION}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin") # /lib

option(ENABLE_PROFILER "Enable google perftools" ON)

if(ENABLE_PROFILER)
  find_package(Gperftools) # REQUIRED

  if(GPERFTOOLS_FOUND)
    set(PROFILER_LIBS ${GPERFTOOLS_PROFILER})
    message(STATUS "PROFILER_LIBS: ${GPERFTOOLS_PROFILER}")
    add_definitions("-DHAVE_PROFILER")
  else()
    set(PROFILER_LIBS "")
  endif()
endif()

# avoid build errors caused by chinese character in source code
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
endif()

include_directories(system ${PROJECT_SOURCE_DIR}/3rd/include)

add_subdirectory(src)
