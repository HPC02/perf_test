# Originally from
# https://github.com/tenzir/vast/blob/e9ccdc9beb7652c5e5a83ac103dac0453a8ed368/cmake/FindGperftools.cmake

# Tries to find Gperftools.
#
# Usage of this module as follows:
#
# find_package(Gperftools)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# Gperftools_ROOT_DIR  Set this variable to the root installation of Gperftools
# if the module has problems finding the proper installation path.
#
# Variables defined by this module:
#
# GPERFTOOLS_FOUND              System has Gperftools libs/headers
# GPERFTOOLS_LIBRARIES          The Gperftools libraries (tcmalloc & profiler)
# GPERFTOOLS_INCLUDE_DIR        The location of Gperftools headers

find_library(
  GPERFTOOLS_TCMALLOC
  NAMES tcmalloc
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_library(
  GPERFTOOLS_PROFILER
  NAMES profiler
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_library(
  GPERFTOOLS_TCMALLOC_AND_PROFILER
  NAMES tcmalloc_and_profiler
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_path(
  GPERFTOOLS_INCLUDE_DIR
  NAMES gperftools/heap-profiler.h
  HINTS ${Gperftools_ROOT_DIR}/include)

set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_TCMALLOC_AND_PROFILER})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gperftools DEFAULT_MSG GPERFTOOLS_LIBRARIES
  GPERFTOOLS_INCLUDE_DIR)

mark_as_advanced(
  Gperftools_ROOT_DIR GPERFTOOLS_TCMALLOC GPERFTOOLS_PROFILER
  GPERFTOOLS_TCMALLOC_AND_PROFILER GPERFTOOLS_LIBRARIES GPERFTOOLS_INCLUDE_DIR)

# create IMPORTED targets
if(Gperftools_FOUND AND NOT TARGET gperftools::tcmalloc)
  add_library(gperftools::tcmalloc UNKNOWN IMPORTED)
  set_target_properties(
    gperftools::tcmalloc
    PROPERTIES IMPORTED_LOCATION ${GPERFTOOLS_TCMALLOC}
    INTERFACE_INCLUDE_DIRECTORIES "${GPERFTOOLS_INCLUDE_DIR}")
  add_library(gperftools::profiler UNKNOWN IMPORTED)
  set_target_properties(
    gperftools::profiler
    PROPERTIES IMPORTED_LOCATION ${GPERFTOOLS_PROFILER}
    INTERFACE_INCLUDE_DIRECTORIES "${GPERFTOOLS_INCLUDE_DIR}")
endif()