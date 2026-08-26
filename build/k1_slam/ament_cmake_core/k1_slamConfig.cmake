# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_k1_slam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED k1_slam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(k1_slam_FOUND FALSE)
  elseif(NOT k1_slam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(k1_slam_FOUND FALSE)
  endif()
  return()
endif()
set(_k1_slam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT k1_slam_FIND_QUIETLY)
  message(STATUS "Found k1_slam: 0.0.0 (${k1_slam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'k1_slam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${k1_slam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(k1_slam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${k1_slam_DIR}/${_extra}")
endforeach()
