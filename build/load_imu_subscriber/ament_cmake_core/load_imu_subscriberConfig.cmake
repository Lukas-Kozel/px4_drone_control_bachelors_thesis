# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_load_imu_subscriber_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED load_imu_subscriber_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(load_imu_subscriber_FOUND FALSE)
  elseif(NOT load_imu_subscriber_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(load_imu_subscriber_FOUND FALSE)
  endif()
  return()
endif()
set(_load_imu_subscriber_CONFIG_INCLUDED TRUE)

# output package information
if(NOT load_imu_subscriber_FIND_QUIETLY)
  message(STATUS "Found load_imu_subscriber: 0.0.0 (${load_imu_subscriber_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'load_imu_subscriber' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${load_imu_subscriber_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(load_imu_subscriber_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${load_imu_subscriber_DIR}/${_extra}")
endforeach()
