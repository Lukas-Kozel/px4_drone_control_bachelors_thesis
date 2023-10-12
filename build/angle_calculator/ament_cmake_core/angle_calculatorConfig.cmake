# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_angle_calculator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED angle_calculator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(angle_calculator_FOUND FALSE)
  elseif(NOT angle_calculator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(angle_calculator_FOUND FALSE)
  endif()
  return()
endif()
set(_angle_calculator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT angle_calculator_FIND_QUIETLY)
  message(STATUS "Found angle_calculator: 0.0.0 (${angle_calculator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'angle_calculator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${angle_calculator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(angle_calculator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${angle_calculator_DIR}/${_extra}")
endforeach()
