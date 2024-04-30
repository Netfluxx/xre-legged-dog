# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_chienpanze_controllers_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED chienpanze_controllers_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(chienpanze_controllers_FOUND FALSE)
  elseif(NOT chienpanze_controllers_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(chienpanze_controllers_FOUND FALSE)
  endif()
  return()
endif()
set(_chienpanze_controllers_CONFIG_INCLUDED TRUE)

# output package information
if(NOT chienpanze_controllers_FIND_QUIETLY)
  message(STATUS "Found chienpanze_controllers: 0.0.0 (${chienpanze_controllers_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'chienpanze_controllers' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${chienpanze_controllers_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(chienpanze_controllers_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${chienpanze_controllers_DIR}/${_extra}")
endforeach()
