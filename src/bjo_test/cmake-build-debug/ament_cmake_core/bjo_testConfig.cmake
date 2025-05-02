# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bjo_test_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bjo_test_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bjo_test_FOUND FALSE)
  elseif(NOT bjo_test_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bjo_test_FOUND FALSE)
  endif()
  return()
endif()
set(_bjo_test_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bjo_test_FIND_QUIETLY)
  message(STATUS "Found bjo_test: 0.0.0 (${bjo_test_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bjo_test' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${bjo_test_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bjo_test_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bjo_test_DIR}/${_extra}")
endforeach()
