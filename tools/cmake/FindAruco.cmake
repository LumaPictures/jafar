# $Id$ #

#
# The following are set after configuration is done: 
#  Aruco_FOUND
#  Aruco_INCLUDE_DIRS
#  Aruco_LIBRARIES
#

set(aruco_path_guesses
  ${ARUCO_BASE}
  $ENV{ARUCO_BASE}  
  ${CMAKE_SOURCE_DIR}/../aruco
  ${CMAKE_SOURCE_DIR}/../../aruco
  ${CMAKE_SOURCE_DIR}/../../../aruco
  )

find_path(Aruco_INCLUDE_DIRS
  NAMES aruco.h
  PATHS
  ${aruco_path_guesses}
  ${ROBOTPKG_BASE}/include
  $ENV{ROBOTPKG_BASE}/include
  /usr/include
  /usr/local/include
  /sw/include
  /opt/local/include
  PATH_SUFFIXES
  sources
  src
  sources/src
  DOC "The directory where aruco.h resides")

string(TOLOWER ${CMAKE_BUILD_TYPE} build_type_lower)

find_library(Aruco_LIBRARIES
  NAMES aruco
  PATHS
  ${aruco_path_guesses}
  ${ROBOTPKG_BASE}/lib
  $ENV{ROBOTPKG_BASE}/lib
  /usr/lib
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /sw/lib
  /opt/local/lib
  PATH_SUFFIXES
  src
  build/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}
  build/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}/${build_type_lower}/src
  DOC "The aruco library")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Aruco  DEFAULT_MSG  Aruco_LIBRARIES  Aruco_INCLUDE_DIRS)

mark_as_advanced(Aruco_INCLUDE_DIRS Aruco_LIBRARIES)
