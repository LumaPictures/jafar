# $Id$ #

find_path(MTI_INCLUDE_DIRS
  NAMES MTI.h
  PATHS
  ${ROBOTPKG_BASE}/include
  $ENV{ROBOTPKG_BASE}/include
  /usr/include
  /usr/local/include
  /sw/include
  /opt/local/include
  PATH_SUFFIXES MTI-clients
  DOC "The directory where MTI.h resides")

find_library(MTI_LIBRARIES
  NAMES MTI
  PATHS
  ${ROBOTPKG_BASE}/lib
  $ENV{ROBOTPKG_BASE}/lib
  /usr/lib
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /sw/lib
  /opt/local/lib
  DOC "The MTI library")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MTI  DEFAULT_MSG  MTI_LIBRARIES  MTI_INCLUDE_DIRS)

mark_as_advanced(MTI_INCLUDE_DIRS MTI_LIBRARIES)
