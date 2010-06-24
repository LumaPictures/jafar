# $Id: $ #

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

  # if(EXISTS ${MTI_LIBRARIES} AND EXISTS ${MTI_INCLUDE_DIRS})
  #   set(MTI_FOUND)
  # endif(EXISTS ${MTI_LIBRARIES} AND EXISTS ${MTI_INCLUDE_DIRS})
  # mark_as_advanced( FORCE MTI_FOUND )

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MTI  DEFAULT_MSG  MTI_LIBRARIES  MTI_INCLUDE_DIRS)

MARK_AS_ADVANCED(MTI_INCLUDE_DIRS MTI_LIBRARIES)
