# $Id$ #

# Try to find the T3D includes and library
#
# 
# If found, these variables are set
# T3D_INCLUDE_DIR - where to find t3d.h
# T3D_LIBRARIES   - List of fully qualified libraries to link against.
# T3D_FOUND       - Do not attempt to use if "no" or undefined.

include(LibFindMacros)

# Dependencies
libfind_package(t3d)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(t3d_PKGCONF t3d)

# Include dir
find_path(t3d_INCLUDE_DIR
  NAMES t3d.h
  PATHS ${t3d_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(t3d_LIBRARY
  NAMES t3d
  PATHS ${t3d_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries that this lib depends on.
set(t3d_PROCESS_INCLUDES t3d_INCLUDE_DIR t3d_INCLUDE_DIRS)
set(t3d_PROCESS_LIBS t3d_LIBRARY t3d_LIBRARIES)
libfind_process(t3d)
