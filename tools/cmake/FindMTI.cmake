
macro(FIND_MTI)
  message(STATUS "looking specifically for MTI")
#  string(TOLOWER "${internal_library}" library_name)
#  string(TOUPPER "${internal_library}" LIBRARY_NAME)
#  string(TOUPPER "${internal_library}_FOUND" LIBRARY_NAME_FOUND)
 # string(TOUPPER "${internal_library}_INCLUDE_DIRS" LIBRARY_NAME_INCLUDE_DIRS)
#  string(TOUPPER "${internal_library}_LIBRARIES" LIBRARY_NAME_LIBRARIES)

  set(MTI_FOUND 0)

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
  message(STATUS "\tMTI headers: ${MTI_INCLUDE_DIRS}")

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
  message(STATUS "\tMTI libraries: ${MTI_LIBRARIES}")

  if(EXISTS ${MTI_LIBRARIES} AND EXISTS ${MTI_INCLUDE_DIRS})
    set(MTI_FOUND 1)
  endif(EXISTS ${MTI_LIBRARIES} AND EXISTS ${MTI_INCLUDE_DIRS})
  mark_as_advanced( FORCE ${MTI_FOUND} )

endmacro(FIND_MTI)
