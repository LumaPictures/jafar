# $Id: FindInternalLibrary.cmake 4624 2010-06-24 11:59:28Z croussil $ #

macro(FIND_INTERNAL_LIBRARY internal_library)
  message(STATUS "looking for ${internal_library}")
  string(TOLOWER "${internal_library}" library_name)
  string(TOUPPER "${internal_library}" LIBRARY_NAME)
  string(TOUPPER "${internal_library}_FOUND" LIBRARY_NAME_FOUND)
  string(TOUPPER "${internal_library}_INCLUDE_DIRS" LIBRARY_NAME_INCLUDE_DIRS)
  string(TOUPPER "${internal_library}_LIBRARIES" LIBRARY_NAME_LIBRARIES)

  set(${LIBRARY_NAME_FOUND} 0)

  find_path(${LIBRARY_NAME_INCLUDE_DIRS}  
    NAMES ${internal_library}.h ${library_name}.h ${LIBRARY_NAME}.h lib${internal_library}.h lib${library_name}.h lib${LIBRARY_NAME}.h ${internal_library}lib.h ${library_name}lib.h ${LIBRARY_NAME}lib.h
    PATHS
    ${ROBOTPKG_BASE}/include
    $ENV{ROBOTPKG_BASE}/include
    PATH_SUFFIXES ${internal_library} ${library_name} ${LIBRARY_NAME}
    DOC "The directory where ${internal_library}/${internal_library.h} resides")
  message(STATUS "\t${internal_library} headers: ${${LIBRARY_NAME_INCLUDE_DIRS}}")

  find_library(${LIBRARY_NAME_LIBRARIES} 
    NAMES ${internal_library} ${LIBRARY_NAME}
    PATHS
    ${ROBOTPKG_BASE}/lib
    $ENV{ROBOTPKG_BASE}/lib
    DOC "The ${internal_library} library")
  message(STATUS "\t${internal_library} libraries: ${${LIBRARY_NAME_LIBRARIES}}")

  if(EXISTS ${${LIBRARY_NAME_LIBRARIES}} AND EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS}})
    set(${LIBRARY_NAME_FOUND} 1)
  endif(EXISTS ${${LIBRARY_NAME_LIBRARIES}} AND EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS}})
  mark_as_advanced( FORCE ${LIBRARY_NAME_FOUND} )
endmacro(FIND_INTERNAL_LIBRARY internal_library)
