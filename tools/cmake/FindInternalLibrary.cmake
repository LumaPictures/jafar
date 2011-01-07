# $Id$ #

macro(FIND_INTERNAL_LIBRARY internal_library)
  message(STATUS "looking for ${internal_library}")
  string(TOLOWER "${internal_library}" library_name)
  string(TOUPPER "${internal_library}" LIBRARY_NAME)
  string(TOUPPER "${internal_library}_FOUND" LIBRARY_NAME_FOUND)
  string(TOUPPER "${internal_library}_INCLUDE_DIRS" LIBRARY_NAME_INCLUDE_DIRS)
  string(TOUPPER "${internal_library}_LIBRARIES" LIBRARY_NAME_LIBRARIES)

  string(TOUPPER "${internal_library}_INCLUDE_DIRS1" LIBRARY_NAME_INCLUDE_DIRS1)
  string(TOUPPER "${internal_library}_INCLUDE_DIRS2" LIBRARY_NAME_INCLUDE_DIRS2)

  set(${LIBRARY_NAME_FOUND} 0)

  # if a header is in root/include/<library>/<library>.h, we want both
  # root/include and root/include/<library> as include_dir
  # the first find_path is more general and always work if the lib is present
  # we should try to improve the way it is implemented for now...

  find_path(${LIBRARY_NAME_INCLUDE_DIRS1}
    NAMES ${internal_library}.h ${library_name}.h ${LIBRARY_NAME}.h lib${internal_library}.h lib${library_name}.h lib${LIBRARY_NAME}.h ${internal_library}lib.h ${library_name}lib.h ${LIBRARY_NAME}lib.h
    PATHS
    ${ROBOTPKG_BASE}/include
    $ENV{ROBOTPKG_BASE}/include
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    PATH_SUFFIXES ${internal_library} ${library_name} ${LIBRARY_NAME}
    DOC "The directory where ${internal_library.h} resides")

  find_path(${LIBRARY_NAME_INCLUDE_DIRS2}
    NAMES
    ${library_name}/${internal_library}.h ${library_name}/${library_name}.h ${library_name}/${LIBRARY_NAME}.h ${library_name}/lib${internal_library}.h ${library_name}/lib${library_name}.h ${library_name}/lib${LIBRARY_NAME}.h ${library_name}/${internal_library}lib.h ${library_name}/${library_name}lib.h ${library_name}/${LIBRARY_NAME}lib.h
    ${internal_library}/${internal_library}.h ${internal_library}/${library_name}.h ${internal_library}/${LIBRARY_NAME}.h ${internal_library}/lib${internal_library}.h ${internal_library}/lib${library_name}.h ${internal_library}/lib${LIBRARY_NAME}.h ${internal_library}/${internal_library}lib.h ${internal_library}/${library_name}lib.h ${internal_library}/${LIBRARY_NAME}lib.h
    ${LIBRARY_NAME}/${internal_library}.h ${LIBRARY_NAME}/${library_name}.h ${LIBRARY_NAME}/${LIBRARY_NAME}.h ${LIBRARY_NAME}/lib${internal_library}.h ${LIBRARY_NAME}/lib${library_name}.h ${LIBRARY_NAME}/lib${LIBRARY_NAME}.h ${LIBRARY_NAME}/${internal_library}lib.h ${LIBRARY_NAME}/${library_name}lib.h ${LIBRARY_NAME}/${LIBRARY_NAME}lib.h
    PATHS
    ${ROBOTPKG_BASE}/include
    $ENV{ROBOTPKG_BASE}/include
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where ${internal_library}/${internal_library.h} resides")

  SET(${LIBRARY_NAME_INCLUDE_DIRS} "${${LIBRARY_NAME_INCLUDE_DIRS1}}")
  if (EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS2}})
    SET(${LIBRARY_NAME_INCLUDE_DIRS} "${${LIBRARY_NAME_INCLUDE_DIRS}};${${LIBRARY_NAME_INCLUDE_DIRS2}}")
  endif(EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS2}})

  message(STATUS "\t${internal_library} headers: ${${LIBRARY_NAME_INCLUDE_DIRS}}")

  find_library(${LIBRARY_NAME_LIBRARIES} 
    NAMES ${internal_library} ${LIBRARY_NAME}
    PATHS
    ${ROBOTPKG_BASE}/lib
    $ENV{ROBOTPKG_BASE}/lib
    /usr/lib
    /usr/lib64
    /usr/local/lib
    /usr/local/lib64
    /sw/lib
    /opt/local/lib
    DOC "The ${internal_library} library")
  message(STATUS "\t${internal_library} libraries: ${${LIBRARY_NAME_LIBRARIES}}")

  if(EXISTS ${${LIBRARY_NAME_LIBRARIES}} AND (EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS1}} OR EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS2}}))
    set(${LIBRARY_NAME_FOUND} 1)
  endif(EXISTS ${${LIBRARY_NAME_LIBRARIES}} AND (EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS1}} OR EXISTS ${${LIBRARY_NAME_INCLUDE_DIRS2}}))
  mark_as_advanced( FORCE ${LIBRARY_NAME_FOUND} )

endmacro(FIND_INTERNAL_LIBRARY internal_library)

macro(FIND_THIS_PACKAGE package_name)
		string(TOUPPER "${package_name}" PACKAGE_NAME)
		pkg_check_modules(${PACKAGE_NAME} ${package_name})
endmacro(FIND_THIS_PACKAGE package_name)
