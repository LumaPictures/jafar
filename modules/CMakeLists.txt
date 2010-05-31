# $Id$ #

#----------------------------------------------------------------------
# return a list of directories that we should add_subdirectory()
#----------------------------------------------------------------------#
# Copied from Troy D. Straszheim                                       #
#                                                                      #
#                                                                      #
# Find each subdirectory containing a CMakeLists.txt file, and include #
# it. This avoids the need to manually list which libraries in Jafar   #
# have CMakeLists.txt files.                                           #
#----------------------------------------------------------------------#
macro(JAFAR_COLLECT_SUBPROJECT_DIRECTORY_NAMES dirname varname filename)
  file(GLOB globbed RELATIVE "${dirname}" "${dirname}/*/${filename}")
  foreach(file ${globbed})
    get_filename_component(dir ${file} PATH)
    set(${varname} ${${varname}} ${dir})
  endforeach()
endmacro()

# Find all of the subdirectories with CMakeLists.txt files in
# them. This contains all of the Jafar libraries.
jafar_collect_subproject_directory_names(${JAFAR_MODULES_PARENT_DIR} JAFAR_MODULES_DIRS "CMakeLists.txt")
foreach(subdir ${JAFAR_MODULES_DIRS})
  file(STRINGS ${JAFAR_MODULES_PARENT_DIR}/${subdir}/CMakeLists.txt  DEPENDENCIES 
       REGEX "^[ ]*REQUIRED_MODULES[ ]*.*")
  string(STRIP "${DEPENDENCIES}" DEPENDENCIES)
  string(REGEX REPLACE "REQUIRED_MODULES[ ]*" "" DEPENDENCIES "${DEPENDENCIES}")
  string(REGEX REPLACE " " ";" DEPENDENCIES "${DEPENDENCIES}")
  #message(STATUS "${subdir} DEPENDENCIES \"${DEPENDENCIES}\"")
  if(NOT("${DEPENDENCIES}" STREQUAL ""))
    string(TOUPPER "JAFAR_${subdir}_DEPENDS" THIS_MODULE_DEPENDS)
    set(${THIS_MODULE_DEPENDS} ${DEPENDENCIES})
    set(${THIS_MODULE_INCLUDES} ${DEPENDENCIES})
  endif(NOT("${DEPENDENCIES}" STREQUAL ""))
endforeach(subdir)

# Add all of the Jafar projects in reverse topological order, so that
# a library's dependencies show up before the library itself.
list(SORT JAFAR_MODULES_DIRS)
topological_sort(JAFAR_MODULES_DIRS JAFAR_ _DEPENDS)

set(JAFAR_SUBPROJECT_DIRS ${JAFAR_MODULES_DIRS})
message(STATUS "JAFAR SUBPROJECT DIRS: ${JAFAR_SUBPROJECT_DIRS}")

#
# Sanity-check for typos: all projects in BUILD_PROJECTS must exist
#
if ((NOT BUILD_PROJECTS STREQUAL "ALL") AND (NOT BUILD_PROJECTS STREQUAL "NONE"))
  foreach(project ${BUILD_PROJECTS})
    list(FIND JAFAR_MODULES_DIRS ${project} THIS_MODULE_INDEX)
    set(error "")
    if (THIS_MODULE_INDEX LESS 0)
      # checkout_jafar_module(${project} ${error})
      # if(error)
	message(FATAL_ERROR "Non existant project \"${project}\" specified in BUILD_PROJECTS couldn't be ckecked out.")
      # endif(error)
    endif()
  endforeach()
endif()

#
# include only directories of projects in BUILD_PROJECTS
#
message(STATUS "")
message(STATUS "Reading jafar project directories...")
message(STATUS "")
set(JAFAR_ALL_COMPONENTS "")

foreach(subdir ${JAFAR_SUBPROJECT_DIRS})
  list(FIND BUILD_PROJECTS ${subdir} THIS_BUILD_PROJECTS_INDEX)
  if ((THIS_BUILD_PROJECTS_INDEX GREATER -1) OR (BUILD_PROJECTS STREQUAL "ALL"))
    if(NOT(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}))
      # checkout_jafar_module(${subdir} error)
      # jafar_sort_modules()
      # if(${error})
	message(FATAL_ERROR "Non existant project \"${project}\" specified in BUILD_PROJECTS couldn't be ckecked out.")
      # endif(${error})
    endif(NOT(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}))
    message(STATUS "+ ${subdir}")
    add_subdirectory(${CMAKE_SOURCE_DIR}/modules/${subdir})
  endif()
endforeach()

set(JAFAR_ALL_COMPONENTS ${JAFAR_ALL_COMPONENTS} PARENT_SCOPE)

# message(STATUS "")
# message(STATUS "Reading jafar project test directories...")
# message(STATUS "")
# foreach(PROJ ${JAFAR_TEST_PROJECTS})
#   string(TOLOWER ${PROJ} proj)
#   project(${proj})
#   set(JAFAR_PROJECT_NAME ${proj})
#   foreach(dir ${JAFAR_${PROJ}_TESTDIRS})
#     message(STATUS "+ ${proj}")
#     add_subdirectory(${dir} ${CMAKE_BINARY_DIR}/modules/${proj}/test_suite)
#   endforeach()
# endforeach()

# Write out a GraphViz file containing inter-library dependencies. 
set(JAFAR_DEPENDENCY_GRAPHVIZ_FILE "${Jafar_BINARY_DIR}/doc/dependencies.dot")
file(WRITE ${JAFAR_DEPENDENCY_GRAPHVIZ_FILE} "digraph jafar {\n")
foreach(SUBDIR ${JAFAR_SUBPROJECT_DIRS})
  string(TOUPPER "JAFAR_${SUBDIR}_COMPILED_LIB" JAFAR_COMPILED_LIB_VAR)
  if (${JAFAR_COMPILED_LIB_VAR})
    file(APPEND ${JAFAR_DEPENDENCY_GRAPHVIZ_FILE} "  \"${SUBDIR}\" [style=\"filled\" fillcolor=\"#A3A27C\" shape=\"box\"];\n ")
  endif (${JAFAR_COMPILED_LIB_VAR})
  string(TOUPPER "JAFAR_${SUBDIR}_DEPENDS" DEPENDS_VAR)
  if(DEFINED ${DEPENDS_VAR})
    foreach(DEP ${${DEPENDS_VAR}})
      file(APPEND ${JAFAR_DEPENDENCY_GRAPHVIZ_FILE} 
        "  \"${SUBDIR}\" -> \"${DEP}\";\n")
    endforeach()
  endif()
endforeach()
file(APPEND ${JAFAR_DEPENDENCY_GRAPHVIZ_FILE} "  \"test\" [style=\"filled\" fillcolor=\"#A3A27C\" shape=\"box\"];\n ")
file(APPEND ${JAFAR_DEPENDENCY_GRAPHVIZ_FILE} "}\n")
