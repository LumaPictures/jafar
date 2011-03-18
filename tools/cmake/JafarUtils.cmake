# $ Id: $ #

##########################################################################
# This macro was copied from here					 #
# http://www.cmake.org/Wiki/CMakeMacroParseArguments                 	 #
# with courtesy of KitWare community					 #
##########################################################################
# There is a general convention for CMake commands that take optional    
# flags and/or variable arguments. Optional flags are all caps and 	 
# are added to the arguments to turn on. Variable arguments have an all  
# caps identifier to determine where each variable argument list starts. 
# The PARSE_ARGUMENTS macro, defined below, can be used by other macros  
# to parse arguments defined in this way.				 
#

MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})    
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})            
    SET(larg_names ${arg_names})    
    LIST(FIND larg_names "${arg}" is_arg_name)                   
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})    
      LIST(FIND loption_names "${arg}" is_option)            
      IF (is_option GREATER -1)
	     SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
	     SET(current_arg_list ${current_arg_list} ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

##########################################################################
# This function were copied from boost-cmake project.                    #
# The license terms is as follow                                         #
##########################################################################
# Copyright (C) 2007 Douglas Gregor <doug.gregor@gmail.com>              #
# Copyright (C) 2007 Troy Straszheim                                     #
#                                                                        #
# Distributed under the Boost Software License, Version 1.0.             #
# See accompanying file LICENSE_1_0.txt or copy at                       #
#   http://www.boost.org/LICENSE_1_0.txt                                 #
##########################################################################
# Perform a reverse topological sort on the given LIST. 
#   
#   topological_sort(my_list "MY_" "_EDGES")
#
# LIST is the name of a variable containing a list of elements to be
# sorted in reverse topological order. Each element in the list has a
# set of outgoing edges (for example, those other list elements that
# it depends on). In the resulting reverse topological ordering
# (written back into the variable named LIST), an element will come
# later in the list than any of the elements that can be reached by
# following its outgoing edges and the outgoing edges of any vertices
# they target, recursively. Thus, if the edges represent dependencies
# on build targets, for example, the reverse topological ordering is
# the order in which one would build those targets.
#
# For each element E in this list, the edges for E are contained in
# the variable named ${PREFIX}${E}${SUFFIX}, where E is the
# upper-cased version of the element in the list. If no such variable
# exists, then it is assumed that there are no edges. For example, if
# my_list contains a, b, and c, one could provide a dependency graph
# using the following variables:
#
#     MY_A_EDGES     b
#     MY_B_EDGES     
#     MY_C_EDGES     a b
#
#  With the involcation of topological_sort shown above and these
#  variables, the resulting reverse topological ordering will be b, a,
#  c.

function(topological_sort LIST PREFIX SUFFIX)
  # Clear the stack and output variable
  set(VERTICES "${${LIST}}")
  set(STACK)
  set(${LIST})

  # Loop over all of the vertices, starting the topological sort from
  # each one.
  foreach(VERTEX ${VERTICES})
    string(TOUPPER ${VERTEX} UPPER_VERTEX)

    # If we haven't already processed this vertex, start a depth-first
    # search from where.
    if (NOT FOUND_${UPPER_VERTEX})
      # Push this vertex onto the stack with all of its outgoing edges
      string(REPLACE ";" " " NEW_ELEMENT 
        "${VERTEX};${${PREFIX}${UPPER_VERTEX}${SUFFIX}}")
      list(APPEND STACK ${NEW_ELEMENT})

      # We've now seen this vertex
      set(FOUND_${UPPER_VERTEX} TRUE)

      # While the depth-first search stack is not empty
      list(LENGTH STACK STACK_LENGTH)
      while(STACK_LENGTH GREATER 0)
        # Remove the vertex and its remaining out-edges from the top
        # of the stack
        list(GET STACK -1 OUT_EDGES)
        list(REMOVE_AT STACK -1)

        # Get the source vertex and the list of out-edges
        separate_arguments(OUT_EDGES)
        list(GET OUT_EDGES 0 SOURCE)
        list(REMOVE_AT OUT_EDGES 0)

        # While there are still out-edges remaining
        list(LENGTH OUT_EDGES OUT_DEGREE)
        while (OUT_DEGREE GREATER 0)
          # Pull off the first outgoing edge
          list(GET OUT_EDGES 0 TARGET)
          list(REMOVE_AT OUT_EDGES 0)

          string(TOUPPER ${TARGET} UPPER_TARGET)
          if (NOT FOUND_${UPPER_TARGET})
            # We have not seen the target before, so we will traverse
            # its outgoing edges before coming back to our
            # source. This is the key to the depth-first traversal.

            # We've now seen this vertex
            set(FOUND_${UPPER_TARGET} TRUE)

            # Push the remaining edges for the current vertex onto the
            # stack
            string(REPLACE ";" " " NEW_ELEMENT 
              "${SOURCE};${OUT_EDGES}")
            list(APPEND STACK ${NEW_ELEMENT})

            # Setup the new source and outgoing edges
            set(SOURCE ${TARGET})
            string(TOUPPER ${SOURCE} UPPER_SOURCE)
            set(OUT_EDGES 
              ${${PREFIX}${UPPER_SOURCE}${SUFFIX}})
          endif(NOT FOUND_${UPPER_TARGET})

          list(LENGTH OUT_EDGES OUT_DEGREE)
        endwhile (OUT_DEGREE GREATER 0)

        # We have finished all of the outgoing edges for
        # SOURCE; add it to the resulting list.
        list(APPEND ${LIST} ${SOURCE})

        # Check the length of the stack
        list(LENGTH STACK STACK_LENGTH)
      endwhile(STACK_LENGTH GREATER 0)
    endif (NOT FOUND_${UPPER_VERTEX})
  endforeach(VERTEX)

  set(${LIST} ${${LIST}} PARENT_SCOPE)
endfunction(topological_sort)

#-------------------------------------------------------------------------------
# This macro checks out a required jafar module that it is not located under
# JAFAR_MODULES_PARENT_DIR.
# it gets LAAS subversion user name and tries to checkout the module from LAAS
# svn server in JAFAR_MODULES_PARENT_DIR.
#-------------------------------------------------------------------------------
macro(CHECKOUT_JAFAR_MODULE MODULE_TO_CHECK CHECKOUT_FAILED)
  string(TOLOWER ${MODULE_TO_CHECK} MODULE_TO_CHECK)
  if(Subversion_FOUND)
    message(STATUS "I need to checkout module: ${MODULE_TO_CHECK}, please wait...")
    # get subversion user name, at least try
    if($ENV{SVN_LAAS_USER})
      set(SVNUSER $ENV{SVN_LAAS_USER})
    else($ENV{SVN_LAAS_USER})
      set(SVNUSER $ENV{USER})
    endif($ENV{SVN_LAAS_USER})
    # checkout the module
    execute_process(
      COMMAND ${Subversion_SVN_EXECUTABLE} co svn+ssh://${SVNUSER}@svn.laas.fr/svn/jafar/jafarModules/trunk/${MODULE_TO_CHECK}
      WORKING_DIRECTORY ${JAFAR_MODULES_PARENT_DIR}
      ERROR_VARIABLE CHECKOUT_FAILED)
  else(Subversion_FOUND)
    message(FATAL_ERROR "Subversion wasn't found in your system.")
    message(FATAL_ERROR "\t${MODULE_TO_CHECK} couldn't be checked")
  endif(Subversion_FOUND)
endmacro(CHECKOUT_JAFAR_MODULE)


#-------------------------------------------------------------------------------
# This macro returns a library component to link with
#-------------------------------------------------------------------------------
macro(GET_COMPONENT_LIBRARY library component component_library)
	string(TOUPPER LIBS_MAP_${library} LIBRARY_KEY)
  string(LENGTH "${component}" component_length)
  string(LENGTH "${library}_" library_word_length)
  math(EXPR length "${component_length} - ${library_word_length}")
  string(SUBSTRING ${component} ${library_word_length} ${length} component)
  string(TOUPPER ${component} component)
  set(component_library_name "${${LIBRARY_KEY}}_${component}_LIBRARY")
  set(${component_library} ${${component_library_name}})
endmacro(GET_COMPONENT_LIBRARY)

#-------------------------------------------------------------------------------
# This macro returns external library exact name.
# Indeed, from CMakeList.txt the libraries name aren't exactly the same as in 
# CMakeLists.cmake. This macro finds the right name.
#-------------------------------------------------------------------------------
macro(GET_EXT_LIBRARY_EXACT_NAME LIB EXACT_NAME COMPONENT)
  # a slight workaround for boost libraries hence we treat them apart
  string(TOUPPER "${LIB}" EXT_LIB)
  if(EXT_LIB MATCHES "_" AND NOT ("${LIB}" STREQUAL "boost_sandbox"))
		string(REGEX REPLACE "([A-Z]*)_.*" "\\1" PREFIX "${EXT_LIB}")
		set(LIBS_MAP_PREFIX_COMPONENTS "LIBS_MAP_${PREFIX}_COMPONENTS")
		if(${LIBS_MAP_PREFIX_COMPONENTS})
			string(TOUPPER "LIBS_MAP_${PREFIX}" THIS_LIB_KEY)
			set(${EXACT_NAME} "${${THIS_LIB_KEY}}")
			string(REGEX REPLACE "[a-zA-Z]*_(.*)" "\\1" ${COMPONENT} "${LIB}")
		endif(${LIBS_MAP_PREFIX_COMPONENTS})
	else(EXT_LIB MATCHES "_" AND NOT ("${LIB}" STREQUAL "boost_sandbox"))
  # get exact library name from LIBS_MAP
  string(TOUPPER "LIBS_MAP_${EXT_LIB}" THIS_LIB_KEY)
  set(${EXACT_NAME} "${${THIS_LIB_KEY}}")
	set(${COMPONENT} "")
  endif(EXT_LIB MATCHES "_" AND NOT ("${LIB}" STREQUAL "boost_sandbox"))
endmacro(GET_EXT_LIBRARY_EXACT_NAME)

#-------------------------------------------------------------------------------
# This macro returns boost component library to link with
#-------------------------------------------------------------------------------
macro(GET_BOOST_COMPONENT_LIBRARY boost_component boost_library)
  string(LENGTH "${boost_component}" boost_component_length)
  string(LENGTH "boost_" boost_word_length)
  math(EXPR length "${boost_component_length} - ${boost_word_length}")
  string(SUBSTRING ${boost_component} ${boost_word_length} ${length} component)
  string(TOUPPER ${component} component)
  set(boost_library_name "Boost_${component}_LIBRARY")
  set(${boost_library} ${${boost_library_name}})
endmacro(GET_BOOST_COMPONENT_LIBRARY)

#-------------------------------------------------------------------------------
# This macro retrieves files given files mames and paths
#-------------------------------------------------------------------------------
function(RETRIEVE_FULL_FILES_NAMES PATHS NAMES FULL_FILE_NAMES)
  list(LENGTH ${NAMES} names_length)
#  message(STATUS "names_length ${names_length}")
  list(LENGTH ${PATHS} paths_length)
#  message(STATUS "paths_length ${paths_length}")
  if((${paths_length} EQUAL 1) AND (${names_length} EQUAL 1))
    set(FULL_FILE_NAMES "${${PATHS}}/lib${${NAMES}}.so")
#    message(STATUS "FULL_FILE_NAMES ${FULL_FILE_NAMES}")
  elseif((${paths_length} EQUAL 1) AND (${names_length} GREATER 1))
    foreach(name ${${NAMES}})
      if(EXISTS ${${PATHS}}/lib${name}.so)
	set(FULL_FILE_NAMES ${FULL_FILE_NAMES} ${${PATHS}}/lib${name}.so)
	list(REMOVE_ITEM ${NAMES} ${name})
      endif(EXISTS ${${PATHS}}/lib${name}.so)
    endforeach(name)
  else((${paths_length} EQUAL 1) AND (${names_length} EQUAL 1))
    foreach(path ${${PATHS}})
      foreach(name ${${NAMES}})
	if(EXISTS ${path}/lib${name}.so)
	  set(FULL_FILE_NAMES ${FULL_FILE_NAMES} ${path}/lib${name}.so)
	  list(REMOVE_ITEM ${NAMES} ${name})
	endif(EXISTS ${path}/lib${name}.so)
      endforeach(name)
    endforeach(path)
  endif((${paths_length} EQUAL 1) AND (${names_length} EQUAL 1))
endfunction(RETRIEVE_FULL_FILES_NAMES)

macro(FIND_MODULE_INTERNAL_LIBRARIES MODULE PATHS NAMES)
  string(TOUPPER "${MODULE}_internal_libraries" MODULE_INTERNAL_LIBRARIES)
  foreach(name ${${NAMES}})
    string(TOUPPER "internal_lib_${name}" INTERNAL_LIB_NAME)
    find_library(${INTERNAL_LIB_NAME} 
      NAMES ${name}
      PATHS ${${PATHS}})
    set(${MODULE_INTERNAL_LIBRARIES} ${${MODULE_INTERNAL_LIBRARIES}} ${${INTERNAL_LIB_NAME}})
  endforeach(name)
endmacro(FIND_MODULE_INTERNAL_LIBRARIES)

