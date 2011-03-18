# $Id:$ #
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0007 OLD)
endif()
#-------------------------------------------------------------------------------
# List arguments are passed as strings so we need to split them
#-------------------------------------------------------------------------------
separate_arguments(THIS_MODULE_REQUIRED_MODULES UNIX_COMMAND 
  "${THIS_MODULE_REQUIRED_MODULES}")
separate_arguments(THIS_MODULE_OPTIONAL_MODULES UNIX_COMMAND 
  "${THIS_MODULE_OPTIONAL_MODULES}")
separate_arguments(THIS_MODULE_REQUIRED_EXTLIBS_EXACT_NAMES UNIX_COMMAND 
  "${THIS_MODULE_REQUIRED_EXTLIBS_EXACT_NAMES}")
separate_arguments(THIS_MODULE_OPTIONAL_EXTLIBS_EXACT_NAMES UNIX_COMMAND 
  "${THIS_MODULE_OPTIONAL_EXTLIBS_EXACT_NAMES}")
separate_arguments(THIS_MODULE_ROBOTPKG_REQ_MODULES UNIX_COMMAND "${THIS_MODULE_ROBOTPKG_REQ_MODULES}")
separate_arguments(THIS_MODULE_ROBOTPKG_OPT_MODULES UNIX_COMMAND "${THIS_MODULE_ROBOTPKG_OPT_MODULES}")
separate_arguments(THIS_MODULE_ROBOTPKG_REQ_EXTLIBS UNIX_COMMAND "${THIS_MODULE_ROBOTPKG_REQ_EXTLIBS}")
separate_arguments(THIS_MODULE_ROBOTPKG_OPT_EXTLIBS UNIX_COMMAND "${THIS_MODULE_ROBOTPKG_OPT_EXTLIBS}")

list(REMOVE_DUPLICATES THIS_MODULE_REQUIRED_EXTLIBS_EXACT_NAMES)
list(REMOVE_DUPLICATES THIS_MODULE_OPTIONAL_EXTLIBS_EXACT_NAMES)

#-------------------------------------------------------------------------------
# This macro retrieves today's date and output it works only on mac and linux
#-------------------------------------------------------------------------------
macro(get_current_date today)
  if(("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux") OR ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin"))
    find_program(date_program NAMES date)
    if(NOT("${date_program}" STREQUAL ""))
      execute_process(COMMAND ${date_program} "+\"%a %b %d %Y\""
	OUTPUT_VARIABLE ${today})
      #stupid but I don't wanna damn quotes
      string(REGEX REPLACE "\"" "" ${today} "${${today}}")
    endif(NOT("${date_program}" STREQUAL ""))
  endif(("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux") OR ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin"))
endmacro(get_current_date)

#-------------------------------------------------------------------------------
# This macro retrieves last module git tag
#-------------------------------------------------------------------------------
macro(get_last_tag last_tag)
	if(DEFINED GIT_EXECUTABLE)
		execute_process(
			COMMAND ${CMAKE_COMMAND} -E chdir ${THIS_MODULE_SOURCE_DIR} ${GIT_EXECUTABLE} "tag"
			OUTPUT_VARIABLE tags)
		string(REPLACE "\n" ";" tags "${tags}")
		list(LENGTH tags nb_tags)
		if(${nb_tags} GREATER 0)
			list(GET tags -1 ${last_tag})
		else()
			message(FATAL_ERROR "\tno available tags for this module!")
		endif()
	endif(DEFINED GIT_EXECUTABLE)
endmacro(get_last_tag)

#TODO: use git archive git archive --format=tar --prefix=kernel-0.1/ kernel-0.1

#get current date
get_current_date(OH_HAPPY_DAY)
#get last tag
get_last_tag(THIS_MODULE_LAST_TAG)
message(STATUS "I will release ${MODULENAME} identified tag ${THIS_MODULE_LAST_TAG}")
if(NOT EXISTS ${THIS_MODULE_BINARY_DIR}/package)
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${THIS_MODULE_BINARY_DIR}/package)
  else(NOT EXISTS ${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG})
	  execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG})
endif()

file(COPY ${Jafar_SOURCE_DIR}/modules/${MODULENAME}/
	DESTINATION ${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG}
	PATTERN "*/.svn" EXCLUDE
	PATTERN "*/.git" EXCLUDE
	PATTERN "*/.gitignore" EXCLUDE
	PATTERN "CMakeLists.txt" EXCLUDE
	#!!! nizar 20110207 : those are the symlinks we added
	PATTERN "code" EXCLUDE
	PATTERN "data" EXCLUDE
  #!!! nizar 20100702 : if nasty people do build in source then ignore what they did
  PATTERN "CMakeFiles" EXCLUDE
  PATTERN "cmake_install.cmake" EXCLUDE
  PATTERN "CTestTestfile.cmake" EXCLUDE
  PATTERN "lib" EXCLUDE
  PATTERN "macro" EXCLUDE
  PATTERN "Makefile" EXCLUDE
  PATTERN "objs" EXCLUDE
  PATTERN "User.make" EXCLUDE
  PATTERN "Testing" EXCLUDE)

##
# create the options list of this package
##

set(THIS_MODULE_ROBOTPKG_OPTIONS "")

list(LENGTH THIS_MODULE_OPTIONAL_MODULES NB_OPT_MODS)
if(${NB_OPT_MODS} GREATER 0)
  math(EXPR NB_OPT_MODS "${NB_OPT_MODS} - 1")
  foreach(counter RANGE ${NB_OPT_MODS})
    list (GET THIS_MODULE_OPTIONAL_MODULES ${counter} optmodule)
    list (GET THIS_MODULE_ROBOTPKG_OPT_MODULES ${counter} optcategory)
    string(TOUPPER "${optmodule}" OPTIONAL_MODULE)
    set(THIS_MODULE_ROBOTPKG_OPTIONS "${THIS_MODULE_ROBOTPKG_DEPENDENCIES}\n
PKG_SUPPORTED_OPTIONS+=		${optmodule}
PKG_OPTION_DESCR.${optmodule}=	Enable jafar module ${optmodule} support.
define PKG_OPTION_SET.${optmodule}
  CMAKE_ARGS+=			-DHAVE_MODULE_${OPTIONAL_MODULE}=ON
  include ../../${optcategory}/depend.mk
endef
define PKG_OPTION_UNSET.${optmodule}
  CMAKE_ARGS+=			-DHAVE_MODULE_${OPTIONAL_MODULE}=OFF
endef")
  endforeach(counter)
endif(${NB_OPT_MODS} GREATER 0)

#set options corresponding to optional external libraries
list(LENGTH THIS_MODULE_OPTIONAL_EXTLIBS_EXACT_NAMES NB_OPT_LIBS)
if(${NB_OPT_LIBS} GREATER 0)
  math(EXPR NB_OPT_LIBS "${NB_OPT_LIBS} - 1")
  foreach(counter RANGE ${NB_OPT_LIBS})
    list(GET THIS_MODULE_OPTIONAL_EXTLIBS_EXACT_NAMES ${counter} optlib)
    list(GET THIS_MODULE_ROBOTPKG_OPT_EXTLIBS ${counter} optcategory)
    string(TOUPPER "${optlib}" OPTLIB)
    set(THIS_MODULE_ROBOTPKG_OPTIONS "${THIS_MODULE_ROBOTPKG_DEPENDENCIES}\n
PKG_SUPPORTED_OPTIONS+=		${optlib}
PKG_OPTION_DESCR.${optlib}=	Enable ${optlib} support.
define PKG_OPTION_SET.${optlib}
  CMAKE_ARGS+=			-DHAVE_${OPTLIB}=ON
  include ../../${optcategory}/depend.mk
endef
define PKG_OPTION_UNSET.${optlib}
  CMAKE_ARGS+=			-DHAVE_${OPTLIB}=OFF
endef")
  endforeach(counter)
endif(${NB_OPT_LIBS} GREATER 0)

##
#create the dependencies list of this package
##
set(THIS_MODULE_ROBOTPKG_DEPENDENCIES "")
set(THIS_MODULE_REQUIRED_MODULES_LINKAGE "")

#add required jafar modules depend.mk
list(LENGTH THIS_MODULE_REQUIRED_MODULES NB_REQ_MODS)
if(${NB_REQ_MODS} GREATER 0)
  math(EXPR NB_REQ_MODS "${NB_REQ_MODS} - 1")
  foreach(counter RANGE ${NB_REQ_MODS})
    list (GET THIS_MODULE_REQUIRED_MODULES ${counter} reqmodule)
    list (GET THIS_MODULE_ROBOTPKG_REQ_MODULES ${counter} reqcategory)
    set(THIS_MODULE_ROBOTPKG_DEPENDENCIES "${THIS_MODULE_ROBOTPKG_DEPENDENCIES}
include ../../${reqcategory}/depend.mk")
    set(THIS_MODULE_REQUIRED_MODULES_LINKAGE "${THIS_MODULE_REQUIRED_MODULES_LINKAGE}
include(\${CMAKE_INSTALL_PREFIX}/share/cmake/jafar/${reqmodule}.cmake)
target_link_libraries(jafar-${MODULENAME} jafar-${reqmodule})
message(STATUS \"--> linking ${MODULENAME} to ${reqmodule}\")
include_directories(\${JAFAR_${reqmodule}_IMPORTED_HEADERS})
message(STATUS \"--> including headers from \${JAFAR_${reqmodule}_IMPORTED_HEADERS}\")")
  endforeach(counter)
	string(REPLACE ";" ",jafar-" THIS_MODULE_REQUIRED_MODULES "jafar-${THIS_MODULE_REQUIRED_MODULES}")
endif(${NB_REQ_MODS} GREATER 0)
	


#add required external libraries depend.mk
foreach(extlib ${THIS_MODULE_ROBOTPKG_REQ_EXTLIBS})
  set(THIS_MODULE_ROBOTPKG_DEPENDENCIES "${THIS_MODULE_ROBOTPKG_DEPENDENCIES}\ninclude ../../${extlib}/depend.mk")
endforeach(extlib)

#add required external libraries finders
foreach(extlib ${THIS_MODULE_REQUIRED_EXTLIBS_EXACT_NAMES})
  string(TOUPPER "${extlib}" EXTLIB)
  if(EXISTS ${Jafar_SOURCE_DIR}/tools/cmake/Find${extlib}.cmake)
    file(COPY ${Jafar_SOURCE_DIR}/tools/cmake/Find${extlib}.cmake 
		DESTINATION ${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG})
    if("${extlib}" STREQUAL Qt4)
      set(THIS_MODULE_REQUIRES "${THIS_MODULE_REQUIRES}
#-----------------------------------------------------------------------------
# Check for Qt4
#-----------------------------------------------------------------------------
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtOpenGL QtSvg)
if(QT4_FOUND)
  include(\${QT_USE_FILE})
  include_directories(\${QT_INCLUDES})
  list(APPEND IMPORTED_HEADERS \${QT_INCLUDES})
  set(LIBS \${LIBS} \${QT_LIBRARIES})
  set(Qt4_LIBS \${QT_LIBRARIES})
endif(QT4_FOUND)\n")
    else("${extlib}" STREQUAL Qt4)
      set(THIS_MODULE_REQUIRES "${THIS_MODULE_REQUIRES}
#-----------------------------------------------------------------------------
# Check for ${extlib}
#-----------------------------------------------------------------------------
include(Find${extlib}.cmake)
find_package(${extlib} REQUIRED)
if(${extlib}_FOUND)
  set(EXTRA_COMPILE_FLAGS \"\${EXTRA_COMPILE_FLAGS} -DHAVE_${EXTLIB}\")
  if(NOT ${extlib}_INCLUDE_DIRS)
    set(${extlib}_INCLUDE_DIRS \${${extlib}_INCLUDE_DIR})
  endif(NOT ${extlib}_INCLUDE_DIRS)
  if(NOT ${extlib}_LIBS)
    set(${extlib}_LIBS \${${extlib}_LIB})
  endif(NOT ${extlib}_LIBS)
  include_directories(\${${extlib}_INCLUDE_DIRS})
  list(APPEND IMPORTED_HEADERS \${${extlib}_INCLUDE_DIRS})
  set(LIBS \${LIBS} \${${extlib}_LIBRARIES})
endif(${extlib}_FOUND)\n")
    endif("${extlib}" STREQUAL Qt4)
    set(THIS_MODULE_EXTLIBS_LINKAGE "${THIS_MODULE_EXTLIBS_LINKAGE}
target_link_libraries(jafar-${MODULENAME} \${${extlib}_LIBRARIES})
message(STATUS \"--> linking ${MODULENAME} to ${extlib} libraries\")")
  else(EXISTS ${Jafar_SOURCE_DIR}/tools/cmake/Find${extlib}.cmake)
    set(ROBOTS_PACKAGES_TO_FIND "${ROBOTS_PACKAGES_TO_FIND} ${extlib}")
  endif(EXISTS ${Jafar_SOURCE_DIR}/tools/cmake/Find${extlib}.cmake)
endforeach(extlib)

configure_file(${Jafar_SOURCE_DIR}/share/template_package/Makefile.in
  ${THIS_MODULE_BINARY_DIR}/package/Makefile @ONLY)

configure_file(${Jafar_SOURCE_DIR}/share/template_package/depend.mk.in
  ${THIS_MODULE_BINARY_DIR}/package/depend.mk @ONLY)

configure_file(${Jafar_SOURCE_DIR}/share/template_package/DESCR.in
  ${THIS_MODULE_BINARY_DIR}/package/DESCR @ONLY)

configure_file(${Jafar_SOURCE_DIR}/share/template_package/CMakeLists.txt.in
	${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG}/CMakeLists.txt @ONLY)

configure_file(${Jafar_SOURCE_DIR}/share/template_package/jafar-module.pc.in
	${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG}/jafar-${MODULENAME}.pc.in @ONLY)

execute_process(
	COMMAND ${CMAKE_COMMAND} -E tar czf jafar-${THIS_MODULE_LAST_TAG}.tar.gz jafar-${THIS_MODULE_LAST_TAG}
  WORKING_DIRECTORY ${THIS_MODULE_BINARY_DIR}/package
  OUTPUT_FILE ${THIS_MODULE_BINARY_DIR}/package/jafar-${THIS_MODULE_LAST_TAG}.tar.gz)

execute_process(COMMAND ${CMAKE_COMMAND} -E md5sum jafar-${THIS_MODULE_LAST_TAG}.tar.gz
  OUTPUT_FILE ${THIS_MODULE_BINARY_DIR}/package/distinfo
  WORKING_DIRECTORY ${THIS_MODULE_BINARY_DIR}/package)
