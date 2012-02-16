# $Id$ #
# ----------------------------------------------------------------------------
#  Root CMake file for Jafar
#
#    From the off-tree build directory, invoke:
#      $ cmake <JAFAR_DIR>
#
# ----------------------------------------------------------------------------

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 0)

# it MUST go before PROJECT(Jafar) in order to work
if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE INTERNAL "" FORCE)
endif()

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(Jafar CXX C)

#-----------------------------------------------------------------------------
# Compilation flags for release and debug
#-----------------------------------------------------------------------------
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;FastDebug" CACHE STRING "possible configurations" FORCE)
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Debug
if (${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Debug)
    endif()
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -march=native -g0 -DNDEBUG -DJFR_NDEBUG -DBOOST_UBLAS_NDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -ggdb")
set(CMAKE_CXX_FLAGS_FASTDEBUG "-O1 -g3 -ggdb -DNDEBUG -DJFR_NDEBUG -DBOOST_UBLAS_NDEBUG")
#set(CMAKE_CXX_FLAGS_FASTDEBUG "-O1 -g3 -ggdb -pg -DNDEBUG -DJFR_NDEBUG -DBOOST_UBLAS_NDEBUG")
#set(JAFAR_COMPILE_FLAGS_MODULE "-pedantic -pedantic-errors -Werror -Wno-long-long -Wall")
set(JAFAR_COMPILE_FLAGS_MODULE "-pedantic -Wno-long-long -Wall")

#-----------------------------------------------------------------------------
# enable test to generate test and run them
#-----------------------------------------------------------------------------
enable_testing()

#-----------------------------------------------------------------------------
# Microsoft Visual Studio special
#-----------------------------------------------------------------------------
if(MSVC)
  set(CMAKE_USE_RELATIVE_PATHS ON CACHE INTERNAL "" FORCE)
endif(MSVC)

# ----------------------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# ----------------------------------------------------------------------------
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0007 OLD)
endif()

# ----------------------------------------------------------------------------
#  Current version number:
# ----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tools/cmake/)
include(JafarConfig)

set(INCLUDES_INSTALL_DIR 
    "${Jafar_SOURCE_DIR}/include"
    CACHE PATH "Destination path under CMAKE_INSTALL_PREFIX for header files")

set(LIBRARIES_INSTALL_DIR
    "${Jafar_SOURCE_DIR}/lib/${BUILDNAME}"
    CACHE PATH "Destination path under CMAKE_INSTALL_PREFIX for libraries")

set(INCLUDES_OUTPUT_DIR 
    "${Jafar_BINARY_DIR}/include"
    CACHE PATH "Destination path under CMAKE_OUTPUT_PREFIX for header files")

set(LIBRARIES_OUTPUT_DIR
    "${Jafar_BINARY_DIR}/lib/${BUILDNAME}"
    CACHE PATH "Destination path under CMAKE_OUTPUT_PREFIX for libraries")

set(JAFAR_MODULES_PARENT_DIR 
    "${Jafar_SOURCE_DIR}/modules"
    CACHE PATH "Where to find Jafar modules")
mark_as_advanced(JAFAR_MODULES_PARENT_DIR)

message(STATUS "Install include dir: " ${INCLUDES_OUTPUT_DIR})
message(STATUS "Install lib dir: " ${LIBRARIES_OUTPUT_DIR})
message(STATUS "Build include dir: " ${INCLUDES_OUTPUT_DIR})
message(STATUS "Build lib dir: " ${LIBRARIES_OUTPUT_DIR})

if(NOT EXISTS ${LIBRARIES_OUTPUT_DIR})
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARIES_OUTPUT_DIR})
endif(NOT EXISTS ${LIBRARIES_OUTPUT_DIR})

if(NOT EXISTS ${INCLUDES_OUTPUT_DIR})
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${INCLUDES_OUTPUT_DIR})
endif(NOT EXISTS ${INCLUDES_OUTPUT_DIR})

if(NOT EXISTS ${LIBRARIES_INSTALL_DIR})
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARIES_INSTALL_DIR})
endif(NOT EXISTS ${LIBRARIES_INSTALL_DIR})

if(NOT EXISTS ${INCLUDES_INSTALL_DIR})
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${INCLUDES_INSTALL_DIR})
endif(NOT EXISTS ${INCLUDES_INSTALL_DIR})

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARIES_OUTPUT_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARIES_OUTPUT_DIR})
# set(CMAKE_INCLUDE_OUTPUT_DIRECTORY ${INCLUDES_OUTPUT_DIR})
# set(CMAKE_DOCUMENTATION_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/doc)

#------------------------------------------------------------------------------
# External dependencies: 
#------------------------------------------------------------------------------
include(CheckIncludeFile)

###
# Boost v1.40 or above  -- REQUIRED
###
find_package(Boost 1.40 REQUIRED COMPONENTS date_time serialization unit_test_framework thread graph system filesystem regex)
set(BOOST_FOUND ${Boost_FOUND})
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "boost libraries "${Boost_LIBRARIES})
set(LIBS ${LIBS} ${Boost_LIBRARIES})
set(HAVE_BOOST ${Boost_FOUND})
set(LIBS_MAP_BOOST "Boost")
set(LIBS_MAP_BOOST_COMPONENTS date_time serialization unit_test_framework thread graph system filesystem regex)
set(ROBOTPKG_MAP_BOOST "devel/boost-headers" "devel/boost-libs")

###
# SWIG -- REQUIRED
###
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
include_directories(${SWIG_INCLUDE_DIRS})
set(LIBS ${LIBS} ${SWIG_LIBRARIES})

##
# Qt4
##
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtOpenGL QtSvg)
if(QT4_FOUND)
  include(${QT_USE_FILE})
  include_directories(${QT_INCLUDES})
  set(LIBS ${LIBS} ${QT_LIBRARIES})
  set(LIBS_MAP_QT4 "Qt4")
	set(LIBS_MAP_QT4_COMPONENTS QtCore QtGui QtXml QtOpenGL QtSvg)
endif(QT4_FOUND)
message(STATUS "QT_LIBRARIES ${QT_LIBRARIES}")

###
# PkgConfig
###
include(FindPkgConfig)

###
# OpenCV
###
#include(FindOpenCV)
#find_package(OpenCV)
pkg_check_modules(OpenCV opencv)
set(HAVE_OPENCV ${OpenCV_FOUND})
set(OPENCV_FOUND ${OpenCV_FOUND})
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  set(LIBS ${LIBS} ${OpenCV_LIBRARIES})
	message(STATUS "version ${OpenCV_VERSION}")
  set(LIBS_MAP_OPENCV "OpenCV")
endif(OpenCV_FOUND)
set(ROBOTPKG_MAP_OPENCV "image/opencv")

###
# OpenSceneGraph
###
find_package(OpenSceneGraph 2.0.0 REQUIRED osgViewer osgGA osgDB)
#find_package(OpenSceneGraph 2.0.0)
set(HAVE_OSG ${OPENSCENEGRAPH_FOUND})
if(OPENSCENEGRAPH_FOUND)
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OPENSCENEGRAPH_LIBRARIES})
  set(LIBS_MAP_OPENSCENEGRAPH "OPENSCENEGRAPH")
	set(LIBS_MAP_OPENSCENEGRAPH_COMPONENTS osgViewer osgGA osgDB)
endif(OPENSCENEGRAPH_FOUND)

###
# OpenGL
###
find_package(OpenGL)
set(HAVE_OPENGL ${OPENGL_FOUND})
if(OPENGL_FOUND)
  set(OPENGL_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
  include_directories(${OPENGL_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OPENGL_LIBRARIES})
  set(LIBS_MAP_OPENGL "OPENGL")
endif(OPENGL_FOUND)

###
# boost-sandbox
###
# set(PATH_TO_BOOST_SANDBOX "" CACHE PATH "path to boost_sandbox")
# if (EXISTS ${PATH_TO_BOOST_SANDBOX})
#   if(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack/lapack.hpp) 
#     include_directories(${PATH_TO_BOOST_SANDBOX})
#     SET(HAVE_BOOST_SANDBOX TRUE)
#   endif(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack/lapack.hpp) 
# endif(EXISTS ${PATH_TO_BOOST_SANDBOX})
# set(ROBOTPKG_MAP_BOOST_SANDBOX "devel/boost-numeric-bindings-v1")

set(PATH_TO_BOOST_SANDBOX "" CACHE PATH "path to boost_sandbox")
if (EXISTS ${PATH_TO_BOOST_SANDBOX})
  if(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack.hpp) 
    include_directories(${PATH_TO_BOOST_SANDBOX})
    set(HAVE_BOOST_SANDBOX TRUE)
		set(BOOST_SANDBOX_FOUND TRUE)
  endif(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack.hpp) 
endif(EXISTS ${PATH_TO_BOOST_SANDBOX})
set(ROBOTPKG_MAP_BOOST_SANDBOX "devel/boost-numeric-bindings")

###
# GDAL
###
find_package(GDAL)
set(HAVE_GDAL ${GDAL_FOUND})
if(GDAL_FOUND)
	set(GDAL_INCLUDE_DIRS ${GDAL_INCLUDE_DIR})
	include_directories(${GDAL_INCLUDE_DIRS})
	set(LIBS ${LIBS} ${GDAL_LIBRARY})
	set(LIBS_MAP_GDAL "GDAL")
endif(GDAL_FOUND)

###
# MTI
###
option(ENABLE_MTI "enable use of MTI" ON)
if(ENABLE_MTI)
	find_package(MTI)
	set(HAVE_MTI ${MTI_FOUND})
	if(MTI_FOUND)
		include_directories(${MTI_INCLUDE_DIRS})
		set(LIBS ${LIBS} ${MTI_LIBRARIES})
		set(LIBS_MAP_MTI "MTI")
	endif(MTI_FOUND)
	set(ROBOTPKG_MAP_MTI "hardware/MTI")
else(ENABLE_MTI)
	set(HAVE_MTI FALSE)
endif(ENABLE_MTI)

###
# GIT to play around
###
find_package(Git REQUIRED)

##
# CMake tests for lapack and blas requires fortran
##
enable_language(Fortran)
##
# LAPACK
##
find_package(LAPACK)
set(HAVE_LAPACK ${LAPACK_FOUND})
if(LAPACK_FOUND)
  set(LIBS ${LIBS} ${LAPACK_LIBRARIES})
  set(LIBS_MAP_LAPACK "LAPACK")
endif(LAPACK_FOUND)
set(ROBOTPKG_MAP_LAPACK "math/lapack")

##
# BLAS
##
find_package(BLAS)
if(BLAS_FOUND)
  set(LIBS ${LIBS} ${BLAS_LIBRARIES})
  set(LIBS_MAP_BLAS "BLAS")
endif(BLAS_FOUND)
set(ROBOTPKG_MAP_BLAS "math/blas")

###
# vcglib
###
set(PATH_TO_VCG "" CACHE PATH "path to vcg")
if (EXISTS ${PATH_TO_VCG})
    include_directories(${PATH_TO_VCG})
    SET(HAVE_VCG TRUE)
endif(EXISTS ${PATH_TO_VCG})

###
# gsl #!!! nizar 20101116 : to remove as soon as possible
###
find_package(GSL)
if(GSL_FOUND)
	include_directories(${GSL_INCLUDE_DIR})
	string(STRIP "${GSL_LIBRARIES}" GSL_LIBRARIES)
	string(REPLACE " " ";" GSL_LIBRARIES "${GSL_LIBRARIES}")
	message(STATUS "gsl libraries: ${GSL_LIBRARIES}")
  set(LIBS ${LIBS} ${GSL_LIBRARIES})
  set(LIBS_MAP_GSL "GSL")
endif(GSL_FOUND)
set(ROBOTPKG_MAP_GSL "math/gsl")

###
# OpenNI, Nite and Sensor
###
include(FindOpenNIX)
if(OPENNIX_FOUND)
	include_directories(${OPENNIX_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OPENNIX_LIBRARIES})
	set(LIBS_MAP_OPENNIX "OPENNIX")
endif(OPENNIX_FOUND)
set(ROBOTPKG_MAP_OPENNIX "wip/openni")

###
# UEYE
###
option(ENABLE_UEYE "enable use of UEYE" ON)
if(ENABLE_UEYE)
	find_package(UEYE)
	set(HAVE_UEYE ${UEYE_FOUND})
	if(UEYE_FOUND)
		include_directories(${UEYE_INCLUDE_DIR})
		set(LIBS ${LIBS} ${UEYE_LIBRARIES})
		set(LIBS_MAP_UEYE "UEYE")
		MESSAGE(STATUS "Found Ueye " ${LIBS})
	endif(UEYE_FOUND)
else(ENABLE_UEYE)
	set(HAVE_UEYE FALSE)
endif(ENABLE_UEYE)


#------------------------------------------------------------------------------
# pkgconfig findable
#------------------------------------------------------------------------------

include(FindInternalLibrary)
if(PKG_CONFIG_FOUND)
	foreach(package
			flann
			libfreenect)
		string(TOUPPER "${package}" PACKAGE)
		string(TOUPPER "HAVE_${package}" HAVE_PACKAGE)
		string(TOUPPER "${package}_FOUND" PACKAGE_FOUND)
		find_this_package(${package})
		set(${HAVE_PACKAGE} ${${PACKAGE_FOUND}})
		if(${PACKAGE_FOUND})
			string(TOUPPER "LIBS_MAP_${package}" LIBS_MAP_PACKAGE)
			set(${LIBS_MAP_PACKAGE} "${PACKAGE}")
			set(LIBS ${LIBS} ${${PACKAGE}_LIBRARIES})
			include_directories(${${PACKAGE}_INCLUDE_DIRS})		
			link_directories(${${PACKAGE}_LIBRARY_DIRS})
		endif(${PACKAGE_FOUND})
	endforeach(package)
else(PKG_CONFIG_FOUND)
  message(ERROR " pkgconfig not found you won't be able to build some modules")
endif(PKG_CONFIG_FOUND)
set(ROBOTPKG_MAP_FLANN "wip/flann")

#------------------------------------------------------------------------------
# Internal dependencies
#------------------------------------------------------------------------------
foreach(internal
	dtm
	images3d
	p3d
	t3d
	gdhe
	stereopixel
	sift
	viam
	lbfgs
	ANN
	posterLib)
  find_internal_library("${internal}")
  string(TOUPPER "HAVE_${internal}" HAVE_INTERNAL)
  string(TOUPPER "${internal}_FOUND" INTERNAL_FOUND)
  set(${HAVE_INTERNAL} ${${INTERNAL_FOUND}})
  string(TOUPPER "LIBS_MAP_${internal}" LIBS_MAP_INTERNAL)
  string(TOUPPER "${internal}" INTERNAL)
  set(${LIBS_MAP_INTERNAL} "${INTERNAL}")
  if(${INTERNAL_FOUND})
    set(LIBS ${LIBS} ${${INTERNAL}_LIBRARIES})
    include_directories(${${INTERNAL}_INCLUDE_DIRS})
  endif(${INTERNAL_FOUND})
endforeach(internal)

set(ROBOTPKG_MAP_DTM "mapping/libdtm")
set(ROBOTPKG_MAP_IMAGES3D "image/libimages3d")
set(ROBOTPKG_MAP_P3D "path/libp3d")
set(ROBOTPKG_MAP_T3D "math/libt3d")
set(ROBOTPKG_MAP_GDHE "graphics/gdhe")
set(ROBOTPKG_MAP_STEREOPIXEL "mapping/libstereopixel")
set(ROBOTPKG_MAP_SIFT "image/sift")
set(ROBOTPKG_MAP_VIAM "image/viam-libs")
set(ROBOTPKG_MAP_VCGLIB "graphics/vcglib")
set(ROBOTPKG_MAP_LBFGS "math/liblbfgs")
set(ROBOTPKG_MAP_ANN "wip/libANN")

#------------------------------------------------------------------------------
# Scripting languages
#------------------------------------------------------------------------------

option(ENABLE_TCL "enable tcl scripting language" ON)
if(ENABLE_TCL)
  find_package(TCL)
  if(TCL_FOUND)
    list(APPEND AVAILABLE_LANGUAGES tcl)
    include_directories(${TCL_INCLUDE_PATH})
   set(TCLZ_OUTPUT_DIR "${Jafar_BINARY_DIR}/tclpkg/${BUILDNAME}"
     CACHE PATH "Destination for tcl wrapped libraries when built")
   mark_as_advanced(TCLZ_OUTPUT_DIR)
   if(NOT EXISTS ${TCLZ_OUTPUT_DIR})
     execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${TCLZ_OUTPUT_DIR})
   endif(NOT EXISTS ${TCLZ_OUTPUT_DIR})
   set(TCLZ_INSTALL_DIR "${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}"
     CACHE PATH "Destination for tcl wrapped libraries when installed")
   mark_as_advanced(TCLZ_INSTALL_DIR)
   if(NOT EXISTS ${TCLZ_INSTALL_DIR})
     execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${TCLZ_INSTALL_DIR})
   endif(NOT EXISTS ${TCLZ_INSTALL_DIR}) 
 endif(TCL_FOUND)
else(ENABLE_TCL)
  set(TCL_FOUND FALSE)
endif(ENABLE_TCL)

if(NOT TCL_FOUND AND ENABLE_TCL )
  message(FATAL_ERROR "tcl is required as scripting language but is not available")
endif(NOT TCL_FOUND AND ENABLE_TCL)

option(ENABLE_RUBY "enable ruby scripting language" ON)
if(ENABLE_RUBY)
  find_package(Ruby)
  set(RUBY_FOUND FALSE)
  if(EXISTS ${RUBY_INCLUDE_PATH} AND 
     EXISTS ${RUBY_EXECUTABLE} AND 
     EXISTS ${RUBY_LIBRARY})
   set(RUBY_FOUND ON)
   include_directories(${RUBY_INCLUDE_PATH})
   exec_program(${RUBY_EXECUTABLE}
     ARGS --version
     OUTPUT_VARIABLE RUBY_VERSION)
   string(REGEX MATCH "[0-9]+\\.[0-9]+" RUBY_VERSION "${RUBY_VERSION}")
   set(RUBIES_OUTPUT_DIR "${LIBRARIES_OUTPUT_DIR}/ruby/${RUBY_VERSION}/jafar"
     CACHE PATH "Destination for ruby wrapped libraries when built")
   mark_as_advanced(RUBIES_OUTPUT_DIR)
   if(NOT EXISTS ${RUBIES_OUTPUT_DIR})
     execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${RUBIES_OUTPUT_DIR})
   endif(NOT EXISTS ${RUBIES_OUTPUT_DIR})
   set(RUBIES_INSTALL_DIR "${LIBRARIES_INSTALL_DIR}/ruby/${RUBY_VERSION}/jafar"
     CACHE PATH "Destination for ruby wrapped libraries when installed")
   mark_as_advanced(RUBIES_INSTALL_DIR)
   if(NOT EXISTS ${RUBIES_INSTALL_DIR})
     execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${RUBIES_INSTALL_DIR})
   endif(NOT EXISTS ${RUBIES_INSTALL_DIR})    
	 configure_file(${CMAKE_SOURCE_DIR}/share/jafar.rb.in
              		${CMAKE_SOURCE_DIR}/share/jafar.rb)
 endif(EXISTS ${RUBY_INCLUDE_PATH} AND 
   EXISTS ${RUBY_EXECUTABLE} AND 
   EXISTS ${RUBY_LIBRARY})
else(ENABLE_RUBY)
  set(RUBY_FOUND FALSE)
endif(ENABLE_RUBY)

if(NOT RUBY_FOUND AND ENABLE_RUBY )
  message(FATAL_ERROR "ruby is required as scripting language but is not available")
endif(NOT RUBY_FOUND AND ENABLE_RUBY)

#------------------------------------------------------------------------------
# configure jafarConfig.h header:
#------------------------------------------------------------------------------

configure_file(${CMAKE_SOURCE_DIR}/include/jafarConfig.h.in
               ${INCLUDES_OUTPUT_DIR}/jafarConfig.h)
install(FILES ${INCLUDES_OUTPUT_DIR}/jafarConfig.h DESTINATION ${INCLUDES_INSTALL_DIR})

#------------------------------------------------------------------------------
# Jafar CMake modules:
#------------------------------------------------------------------------------

#include(JafarUtils.cmake)
SET(BUILD_MODULARIZED ON CACHE OPTION "build each module from its location, default is on")
if (NOT BUILD_MODULARIZED)
  add_subdirectory(doc)
  add_subdirectory(modules)
endif(NOT BUILD_MODULARIZED)

include(JafarModule)
add_subdirectory(modules)

#------------------------------------------------------------------------------
# Jafar documentation
#------------------------------------------------------------------------------
find_package(Doxygen)

if(DOXYGEN_FOUND)
  configure_file(${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile.in
    ${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile)
  
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile)

  add_custom_command(
    TARGET doc
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy_if_different ${Jafar_SOURCE_DIR}/doc/doxygen/images/LAASsmall.png ${Jafar_SOURCE_DIR}/doc/html/LAASsmall.png
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy_if_different ${Jafar_SOURCE_DIR}/doc/doxygen/images/jafar.jpg ${Jafar_SOURCE_DIR}/doc/html/jafar.jpg
  )

  foreach(module ${JAFAR_MODULES_DIRS})
	  add_custom_command(
	    TARGET doc
	    POST_BUILD
	    COMMAND ${CMAKE_COMMAND}
	    ARGS -E copy_directory ${JAFAR_MODULES_PARENT_DIR}/${module}/doc/images ${Jafar_SOURCE_DIR}/doc/html/
	  )
  endforeach(module)
endif(DOXYGEN_FOUND)
