# $Id$ #
# ----------------------------------------------------------------------------
#  Root CMake file for Jafar
#
#    From the off-tree build directory, invoke:
#      $ cmake <JAFAR_DIR>
#
# ----------------------------------------------------------------------------

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

# it MUST go before PROJECT(Jafar) in order to work
if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE INTERNAL "" FORCE)
endif()

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(Jafar CXX C)

#set(CMAKE_SKIP_RPATH 1)
#set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Byuilding type, choose either Debug or Release")
#set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE LIST)

enable_testing()

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
endif()

# ----------------------------------------------------------------------------
#  Current version number:
# ----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tools/cmake/)
include(JafarConfig)

set(INCLUDES_INSTALL_DIR 
    "${Jafar_BINARY_DIR}/include"
    CACHE PATH "Destination path under CMAKE_INSTALL_PREFIX for header files")

set(LIBRARIES_INSTALL_DIR
    "${Jafar_BINARY_DIR}/lib/${BUILDNAME}"
    CACHE PATH "Destination path under CMAKE_INSTALL_PREFIX for libraries")

set(JAFAR_MODULES_PARENT_DIR 
    "${Jafar_SOURCE_DIR}/modules"
    CACHE PATH "Where to find Jafar modules")
mark_as_advanced(JAFAR_MODULES_PARENT_DIR)

message(STATUS "Install include dir: " ${INCLUDES_INSTALL_DIR})
message(STATUS "Install lib dir: " ${LIBRARIES_INSTALL_DIR})
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARIES_INSTALL_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARIES_INSTALL_DIR})
# set(CMAKE_INCLUDE_OUTPUT_DIRECTORY ${INCLUDES_INSTALL_DIR})
# set(CMAKE_DOCUMENTATION_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/doc)

#------------------------------------------------------------------------------
# External dependencies: 
#------------------------------------------------------------------------------

include(CheckIncludeFile)

find_package(Boost 1.34 REQUIRED COMPONENTS date_time serialization unit_test_framework thread graph)
set(BOOST_FOUND ${Boost_FOUND})
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "boost libraries "${Boost_LIBRARIES})
set(LIBS ${LIBS} ${Boost_LIBRARIES})
set(HAVE_BOOST Boost_FOUND)
set(LIBS_MAP_BOOST "Boost")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
include_directories(${SWIG_INCLUDE_DIRS})
set(LIBS ${LIBS} ${SWIG_LIBRARIES})

##
# qt4
##
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtOpenGL QtSvg)
if(QT4_FOUND)
  include(${QT_USE_FILE})
  include_directories(${QT_INCLUDES})
#  message(STATUS "QT_LIBRARIES: ${QT_LIBRARIES}")
  set(LIBS ${LIBS} ${QT_LIBRARIES})
  set(LIBS_MAP_QT4 "QT")
endif(QT4_FOUND)

###
# opencv
###
include(FindOpenCV)
find_package(OpenCV)
set(HAVE_OPENCV OpenCV_FOUND)
set(OPENCV_FOUND ${OpenCV_FOUND})
if(OpenCV_FOUND)
  #this is a hack be cause the FindOpenCV.cmake provides duplicated values
  LIST(REMOVE_DUPLICATES OpenCV_INCLUDE_DIRS)
  LIST(REMOVE_DUPLICATES OpenCV_LIBRARIES)
  #end of hack
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenCV_LIBRARIES})
  set(LIBS_MAP_OPENCV "OpenCV")
endif(OpenCV_FOUND)

###
# openscenegraph
###
#find_package(OpenSceneGraph 2.0.0 REQUIRED osgViewer osgGA)
find_package(OpenSceneGraph 2.0.0)
set(HAVE_OSG OPENSCENEGRAPH_FOUND)
if(OPENSCENEGRAPH_FOUND)
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OPENSCENEGRAPH_LIBRARIES})
  set(LIBS_MAP_OPENSCENEGRAPH "OPENSCENEGRAPH")
endif(OPENSCENEGRAPH_FOUND)

###
# opengl
###
find_package(OpenGL)
set(HAVE_OPENGL OPENGL_FOUND)
if(OPENGL_FOUND)
  set(OPENGL_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
  include_directories(${OPENGL_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OPENGL_LIBRARIES})
  set(LIBS_MAP_OPENGL "OPENGL")
endif(OPENGL_FOUND)

###
# boost_sandbox
###
set(PATH_TO_BOOST_SANDBOX "" CACHE PATH "path to boost_sandbox")
if (EXISTS ${PATH_TO_BOOST_SANDBOX})
  if(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack/lapack.hpp) 
    include_directories(${PATH_TO_BOOST_SANDBOX})
    SET(HAVE_BOOST_SANDBOX TRUE)
  endif(EXISTS ${PATH_TO_BOOST_SANDBOX}/boost/numeric/bindings/lapack/lapack.hpp) 
endif(EXISTS ${PATH_TO_BOOST_SANDBOX})

###
# GDAL
###
find_package(GDAL)
set(HAVE_GDAL GDAL_FOUND)
if(GDAL_FOUND)
  set(GDAL_INCLUDE_DIRS ${GDAL_INCLUDE_DIR})
  include_directories(${GDAL_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${GDAL_LIBRARY})
  set(LIBS_MAP_GDAL "GDAL")
endif(GDAL_FOUND)

# include(FindCGAL)
# find_package(CGAL)
# include_directories(${OPENGL_INCLUDE_DIR})
# set(LIBS ${LIBS} ${OPENGL_LIBRARIES})
# check_include_file(GL/gl.h HAVE_OPENGL)

find_package(Subversion REQUIRED)

##
# CMake tests for lapack and blas requires fortran
##
enable_language(Fortran)
##
# lapack
##
find_package(LAPACK)
set(HAVE_LAPACK LAPACK_FOUND)
if(LAPACK_FOUND)
  set(LIBS ${LIBS} ${LAPACK_LIBRARIES})
  set(LIBS_MAP_LAPACK "LAPACK")
endif(LAPACK_FOUND)
##
# blas
##
find_package(BLAS)
if(BLAS_FOUND)
  set(LIBS ${LIBS} ${BLAS_LIBRARIES})
  set(LIBS_MAP_BLAS "BLAS")
endif(BLAS_FOUND)

#------------------------------------------------------------------------------
# Internal dependencies
#------------------------------------------------------------------------------

include(FindInternalLibrary)

foreach(internal
	dtm
    images3d
	p3d
    t3d
    gdhe
    stereopixel)
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


#------------------------------------------------------------------------------
# Scripting languages
#------------------------------------------------------------------------------

set(ENABLE_TCL ON CACHE OPTION "enable tcl scripting language")
if(ENABLE_TCL)
  find_package(TCL)
  if(TCL_FOUND)
    list(APPEND AVAILABLE_LANGUAGES tcl)
    include_directories(${TCL_INCLUDE_PATH})
  endif(TCL_FOUND)
else(ENABLE_TCL)
  set(TCL_FOUND OFF)
endif(ENABLE_TCL)

find_package(Ruby)
set(RUBY_FOUND FALSE)
if(EXISTS ${RUBY_INCLUDE_PATH} AND 
   EXISTS ${RUBY_EXECUTABLE} AND 
   EXISTS ${RUBY_LIBRARY})
  set(RUBY_FOUND ON)
  include_directories(${RUBY_INCLUDE_PATH})
endif(EXISTS ${RUBY_INCLUDE_PATH} AND 
      EXISTS ${RUBY_EXECUTABLE} AND 
      EXISTS ${RUBY_LIBRARY})

if(NOT TCL_FOUND AND NOT RUBY_FOUND)
  message(FATAL_ERROR "no scripting language available please install one")
endif(NOT TCL_FOUND AND NOT RUBY_FOUND)

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

configure_file(${CMAKE_SOURCE_DIR}/include/jafarConfig.h.in
               ${INCLUDES_INSTALL_DIR}/jafarConfig.h)

configure_file(${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile.in
               ${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile)

add_custom_target(doc
  ${DOXYGEN_EXECUTABLE} ${Jafar_SOURCE_DIR}/tools/doxygen/Doxyfile
)

add_custom_command(
  TARGET doc
  POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy ${Jafar_SOURCE_DIR}/doc/doxygen/images/LAASsmall.png ${Jafar_BINARY_DIR}/doc/html/LAASsmall.png
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy ${Jafar_SOURCE_DIR}/doc/doxygen/images/jafar.jpg ${Jafar_BINARY_DIR}/doc/html/jafar.jpg
)
