# $Id$ #

include(JafarUtils)

#------------------------------------------------------------------------------
#This macro generates ruby wrapping for a jafar module
#------------------------------------------------------------------------------
macro(WRAP_JAFAR_MODULE_TO_RUBY jafar_modulename)
  set(SWIGRUBY 1)

  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ruby)
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/src/ruby)
  endif()

  foreach(inc ${THIS_PROJECT_INCLUDES_ALL_LIST})
    set(INTERNAL_INCLUDES ${INTERNAL_INCLUDES} -I${JAFAR_MODULES_PARENT_DIR}/${inc}/include)
  endforeach(inc)
  set(INTERNAL_INCLUDES ${INTERNAL_INCLUDES} -I${INCLUDES_INSTALL_DIR})
  add_custom_command(
    OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/src/ruby/${jafar_modulename}_wrap.cpp
    COMMAND ${SWIG_EXECUTABLE} 
    ARGS -c++ -Wall -v -ruby -prefix "jafar::" ${INTERNAL_INCLUDES} -I${Boost_INCLUDE_DIRS} -I${Jafar_SOURCE_DIR}/tools/swig -I${RUBY_INCLUDE_PATH} -I${CMAKE_CURRENT_SOURCE_DIR}/include "-D_JFR_MODULE_=\\\"${jafar_modulename}\\\"" -o src/ruby/${jafar_modulename}_wrap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${jafar_modulename}.i
    MAIN_DEPENDENCY include/${jafar_modulename}.i
    DEPENDS include/${jafar_modulename}Tools.i include/${jafar_modulename}Exception.i
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   )
  add_library(${jafar_modulename}_ruby_wrap SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/ruby/${jafar_modulename}_wrap.cpp)
  target_link_libraries(${jafar_modulename}_ruby_wrap ${jafar_modulename} stdc++ ${RUBY_LIBRARY})
  set_target_properties(${jafar_modulename}_ruby_wrap 
    PROPERTIES OUTPUT_NAME ${jafar_modulename} 
               LIBRARY_OUTPUT_DIRECTORY ${LIBRARIES_OUTPUT_DIR}/ruby/${RUBY_VERSION}/jafar/${jafar_modulename}
	       LINK_FLAGS "${THIS_MODULE_LDFLAGS}")
  file(GLOB ruby_macros ${CMAKE_CURRENT_SOURCE_DIR}/macro/*.rb)
  foreach(macro ${ruby_macros})
    get_filename_component(MACRO_NAME ${macro} NAME_WE)
    if(${MACRO_NAME} STREQUAL ${jafar_modulename})
      add_custom_command(
	TARGET ${jafar_modulename}_ruby_wrap
	POST_BUILD
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy_if_different ${macro} ${LIBRARIES_OUTPUT_DIR}/ruby/${RUBY_VERSION}/jafar/${MACRO_NAME}.rb
	)
    else(${MACRO_NAME} STREQUAL ${jafar_modulename})
      add_custom_command(
	TARGET ${jafar_modulename}_ruby_wrap
	POST_BUILD
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy_if_different ${macro} ${LIBRARIES_OUTPUT_DIR}/ruby/${RUBY_VERSION}/jafar/${jafar_modulename}/${MACRO_NAME}.rb
	)
    endif(${MACRO_NAME} STREQUAL ${jafar_modulename})
  endforeach(macro)

# install instructions for ruby library and macros
  install(TARGETS ${jafar_modulename}_ruby_wrap DESTINATION ${LIBRARIES_INSTALL_DIR}/ruby/${RUBY_VERSION}/jafar/${jafar_modulename})
  file(GLOB ruby_macros macro/*.rb)
  foreach(macro ${ruby_macros})
    get_filename_component(MACRO_NAME ${macro} NAME_WE)
    if(${MACRO_NAME} STREQUAL ${jafar_modulename})
      install(FILES ${macro} DESTINATION ${LIBRARIES_INSTALL_DIR}/ruby/${RUBY_VERSION}/jafar)
    else(${MACRO_NAME} STREQUAL ${jafar_modulename})
      install(FILES ${macro} DESTINATION ${LIBRARIES_INSTALL_DIR}/ruby/${RUBY_VERSION}/jafar/${jafar_modulename})
    endif(${MACRO_NAME} STREQUAL ${jafar_modulename})
  endforeach(macro ${ruby_macros})

  # include(${SWIG_USE_FILE})
  # include_directories(${RUBY_INCLUDE_PATH})
  # include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
  # include_directories(${Jafar_SOURCE_DIR}/tools/swig)
  # foreach(inc ${THIS_PROJECT_INCLUDES_ALL_LIST})
  #   include_directories(${JAFAR_MODULES_PARENT_DIR}/${inc}/include)
  # endforeach(inc)
  # set_source_files_properties(include/${jafar_modulename}.i PROPERTIES CPLUSPLUS ON PREFIX "jafar::" )
  # set($ENV{SWIG_CPP_EXTENSION} cpp})
  # SWIG_ADD_MODULE(
  #   ${jafar_modulename}_ruby_wrap 
  #   ruby 
  #   include/${jafar_modulename}.i 
  #   include/${jafar_modulename}RUBY_wrap.cxx
  # )
  # SWIG_LINK_LIBRARIES(${jafar_modulename}_ruby_wrap ${RUBY_LIBRARY})
endmacro(WRAP_JAFAR_MODULE_TO_RUBY)

#------------------------------------------------------------------------------
#This macro generates tcl wrapping for a jafar module
#------------------------------------------------------------------------------
macro(WRAP_JAFAR_MODULE_TO_TCL jafar_modulename)
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/tcl)
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/src/tcl)
  endif()
  foreach(inc ${THIS_PROJECT_INCLUDES_ALL_LIST})
    set(INTERNAL_INCLUDES ${INTERNAL_INCLUDES} -I${JAFAR_MODULES_PARENT_DIR}/${inc}/include)
  endforeach(inc)
  add_custom_command(
    OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/src/tcl/${jafar_modulename}_wrap.cpp
    COMMAND ${SWIG_EXECUTABLE} 
    ARGS -c++ -Wall -v -tcl -namespace -pkgversion ${FULL_VERSION} ${INTERNAL_INCLUDES} -I${Boost_INCLUDE_DIRS} -I${Jafar_SOURCE_DIR}/tools/swig -I${TCL_INCLUDE_PATH} -I${TK_INCLUDE_PATH} "-D_JFR_MODULE_=\\\"${jafar_modulename}\\\"" -o ${CMAKE_CURRENT_SOURCE_DIR}/src/tcl/${jafar_modulename}_wrap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${jafar_modulename}.i
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/include/${jafar_modulename}.i
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${jafar_modulename}Tools.i ${CMAKE_CURRENT_SOURCE_DIR}/include/${jafar_modulename}Exception.i
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   )
  add_library(${jafar_modulename}_tcl_wrap SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/tcl/${jafar_modulename}_wrap.cpp)
  target_link_libraries(${jafar_modulename}_tcl_wrap ${jafar_modulename} stdc++ ${TCL_LIBRARY} ${TK_LIBRARY})
  set_target_properties(${jafar_modulename}_tcl_wrap 
    PROPERTIES OUTPUT_NAME ${jafar_modulename} 
               PREFIX ""
	       LIBRARY_OUTPUT_DIRECTORY ${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename})
  file(GLOB tcl_macros ${CMAKE_CURRENT_SOURCE_DIR}/macro/*.tcl)
# copy macros
  foreach(macro ${tcl_macros})
    get_filename_component(MACRO_NAME ${macro} NAME)
    add_custom_command(
      TARGET ${jafar_modulename}_tcl_wrap
      POST_BUILD
      COMMAND ${CMAKE_COMMAND}
      ARGS -E copy ${macro} ${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename}/${MACRO_NAME} 
      )
  endforeach(macro)
# generate tcl package
  add_custom_command(
    TARGET ${jafar_modulename}_tcl_wrap
    POST_BUILD
    COMMAND ${TCL_TCLSH} ARGS ${Jafar_SOURCE_DIR}/tools/swig/makePkg.tcl ${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename}
    WORKING_DIRECTORY ${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename}
  )

# install instructions for tcl library and macros
  install(TARGETS ${jafar_modulename}_tcl_wrap DESTINATION ${Jafar_BINARY_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename})
#  install(FILES ${tcl_macros} DESTINATION ${Jafar_BINARY_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename})
  file(GLOB tcl_files ${Jafar_SOURCE_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename}/*)
  install(FILES ${tcl_files} DESTINATION ${Jafar_BINARY_DIR}/tclpkg/${BUILDNAME}/${jafar_modulename})
endmacro(WRAP_JAFAR_MODULE_TO_TCL)

#------------------------------------------------------------------------------
#This macro generates qt special files
#------------------------------------------------------------------------------
macro(GENERATE_QT_FILES JAFAR_MODULENAME)
  #include qt specific macros
  include(${QT_USE_FILE})

  string(TOUPPER "${JAFAR_MODULENAME}_WRAPPED_HEADERS" QTUI_H_SRC)
  string(TOUPPER "${JAFAR_MODULENAME}_WRAPPED_CPPS" QT_MOC_SRCS)

  #generate headers from ui files
  file(GLOB UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.ui)
  QT4_WRAP_UI(${QTUI_H_SRC} ${UI_FILES})

  # generate moc files from headers contining \"Q_OBJECT\"
  file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/${JAFAR_MODULENAME}/*.hpp")
  foreach(header ${HEADERS})
    file(STRINGS ${header} Q_OBJECT_STRING REGEX "^[ ]*Q_OBJECT[ ]*$")
    if(NOT("${Q_OBJECT_STRING}" STREQUAL ""))
      get_filename_component(HEADER_NAME ${header} NAME_WE)
      execute_process(COMMAND ${QT_MOC_EXECUTABLE} ${header} -o ${CMAKE_CURRENT_SOURCE_DIR}/src/${HEADER_NAME}.moc
#	INPUT_FILE ${header} 
#	OUTPUT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/${HEADER_NAME}.moc
)
#      QT4_GENERATE_MOC(${header} ${CMAKE_CURRENT_SOURCE_DIR}/src/${HEADER_NAME}.moc)
    endif(NOT("${Q_OBJECT_STRING}" STREQUAL ""))
  endforeach(header)

#  QT4_WRAP_CPP(${QT_MOC_SRCS} ${HEADERS_TO_MOC})

endmacro(GENERATE_QT_FILES)

#------------------------------------------------------------------------------
# This macro builds jafar module.
# It computes its dependencies from CMakeLists.txt, builds the library and
# wraps it to the available script languages
#------------------------------------------------------------------------------
macro(BUILD_JAFAR_MODULE modulename)
  #----------------------------------------------------------------------------
  # computing module dependencies internal as well as external, 
  # required as well as optional ones
  #----------------------------------------------------------------------------
  parse_arguments(THIS_MODULE
    "VERSION;REVISION;REQUIRED_MODULES;OPTIONAL_MODULES;REQUIRED_EXTLIBS;OPTIONAL_EXTLIBS;CXXFLAGS;CPPFLAGS;LDFLAGS"
    ""
    ${ARGN}
    )
  # include headers from required modules
  foreach(required_module ${THIS_MODULE_REQUIRED_MODULES})
    include_directories(${JAFAR_MODULES_PARENT_DIR}/${required_module}/include)
  endforeach(required_module)

  # Set THIS_PROJECT_DEPENDS_ALL to the set of all of its
  # dependencies, its dependencies' dependencies, etc., transitively.
  string(TOUPPER "JAFAR_${modulename}_DEPENDS" THIS_PROJECT_DEPENDS)
  set(THIS_PROJECT_DEPENDS_ALL ${${THIS_PROJECT_DEPENDS}})
  set(ADDED_DEPS TRUE)
  while (ADDED_DEPS)
    set(ADDED_DEPS FALSE)
    foreach(DEP ${THIS_PROJECT_DEPENDS_ALL})
      string(TOUPPER "JAFAR_${DEP}_DEPENDS" DEP_DEPENDS)
      foreach(DEPDEP ${${DEP_DEPENDS}})
        list(FIND THIS_PROJECT_DEPENDS_ALL ${DEPDEP} DEPDEP_INDEX)
        if (DEPDEP_INDEX EQUAL -1)
          list(APPEND THIS_PROJECT_DEPENDS_ALL ${DEPDEP})
          set(ADDED_DEPS TRUE)
        endif()
      endforeach()
    endforeach()
  endwhile()

  string(REGEX REPLACE "[ ]+" ";" THIS_PROJECT_DEPENDS_ALL_LIST "${THIS_PROJECT_DEPENDS_ALL}")
  set(THIS_PROJECT_INCLUDES_ALL_LIST ${THIS_PROJECT_DEPENDS_ALL_LIST})
  message(STATUS "- project ${modulename} dependencies ${THIS_PROJECT_INCLUDES_ALL_LIST}")
  foreach(inc ${THIS_PROJECT_INCLUDES_ALL_LIST})
    include_directories(${JAFAR_MODULES_PARENT_DIR}/${inc}/include)
  endforeach(inc)

  # check required external libraries dependencies
  list(LENGTH THIS_MODULE_REQUIRED_EXTLIBS NB_EXTLIBS)
  if(${NB_EXTLIBS} GREATER 0)
    foreach(extlib ${THIS_MODULE_REQUIRED_EXTLIBS})
      get_ext_library_exact_name(${extlib} LIB_EXACT_NAME)
      # boost is always found as it is a requirement
      # besides, boost libraries are located in components, we treat them apart
      if("${LIB_EXACT_NAME}" STREQUAL "Boost")
	get_boost_component_library(${extlib} BOOST_COMPONENT_LIBRARY)
	list(APPEND THIS_PROJECT_DEPENDS_ALL_LIST ${BOOST_COMPONENT_LIBRARY})
      else("${LIB_EXACT_NAME}" STREQUAL "Boost")
	if("${LIB_EXACT_NAME}" STREQUAL "QT")
	  set(QT_WRAPPING_REQUIRED 1)
	  list(APPEND THIS_PROJECT_DEPENDS_ALL_LIST ${QT_LIBRARIES})
	else("${LIB_EXACT_NAME}" STREQUAL "QT")
	  string(TOUPPER "${LIB_EXACT_NAME}_FOUND" THIS_LIB_FOUND)
	  if(${THIS_LIB_FOUND})
	    set(LIB_EXACT_NAME_LIBRARIES "${LIB_EXACT_NAME}_LIBRARIES")
	    list(APPEND THIS_PROJECT_DEPENDS_ALL_LIST ${${LIB_EXACT_NAME_LIBRARIES}})
	  else(${EXTLIB_FOUND})
	    message(FATAL_ERROR "this module requires ${extlib} but it wasn't found, please install it and rerun cmake on top of jafar!")
	  endif(${THIS_LIB_FOUND})
	endif("${LIB_EXACT_NAME}" STREQUAL "QT")
      endif("${LIB_EXACT_NAME}" STREQUAL "Boost")
    endforeach(extlib)
  endif(${NB_EXTLIBS} GREATER 0)

  # check optional external libraries dependencies
  list(LENGTH THIS_MODULE_OPTIONAL_EXTLIBS NB_EXTLIBS)
  if(${NB_EXTLIBS} GREATER 0)
    foreach(extlib ${THIS_MODULE_OPTIONAL_EXTLIBS})
      get_ext_library_exact_name(${extlib} LIB_EXACT_NAME)
      if("${LIB_EXACT_NAME}" STREQUAL "Boost")
	get_boost_component_library(${extlib} BOOST_COMPONENT_LIBRARY)
	list(APPEND THIS_PROJECT_DEPENDS_ALL ${BOOST_COMPONENT_LIBRARY})
      else("${LIB_EXACT_NAME}" STREQUAL "Boost")
	if("${LIB_EXACT_NAME}" STREQUAL "QT" AND QT4_FOUND)
	  set(QT_WRAPPING_REQUIRED 1)
	  list(APPEND THIS_PROJECT_DEPENDS_ALL_LIST ${QT_LIBRARIES})
	else("${LIB_EXACT_NAME}" STREQUAL "QT" AND QT4_FOUND)
	  string(TOUPPER "${LIB_EXACT_NAME}_FOUND" THIS_LIB_FOUND)
	  if(${THIS_LIB_FOUND})
	    set(LIB_EXACT_NAME_LIBRARIES "${LIB_EXACT_NAME}_LIBRARIES")
	    list(APPEND THIS_PROJECT_DEPENDS_ALL_LIST ${${LIB_EXACT_NAME_LIBRARIES}})
	  else(${EXTLIB_FOUND})
	    message(STATUS "this module requires ${extlib} but it wasn't found, some features may be disabled!")
	  endif(${THIS_LIB_FOUND})
	endif("${LIB_EXACT_NAME}" STREQUAL "QT" AND QT4_FOUND)
      endif("${LIB_EXACT_NAME}" STREQUAL "Boost")
    endforeach(extlib)
  endif(${NB_EXTLIBS} GREATER 0)

  string(TOLOWER "${modulename}" MODULENAME)
  string(TOLOWER "jafar_module_${MODULENAME}" JAFAR_MODULE_PROJECT)

  #------------------------------------------------------------------------------
  # instantiate module project and targets: library, test and wrappers
  #------------------------------------------------------------------------------
  project(${JAFAR_MODULE_PROJECT})
  #------------------------------------------------------------------------------
  # building module library
  #------------------------------------------------------------------------------

  #read all the project properties
  set(FULL_VERSION "${THIS_MODULE_VERSION}.${THIS_MODULE_REVISION}")
  set(ALL_COMPILER_FLAGS ${THIS_MODULE_CXXFLAGS} ${THIS_MODULE_CPPFLAGS} "-D_JFR_MODULE_=\\\"${MODULENAME}\\\"")
  message(STATUS "- compiling flags ${ALL_COMPILER_FLAGS}")

  # wrap QT files if needed
  if(QT_WRAPPING_REQUIRED)
    generate_qt_files(${MODULENAME})
  endif(QT_WRAPPING_REQUIRED)

  # add headers
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
  include_directories(${INCLUDES_INSTALL_DIR})
  set(LIBS ${LIBS} ${LIBRARIES_INSTALL_DIR})

  # add sources
  file(GLOB module_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  if(QT_WRAPPING_REQUIRED)
    string(TOUPPER "${MODULENAME}_WRAPPED_CPPS" generated_cxx)
    set(module_sources ${module_sources} ${${generated_cxx}})
  endif(QT_WRAPPING_REQUIRED)

  # build the library
  add_library(${MODULENAME} SHARED ${module_sources})

  # link module to dependencies
  foreach(dependency ${THIS_PROJECT_DEPENDS_ALL_LIST})
    target_link_libraries(${MODULENAME} ${dependency})
    message(STATUS "--> linking ${MODULENAME} to ${dependency}")
  endforeach(dependency)

  # set library properties: version, output directory, compiler flags and link flags
  set_target_properties(${MODULENAME} PROPERTIES VERSION ${FULL_VERSION} SOVERSION ${THIS_MODULE_VERSION})
  set_target_properties(${MODULENAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARIES_OUTPUT_DIR})
  set_target_properties(${MODULENAME} PROPERTIES COMPILER_FLAGS "${ALL_COMPILER_FLAGS}")
  set_target_properties(${MODULENAME} PROPERTIES LINK_FLAGS "${THIS_MODULE_LDFLAGS}")
#  set_target_properties(${MODULENAME} PROPERTIES LINK_FLAGS_RELEASE )

  # copy headers
  file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/include/${MODULENAME}/*.hpp)
  foreach(header ${headers})
    get_filename_component(HEADER_NAME ${header} NAME)
    add_custom_command(
      TARGET ${MODULENAME}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND}
      ARGS -E copy_if_different ${header} ${INCLUDES_OUTPUT_DIR}/${MODULENAME}/${HEADER_NAME} 
      )
  endforeach(header)

  # install headers and libraries
  install(TARGETS ${MODULENAME} DESTINATION ${LIBRARIES_INSTALL_DIR})
  file(GLOB module_headers include/${MODULENAME}/*.hpp)
  install(FILES ${module_headers} DESTINATION ${INCLUDES_INSTALL_DIR}/${MODULENAME})

  #------------------------------------------------------------------------------
  # building test for module
  #------------------------------------------------------------------------------
  file(GLOB test_sources ${CMAKE_CURRENT_SOURCE_DIR}/test_suite/*.cpp)
  add_executable(test_suite_${MODULENAME} ${test_sources})
  target_link_libraries(test_suite_${MODULENAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${MODULENAME})
  set_target_properties(test_suite_${MODULENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY test_suite/${BUILDNAME})
  # set_target_properties(test_suite_${MODULENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY test_suite/${BUILDNAME} 
  #   LINK_FLAGS "${THIS_MODULE_LDFLAGS}"
  #   COMPILER_FLAGS "${ALL_COMPILER_FLAGS}")
  add_test(test_suite_${MODULENAME} test_suite/${BUILDNAME}/test_suite_${MODULENAME})

  #------------------------------------------------------------------------------
  # building demos for module
  #------------------------------------------------------------------------------
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo_suite)
    file(GLOB demo_sources ${CMAKE_CURRENT_SOURCE_DIR}/demo_suite/*.cpp)
    foreach(source ${demo_sources})
      get_filename_component(demo ${source} NAME_WE)
      add_executable(${MODULENAME}_${demo} ${source})
      target_link_libraries(${MODULENAME}_${demo} ${MODULENAME})
      set_target_properties(${MODULENAME}_${demo} PROPERTIES RUNTIME_OUTPUT_DIRECTORY demo_suite/${BUILDNAME}
	                                                     OUTPUT_NAME ${demo})
    endforeach(source)
  endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo_suite)

  #------------------------------------------------------------------------------
  # building wrapped libraries in available script languages
  #------------------------------------------------------------------------------
  # wrap module to ruby
  if(ENABLE_RUBY)
    wrap_jafar_module_to_ruby(${modulename})
  endif(ENABLE_RUBY)

  # wrap module to tcl
  if(ENABLE_TCL)
    wrap_jafar_module_to_tcl(${modulename})
  endif(ENABLE_TCL)

endmacro(BUILD_JAFAR_MODULE)
