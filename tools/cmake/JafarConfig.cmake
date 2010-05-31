include(CheckCXXSourceCompiles)


# Toolset detection.
if (NOT JAFAR_TOOLSET)
  set(JAFAR_TOOLSET "unknown")
  if (MSVC60)
    set(JAFAR_TOOLSET "vc6")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "6.0")
  elseif(MSVC70)
    set(JAFAR_TOOLSET "vc7")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "7.0")
  elseif(MSVC71)
    set(JAFAR_TOOLSET "vc71")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "7.1")
  elseif(MSVC80)
    set(JAFAR_TOOLSET "vc80")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "8.0")
  elseif(MSVC90)
    set(JAFAR_TOOLSET "vc90")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "9.0")
  elseif(MSVC)
    set(JAFAR_TOOLSET "vc")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "unknown")
  elseif(BORLAND)
    set(JAFAR_TOOLSET "bcb")
    set(JAFAR_COMPILER "msvc")
    set(JAFAR_COMPILER_VERSION "unknown")
  elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(JAFAR_COMPILER "gnu")

    # Execute GCC with the -dumpversion option, to give us a version string
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" 
      OUTPUT_VARIABLE GCC_VERSION_STRING)
    
    # Match only the major and minor versions of the version string
    string(REGEX MATCH "[0-9]+.[0-9]+" GCC_MAJOR_MINOR_VERSION_STRING
      "${GCC_VERSION_STRING}")

    # Match the full compiler version for the build name
    string(REGEX MATCH "[0-9]+.[0-9]+.[0-9]+" JAFAR_COMPILER_VERSION
      "${GCC_VERSION_STRING}")
    
    # Strip out the period between the major and minor versions
    string(REGEX REPLACE "\\." "" JAFAR_VERSIONING_GCC_VERSION
      "${GCC_MAJOR_MINOR_VERSION_STRING}")
    
    # Set the GCC versioning toolset
    set(JAFAR_TOOLSET "gcc${JAFAR_VERSIONING_GCC_VERSION}")
  elseif(CMAKE_CXX_COMPILER MATCHES "/icpc$" 
      OR CMAKE_CXX_COMPILER MATCHES "/icpc.exe$" 
      OR CMAKE_CXX_COMPILER MATCHES "/icl.exe$")
    set(JAFAR_TOOLSET "intel")
    set(JAFAR_COMPILER "intel")
    set(CMAKE_COMPILER_IS_INTEL ON)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion"
      OUTPUT_VARIABLE INTEL_VERSION_STRING
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(JAFAR_COMPILER_VERSION ${INTEL_VERSION_STRING})
  endif(MSVC60)
endif (NOT JAFAR_TOOLSET)

MESSAGE(STATUS "Jafar compiler: " ${JAFAR_COMPILER})
MESSAGE(STATUS "Jafar toolset: "  ${JAFAR_TOOLSET})

# create cache entry
set(JAFAR_PLATFORM "unknown")

# Multi-threading support
if(CMAKE_SYSTEM_NAME STREQUAL "SunOS")
  set(JAFAR_PLATFORM "sunos")
elseif(CMAKE_SYSTEM_NAME STREQUAL "BeOS")
  # No threading options necessary for BeOS
  set(JAFAR_PLATFORM "beos")
elseif(CMAKE_SYSTEM_NAME MATCHES ".*BSD")
  set(JAFAR_PLATFORM "bsd")
elseif(CMAKE_SYSTEM_NAME STREQUAL "DragonFly")
  set(JAFAR_PLATFORM "dragonfly")
elseif(CMAKE_SYSTEM_NAME STREQUAL "IRIX")
  set(JAFAR_PLATFORM "irix")
elseif(CMAKE_SYSTEM_NAME STREQUAL "HP-UX")
  set(JAFAR_PLATFORM "hpux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(JAFAR_PLATFORM "darwin")
elseif(UNIX)
  if (MINGW)
    set(JAFAR_PLATFORM "mingw")
  elseif(CYGWIN)
    set(JAFAR_PLATFORM "cygwin")
  elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(JAFAR_PLATFORM "linux")
  else()
    set(JAFAR_PLATFORM "unix")
  endif()
elseif(WIN32)
  set(JAFAR_PLATFORM "windows")
else()
  set(JAFAR_PLATFORM "unknown")
endif()

# create cache entry
set(JAFAR_PLATFORM ${JAFAR_PLATFORM} CACHE STRING "Jafar platform name")

message(STATUS "Jafar platform: " ${JAFAR_PLATFORM})

# Set the build name 
set(BUILDNAME "${CMAKE_SYSTEM_PROCESSOR}-${JAFAR_PLATFORM}-${JAFAR_COMPILER}" CACHE STRING "build name")
message(STATUS "Build name: " ${BUILDNAME})

set(BUILD_PROJECTS "ALL"  CACHE STRING "Semicolon-separated list of project to build, or \"ALL\"")
