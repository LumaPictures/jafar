message(STATUS "looking for opennix")

find_path(OPENNI_INCLUDE_DIR
  NAMES XnOpenNI.h
  PATHS
  ${ROBOTPKG_BASE}/include
  $ENV{ROBOTPKG_BASE}/include
  /usr/include
  /usr/local/include
  /sw/include
  /opt/local/include
  PATH_SUFFIXES ni
  DOC "The directory where XnOpenNI.h resides")
if(EXISTS ${OPENNI_INCLUDE_DIR})
	set(OPENNI_INCLUDE_DIR_FOUND 1)
	mark_as_advanced(FORCE OPENNI_INCLUDE_DIR)
endif()

find_path(OPENNI_NITE_INCLUDE_DIR
  NAMES XnVNite.h
  PATHS
  ${ROBOTPKG_BASE}/include
  $ENV{ROBOTPKG_BASE}/include
  /usr/include
  /usr/local/include
  /sw/include
  /opt/local/include
  PATH_SUFFIXES nite 
  DOC "The directory where XnVNite.h resides")
if(EXISTS ${OPENNI_NITE_INCLUDE_DIR})
	set(OPENNI_NITE_INCLUDE_DIR_FOUND 1)
	mark_as_advanced(FORCE OPENNI_NITE_INCLUDE_DIR)
endif()

if(OPENNI_INCLUDE_DIR_FOUND AND OPENNI_NITE_INCLUDE_DIR_FOUND)
	SET(OPENNIX_INCLUDE_DIRS ${OPENNI_NITE_INCLUDE_DIR};${OPENNI_INCLUDE_DIR})
	SET(OPENNIX_INCLUDE_DIRS_FOUND 1)
endif()

message(STATUS "\topennix headers: ${OPENNIX_INCLUDE_DIRS}")

find_library(OPENNI_LIBRARIES
  NAMES OpenNI Sample-NiSampleModule nimCodecs nimMockNodes nimRecorder
  PATHS
  ${ROBOTPKG_BASE}/lib
  $ENV{ROBOTPKG_BASE}/lib
  /usr/lib
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /sw/lib
  /opt/local/lib
  DOC "The OpenNI library")
if(EXISTS ${OPENNI_LIBRARIES})
	set(OPENNI_LIBRARIES_FOUND 1)
	mark_as_advanced(FORCE OPENNI_LIBRARIES)
endif()

find_library(OPENNI_NITE_LIBRARIES
  NAMES XnVNite XnVFeatures XnVHandGenerator
  PATHS
  ${ROBOTPKG_BASE}/lib
  $ENV{ROBOTPKG_BASE}/lib
  /usr/lib
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /sw/lib
  /opt/local/lib
  DOC "The OpenNI Nite library")
if(EXISTS ${OPENNI_NITE_LIBRARIES})
	set(OPENNI_NITE_LIBRARIES_FOUND 1)
	mark_as_advanced(FORCE OPENNI_NITE_LIBRARIES)
endif()

find_library(OPENNI_SENSOR_LIBRARIES
  NAMES XnCore XnDDK XnDeviceFile XnDeviceSensorV2 XnFormats
  PATHS
  ${ROBOTPKG_BASE}/lib
  $ENV{ROBOTPKG_BASE}/lib
  /usr/lib
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /sw/lib
  /opt/local/lib
  DOC "The OpenNI Sensor library")
if(EXISTS ${OPENNI_SENSOR_LIBRARIES})
	set(OPENNI_SENSOR_LIBRARIES_FOUND 1)
	mark_as_advanced(FORCE OPENNI_SENSOR_LIBRARIES)
endif()

if(OPENNI_LIBRARIES_FOUND AND 
	 OPENNI_SENSOR_LIBRARIES_FOUND AND 
	 OPENNI_NITE_LIBRARIES_FOUND)
 SET(OPENNIX_LIBRARIES ${OPENNI_NITE_LIBRARIES};${OPENNI_LIBRARIES};${OPENNI_SENSOR_LIBRARIES})
 SET(OPENNIX_LIBRARIES_FOUND 1)
endif()
message(STATUS "\topennix libraries: ${OPENNIX_LIBRARIES}")

if(OPENNIX_LIBRARIES_FOUND AND OPENNIX_INCLUDE_DIRS_FOUND)
  set(OPENNIX_FOUND 1)
else()
  set(OPENNIX_FOUND 0)
endif()

