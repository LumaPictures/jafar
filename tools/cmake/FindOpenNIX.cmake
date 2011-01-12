message(STATUS "looking for opennix")
# set(OPENNI_NITE_INCLUDE_DIR)
# set(OPENNI_INCLUDE_DIR)
# set(OPENNI_FOUND 0)

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

SET(OPENNIX_INCLUDE_DIRS ${OPENNI_NITE_INCLUDE_DIR};${OPENNI_INCLUDE_DIR})

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

SET(OPENNIX_LIBRARIES ${OPENNI_NITE_LIBRARIES};${OPENNI_LIBRARIES};${OPENNI_SENSOR_LIBRARIES})
message(STATUS "\topennix libraries: ${OPENNIX_LIBRARIES}")
if(OPENNIX_FOUND)
  set(OPENNIX_FOUND 1)
else()
  set(OPENNIX_FOUND 0)
endif()

