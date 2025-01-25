# (C) Copyright 2021-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# FindAtracsysSDK.cmake
#
# Find the Atracsys fusionTrack SDK. It is sufficient to set AtracsysSDK_INCLUDE_DIR
# because the other files should then be found automatically.
#
#    AtracsysSDK_INCLUDE_DIR  -- path to header files
#    AtracsysSDK_LIBRARY_DIR  -- path to library files
#    AtracsysSDK_LIBRARIES    -- list of library names
#    AtracsysSDK_FOUND        -- true if package found
#
# Also sets the following:
#    AtracsysSDK_LIBRARY_fusionTrack -- full path to fusionTrack library
#    AtracsysSDK_LIBRARY_device      -- full path to device library

find_path (AtracsysSDK_INCLUDE_DIR
           NAMES "ftkInterface.h"
           DOC "Directory for Atracsys SDK header files"
           PATH_SUFFIXES "src/include" "include"
           PATHS "C:/Program Files/Atracsys/Passive Tracking SDK")

# Determine whether to look for 32-bit or 64-bit libraries
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(AtracsysSDK_ARCH "64")
else ()
  set(AtracsysSDK_ARCH "32")
endif()

find_library (AtracsysSDK_LIBRARY_fusionTrack
              NAMES "fusionTrack${AtracsysSDK_ARCH}"
              DOC "Atracsys SDK library"
              PATH_SUFFIXES "lib"
              PATHS "${AtracsysSDK_INCLUDE_DIR}/..")
mark_as_advanced (AtracsysSDK_LIBRARY_fusionTrack)
get_filename_component(AtracsysSDK_LIBRARY_DIR ${AtracsysSDK_LIBRARY_fusionTrack} DIRECTORY)

if (UNIX)
  find_library (AtracsysSDK_LIBRARY_device
                NAMES "device${AtracsysSDK_ARCH}"
                PATH_SUFFIXES "lib"
                PATHS "${AtracsysSDK_LIBRARY_DIR}")
  mark_as_advanced (AtracsysSDK_LIBRARY_device)
endif (UNIX)

set (AtracsysSDK_FOUND FALSE)
if (AtracsysSDK_INCLUDE_DIR AND AtracsysSDK_LIBRARY_DIR)
  set (AtracsysSDK_FOUND TRUE)

  get_filename_component(LIBRARY_fusionTrack ${AtracsysSDK_LIBRARY_fusionTrack} NAME)
  set (AtracsysSDK_LIBRARIES ${LIBRARY_fusionTrack})
  unset(LIBRARY_fusionTrack)
  if (WIN32)
    set (AtracsysSDK_LIBRARIES ${AtracsysSDK_LIBRARIES} winmm)
  endif (WIN32)
  if (UNIX)
    get_filename_component(LIBRARY_device ${AtracsysSDK_LIBRARY_device} NAME)
    set (AtracsysSDK_LIBRARIES ${AtracsysSDK_LIBRARIES} ${LIBRARY_device} pthread rt)
    unset(LIBRARY_device)
  endif (UNIX)

  # Test what SDK supports
  set (CMAKE_REQUIRED_INCLUDES "${CMAKE_REQUIRED_INCLUDES};${AtracsysSDK_INCLUDE_DIR}")
  include (CheckCXXSourceCompiles)
  # Check for FTK_OK
  unset (AtracsysSDK_HAS_FTK_OK CACHE)
  check_cxx_source_compiles ("
    #include <ftkInterface.h>
    int main(void) {
      static ftkError _error = FTK_OK;
    }" AtracsysSDK_HAS_FTK_OK)
  # Check for QS_OK
  unset (AtracsysSDK_HAS_QS_OK CACHE)
  check_cxx_source_compiles ("
    #include <ftkInterface.h>
    int main(void) {
      static ftkQueryStatus _status = QS_OK;
    }" AtracsysSDK_HAS_QS_OK)
  unset (AtracsysSDK_HAS_ftkCameraParameters CACHE)
  check_cxx_source_compiles ("
    #include <ftkInterface.h>
    int main(void) {
      static ftkCameraParameter parms;
    }" AtracsysSDK_HAS_ftkCameraParameters)

endif (AtracsysSDK_INCLUDE_DIR AND AtracsysSDK_LIBRARY_DIR)
