#
# (C) Copyright 2014 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawAtracsysFusionTrack requires cisst to be built with JSON" )
  endif ()

  # this should be replaced by a find AtracsysFusionTrack
  set (atracsys_DIR "/home/paul/dev/fusiontrack")
  include_directories (${atracsys_DIR}/soft.atr.2cams.sdk/dev/include
                       ${atracsys_DIR}/soft.atr.2cams.sdk/dev/include/interface
      ${atracsys_DIR}/soft.atr.2cams.samples/dev/include/public)
  set (atracsys_LIBRARIES pthread rt "/usr/lib/libfusionTrack64.so")

  # create/configure file for find_package (sawAtracsysFusionTrack)
  set (sawAtracsysFusionTrack_INCLUDE_DIR "${sawAtracsysFusionTrack_SOURCE_DIR}/include")
  set (sawAtracsysFusionTrack_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawAtracsysFusionTrack_LIBRARIES sawAtracsysFusionTrack ${atracsys_LIBRARIES})

  include_directories (${sawAtracsysFusionTrack_INCLUDE_DIR} ${sawAtracsysFusionTrack_BINARY_DIR})
  set (sawAtracsysFusionTrack_HEADER_DIR "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack")

  add_library (sawAtracsysFusionTrack
               ${sawAtracsysFusionTrack_HEADER_DIR}/sawAtracsysFusionTrackExport.h
               mtsAtracsysFusionTrack.cpp
               ${sawAtracsysFusionTrack_HEADER_DIR}/mtsAtracsysFusionTrack.h)
  set_property (TARGET sawAtracsysFusionTrack PROPERTY FOLDER "sawAtracsysFusionTrack")
  cisst_target_link_libraries (sawAtracsysFusionTrack ${REQUIRED_CISST_LIBRARIES})
  target_link_libraries (sawAtracsysFusionTrack ${atracsys_LIBRARIES})

  # add Qt code
  add_subdirectory (Qt)
  set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} ${sawIntuitiveResearchKitQt_LIBRARIES})

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawAtracsysFusionTrackConfig.cmake.in"
                  "${sawAtracsysFusionTrack_BINARY_DIR}/sawAtracsysFusionTrackConfig.cmake"
                   @ONLY@)

  # Install target for headers and library
  install (DIRECTORY
           "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawAtracsysFusionTrack
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawAtracsysFusionTrack_BINARY_DIR}/sawAtracsysFusionTrackConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
