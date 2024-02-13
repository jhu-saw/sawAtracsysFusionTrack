#
# (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawAtracsysFusionTrack VERSION 1.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # we need json for general config file and added support for tool geometry to replace .ini
  if (NOT CISST_HAS_JSON)
    message (FATAL_ERROR "sawAtracsysFusionTrack requires cisst to be built with JSON")
  endif ()

  # still looking for .ini parser
  if (WIN32)
    # Windows, not sure if there's a system provided library
    set (INI_PARSER_FOUND 0)
  elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # Linux, look for inih
    find_path (INI_PARSER_INCLUDE_DIR iniparser/iniparser.h)
    find_library (INI_PARSER_LIBRARIES NAMES iniparser)
    if (INI_PARSER_INCLUDE_DIR AND INI_PARSER_LIBRARIES)
      set (INI_PARSER_FOUND 1)
      mark_as_advanced (INI_PARSER_INCLUDE_DIR INI_PARSER_LIBRARIES)
    else ()
      set (INI_PARSER_FOUND 0)
    endif ()
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  find_package (AtracsysSDK)

  if (AtracsysSDK_FOUND)
    include_directories(${AtracsysSDK_INCLUDE_DIR})

    if (UNIX)
      # copy the binary provided for fusionTrack in cisst/saw library directory
      file (MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
      file (COPY ${AtracsysSDK_LIBRARY_fusionTrack} ${AtracsysSDK_LIBRARY_device}
            DESTINATION ${LIBRARY_OUTPUT_PATH})
    endif (UNIX)

    # create/configure file for find_package (sawAtracsysFusionTrack)
    set (sawAtracsysFusionTrack_INCLUDE_DIR
      "${sawAtracsysFusionTrack_SOURCE_DIR}/include"
      "${sawAtracsysFusionTrack_BINARY_DIR}/include")
    set (sawAtracsysFusionTrack_HEADER_DIR "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack")
    set (sawAtracsysFusionTrack_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${AtracsysSDK_LIBRARY_DIR}")
    set (sawAtracsysFusionTrack_LIBRARIES sawAtracsysFusionTrack ${AtracsysSDK_LIBRARIES})
    if (INI_PARSER_FOUND)
      set (sawAtracsysFusionTrack_LIBRARIES ${sawAtracsysFusionTrack_LIBRARIES} ${INI_PARSER_LIBRARIES})
    endif ()

    include_directories (${sawAtracsysFusionTrack_INCLUDE_DIR})
    link_directories (${sawAtracsysFusionTrack_LIBRARY_DIR})

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawAtracsysFusionTrack_LIBRARIES ${sawAtracsysFusionTrack_LIBRARIES} sawAtracsysFusionTrackQt)
    endif (CISST_HAS_QT)

    cisst_add_config_files (sawAtracsysFusionTrack)

    add_library (sawAtracsysFusionTrack ${IS_SHARED}
      ${sawAtracsysFusionTrack_HEADER_DIR}/sawAtracsysFusionTrackExport.h
      code/mtsAtracsysFusionTrack.cpp
      ${sawAtracsysFusionTrack_HEADER_DIR}/mtsAtracsysFusionTrack.h)
    set_target_properties (sawAtracsysFusionTrack PROPERTIES
      VERSION ${sawAtracsysFusionTrack_VERSION}
      FOLDER "sawAtracsysFusionTrack")
    cisst_target_link_libraries (sawAtracsysFusionTrack ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries (sawAtracsysFusionTrack ${AtracsysSDK_LIBRARIES})

    configure_file (
      "${sawAtracsysFusionTrack_SOURCE_DIR}/code/sawAtracsysFusionTrackConfig.h.in"
      "${sawAtracsysFusionTrack_BINARY_DIR}/include/sawAtracsysFusionTrack/sawAtracsysFusionTrackConfig.h")

    # Install target for headers and library
    install (DIRECTORY
      "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack"
      "${sawAtracsysFusionTrack_BINARY_DIR}/include/sawAtracsysFusionTrack"
      DESTINATION include
      COMPONENT sawAtracsysFusionTrack-dev)

    install (TARGETS sawAtracsysFusionTrack
      COMPONENT sawAtracsysFusionTrack
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  else (AtracsysSDK_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Atracsys SDK.")
  endif (AtracsysSDK_FOUND)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
