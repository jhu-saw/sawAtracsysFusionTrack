#
# (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawAtracsysFusionTrack)

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.0.6 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message (FATAL_ERROR "sawAtracsysFusionTrack requires cisst to be built with JSON")
  endif ()

  # catkin/ROS paths
  cisst_is_catkin_build (sawAtracsysFusionTrack_IS_CATKIN_BUILT)
  if (sawAtracsysFusionTrack_IS_CATKIN_BUILT)
    set (sawAtracsysFusionTrack_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawAtracsysFusionTrack/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawAtracsysFusionTrack_BINARY_DIR}/lib")
  else ()
    set (sawAtracsysFusionTrack_CONFIG_FILE_DIR "${sawAtracsysFusionTrack_BINARY_DIR}")
  endif ()

  # this should be replaced by a find AtracsysFusionTrack
  set (atracsys_FOUND 0)
  if (WIN32)
    # Windows
    set (atracsys_DIR "C:/Program Files/Atracsys/Passive Tracking SDK")
    include_directories (${atracsys_DIR}/include
                         ${atracsys_DIR}/bin)
    link_directories (${atracsys_DIR}/bin)
    set (atracsys_LIBRARIES winmm fusionTrack64.lib)
    set (atracsys_FOUND 1)
    # Linux
  elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_path (atracsys_DIR
               NAMES "ftkInterface.h"
               PATH_SUFFIXES "src/include" "include")
    if (EXISTS "${atracsys_DIR}/src/include/ftkInterface.h" OR EXISTS "${atracsys_DIR}/include/ftkInterface.h")
      set (atracsys_FOUND 1)
      include_directories (${atracsys_DIR}/src/include ${atracsys_DIR}/include)
      # libraries
      link_directories (${atracsys_DIR}/bin)
      set (atracsys_LIBRARIES pthread rt fusionTrack64 device64)
      # copy the binary provided for fusionTrack in cisst/saw library directory
      find_library (atracsys_LIBRARY_fusionTrack
                    NAMES fusionTrack64
                    PATHS ${atracsys_DIR}/bin ${atracsys_DIR}/lib)
      find_library (atracsys_LIBRARY_device
                    NAMES device64
                    PATHS ${atracsys_DIR}/bin ${atracsys_DIR}/lib)
      file (MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
      file (COPY ${atracsys_LIBRARY_fusionTrack} ${atracsys_LIBRARY_device}
            DESTINATION ${LIBRARY_OUTPUT_PATH})
    endif ()
  else (WIN32)
    # Everything else
    set (atracsys_LIBRARIES pthread rt fusionTrack64)
  endif (WIN32)

  # if we found the actracsys SDK
  if (atracsys_FOUND)
    # create/configure file for find_package (sawAtracsysFusionTrack)
    set (sawAtracsysFusionTrack_INCLUDE_DIR
         "${sawAtracsysFusionTrack_SOURCE_DIR}/include"
         "${sawAtracsysFusionTrack_BINARY_DIR}/include")
    set (sawAtracsysFusionTrack_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${atracsys_DIR}/lib")
    set (sawAtracsysFusionTrack_LIBRARIES sawAtracsysFusionTrack ${atracsys_LIBRARIES})

    # Set the version number
    set (sawAtracsysFusionTrack_VERSION_MAJOR "1")
    set (sawAtracsysFusionTrack_VERSION_MINOR "1")
    set (sawAtracsysFusionTrack_VERSION_PATCH "0")
    set (sawAtracsysFusionTrack_VERSION "${sawAtracsysFusionTrack_VERSION_MAJOR}.${sawAtracsysFusionTrack_VERSION_MINOR}.${sawAtracsysFusionTrack_VERSION_PATCH}")

    # Generate sawAtracsysFusionTrackRevision.h
    configure_file ("${sawAtracsysFusionTrack_SOURCE_DIR}/code/sawAtracsysFusionTrackRevision.h.in"
                    "${sawAtracsysFusionTrack_BINARY_DIR}/include/sawAtracsysFusionTrack/sawAtracsysFusionTrackRevision.h")

    include_directories (${sawAtracsysFusionTrack_INCLUDE_DIR})
    link_directories (${sawAtracsysFusionTrack_LIBRARY_DIR})

    set (sawAtracsysFusionTrack_HEADER_DIR "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack")

    add_library (sawAtracsysFusionTrack ${IS_SHARED}
                 ${sawAtracsysFusionTrack_HEADER_DIR}/sawAtracsysFusionTrackExport.h
                 code/mtsAtracsysFusionTrack.cpp
                 ${sawAtracsysFusionTrack_HEADER_DIR}/mtsAtracsysFusionTrack.h)
    set_property (TARGET sawAtracsysFusionTrack PROPERTY FOLDER "sawAtracsysFusionTrack")
    cisst_target_link_libraries (sawAtracsysFusionTrack ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries (sawAtracsysFusionTrack ${atracsys_LIBRARIES})

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawAtracsysFusionTrack_LIBRARIES ${sawAtracsysFusionTrack_LIBRARIES} ${sawAtracsysFusionTrackQt_LIBRARIES})
    endif (CISST_HAS_QT)

    set (sawAtracsysFusionTrack_CMAKE_CONFIG_FILE
         "${sawAtracsysFusionTrack_CONFIG_FILE_DIR}/sawAtracsysFusionTrackConfig.cmake")

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawAtracsysFusionTrackConfig.cmake.in"
                    "${sawAtracsysFusionTrack_CMAKE_CONFIG_FILE}"
                    @ONLY)

    cisst_add_config_version ("${sawAtracsysFusionTrack_CONFIG_FILE_DIR}/sawAtracsysFusionTrackConfigVersion.cmake"
                              VERSION ${sawAtracsysFusionTrack_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawAtracsysFusionTrack)

    # Install target for headers and library
    install (DIRECTORY
             "${sawAtracsysFusionTrack_SOURCE_DIR}/include/sawAtracsysFusionTrack"
             "${sawAtracsysFusionTrack_BINARY_DIR}/include/sawAtracsysFusionTrack"
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawAtracsysFusionTrack
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawAtracsysFusionTrack_BINARY_DIR}/sawAtracsysFusionTrackConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (atracsys_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Atracsys SDK.  Please set atracsys_DIR using CMake, e.g. ccmake ${CMAKE_CURRENT_BINARY_DIR}")
  endif (atracsys_FOUND)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
