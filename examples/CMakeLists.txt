#
# (C) Copyright 2014-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawAtracsysFusionTrackExamples_IS_CATKIN_BUILT)
  if (sawAtracsysFusionTrackExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawAtracsysFusionTrack has been compiled within cisst, we should find it automatically
  find_package (sawAtracsysFusionTrack)

  if (sawAtracsysFusionTrack_FOUND AND CISST_HAS_QT)

    # sawAtracsysFusionTrack configuration
    include_directories (${sawAtracsysFusionTrack_INCLUDE_DIR})
    link_directories (${sawAtracsysFusionTrack_LIBRARY_DIR})

    if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)

    add_executable (sawAtracsysFusionTrackQtExample mainQt.cpp)
    set_property (TARGET sawAtracsysFusionTrackQtExample PROPERTY FOLDER "sawAtracsysFusionTrack")

    # link against non cisst libraries and saw components
    target_link_libraries (sawAtracsysFusionTrackQtExample
                           ${sawAtracsysFusionTrack_LIBRARIES}
                           ${sawAtracsysFusionTrackQt_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawAtracsysFusionTrackQtExample ${REQUIRED_CISST_LIBRARIES})

  else (sawAtracsysFusionTrack_FOUND AND CISST_HAS_QT)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawAtracsysFusionTrack and cisstQt")
  endif (sawAtracsysFusionTrack_FOUND AND CISST_HAS_QT)

endif (cisst_FOUND_AS_REQUIRED)
