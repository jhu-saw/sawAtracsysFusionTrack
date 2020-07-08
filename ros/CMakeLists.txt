#
# (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8.3)

project (atracsys_ros)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstQt)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  # sawAtracsysFusionTrack has been compiled within cisst, we should find it automatically
  find_package (sawAtracsysFusionTrack 1.1.0)

  if (sawAtracsysFusionTrack_FOUND)

    include_directories (${CATKIN_DEVEL_PREFIX}/include
                         ${sawAtracsysFusionTrack_INCLUDE_DIR})

    link_directories (${sawAtracsysFusionTrack_LIBRARY_DIR})

    add_executable (atracsys_json src/atracsys_json.cpp)
    target_link_libraries (atracsys_json
                           ${sawAtracsysFusionTrack_LIBRARIES}
                           ${catkin_LIBRARIES})
    cisst_target_link_libraries (atracsys_json ${REQUIRED_CISST_LIBRARIES})

  else (sawAtracsysFusionTrack_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawAtracsysFusionTrack")
  endif (sawAtracsysFusionTrack_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
