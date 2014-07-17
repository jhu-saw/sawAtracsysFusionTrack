#
# (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.
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
                              cisstCommonQt
                              cisstVector
                              cisstVectorQt
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawNDITracker has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawAtracsysFusionTrack REQUIRED)

  if (sawAtracsysFusionTrack_FOUND)

    # sawAtracsysFusionTrack configuration
    include_directories (${sawAtracsysFusionTrack_INCLUDE_DIR})
    link_directories (${sawAtracsysFusionTrack_LIBRARY_DIR})

    add_executable (sawAtracsysFusionTrackQtExample mainQt.cpp)
    set_property (TARGET sawAtracsysFusionTrackQtExample PROPERTY FOLDER "sawAtracsysFusionTrack")

    # link against non cisst libraries and saw components
    target_link_libraries (sawAtracsysFusionTrackQtExample
                           ${sawAtracsysFusionTrack_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawAtracsysFusionTrackQtExample ${REQUIRED_CISST_LIBRARIES})

  endif (sawAtracsysFusionTrack_FOUND)

endif (cisst_FOUND)
