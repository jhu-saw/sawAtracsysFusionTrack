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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
	 cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  if (CISST_HAS_QT4)
    qt4_wrap_cpp (SAW_INTUITIVE_RESEARCH_KIT_QT_WRAP_CPP
                  ${sawAtracsysFusionTrack_HEADER_DIR}/mtsAtracsysFusionTrackToolQtWidget.h)
  else (CISST_HAS_QT4)
    set (CMAKE_AUTOMOC ON)
  endif (CISST_HAS_QT4)

  add_library (sawAtracsysFusionTrackQt
               ${sawAtracsysFusionTrack_HEADER_DIR}/mtsAtracsysFusionTrackToolQtWidget.h
			   ${sawAtracsysFusionTrack_HEADER_DIR}/sawAtracsysFusionTrackQtExport.h
               mtsAtracsysFusionTrackToolQtWidget.cpp
               ${SAW_INTUITIVE_RESEARCH_KIT_QT_WRAP_CPP})
  set_property (TARGET sawAtracsysFusionTrackQt PROPERTY FOLDER "sawAtracsysFusionTrack")
  cisst_target_link_libraries (sawAtracsysFusionTrackQt ${REQUIRED_CISST_LIBRARIES})

  # make sure the new library is known by the parent folder to add to the config file
  set (sawAtracsysFusionTrackQt_LIBRARIES sawAtracsysFusionTrackQt PARENT_SCOPE)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
