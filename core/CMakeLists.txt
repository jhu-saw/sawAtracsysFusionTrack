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
project (sawAtracsysFusionTrackCore VERSION 1.1.0)

find_package (cisst REQUIRED)
include (${CISST_USE_FILE})
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

set (sawAtracsysFusionTrack_DIR "${sawAtracsysFusionTrackCore_BINARY_DIR}/components")
add_subdirectory (examples)
add_subdirectory (share)

include (CPack)
cpack_add_component (sawAtracsysFusionTrack)
cpack_add_component (sawAtracsysFusionTrack-dev
                     DEPENDS sawAtracsysFusionTrack)
cpack_add_component (sawAtracsysFusionTrack-Examples
                     DEPENDS sawAtracsysFusionTrack)
cpack_add_component (sawAtracsysFusionTrack-Share
                     DEPENDS sawAtracsysFusionTrack)
