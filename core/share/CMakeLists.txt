cmake_minimum_required (VERSION 3.10)
project (atracsys_config VERSION 1.1.1)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (DIRECTORY
         "${atracsys_config_SOURCE_DIR}"
         DESTINATION share/sawAtracsysFusionTrack
         COMPONENT sawAtracsysFusionTrack-Share
         PATTERN "CMakeLists.txt" EXCLUDE)
