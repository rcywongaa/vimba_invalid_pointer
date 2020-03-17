cmake_minimum_required(VERSION 2.8.3)
project(vimba)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  LIBRARIES
    ${PROJECT_SOURCE_DIR}/lib/libVimbaC.so
    ${PROJECT_SOURCE_DIR}/lib/libVimbaCPP.so
    ${PROJECT_SOURCE_DIR}/lib/libVimbaImageTransform.so
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
)
