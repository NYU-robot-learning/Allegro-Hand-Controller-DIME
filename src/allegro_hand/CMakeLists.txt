cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ll4ma_kdl_DIR "/home/grail/catkin_ws/src/allegro_hand_ros_v4/src/ll4ma_kdl/")

find_package(catkin REQUIRED COMPONENTS
  allegro_hand_driver
  libpcan
  roscpp
  sensor_msgs
  ll4ma_kdl
#  bhand
# roslaunch  # Disabled due to testing
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_driver libpcan roscpp sensor_msgs # ll4ma_kdl bhand 
#  DEPENDS
)

###########
## Build ##
###########
message( ${catkin_INCLUDE_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare executables


# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)

## Setup for Python package
catkin_python_setup()
