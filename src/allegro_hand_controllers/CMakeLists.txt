cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_controllers)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  allegro_hand_driver
#  bhand
  libpcan
  roscpp
  sensor_msgs
  ll4ma_kdl
# roslaunch  # Disabled due to testing
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_driver libpcan roscpp sensor_msgs ll4ma_kdl # bhand
  DEPENDS Eigen
)
#add_library(allegro_node src/kdl_controller.cpp)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(allegro_node src/allegro_node.cpp src/kdl_controller.cpp)

## Declare executables
add_executable(allegro_node_pd src/allegro_node_pd.cpp)
target_link_libraries(allegro_node_pd allegro_node ${catkin_LIBRARIES})

# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  allegro_node
  allegro_node_pd
  #kdl_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)