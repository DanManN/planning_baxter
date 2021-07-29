cmake_minimum_required(VERSION 2.8.3)
project(planit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_commander
  moveit_msgs
  shape_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_message_files(
  DIRECTORY msg
  FILES
  PercievedObject.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  PercieveObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/gazeboPerception.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
