cmake_minimum_required(VERSION 2.8.3)
project(baxter_planit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  planit
  baxter_core_msgs
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
   scripts/demo_table
   scripts/demo_tasks
   scripts/demo_objs_6
   scripts/init_rand
   nodes/baxter_planner
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
