cmake_minimum_required(VERSION 3.0.2)
project(ref_gvn_lite)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  erl_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ref_gvn_ros
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package()
