cmake_minimum_required(VERSION 3.0.2)
project(erl_moving_obstacle)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()

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
