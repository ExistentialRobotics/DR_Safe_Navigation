cmake_minimum_required(VERSION 3.0)
project(erl_models)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  topic_tools
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs nav_msgs tf2_ros topic_tools
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})


add_executable(mesh_visualization src/mesh_visualization.cpp)
target_link_libraries(mesh_visualization ${catkin_LIBRARIES})
