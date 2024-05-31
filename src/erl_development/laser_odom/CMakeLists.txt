cmake_minimum_required(VERSION 3.0)
project(laser_odom)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# This forces the -march=native flag to be removed
# PCL crashes if  -march=native is added
#string(REPLACE "-march=native" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
#message( STATUS "***PCL CMAKE FLAGS***:" ${CMAKE_CXX_FLAGS})

find_package(Eigen3 REQUIRED)
if (EXISTS /usr/lib/libOpenGL.so)
  set(OpenGL_GL_PREFERENCE "GLVND")
else ()
  set(OpenGL_GL_PREFERENCE "LEGACY")
endif ()
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag angles tf sensor_msgs 
  geometry_msgs visualization_msgs pcl_ros laser_geometry)

catkin_package(
   CATKIN_DEPENDS roscpp rosbag angles tf sensor_msgs pcl_ros laser_geometry
  geometry_msgs visualization_msgs)

include_directories(${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(matcher src/matcher.cpp)
target_link_libraries(matcher ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(laser_odom_bag src/laser_odom_bag.cpp)
target_link_libraries(laser_odom_bag matcher ${catkin_LIBRARIES})

add_executable(laser_odom src/laser_odom.cpp)
target_link_libraries(laser_odom matcher ${catkin_LIBRARIES})

add_executable(truncate_scan src/truncate_scan.cpp)
target_link_libraries(truncate_scan ${catkin_LIBRARIES})
