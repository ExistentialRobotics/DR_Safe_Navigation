
cmake_minimum_required(VERSION 3.1)
project(erl_visualization)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


################################################################################
FIND_PACKAGE(catkin REQUIRED COMPONENTS rviz roscpp)
FIND_PACKAGE(catkin_simple REQUIRED)
FIND_PACKAGE(cmake_modules)
FIND_PACKAGE(Eigen3 REQUIRED)
if (NOT DEFINED EIGEN3_INCLUDE_DIR)  # latest Eigen3 uses newer CMake syntax: Eigen3::Eigen
  get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})  # for catkin
endif ()
include_directories(include ${EIGEN3_INCLUDE_DIRS})
#include_directories(${catkin_INCLUDE_DIRS})
#link_directories(${catkin_LIBRARY_DIRS})
find_package(OpenCV REQUIRED)
set(EIGEN3_VERSION_STRING "3.4.90")  # latest Eigen3 (3.4.90) does not provide this variable anymore.
find_package(gazebo REQUIRED)  # a warning about protobuf version may appear, but it is a cmake bug.
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})


# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# YAML-CPP
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()


#FIND_PACKAGE(VTK REQUIRED)
#FIND_PACKAGE(motion_primitive_library REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS})
################################################################################

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

catkin_simple()

set(SOURCE_FILES
        src/erl_rviz_plugins/point_trajectory_visual.cpp
        src/erl_rviz_plugins/point_trajectory_display.cpp
        src/erl_rviz_plugins/point_trajectory_array_display.cpp
        src/erl_rviz_plugins/primitive_trajectory_visual.cpp
        src/erl_rviz_plugins/primitive_trajectory_display.cpp
  ${MOC_FILES}
)

cs_add_library(erl_rviz_plugins ${SOURCE_FILES})
target_link_libraries(erl_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
#
#cs_add_library(erl_gazebo_odometry_plugin src/erl_gazebo_plugins/gazebo_odometry_plugin.cpp)
#target_link_libraries(erl_gazebo_odometry_plugin ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(erl_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})
#
#cs_add_library(erl_gazebo_imu_plugin src/erl_gazebo_plugins/gazebo_imu_plugin.cpp)
#target_link_libraries(erl_gazebo_imu_plugin ${catkin_LIBRARIES} )
#add_dependencies(erl_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})


cs_add_executable(test_point_trajectory_display test/test_point_trajectory_display.cpp)
cs_add_executable(test_primitive_trajectory_display test/test_primitive_trajectory_display.cpp)
#target_link_libraries(test_point_trajectory)
#
#cs_add_executable(mesh_visualization src/erl_rviz_markers/mesh_visualization.cpp)
#target_link_libraries(mesh_visualization ${catkin_LIBRARIES})

cs_add_executable(mesh_msg_visualization src/erl_rviz_markers/mesh_msg_visualization.cpp)
target_link_libraries(mesh_msg_visualization ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(grid_visualization src/erl_rviz_markers/grid_visualization.cpp)
target_link_libraries(grid_visualization ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(grid_msg_visualization src/erl_rviz_markers/grid_msg_visualization.cpp)
target_link_libraries(grid_msg_visualization ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(landmark_msg_visualization src/erl_rviz_markers/landmark_msg_visualization.cpp)
target_link_libraries(landmark_msg_visualization ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})


cs_add_executable(multi_belief_visualization src/erl_rviz_markers/multi_belief_visualization.cpp)
target_link_libraries(landmark_msg_visualization ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(grid_msg_publisher src/erl_rviz_markers/grid_msg_publisher.cpp)
target_link_libraries(grid_msg_publisher ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

cs_install_scripts(scripts/range_viewer.py)
cs_install_scripts(scripts/map_viewer.py)

catkin_python_setup()  # This is needed to install the Python Modules.

cs_install()

cs_export()
