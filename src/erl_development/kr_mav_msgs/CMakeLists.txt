cmake_minimum_required(VERSION 3.10)
project(kr_mav_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)
find_package(Eigen3 REQUIRED)
if (NOT EIGEN3_INCLUDE_DIRS AND TARGET Eigen3::Eigen)
  get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
endif ()

add_message_files(
  DIRECTORY
  msg
  FILES
  AuxCommand.msg
  Corrections.msg
  MotorRPM.msg
  OutputData.msg
  PWMCommand.msg
  PositionCommand.msg
  SO3Command.msg
  Serial.msg
  StatusData.msg
  TRPYCommand.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  DEPENDS
  EIGEN3)
