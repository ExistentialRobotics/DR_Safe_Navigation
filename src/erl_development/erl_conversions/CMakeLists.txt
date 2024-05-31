cmake_minimum_required(VERSION 3.1)
project(erl_conversions)

################################################################################
# user build settings
option(BUILD_PYTHON "Build Python Bindings" ON) # Set ON if you want to build this package with Python bindings
################################################################################

# set default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


# Find dependencies
INCLUDE_DIRECTORIES(include)

# Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system filesystem program_options date_time timer chrono serialization unit_test_framework)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# PCL
#FIND_PACKAGE(PCL 1.7 REQUIRED)
#INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
#LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
#ADD_DEFINITIONS(${PCL_DEFINITIONS})

# YAML-CPP
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
IF (YAMLCPP_FOUND)
    MESSAGE(STATUS "Found yaml-cpp.")
ELSE ()
    MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF ()

##########################################################################################################
FIND_PACKAGE(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


cs_add_library(conversions src/conversions.cpp)

cs_add_executable(msg_to_tf src/msg_to_tf.cpp)
# New Odom to TF
#cs_add_executable(odom_to_tf src/odom_to_tf.cpp)


catkin_python_setup()  # This is needed to install the Python Modules.
##########################################################################################################
IF(CATKIN_ENABLE_TESTING)
 add_subdirectory(test)
ENDIF(CATKIN_ENABLE_TESTING)

##########################################################################################################
IF(BUILD_PYTHON)
  add_subdirectory(bindings/python)
ENDIF(BUILD_PYTHON)


##########################################################################################################
cs_install()
cs_export(DEPENDS Boost YAMLCPP)
##########################################################################################################
