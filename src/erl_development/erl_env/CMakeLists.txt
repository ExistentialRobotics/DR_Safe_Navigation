cmake_minimum_required(VERSION 3.1)
project(erl_env)

################################################################################
# user build settings
#option(BUILD_TESTS "Build the Tests" OFF)
option(BUILD_PYTHON "Build Python Bindings" ON)
################################################################################

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


# Set Output Folders
#SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)


# Find dependencies
INCLUDE_DIRECTORIES(include)

FIND_PACKAGE(Eigen3 REQUIRED)
if (NOT DEFINED EIGEN3_INCLUDE_DIR)  # latest Eigen3 uses newer CMake syntax: Eigen3::Eigen
    get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})  # for catkin
endif ()
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
set(Boost_DEBUG 1)
FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
IF (YAMLCPP_FOUND)
    MESSAGE(STATUS "Found yaml-cpp.")
    #MESSAGE(STATUS "YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
    #MESSAGE(STATUS "YAMLCPP_LIBRARIES: ${YAMLCPP_LIBRARIES}")
    #MESSAGE(STATUS "YAMLCPP_LIBRARY_DIRS: ${YAMLCPP_LIBRARY_DIRS}")
ELSE ()
    MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF ()


##################################################################################
# Catkin Simple. Needed for finding this package by others.
FIND_PACKAGE(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


##################################################################################
if (CATKIN_ENABLE_TESTING)
    #enable_testing()
    add_subdirectory(test)
endif (CATKIN_ENABLE_TESTING)

##################################################################################
if (BUILD_PYTHON)
    add_subdirectory(bindings/python)
endif (BUILD_PYTHON)

##########################################################################################################
cs_install()
cs_export()
##########################################################################################################


