cmake_minimum_required(VERSION 3.1)
project(erl_map)

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
if (NOT DEFINED EIGEN3_INCLUDE_DIR)  # latest Eigen3 uses newer CMake syntax: Eigen3::Eigen
    get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})  # for catkin
endif ()
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# CGAL NOTE::::: This needs to appear before Boost, because it overwrites the requested Boost libraries. SIGH.
find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})
INCLUDE_DIRECTORIES(${CGAL_INCLUDE_DIRS})

# Boost
FIND_PACKAGE(Boost COMPONENTS serialization iostreams system filesystem REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

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
cs_export(DEPENDS Boost CGAL YAMLCPP
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)
##########################################################################################################



















