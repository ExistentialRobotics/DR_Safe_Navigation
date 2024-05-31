CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(erl_astar)


################################################################################
# user build settings
option(BUILD_PYTHON "Build Python Bindings" ON)
option(BUILD_MATLAB "Build MEX Bindings" OFF)
################################################################################


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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


# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# YAML-CPP
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

##########################################################################################################
FIND_PACKAGE(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(erl_astar
  src/astar_nx.cpp)
target_link_libraries(erl_astar ${Boost_LIBRARIES})

cs_add_library(erl_jps
  src/jps_2D.cpp
  src/jps_3D.cpp)
target_link_libraries(erl_jps ${Boost_LIBRARIES})


##########################################################################################################
#IF(CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#ENDIF(CATKIN_ENABLE_TESTING)

IF(BUILD_PYTHON)
  add_subdirectory(bindings/python)
ENDIF(BUILD_PYTHON)

IF(BUILD_MATLAB)
  add_subdirectory(bindings/matlab)
ENDIF(BUILD_MATLAB)


##########################################################################################################
cs_install()
cs_export()
##########################################################################################################







#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#PROJECT(erl_astar)

#if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

## Set Output Paths
#SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

## Optional Build Flags
#option(USE_ROS "Build with Catkin" ON)
#option(BUILD_TESTS "Build the Tests" OFF)
#option(BUILD_PYTHON "Build Python Bindings" OFF)
#option(BUILD_MATLAB "Build MEX Bindings" OFF) # Do not build MATLAB by default.


#FIND_PACKAGE(Eigen3 REQUIRED)
#FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
#FIND_PACKAGE(PkgConfig REQUIRED)
#PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
#IF(YAMLCPP_FOUND)
#  MESSAGE(STATUS "Found yaml-cpp.")
#  MESSAGE(STATUS "YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
#  MESSAGE(STATUS "YAMLCPP_LIBRARIES: ${YAMLCPP_LIBRARIES}")
#  MESSAGE(STATUS "YAMLCPP_LIBRARY_DIRS: ${YAMLCPP_LIBRARY_DIRS}")
#ELSE()
#  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
#ENDIF()

###################################################################################
## Catkin based Build (Default)
#if (USE_ROS)
#  find_package(catkin_simple REQUIRED)
#  FIND_PACKAGE(catkin REQUIRED COMPONENTS erl_utilities erl_env)
#  catkin_simple() # Get Dependencies from Catkin Workspace

#  cs_install()
#  cs_export()

## Pure CMake Build
#else()
#  # Initialize Dependencies as Submodules
#  execute_process(COMMAND git submodule update --init
#                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
#  # Include Headers
#  include_directories(ext/erl_utilities/include)
#  include_directories(ext/erl_env/include)
#endif()

## Includes
#INCLUDE_DIRECTORIES(
#  include
#  ${EIGEN3_INCLUDE_DIRS}
#)
###################################################################################

###################################################################################
## Tests
#if (BUILD_TESTS)
#  enable_testing()
#  add_subdirectory(test)
#endif()

###################################################################################
## Python Bindings
#if (BUILD_PYTHON)
#  add_subdirectory(python) # Add AStar Library
#  add_subdirectory(ext/erl_env/python) # Add ENV Library
#  add_subdirectory(ext/erl_utilities/python) # Add Utilities Library
#endif()

###################################################################################
## MEX Bindings
#if (BUILD_MATLAB)
#  add_subdirectory(matlab)
#endif()
###################################################################################
## OLD
## Hashing Tests.
##add_executable(test_hashing src/test/test_hashing.cpp)
##target_link_libraries(test_hashing ${Boost_LIBRARIES})
##add_executable(test_hashing2 src/test/test_hashing2.cpp)
##target_link_libraries(test_hashing2 ${Boost_LIBRARIES})
##add_executable(test_hashing3 src/test/test_hashing3.cpp)
##target_link_libraries(test_hashing3 ${Boost_LIBRARIES})
###################################################################################

###################################################################################
## STUFF FOR COMPLING LIBRARIRES
### Set link libraries
##LIST(APPEND LINK_LIBS
##  #${YAMLCPP_LIBRARIES}
##  #${JPEG_LIBRARIES}
##  #${LIBXML2_LIBRARIES}
##  #${FFTW3_LIBRARIES}
##)


### Set sourcefiles
##LIST(APPEND SOURCEFILES
##  src/arastar_nx.cpp)

##LIST(APPEND HEADERFILES
##  include/arastar_nx.h
##  include/env_2d.h
##  include/env_int.h
##  include/map_nx.h
##  include/utils_nx.h)

#################################################################################
##ADD_LIBRARY(${PROJECT_NAME} STATIC ${HEADERFILES} ${SOURCEFILES})
##ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
##TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})
#################################################################################


