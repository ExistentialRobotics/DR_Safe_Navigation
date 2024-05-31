cmake_minimum_required(VERSION 3.24)

project(erl_geometry
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_geometry is a C++ library of geometry data structures, algorithms and datasets.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()
erl_project_setup(
        ${PROJECT_NAME}
        ERL_PACKAGES erl_common)

#erl_find_package(
#        PACKAGE Qhull
#        REQUIRED
#        COMMANDS UBUNTU_LINUX "try `sudo apt-get install libqhull-dev`"
#        COMMANDS ARCH_LINUX "try `sudo pacman -S qhull`")
#get_target_property(Qhull_INCLUDE_DIRS Qhull::qhullcpp INTERFACE_INCLUDE_DIRECTORIES)
#get_target_property(qhull_r_location Qhull::qhull_r LOCATION)
#get_target_property(qhullcpp_location Qhull::qhullcpp LOCATION)
#list(APPEND Qhull_LIBRARIES ${qhull_r_location} ${qhullcpp_location})
#
#erl_find_package(
#        PACKAGE Open3D
#        REQUIRED
#        COMMANDS GENERAL "visit http://www.open3d.org/"
#        COMMANDS ARCH_LINUX "try `paru -S open3d`"
#)
#get_target_property(Open3D_LIBRARIES Open3D::Open3D LOCATION)

erl_setup_ros()
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDS}
        DEPENDS ${${PROJECT_NAME}_DEPENDS})   # non-catkin dependencies are collected via erl_find_package

#######################################################################################################################
# BUILD
#######################################################################################################################

########################
# define the libraries #
########################
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
if (catkin_LIBRARIES)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
else ()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_ERL_PACKAGES})
endif ()

#######################################################################################################################
# INSTALL
#######################################################################################################################
erl_install(LIBRARIES ${PROJECT_NAME})

#######################################################################################################################
# export variables
#######################################################################################################################
erl_mark_project_found(${PROJECT_NAME})
