cmake_minimum_required(VERSION 3.24)

project(erl_sdf_mapping
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_sdf_mapping is a C++ library for SDF estimation.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()
erl_project_setup(
        ${PROJECT_NAME}
        ERL_PACKAGES erl_common erl_gaussian_process erl_geometry
        CATKIN_COMPONENTS roscpp std_msgs sensor_msgs tf2_ros tf2_eigen cv_bridge message_generation
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf2_ros tf2_eigen cv_bridge message_runtime
)
erl_setup_ros(
        CATKIN_COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        SRV_FILES PredictSdf.srv
        MSG_DEPENDENCIES std_msgs
)
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
file(GLOB SRC_FILES src/*.cpp src/gpis/*.cpp)
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

if (ROS_ACTIVATED)
    add_executable(erl_sdf_mapping_node src/ros/erl_sdf_mapping_node.cpp)
    target_link_libraries(erl_sdf_mapping_node PUBLIC ${PROJECT_NAME}::${PROJECT_NAME})
endif ()

#######################################################################################################################
# INSTALL
#######################################################################################################################
erl_install(LIBRARIES ${PROJECT_NAME})

#######################################################################################################################
# export variables
#######################################################################################################################
erl_mark_project_found(${PROJECT_NAME})
