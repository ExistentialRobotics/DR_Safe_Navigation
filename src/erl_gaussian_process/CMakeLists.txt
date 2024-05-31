cmake_minimum_required(VERSION 3.24)

project(erl_gaussian_process
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_gaussian_process is a C++ implementation of Gaussian Process.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_common REQUIRED)
endif ()
erl_project_setup(
        ${PROJECT_NAME}
        ERL_PACKAGES erl_common erl_covariance
)
erl_setup_ros()
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDS}
        DEPENDS ${${PROJECT_NAME}_DEPENDS})

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
