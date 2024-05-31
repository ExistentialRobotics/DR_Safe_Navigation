cmake_minimum_required(VERSION 3.24)

project(erl_common
        LANGUAGES CXX
        VERSION 0.1.0
        DESCRIPTION "erl_common is a C++ module commonly used by other modules")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    include(cmake/erl_common_tools.cmake)
endif ()
erl_project_setup(
        ${PROJECT_NAME}
        CATKIN_COMPONENTS roscpp
)
erl_setup_ros()
erl_catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        DEPENDS ${${PROJECT_NAME}_DEPENDS}  # non-catkin dependencies are collected via erl_find_package
        CFG_EXTRAS erl_common_tools.cmake)

#######################################################################################################################
# BUILD
#######################################################################################################################

########################
# define the libraries #
########################
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}) # this allows namespace style, erl_common::erl_common
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_include_directories(${PROJECT_NAME} SYSTEM
        PUBLIC ${MKL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(${PROJECT_NAME}
        PUBLIC fmt::fmt
        PUBLIC ${OpenCV_LIBRARIES}
        PUBLIC yaml-cpp
        PUBLIC ${absl_LIBRARIES}
        PUBLIC pthread OpenMP::OpenMP_CXX
        PUBLIC Eigen3::Eigen ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES}
        PUBLIC ${catkin_LIBRARIES})

#######################################################################################################################
# INSTALL
#######################################################################################################################
erl_install(LIBRARIES ${PROJECT_NAME})

#######################################################################################################################
# done
#######################################################################################################################
erl_mark_project_found(${PROJECT_NAME})
