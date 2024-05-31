cmake_minimum_required(VERSION 3.1)
project(erl_astar_ros)

# Set Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


# Add External Dependencies
find_package(PkgConfig)
find_package(catkin_simple REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system iostreams filesystem program_options date_time timer chrono serialization unit_test_framework)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})

catkin_simple()

# Set link libraries
LIST(APPEND LINK_LIBS
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(erl_astar_2d_online_node src/erl_astar_2d_online_node.cpp)
target_link_libraries(erl_astar_2d_online_node ${LINK_LIBS})

add_executable(erl_astar_costmap2d_online_node src/erl_astar_costmap2d_online_node.cpp)
target_link_libraries(erl_astar_costmap2d_online_node ${LINK_LIBS})

cs_install()
cs_export()

