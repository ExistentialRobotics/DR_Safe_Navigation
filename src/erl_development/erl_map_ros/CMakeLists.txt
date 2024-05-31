cmake_minimum_required(VERSION 3.1)
project(erl_map_ros)

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


##########################################################################################################
FIND_PACKAGE(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system iostreams filesystem program_options date_time timer chrono serialization unit_test_framework)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})


cs_add_executable(gridmap_server src/gridmap_server.cpp)
target_link_libraries(gridmap_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(online_gridmap_server src/online_gridmap_server.cpp)
target_link_libraries(online_gridmap_server  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(convert_legacy_gridmap src/convert_legacy_gridmap.cpp)
target_link_libraries(convert_legacy_gridmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(convert_mesh_gridmap src/convert_mesh_gridmap.cpp)
target_link_libraries(convert_mesh_gridmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})

cs_add_executable(test_depth_sim src/test_depth_sim.cpp)
target_link_libraries(test_depth_sim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##########################################################################################################
cs_install()
cs_export()
##########################################################################################################



















