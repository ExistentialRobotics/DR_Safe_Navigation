cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sfm_plugin)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
        roscpp
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
if (NOT DEFINED EIGEN3_VERSION_STRING) # needed by gazebo but Eigen may not set it
    set(EIGEN3_VERSION_STRING ${Eigen3_VERSION})
endif ()
find_package(gazebo REQUIRED)


include_directories(include)
include_directories(SYSTEM
        /usr/local/include  #to find lightsfm
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
        ${GAZEBO_LIBRARY_DIRS}
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES PedestrianSFMPlugin
        CATKIN_DEPENDS gazebo_ros roscpp
)


add_library(PedestrianSFMPlugin src/PedestrianSFMPlugin.cpp)
target_link_libraries(PedestrianSFMPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS
        PedestrianSFMPlugin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
