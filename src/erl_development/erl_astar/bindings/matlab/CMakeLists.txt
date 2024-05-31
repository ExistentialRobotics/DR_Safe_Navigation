CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(erl_astar_matlab)

## MATLAB BINDINGS. 
## TODO: Add the remaining MEX bindings to this CMakeLists.txt file.
## TODO: Fix dependencies on catkin packages
############################################################################################################

# Inflate Map
add_custom_target(inflate_map_mex ALL
        COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++14' -I${CMAKE_SOURCE_DIR}/include -outdir ${CMAKE_CURRENT_SOURCE_DIR} inflate_map_mex.cpp"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bindings/matlab
        COMMENT "Building mex files..."
        VERBATIM
        )
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/inflate_map_mex.mexa64) 

# JPS 2D
add_custom_target(jps_2D_mex ALL
        COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++14' -I${CMAKE_SOURCE_DIR}/include -outdir ${CMAKE_CURRENT_SOURCE_DIR} jps_2D_mex.cpp ${CMAKE_SOURCE_DIR}/src/jps_2D.cpp"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bindings/matlab
        COMMENT "Building mex files..."
        VERBATIM
        )
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/jps_2D_mex.mexa64)         

# JPS 3D
add_custom_target(jps_3D_mex ALL
        COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++14' -I${CMAKE_SOURCE_DIR}/include -outdir ${CMAKE_CURRENT_SOURCE_DIR} jps_3D_mex.cpp ${CMAKE_SOURCE_DIR}/src/jps_3D.cpp"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bindings/matlab
        COMMENT "Building mex files..."
        VERBATIM
        )
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/jps_3D_mex.mexa64)        

# AStar 2D
add_custom_target(astar_2D_mex ALL
        COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++14' -I${CMAKE_SOURCE_DIR}/include \
        -I${CMAKE_SOURCE_DIR}/../erl_env/include -I${CMAKE_SOURCE_DIR}/../erl_utilities/include \
        -I${EIGEN3_INCLUDE_DIR} -l${YAMLCPP_LIBRARIES} -lboost_system \
        -outdir ${CMAKE_CURRENT_SOURCE_DIR} astar_2D_mex.cpp ${CMAKE_SOURCE_DIR}/src/astar_nx.cpp"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bindings/matlab
        COMMENT "Building mex files..."
        VERBATIM)
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/astar_2D_mex.mexa64)


############################################################################################################
#IF(USE_ROS)
#  MESSAGE(STATUS "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIRS} astar_ltl_mex.cpp ../astar_nx.cpp")
#  add_custom_target(astar_ltl_mex ALL
#    COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIRS} astar_ltl_mex.cpp ../astar_nx.cpp"
#    COMMAND mv astar_ltl_mex.mexa64 ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test
#    COMMENT "Building mex files..."
#    VERBATIM)
#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/astar_ltl_mex.mexa64)
#ELSE()
#  MESSAGE(STATUS "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIRS} astar_ltl_mex.cpp ../astar_nx.cpp")
#  add_custom_target(astar_ltl_mex ALL
#    COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIRS} astar_ltl_mex.cpp ../astar_nx.cpp"
#    COMMAND mv astar_ltl_mex.mexa64 ${PROJECT_SOURCE_DIR}/bin/
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test
#    COMMENT "Building mex files..."
#    VERBATIM)
#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/bin/astar_ltl_mex.mexa64)
#ENDIF()
############################################################################################################
############################################################################################################
#IF(USE_ROS)
#  MESSAGE("mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIR} -l${YAMLCPP_LIBRARIES} test_astar_SE2_mex.cpp ../astar_nx.cpp")
#  add_custom_target(astar_SE2_mex ALL
#    COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIR} -l${YAMLCPP_LIBRARIES} test_astar_SE2_mex.cpp ../astar_nx.cpp"
#    COMMAND mv test_astar_SE2_mex.mexa64 ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/astar_SE2_mex.mexa64
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test
#    COMMENT "Building mex files..."
#    VERBATIM)
#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/astar_SE2_mex.mexa64)
#ELSE()
#  MESSAGE("mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIR} -l${YAMLCPP_LIBRARIES} test_astar_SE2_mex.cpp ../astar_nx.cpp")
#  add_custom_target(astar_SE2_mex ALL
#    COMMAND eval "mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I../../include -I${EIGEN3_INCLUDE_DIR} -l${YAMLCPP_LIBRARIES} test_astar_SE2_mex.cpp ../astar_nx.cpp"
#    COMMAND mv test_astar_SE2_mex.mexa64 ${PROJECT_SOURCE_DIR}/bin/astar_SE2_mex.mexa64
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/test
#    COMMENT "Building mex files..."
#    VERBATIM)
#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/bin/astar_SE2_mex.mexa64)
#ENDIF()
############################################################################################################
