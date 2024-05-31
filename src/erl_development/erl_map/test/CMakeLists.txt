## Build Tests for Erl Map.


catkin_add_gtest(test_erl_map gtest/test_erl_map.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
TARGET_LINK_LIBRARIES(test_erl_map ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SERIALIZATION_LIBRARY} ${Boost_IOSTREAMS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${CGAL_LIBRARY} gtest gmock pthread)

catkin_add_gtest(test_bresenham gtest/test_bresenham.cpp)
TARGET_LINK_LIBRARIES(test_bresenham ${Boost_LIBRARIES} gtest gmock pthread)

catkin_add_nosetests(${CMAKE_CURRENT_SOURCE_DIR}/pytests/map_test.py WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)



#catkin_add_gtest(test_inflate_map gtest/test_inflate_map.cpp)
#TARGET_LINK_LIBRARIES(test_inflate_map ${Boost_LIBRARIES})

#catkin_add_gtest(test_grid_map gtest/test_grid_map.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#TARGET_LINK_LIBRARIES(test_grid_map ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} gtest gmock)

#catkin_add_gtest(test_mesh_map gtest/test_mesh_map.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#TARGET_LINK_LIBRARIES(test_mesh_map ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${CGAL_LIBRARY} gtest gmock)

#cs_add_executable(test_mesh_map test_mesh_map.cpp)
#TARGET_LINK_LIBRARIES(test_mesh_map ${Boost_LIBRARIES} ${CGAL_LIBRARY})

