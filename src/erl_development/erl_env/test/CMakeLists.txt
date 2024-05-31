## Build Tests for Environment.

# Tests
catkin_add_gtest(test_environment_2d ${CMAKE_CURRENT_SOURCE_DIR}/gtest/test_environment_2d.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/gtest)
TARGET_LINK_LIBRARIES(test_environment_2d ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}  ${catkin_LIBRARIES} gtest gmock)

catkin_add_gtest(test_environment_3d ${CMAKE_CURRENT_SOURCE_DIR}/gtest/test_environment_3d.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/gtest)
TARGET_LINK_LIBRARIES(test_environment_3d ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}  ${catkin_LIBRARIES} gtest gmock)

catkin_add_gtest(test_environment_se2 ${CMAKE_CURRENT_SOURCE_DIR}/gtest/test_environment_se2.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/gtest)
TARGET_LINK_LIBRARIES(test_environment_se2 ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${catkin_LIBRARIES} gtest gmock)

catkin_add_gtest(test_environment_fas ${CMAKE_CURRENT_SOURCE_DIR}/gtest/test_environment_fas.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/gtest)
TARGET_LINK_LIBRARIES(test_environment_fas ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}  ${catkin_LIBRARIES} gtest gmock)

catkin_add_nosetests(${CMAKE_CURRENT_SOURCE_DIR}/pytests/test_env_2d.py)