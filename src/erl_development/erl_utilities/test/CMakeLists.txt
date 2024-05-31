# C++ Unit Tests
catkin_add_gtest(test_poly gtest/test_poly_utils.cpp)
TARGET_LINK_LIBRARIES(test_poly ${Boost_LIBRARIES} gtest gmock)

catkin_add_gtest(test_polynomial gtest/test_trajectories.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/trajectories/polynomial.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/trajectories/piecewise_trajectory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/trajectories/piecewise_polynomial.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

catkin_add_gtest(test_geom_utils gtest/test_geom_utils.cpp)
TARGET_LINK_LIBRARIES(test_geom_utils gtest gmock)

# Python Unit Tests
catkin_add_nosetests(${CMAKE_CURRENT_SOURCE_DIR}/pytests/test_trajectories.py)
