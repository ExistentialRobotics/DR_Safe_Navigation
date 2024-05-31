# Unit Tests for Conversions

catkin_add_gtest(test_conversion ${CMAKE_CURRENT_SOURCE_DIR}/gtest/test_trajectory_conversion.cpp)
target_link_libraries(test_conversion ${catkin_LIBRARIES})
