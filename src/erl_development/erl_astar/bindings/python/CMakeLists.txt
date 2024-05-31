CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
project(erl_astar_python)

pybind_add_module(pyAStar MODULE ${CMAKE_CURRENT_SOURCE_DIR}/bindings.cpp)
target_link_libraries(pyAStar PRIVATE erl_astar)


#FIND_PACKAGE(pybind11 REQUIRED)
#if (pybind11_FOUND)
#  pybind11_add_module(pyAStar MODULE bindings.cpp)
#  set_target_properties(pyAStar PROPERTIES PREFIX "" SUFFIX ".so")
#  target_link_libraries(pyAStar PRIVATE ${Boost_LIBRARIES} ${YAMLCPP_LIBRARIES})
#endif()
