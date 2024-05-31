CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
project(erl_env_python)

pybind_add_module(pyErlEnv MODULE bindings.cpp)
target_link_libraries(pyErlEnv  PRIVATE ${Boost_LIBRARIES} ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
