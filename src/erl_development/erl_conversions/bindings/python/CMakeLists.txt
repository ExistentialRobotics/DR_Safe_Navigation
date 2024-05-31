set(bindings_python_version 3.6)

pybind_add_module(pyErlConversions MODULE ${CMAKE_CURRENT_SOURCE_DIR}/bindings.cpp)
#target_link_libraries(pyErlConversions PRIVATE erl_conversions)
  
#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#FIND_PACKAGE(pybind11 REQUIRED)
#if (pybind11_FOUND)
#    set(bindings_python_version 3.6)
#    pybind11_add_module(pyBarebones MODULE bindings.cpp)
#    set_target_properties(pyBarebones PROPERTIES PREFIX "" SUFFIX ".so")
#    target_link_libraries(pyBarebones PRIVATE erl_barebones)
#endif()