pybind_add_module(pyErlMap MODULE bindings.cpp)
target_link_libraries(pyErlMap PRIVATE ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})

