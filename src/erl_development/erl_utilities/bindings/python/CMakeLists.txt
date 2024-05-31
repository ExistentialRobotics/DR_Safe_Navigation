
pybind_add_module(pyErlUtils MODULE bindings.cpp
            ${PROJECT_SOURCE_DIR}/src/trajectories/piecewise_polynomial.cpp
            ${PROJECT_SOURCE_DIR}/src/trajectories/polynomial.cpp
            ${PROJECT_SOURCE_DIR}/src/trajectories/piecewise_trajectory.cpp)
target_link_libraries(pyErlUtils PRIVATE ${Boost_LIBRARIES} ${YAMLCPP_LIBRARIES})





#set(PYTHON_VERSION 3)
#FIND_PACKAGE(PythonInterp 3 REQUIRED)
#FIND_PACKAGE(PythonLibs 3 REQUIRED)
#set(PYBIND11_PYTHON_VERSION 3)
#set(PYTHON_VERSION "3.6" CACHE STRING "Python Version to use")
#find_package(PythonLibsNew EXACT ${PYTHON_VERSION} MODULE REQUIRED)
#set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION})


#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#project(erl_utils_python)

#FIND_PACKAGE(pybind11 REQUIRED)
#if (pybind11_FOUND)
#    set(bindings_python_version 3.6)
#    pybind11_add_module(pyUtils MODULE bindings.cpp
#            ${CMAKE_SOURCE_DIR}/src/trajectories/piecewise_polynomial.cpp
#            ${CMAKE_SOURCE_DIR}/src/trajectories/polynomial.cpp
#            ${CMAKE_SOURCE_DIR}/src/trajectories/piecewise_trajectory.cpp)
#    set_target_properties(pyUtils PROPERTIES PREFIX "" SUFFIX ".so")
#    target_link_libraries(pyUtils PRIVATE ${Boost_LIBRARIES} ${YAMLCPP_LIBRARIES})
#endif()
