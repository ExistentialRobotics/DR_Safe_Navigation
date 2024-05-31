`erl_common_tools.cmake`
============

This is a CMake module that provides some utilities for configuring C++ projects developed in ERL.

# Getting Started

1. Clone this repository into your project's `src` directory
    ```shell
    cd src
    git clone https://github.com/ExistentialRobotics/erl_common.git
    ```
2. Include the module in your project's `CMakeLists.txt` file
    ```cmake
    include(erl_common/cmake/erl_project_setup.cmake)
    ```
    - Available options:
        - `BUILD_PYTHON`: Build Python bindings for the project (default: `ON`)
        - `USE_LAPACK`: Use LAPACK for linear algebra, e.g. Eigen (default: `ON`)
        - `USE_INTEL_MKL`: Use Intel MKL, will set `USE_LAPACK` to `ON` (default: `ON`)
        - `USE_AOCL`: Use AMD Optimizing CPU Library (default: `OFF`)
        - `USE_SINGLE_THREADED_BLAS`: Use single-threaded BLAS when you have BLAS call in your threaded code (
          default: `ON`)
        - `BUILD_TEST`: Config GoogleTest (default: `ON`)
        - `Python3_ROOT_DIR`: Path to Python3 installation (default: given by CMake `FindPython3` module)
          Or you can load the `erl_common` module directly:
    ```cmake
    add_subdirectory(erl_common)
    ```
3. Then, you can use the utilities provided by `erl_common_tools.cmake` in other `CMakeLists.txt`. For example:
    ```cmake
    cmake_minimum_required(VERSION 3.24)
    
    project(erl_covariance
            LANGUAGES CXX
            VERSION 0.1.0
            DESCRIPTION "erl_covariance is a C++ library of kernel functions")
    message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")
    
    if (NOT COMMAND erl_project_setup)
       find_package(erl_common REQUIRED)
    endif ()
    set(${PROJECT_NAME}_CATKIN_COMPONENTS erl_common)
    erl_project_setup()
    erl_setup_ros()
    erl_catkin_package(
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME} py${PROJECT_NAME}
            CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
            DEPENDS ${${PROJECT_NAME}_DEPENDS})
    # ...
    ```
    Full example can be found in [erl_covariance](https://github.com/ExistentialRobotics/erl_covariance/blob/main/CMakeLists.txt).

# Utilities
## erl_project_setup
This a macro that does the following things:
- Detect ROS environment: call `erl_detect_ros`
- Setup compiler flags: call `erl_setup_compiler`
- Setup LAPACK: call `erl_setup_lapack`
- Setup common packages like Eigen, Boost, OpenCV, etc.: call `erl_setup_common_packages`
- Setup Python: call `erl_setup_python`
- Setup test: call `erl_setup_test`

## erl_setup_ros
This is a macro that does the following things:
- find catkin with required catkin components if ROS1 is detected
- setup python for catkin if `setup.py` is found in the package root directory
- ROS2 is not supported yet

## erl_catkin_package
This is a macro that does the following things:
- define catkin package
- setup paths related to build and install: call `erl_set_project_paths`

## erl_add_python_package
This is a macro that does the following things:
- add pybind11 module if C++ source files are found in `python/binding`
- add three custom targets: `<project_name>_py_wheel`, `<project_name>_py_develop`, `<project_name>_py_install`

## erl_add_test
This is a macro that automatically detects and adds GoogleTest/PythonNose tests stored in `test/gtest` and 
`test/pytest` respectively. e.g.
```cmake
erl_add_test(
        LIBRARIES ${PROJECT_NAME}
)
```
**Notes**:
- For GoogleTest, when building with ROS1 activated, `catkin` cannot provide `gtest` with `main` entrypoint. So you need
  to add the main function in your gtest source code:
   ```c++
   #if defined(ERL_ROS_VERSION_1)
   int
   main(int argc, char **argv) {
       ::testing::InitGoogleTest(&argc, argv);
       return RUN_ALL_TESTS();
   }
   #endif
   ```

## erl_find_package
This is a macro that prints suggestions for installing a required package on different platforms. e.g.
```cmake
erl_find_package(
     PACKAGE OpenMP
     REQUIRED
     COMMANDS APPLE "try `brew install libomp`"
     COMMANDS UBUNTU_LINUX "try `sudo apt install libomp-dev`"
     COMMANDS ARCH_LINUX "try `sudo pacman -S openmp`")
```

## erl_find_path
This is a function to find a directory that contains required files. e.g.
```cmake
erl_find_path(
       OUTPUT LAPACKE_INCLUDE_DIR
       PACKAGE LAPACKE
       REQUIRED
       NAMES lapacke.h
       PATHS /usr/include /usr/local/include /usr/local/Cellar/lapack/*/include
       COMMANDS UBUNTU_LINUX "try `sudo apt install liblapacke-dev`"
       COMMANDS ARCH_LINUX "try `sudo pacman -S lapacke`")
```

## erl_install
This is a macro that generates install rules for executables, libraries, and Python packages. e.g.
```cmake
erl_install(
        LIBRARIES ${PROJECT_NAME}
        PYBIND_MODULES py${PROJECT_NAME})
```

## other utilities
- `erl_detect_ros`: detect ROS environment
- `erl_setup_compiler`: setup compiler flags
- `erl_setup_lapack`: setup LAPACK
- `erl_setup_common_packages`: setup common packages like Eigen, Boost, OpenCV, etc.
- `erl_setup_python`: setup Python
- `erl_setup_test`: setup test
- `erl_set_project_paths`: setup paths related to build and install
- `erl_mark_project_found`: mark a project as found
- `erl_os_release_info`: get OS release information such as Linux distribution name and version
- `erl_parse_key_value_pairs`: parse key-value pairs from a list of key, value, key, value, ...
- `erl_platform_based_message`: print a message based on the platform
- `erl_suggest_cmd_for_assert`: suggest a command when an assertion fails
- `erl_print_variables`: print all CMake variables, useful for debugging
- `erl_print_variable`: print a CMake variable, useful for debugging
