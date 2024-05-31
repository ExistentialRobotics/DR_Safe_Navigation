#######################################################################################################################
# Add this cmake module path to the cmake module path
#######################################################################################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(ERL_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "ERL CMake directory")

#######################################################################################################################
# target_link_libraries_system
#######################################################################################################################
function(target_link_libraries_system target)
    set(options PRIVATE PUBLIC INTERFACE)
    set(oneValueArgs)
    set(multiValueArgs)
    cmake_parse_arguments(ERL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    foreach (op ${options})
        if (ERL_${op})
            set(scope ${op})
        endif ()
    endforeach ()
    if (NOT scope)
        message(FATAL_ERROR "No scope is specified: PRIVATE, PUBLIC, or INTERFACE")
    endif ()
    set(libs ${ERL_UNPARSED_ARGUMENTS})

    foreach (lib ${libs})
        if (TARGET ${lib})
            get_target_property(lib_include_dirs ${lib} INTERFACE_INCLUDE_DIRECTORIES)

            get_target_property(include_dir ${lib} INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
            if (include_dir)
                list(APPEND lib_include_dirs ${include_dir})
            endif ()

            get_target_property(include_dir ${lib} INCLUDE_DIRECTORIES)
            if (include_dir)
                list(APPEND lib_include_dirs ${include_dir})
            endif ()
            unset(include_dir)

            if (lib_include_dirs)
                list(APPEND dirs_to_include ${lib_include_dirs})
            else ()
                message(WARNING "Cannot find [INTERFACE_]INCLUDE_DIRECTORIES for ${lib}")
            endif ()
            get_target_property(lib_type ${lib} TYPE)
            if (NOT lib_type STREQUAL "INTERFACE_LIBRARY")
                list(APPEND libs_to_link ${lib})
            endif ()
        else ()
            list(APPEND libs_to_link ${lib})
        endif ()
    endforeach ()

    if (dirs_to_include)
        list(REMOVE_DUPLICATES dirs_to_include)
        target_include_directories(${target} SYSTEM ${scope} ${dirs_to_include})
    endif ()
    if (libs_to_link)
        list(REMOVE_DUPLICATES libs_to_link)
        target_link_libraries(${target} ${scope} ${libs_to_link})
    endif ()
endfunction()

#######################################################################################################################
# erl_set_gtest_args
#######################################################################################################################
macro(erl_set_gtest_args _gtest_name _gtest_args)
    set(${_gtest_name}_GTEST_ARGS ${_gtest_args} CACHE STRING "GTest arguments for ${_gtest_name}" FORCE)
endmacro()

#######################################################################################################################
# erl_add_test
#######################################################################################################################
macro(erl_add_tests)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs LIBRARIES)
    cmake_parse_arguments(${PROJECT_NAME}_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (ERL_BUILD_TEST_${PROJECT_NAME})
        # add gtest
        file(GLOB GTEST_SOURCES ${${PROJECT_NAME}_TEST_DIR}/gtest/*.cpp)
        if (ROS_ACTIVATED AND ROS_VERSION STREQUAL "1" AND CATKIN_ENABLE_TESTING)
            foreach (file IN LISTS GTEST_SOURCES)
                get_filename_component(name ${file} NAME_WE)
                catkin_add_gtest(${name} ${file})
                target_include_directories(${name} PRIVATE ${catkin_INCLUDE_DIRS})
                target_link_libraries(${name} ${catkin_LIBRARIES} ${${PROJECT_NAME}_TEST_LIBRARIES})
                message(STATUS "Adding gtest ${name}")
            endforeach ()
        else ()
            foreach (file IN LISTS GTEST_SOURCES)
                get_filename_component(name ${file} NAME_WE)
                message(STATUS "Adding gtest ${name}")
                add_executable(${name} ${file})
                if (${PROJECT_NAME}_TEST_UNPARSED_ARGUMENTS)
                    cmake_parse_arguments("${name}" "" "" "${name}_LIBRARIES" ${${PROJECT_NAME}_TEST_UNPARSED_ARGUMENTS})
                    if (${name}_${name}_LIBRARIES)
                        message(STATUS "additional LIBRARIES for ${name}: ${${name}_${name}_LIBRARIES}")
                    endif ()
                    if (${name}_UNPARSED_ARGUMENTS)
                        set(${PROJECT_NAME}_TEST_UNPARSED_ARGUMENTS ${${name}_UNPARSED_ARGUMENTS})
                    endif ()
                endif ()
                target_link_libraries(${name} ${${PROJECT_NAME}_TEST_LIBRARIES} GTest::Main ${${name}_${name}_LIBRARIES})
                #                if (DEFINED ${name}_GTEST_ARGS)
                #                    gtest_discover_tests(
                #                            ${name}
                #                            EXTRA_ARGS ${${name}_GTEST_ARGS}
                #                            WORKING_DIRECTORY ${${PROJECT_NAME}_TEST_DIR}
                #                            DISCOVERY_TIMEOUT 60
                #                    )
                #                else ()
                #                    gtest_discover_tests(
                #                            ${name}
                #                            WORKING_DIRECTORY ${${PROJECT_NAME}_TEST_DIR}
                #                            DISCOVERY_TIMEOUT 60
                #                    )
                #                endif ()
            endforeach ()
        endif ()
        # TODO: add python tests
    endif ()
endmacro()

#######################################################################################################################
# Function erl_os_release_info - Determine and return OS name and version
# Borrowed from https://github.com/intel/compute-runtime/blob/master/os_release_info.cmake
#
# Args:
# 1.  the name of a variable to receive os_name
# 2.  the name of a variable to receive os_version
#
# Return values: (Quotation marks are always stripped).
# Upon failure, return values are null strings.
#
# Examples:
#   os_name           os_version
#   --------------    -------
#   clear-linux-os    21180          (Changes twice daily)
#   ubuntu            12.04  16.04  17.10  18.04
#   fedora            27
#   centos            6.9  7.4.1708
#
# Potential sources are tried (in order of preference) until a
# suitable one is found.

# Implementation documentation:
#
# The potential sources, in order, are as follows.
# - /etc/centos-release
#       Centos 7 also has /etc/os-release.  File /etc/os-release is less
#       precise about the Centos version (e.g., "7" instead of "7.4.1708").
#       For that reason, this file is checked first.
#       Examples:
#       CentOS release 6.9 (Final)
#       CentOS Linux release 7.4.1708 (Core)
# - /usr/lib/os-release
#       Present for Clear Linux, modern Fedora, and Ubuntu since some time
#       between 14.04 and 16.04.  The ID and VERSION_ID values are used.
#       Examples:
#       ID=clear-linux-os      VERSION_ID=21180
#       ID=fedora              VERSION_ID=27
#       ID=ubuntu              VERSION_ID="14.04"
#       ID=ubuntu              VERSION_ID="16.04"
#       ID="ubuntu"            VERSION_ID="17.10"
# - /etc/os-release - Same form as (sometimes a link to) /usr/lib/os-release
#       ID="Ubuntu"            VERSION_ID="12.04"
#       ID="Ubuntu"            VERSION_ID="14.04"
#           with a symbolic link: /etc/os-release -> ../usr/lib/os-release
#       ID="CentOS Linux"      VERSION_ID="7"    Also: ID_LIKE="rhel fedora"
# - /etc/lsb-release
#       For Centos, not too meaningful.
#       Other "OS"s are more reasonable:
#       DISTRIB_ID=Ubuntu      DISTRIB_RELEASE=12.04
#       DISTRIB_ID=Ubuntu      DISTRIB_RELEASE=14.04
#       DISTRIB_ID=Ubuntu      DISTRIB_RELEASE=17.10
#######################################################################################################################
function(erl_os_release_info _vn_id _vn_version_id _vn_codename)
    set(_var_id "")
    set(_var_version_id "")
    set(_var_codename "")

    if ("${_var_id}" STREQUAL "")
        set(file_path "/etc/centos-release")
        if (EXISTS "${file_path}")
            # Example: CentOS release 6.9 (Final)
            file(STRINGS "${file_path}" file_list LIMIT_COUNT 1)
            list(GET file_list 0 file_line)

            # Remove all parenthesized items.
            string(REGEX REPLACE "\\([^)]+\\)" "" file_line "${file_line}")

            # Extract start and end, discard optional "version" or "release"
            string(REGEX MATCH "^([A-Za-z0-9_]+)( +(version|release))? +(.*)$" _dummy "${file_line}")
            #                    1              2  3                    4

            set(_var_id "${CMAKE_MATCH_1}")
            set(_var_version_id "${CMAKE_MATCH_4}")
        endif ()
    endif ()

    if ("${_var_id}" STREQUAL "")
        if (EXISTS "/usr/lib/os-release")
            set(file_path "/usr/lib/os-release")
        elseif (EXISTS "/etc/os-release")
            set(file_path "/etc/os-release")
        else ()
            set(file_path "")
        endif ()

        if (NOT "${file_path}" STREQUAL "")
            file(STRINGS "${file_path}" data_list REGEX "^(ID|VERSION_ID|VERSION_CODENAME)=")

            # Look for lines like "ID="..." and VERSION_ID="..."
            foreach (_var ${data_list})
                if ("${_var}" MATCHES "^(ID)=(.*)$")
                    set(_var_id "${CMAKE_MATCH_2}")
                elseif ("${_var}" MATCHES "^(VERSION_ID)=(.*)$")
                    set(_var_version_id "${CMAKE_MATCH_2}")
                elseif ("${_var}" MATCHES "^(VERSION_CODENAME)=(.*)$")
                    set(_var_codename "${CMAKE_MATCH_2}")
                endif ()
            endforeach ()
        endif ()
    endif ()

    if ("${_var_id}" STREQUAL "")
        set(file_path "/etc/lsb-release")
        if (EXISTS "${file_path}")
            file(STRINGS "${file_path}" data_list REGEX "^(DISTRIB_ID|DISTRIB_RELEASE|DISTRIB_CODENAME)=")

            # Look for lines like "DISTRIB_ID="..." and DISTRIB_RELEASE="..."
            foreach (_var ${data_list})
                if ("${_var}" MATCHES "^(DISTRIB_ID)=(.*)$")
                    set(_var_id "${CMAKE_MATCH_2}")
                elseif ("${_var}" MATCHES "^(DISTRIB_RELEASE)=(.*)$")
                    set(_var_version_id "${CMAKE_MATCH_2}")
                elseif ("${_var}" MATCHES "^(DISTRIB_CODENAME)=(.*)$")
                    set(_var_codename "${CMAKE_MATCH_2}")
                endif ()
            endforeach ()
        endif ()
    endif ()

    string(TOUPPER "${_var_id}" "_var_id")

    string(STRIP "${_var_id}" _var_id)
    string(STRIP "${_var_version_id}" _var_version_id)
    string(STRIP "${_var_codename}" _var_codename)

    # Remove any enclosing quotation marks
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" _var_id "${_var_id}")
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" _var_version_id "${_var_version_id}")
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" _var_codename "${_var_codename}")

    if (NOT "${_vn_id}" STREQUAL "")
        set(${_vn_id} "${_var_id}" PARENT_SCOPE)
    endif ()

    if (NOT "${_vn_version_id}" STREQUAL "")
        set(${_vn_version_id} "${_var_version_id}" PARENT_SCOPE)
    endif ()

    if (NOT "${_vn_codename}" STREQUAL "")
        set(${_vn_codename} "${_var_codename}" PARENT_SCOPE)
    endif ()
endfunction()

#######################################################################################################################
# erl_parse_key_value_pairs
#######################################################################################################################
function(erl_parse_key_value_pairs _pairs prefix)
    # _pairs is read-only and string based, so we need to unpack it into a local variable.
    set(pairs ${${_pairs}})
    list(LENGTH pairs len)
    math(EXPR pair_counts "${len} / 2 - 1")
    math(EXPR left_over "${len} % 2")
    if (NOT "${left_over}" STREQUAL "0")
        message(FATAL_ERROR "pairs must be a list of pairs: <key> <value> <key> <value> ...")
    endif ()
    foreach (index RANGE ${pair_counts})
        math(EXPR key_index "${index} * 2")
        math(EXPR value_index "${key_index} + 1")
        list(GET pairs ${key_index} key)
        list(GET pairs ${value_index} value)
        set(${prefix}_${key} ${value} PARENT_SCOPE)
    endforeach ()
endfunction()

#######################################################################################################################
# erl_platform_based_message
#######################################################################################################################
function(erl_platform_based_message)
    set(options)
    set(oneValueArgs MSG_TYPE MSG_PREFIX)
    set(multiValueArgs MESSAGES)
    cmake_parse_arguments(ERL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (ERL_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "Unparsed arguments: ${ERL_UNPARSED_ARGUMENTS}")
    endif ()

    erl_parse_key_value_pairs(ERL_MESSAGES ERL_MESSAGES)
    string(TOUPPER "${CMAKE_HOST_SYSTEM_NAME}" ERL_PLATFORM)
    if (ERL_PLATFORM STREQUAL "LINUX")
        erl_os_release_info(LINUX_DISTRO LINUX_VERSION LINUX_CODENAME)
        if (DEFINED ERL_MESSAGES_${LINUX_DISTRO}_LINUX)
            set(ERL_PLATFORM ${LINUX_DISTRO}_LINUX)
        endif ()
    endif ()

    if (NOT DEFINED ERL_MESSAGES_${ERL_PLATFORM} AND DEFINED ERL_MESSAGES_GENERAL)
        set(ERL_PLATFORM GENERAL)
    endif ()

    message(${ERL_MSG_TYPE} "${ERL_MSG_PREFIX}: ${ERL_MESSAGES_${ERL_PLATFORM}}")
endfunction()

#######################################################################################################################
# erl_suggest_cmd_for_assert
#######################################################################################################################
function(erl_suggest_cmd_for_assert)
    set(options)
    set(oneValueArgs ASSERT MSG MSG_TYPE)
    set(multiValueArgs COMMANDS)
    cmake_parse_arguments(ERL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (ERL_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "Unparsed arguments: ${ERL_UNPARSED_ARGUMENTS}")
    endif ()

    if (NOT DEFINED ERL_ASSERT)
        message(FATAL_ERROR "ASSERT must be set")
    endif ()

    if (NOT ERL_MSG)
        message(FATAL_ERROR "MSG must be set")
    endif ()

    if (NOT ERL_MSG_TYPE)
        set(ERL_MSG_TYPE "FATAL_ERROR")
    endif ()

    if (NOT ERL_ASSERT)
        if (NOT DEFINED ERL_COMMANDS)
            message(${ERL_MSG_TYPE} "${ERL_MSG}: try ${ERL_GENERAL_CMD}")
        else ()
            erl_platform_based_message(
                    MSG_TYPE ${ERL_MSG_TYPE}
                    MSG_PREFIX ${ERL_MSG}
                    MESSAGES ${ERL_COMMANDS}
            )
        endif ()
    endif ()
endfunction()

#######################################################################################################################
# erl_find_package
#######################################################################################################################
macro(erl_find_package)
    set(options NO_RECORD)
    set(oneValueArgs PACKAGE PKGCONFIG)
    set(multiValueArgs COMMANDS)
    cmake_parse_arguments(ERL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (QUIET IN_LIST ERL_UNPARSED_ARGUMENTS)
        set(ERL_QUIET ON)
    else ()
        if (NOT DEFINED ${ERL_PACKAGE}_VERBOSE_ONCE)  # avoid printing multiple times
            set(ERL_QUIET OFF)
            set(${ERL_PACKAGE}_VERBOSE_ONCE ON CACHE BOOL "Flag of whether print detailed logs for ${ERL_PACKAGE}" FORCE)
        else ()
            set(ERL_QUIET ON)
        endif ()
    endif ()

    if (NOT ERL_QUIET)
        message(STATUS "=================================================================================================")
        if (ERL_PACKAGE STREQUAL "Python3")
            message(STATUS "To specify python interpreter, run `cmake -DPython3_ROOT_DIR=/path/to/python3_bin_folder ..`")
            message(STATUS "With CLion, Python_EXECUTABLE is set to the selected python interpreter")
            if (DEFINED Python_EXECUTABLE)
                get_filename_component(Python3_ROOT_DIR ${Python_EXECUTABLE} DIRECTORY)
            endif ()
        endif ()
        erl_platform_based_message(
                MSG_TYPE STATUS
                MSG_PREFIX "Finding package ${ERL_PACKAGE}, if not found"
                MESSAGES ${ERL_COMMANDS})
    endif ()

    if (ERL_PKGCONFIG)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(${ERL_PACKAGE} ${ERL_UNPARSED_ARGUMENTS})
    else ()
        find_package(${ERL_PACKAGE} ${ERL_UNPARSED_ARGUMENTS})
    endif ()

    if (REQUIRED IN_LIST ERL_UNPARSED_ARGUMENTS)
        set(MSG_TYPE "FATAL_ERROR")
    else ()
        set(MSG_TYPE "WARNING")
    endif ()

    if (${ERL_PACKAGE}_FOUND AND NOT ERL_QUIET)
        foreach (item IN ITEMS FOUND INCLUDE_DIR INCLUDE_DIRS LIBRARY LIBRARIES LIBS DEFINITIONS)
            if (DEFINED ${ERL_PACKAGE}_${item})
                message(STATUS "${ERL_PACKAGE}_${item}: ${${ERL_PACKAGE}_${item}}")
            endif ()
        endforeach ()
    endif ()

    if (ERL_NO_RECORD)
        if (NOT ERL_QUIET)
            message(STATUS "${ERL_PACKAGE} is not added to ${PROJECT_NAME}_DEPENDS")
        endif ()
    else ()
        list(APPEND ${PROJECT_NAME}_DEPENDS ${ERL_PACKAGE})
    endif ()

    unset(ERL_PKGCONFIG)
    unset(ERL_NO_RECORD)
    unset(ERL_PACKAGE)
    unset(ERL_COMMANDS)
endmacro()

#######################################################################################################################
# erl_find_path
#######################################################################################################################
function(erl_find_path)
    set(options)
    set(oneValueArgs OUTPUT PACKAGE)
    set(multiValueArgs COMMANDS)
    cmake_parse_arguments(ERL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (NOT DEFINED ERL_OUTPUT)
        set(ERL_OUTPUT "FILE_FOUND")
    endif ()

    message(STATUS "=================================================================================================")
    if (DEFINED ERL_PACKAGE)
        erl_platform_based_message(
                MSG_TYPE STATUS
                MSG_PREFIX "Checking file for ${ERL_PACKAGE}, if not found"
                MESSAGES ${ERL_COMMANDS})
    else ()
        erl_platform_based_message(
                MSG_TYPE STATUS
                MSG_PREFIX "Checking file, if not found"
                MESSAGES ${ERL_COMMANDS})
    endif ()

    find_path(
            ${ERL_OUTPUT}
            ${ERL_UNPARSED_ARGUMENTS}
    )
    get_filename_component(${ERL_OUTPUT} ${${ERL_OUTPUT}} REALPATH)
    message(STATUS "${ERL_OUTPUT}: ${${ERL_OUTPUT}}")
    set(${ERL_OUTPUT} ${${ERL_OUTPUT}} PARENT_SCOPE)
endfunction()

#######################################################################################################################
# erl_detect_ros
#######################################################################################################################
macro(erl_detect_ros)
    message(STATUS "=================================================================================================")
    if (DEFINED ENV{ROS_VERSION})
        set(ROS_ACTIVATED ON)
        set(ROS_VERSION "$ENV{ROS_VERSION}")
        set(ROS_DISTRO "$ENV{ROS_DISTRO}")
    elseif (DEFINED CATKIN_DEVEL_PREFIX)
        message(STATUS "catkin devel prefix: ${CATKIN_DEVEL_PREFIX}")
        set(ROS_ACTIVATED ON)
        set(ROS_VERSION "1")
    endif ()
    if (ROS_VERSION STREQUAL "1")
        message(STATUS "ROS_VERSION: ${ROS_VERSION}")
        add_definitions(-DERL_ROS_VERSION_1)
    elseif (ROS_VERSION STREQUAL "2")
        message(STATUS "ROS_VERSION: ${ROS_VERSION}")
        add_definitions(-DERL_ROS_VERSION_2)
    endif ()
endmacro()

#######################################################################################################################
# erl_set_project_paths
#######################################################################################################################
macro(erl_set_project_paths)

    # set project paths
    set(${PROJECT_NAME}_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}
            CACHE PATH "Root directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include
            CACHE PATH "Include directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
            CACHE PATH "Source directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test
            CACHE PATH "Test directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake
            CACHE PATH "CMake directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_PYTHON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python
            CACHE PATH "Python directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_PYTHON_BINDING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python/binding
            CACHE PATH "Python binding directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_SCRIPTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/scripts
            CACHE PATH "Script directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config
            CACHE PATH "Config directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/launch
            CACHE PATH "Launch directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/msg
            CACHE PATH "Msg directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_SRV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/srv
            CACHE PATH "Srv directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_ACTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/action
            CACHE PATH "Action directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_DOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc
            CACHE PATH "Doc directory of ${PROJECT_NAME}" FORCE)

    set(${PROJECT_NAME}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}
            CACHE PATH "Build directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_BUILD_PYTHON_DIR ${CMAKE_CURRENT_BINARY_DIR}/python
            CACHE PATH "Build python directory of ${PROJECT_NAME}" FORCE)
    set(${PROJECT_NAME}_BUILD_TEST_DIR ${CMAKE_CURRENT_BINARY_DIR}/test
            CACHE PATH "Build test directory of ${PROJECT_NAME}" FORCE)

    # set project devel & install paths
    if (ROS_VERSION STREQUAL "1")
        set(${PROJECT_NAME}_INSTALL_BINARY_DIR ${CATKIN_GLOBAL_BIN_DESTINATION}  # `bin`
                CACHE PATH "Binary directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_ETC_DIR ${CATKIN_PACKAGE_ETC_DESTINATION}
                CACHE PATH "Etc directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_INCLUDE_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                CACHE PATH "Include directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_LIBRARY_DIR ${CATKIN_PACKAGE_LIB_DESTINATION}
                CACHE PATH "Library directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_PYTHON_DIR ${CATKIN_PACKAGE_PYTHON_DESTINATION}
                CACHE PATH "Python directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_SHARE_DIR ${CATKIN_PACKAGE_SHARE_DESTINATION}
                CACHE PATH "Share directory of ${PROJECT_NAME}" FORCE)
        set(${PROJECT_NAME}_INSTALL_CMAKE_DIR ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
                CACHE PATH "CMake directory of ${PROJECT_NAME}" FORCE)

        if (NOT DEFINED ERL_CATKIN_INSTALL_DIR)
            set(ERL_CATKIN_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
            get_filename_component(ERL_CATKIN_WORKSPACE_DIR ${ERL_CATKIN_INSTALL_DIR} DIRECTORY)
            set(ERL_CATKIN_INSTALL_LIB_DIR ${ERL_CATKIN_INSTALL_DIR}/lib)
            set(ERL_CATKIN_INSTALL_PYTHON_DIR ${ERL_CATKIN_INSTALL_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

            set(ERL_CATKIN_DEVEL_DIR ${ERL_CATKIN_WORKSPACE_DIR}/devel)
            set(ERL_CATKIN_DEVEL_LIB_DIR ${ERL_CATKIN_DEVEL_DIR}/lib)
            set(ERL_CATKIN_DEVEL_PYTHON_DIR ${ERL_CATKIN_DEVEL_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
        endif ()
    elseif (ROS_VERSION STREQUAL "2")
        message(FATAL_ERROR "ROS2 is not supported yet")
    else ()
        include(GNUInstallDirs)
        include(CMakePackageConfigHelpers)
        # /usr/local/bin
        set(${PROJECT_NAME}_INSTALL_BINARY_DIR ${CMAKE_INSTALL_BINDIR}  # `bin`
                CACHE PATH "Path to ${PROJECT_NAME} binary directory during installation" FORCE)
        # /usr/local/etc/PROJECT_NAME
        set(${PROJECT_NAME}_INSTALL_ETC_DIR ${CMAKE_INSTALL_SYSCONFDIR}/${PROJECT_NAME}
                CACHE PATH "Path to ${PROJECT_NAME} etc directory during installation" FORCE)
        # /usr/local/include/PROJECT_NAME
        set(${PROJECT_NAME}_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}
                CACHE PATH "Path to ${PROJECT_NAME} include directory during installation" FORCE)
        # /usr/local/lib/PROJECT_NAME
        set(${PROJECT_NAME}_INSTALL_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
                CACHE PATH "Path to ${PROJECT_NAME} library directory during installation" FORCE)
        # /usr/local/lib/pythonX.Y/dist-packages/PROJECT_NAME
        set(${PROJECT_NAME}_INSTALL_PYTHON_DIR ${Python3_SITELIB}/${PROJECT_NAME}
                CACHE PATH "Path to ${PROJECT_NAME} python directory during installation" FORCE)
        # /usr/local/share/PROJECT_NAME
        set(${PROJECT_NAME}_INSTALL_SHARE_DIR ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
                CACHE PATH "Path to ${PROJECT_NAME} share directory during installation" FORCE)
        # /usr/local/share/PROJECT_NAME/cmake
        set(${PROJECT_NAME}_INSTALL_CMAKE_DIR ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
                CACHE PATH "Path to ${PROJECT_NAME} cmake directory during installation" FORCE)
    endif ()
endmacro()

#######################################################################################################################
# erl_print_variables
#######################################################################################################################
function(erl_print_variables)
    get_cmake_property(VARIABLE_NAMES VARIABLES)
    list(SORT VARIABLE_NAMES)
    foreach (VARIABLE_NAME ${VARIABLE_NAMES})
        message(STATUS "${VARIABLE_NAME}=${${VARIABLE_NAME}}")
    endforeach ()
endfunction()

#######################################################################################################################
# erl_print_variable
#######################################################################################################################
function(erl_print_variable VARIABLE_NAME)
    message(STATUS "${VARIABLE_NAME}=${${VARIABLE_NAME}}")
endfunction()

#######################################################################################################################
# erl_setup_compiler
#######################################################################################################################
macro(erl_setup_compiler)
    option(ERL_IGNORE_CONDA_LIBRARIES "Ignore conda libraries" ON)
    option(ERL_PRINT_HEADER_DEPENDENCIES "Print header dependencies" OFF)
    if (NOT $ENV{CONDA_PREFIX} STREQUAL "" AND ERL_IGNORE_CONDA_LIBRARIES)
        list(APPEND CMAKE_IGNORE_PREFIX_PATH $ENV{CONDA_PREFIX})  # ignore conda libraries
    endif ()

    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif ()
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    if (NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
    endif ()
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    if (ERL_PRINT_HEADER_DEPENDENCIES)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -H")
    endif ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -Wall -Wextra -flto=auto")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color -fdiagnostics-show-template-tree")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops")

    if (NOT CMAKE_OSX_DEPLOYMENT_TARGET)
        set(CMAKE_OSX_DEPLOYMENT_TARGET 13.0)
    endif ()
    find_program(CCACHE_FOUND ccache)
    if (CCACHE_FOUND)
        set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
    else ()
        message(STATUS "ccache is not found")
    endif ()
    if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_definitions(-DNDEBUG)
    else ()
        set(CMAKE_VERBOSE_MAKEFILE ON)
    endif ()
endmacro()

#######################################################################################################################
# erl_enable_cuda
#######################################################################################################################
macro(erl_enable_cuda)
    set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER} CACHE STRING "Host compiler for CUDA" FORCE)
    enable_language(CUDA)
endmacro()

#######################################################################################################################
# erl_setup_lapack
#######################################################################################################################
macro(erl_setup_lapack)
    option(ERL_USE_LAPACK "Use LAPACK" ON)
    option(ERL_USE_LAPACK_STRICT "Use robust LAPACK algorithms only" OFF)
    option(ERL_USE_INTEL_MKL "Use Intel MKL (Math Kernel Library)" ON)
    option(ERL_USE_AOCL "Use AMD Optimizing CPU Library" OFF)
    option(ERL_USE_SINGLE_THREADED_BLAS "Use single-threaded BLAS" ON)

    if (ERL_USE_INTEL_MKL AND ERL_USE_AOCL)
        message(FATAL_ERROR "ERL_USE_INTEL_MKL and ERL_USE_AOCL cannot be both ON")
    endif ()

    if (ERL_USE_INTEL_MKL OR ERL_USE_AOCL)
        set(ERL_USE_LAPACK ON)
    endif ()

    if (ERL_USE_LAPACK)
        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(ERL_USE_LAPACK_STRICT ON)
        endif ()
        erl_find_path(
                OUTPUT LAPACKE_INCLUDE_DIR
                PACKAGE LAPACKE
                REQUIRED
                NAMES lapacke.h
                PATHS /usr/include /usr/local/include /usr/local/Cellar/lapack/*/include
                COMMANDS UBUNTU_LINUX "try `sudo apt install liblapacke-dev`"
                COMMANDS ARCH_LINUX "try `sudo pacman -S lapacke`")

        if (ERL_USE_INTEL_MKL)
            message(STATUS "Use Intel Math Kernel Library")
            if (ERL_USE_LAPACK_STRICT)
                # We don't turn on some unstable MKL routines: https://eigen.tuxfamily.org/dox/TopicUsingIntelMKL.html
                add_definitions(-DEIGEN_USE_BLAS)
                add_definitions(-DEIGEN_USE_LAPACKE_STRICT)
                add_definitions(-DEIGEN_USE_MKL_VML)
            else ()
                add_definitions(-DEIGEN_USE_MKL_ALL)
            endif ()
            if (ERL_USE_SINGLE_THREADED_BLAS)
                # we use MKL inside our OpenMP for loop or threaded code, so we need sequential BLAS
                set(BLA_VENDOR Intel10_64lp_seq)
                set(MKL_THREADING "sequential")
            else ()
                # MKL is used outside OpenMP for loop or threaded code, so we can use threaded BLAS
                set(BLA_VENDOR Intel10_64lp)
                set(MKL_THREADING "intel_thread")
            endif ()
            if (NOT DEFINED ENV{MKLROOT})
                unset(MKL_INCLUDE_DIRS CACHE)
                erl_find_path(
                        OUTPUT MKL_INCLUDE_DIRS
                        PACKAGE MKL
                        mkl.h
                        PATHS /usr/include /usr/local/include /opt/intel/oneapi/mkl/*/include
                        REQUIRED
                        COMMANDS ARCH_LINUX "try `sudo pacman -S intel-oneapi-basekit`"
                        COMMANDS GENERAL "visit https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html")
                get_filename_component(MKLROOT ${MKL_INCLUDE_DIRS} DIRECTORY)
                get_filename_component(MKLROOT ${MKLROOT} REALPATH)
                set(ENV{MKLROOT} ${MKLROOT})
                set(MKL_DIR ${MKLROOT}/lib/cmake/mkl CACHE PATH "Path to MKL cmake directory" FORCE)
                message(STATUS "MKLROOT is set to ${MKLROOT}")
            endif ()
            set(MKL_ARCH "intel64")
            set(MKL_LINK "dynamic")
            set(MKL_INTERFACE_FULL "intel_lp64")  # 32-bit integer indexing, for 64-bit integer indexing, use "intel_ilp64"
            erl_find_package(   # We need to find MKL to get MKL_H
                    PACKAGE MKL # MKL_LIBRARIES contains library names instead of full path, so we cannot use it
                    REQUIRED GLOBAL
                    COMMANDS ARCH_LINUX "try `sudo pacman -S intel-oneapi-basekit`"
                    COMMANDS GENERAL "visit https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html")
            erl_find_package(   # LAPACK will resolve the full paths of MKL libraries
                    PACKAGE LAPACK
                    REQUIRED GLOBAL
                    COMMANDS APPLE "try `brew install lapack`"
                    COMMANDS UBUNTU_LINUX "try `sudo apt install liblapack-dev`"
                    COMMANDS ARCH_LINUX "try `sudo pacman -S lapack`")
            set(MKL_INCLUDE_DIRS ${MKL_H} CACHE PATH "Path to MKL include directory" FORCE)
            # MKL_LIBRARIES contains library names instead of full path, so we cannot use it
            # we must remove MKL_LIBRARIES to avoid adding it to catkin_LIBRARIES when using catkin
            # however, find_package(MKL) will throw an error if it is called again
            unset(MKL_LIBRARIES)  # remove normal variable
            unset(MKL_LIBRARIES CACHE)  # remove CACHE variable
            # let's set the full path of MKL libraries
            set(MKL_LIBRARIES ${LAPACK_LIBRARIES} CACHE FILEPATH "Path to MKL libraries" FORCE)
        elseif (ERL_USE_AOCL)
            message(STATUS "Use AMD Optimizing CPU Library")
            add_definitions(-DEIGEN_USE_BLAS)
            add_definitions(-DEIGEN_ERL_USE_LAPACKE)
            set(BLA_VENDOR AOCL)
            erl_find_path(
                    OUTPUT AOCL_LIB_DIR
                    PACKAGE AMD-AOCL
                    REQUIRED
                    NAMES libblis.so
                    PATHS /opt/AMD/aocl/*/lib
                    COMMANDS GENERAL "visit https://www.amd.com/en/developer/aocl.html"
                    COMMANDS ARCH_LINUX "try `paru -Ss blas-aocl-gcc`")
            get_filename_component(AOCL_ROOT ${AOCL_LIB_DIR} DIRECTORY)
            if (ERL_USE_SINGLE_THREADED_BLAS)
                set(BLAS_LIBRARIES ${AOCL_ROOT}/lib/libblis.so ${AOCL_ROOT}/lib/libalm.so -lm)  # single-threaded BLAS
            else ()
                set(BLAS_LIBRARIES ${AOCL_ROOT}/lib/libblis-mt.so ${AOCL_ROOT}/lib/libalm.so -lm)  # multi-threaded BLAS
            endif ()
            set(LAPACK_LIBRARIES ${AOCL_ROOT}/lib/libflame.so)
        else ()
            message(STATUS "Use OpenBLAS")
            add_definitions(-DEIGEN_USE_BLAS)
            add_definitions(-DEIGEN_ERL_USE_LAPACKE)
            set(BLA_VENDOR OpenBLAS)
            if (ERL_USE_SINGLE_THREADED_BLAS)
                erl_find_path(
                        OUTPUT BLAS_LIB_DIR
                        PACKAGE OpenBLAS
                        REQUIRED
                        COMMANDS APPLE "try `brew install openblas`"
                        COMMANDS UBUNTU_LINUX "try `bash scripts/install_openblas_seq.bash`"
                        NAMES libopenblas.so
                        PATHS /opt/OpenBLAS/lib)
            endif ()
            erl_find_package(
                    PACKAGE LAPACK
                    REQUIRED
                    COMMANDS APPLE "try `brew install lapack`"
                    COMMANDS UBUNTU_LINUX "try `sudo apt install liblapack-dev`"
                    COMMANDS ARCH_LINUX "try `sudo pacman -S lapack`")
        endif ()
        message(STATUS "BLAS_LIBRARIES: ${BLAS_LIBRARIES}")
        message(STATUS "LAPACK_LIBRARIES: ${LAPACK_LIBRARIES}")
    else ()
        unset(BLAS_LIBRARIES)
        unset(LAPACK_LIBRARIES)
    endif ()
endmacro()

#######################################################################################################################
# erl_setup_common_packages
#######################################################################################################################
macro(erl_setup_common_packages)
    erl_find_package(
            PACKAGE fmt
            REQUIRED GLOBAL
            COMMANDS ARCH_LINUX "try `sudo pacman -S fmt`"
    )
    set_target_properties(fmt::fmt PROPERTIES SYSTEM ON)
    set_target_properties(fmt::fmt-header-only PROPERTIES SYSTEM ON)
    get_target_property(fmt_INCLUDE_DIRS fmt::fmt INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(fmt_LIBRARIES fmt::fmt IMPORTED_LOCATION_RELEASE)

    erl_find_package(
            PACKAGE OpenMP
            REQUIRED GLOBAL
            COMMANDS APPLE "try `brew install libomp`"
            COMMANDS UBUNTU_LINUX "try `sudo apt install libomp-dev`"
            COMMANDS ARCH_LINUX "try `sudo pacman -S openmp`")

    erl_find_package(
            PACKAGE absl
            REQUIRED GLOBAL
            COMMANDS APPLE "try `brew install abseil`"
            COMMANDS UBUNTU_LINUX "try `sudo apt install libabseil-dev`"
            COMMANDS ARCH_LINUX "try `sudo pacman -S abseil-cpp`")
    # absl_INCLUDE_DIRS and absl_LIBRARIES are not set by find_package(absl), so we need to set them manually for catkin
    get_target_property(absl_INCLUDE_DIRS absl::core_headers INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(absl_raw_hash_set_path absl::raw_hash_set LOCATION)
    get_filename_component(absl_LIB_DIR ${absl_raw_hash_set_path} DIRECTORY)
    unset(absl_raw_hash_set_path)
    file(GLOB absl_LIBRARIES ${absl_LIB_DIR}/libabsl_*.so.*)
    file(GLOB exclude_absl_LIBRARIES ${absl_LIB_DIR}/libabsl_*test*.so.*)
    foreach (exclude_absl_LIBRARY ${exclude_absl_LIBRARIES})
        list(REMOVE_ITEM absl_LIBRARIES ${exclude_absl_LIBRARY})
    endforeach ()

    # There are some bugs in Eigen3.4.0 when EIGEN_USE_MKL_ALL is defined. We should use the latest version.
    # enable vectorization of Eigen, borrow from https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-06
    if (ERL_USE_INTEL_MKL)
        include(CheckCXXCompilerFlag)
        # check -march=native -xHost -mavx -mavx2 -mfma -mfma4
        unset(_CXX_FLAGS)
        foreach (_flag IN ITEMS "-march=native" "-xHost" "-mavx" "-mavx2" "-mfma" "-mfma4")
            string(REPLACE "=" "_" _flag_works ${_flag})
            string(REPLACE "-" "_" _flag_works ${_flag_works})
            check_cxx_compiler_flag("${_flag}" ${_flag_works})
            if (${_flag_work})
                message(STATUS "Use ${_flag} for Release build")
                set(_CXX_FLAGS "${_CXX_FLAGS} ${_flag}")
            endif ()
        endforeach ()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${_CXX_FLAGS}")
        unset(_CXX_FLAGS)
    endif ()
    if (ERL_USE_INTEL_MKL)  # option from erl_setup_lapack
        set(EIGEN3_VERSION_STRING "3.4.90" CACHE STRING "Eigen3 version" FORCE)  # some other packages may read this variable.
        erl_find_package(
                PACKAGE Eigen3
                ${EIGEN3_VERSION_STRING} REQUIRED CONFIG GLOBAL  # in case some other packages define FindEigen3.cmake
                COMMANDS ARCH_LINUX "try `paru -S eigen-git`"
                COMMANDS GENERAL "visit https://gitlab.com/libeigen/eigen to install the required version")
    else ()
        erl_find_package(
                PACKAGE Eigen3
                REQUIRED CONFIG GLOBAL  # in case some other packages define FindEigen3.cmake
                COMMANDS APPLE "try `brew install eigen`"
                COMMANDS UBUNTU_LINUX "try `sudo apt install libeigen3-dev`"
                COMMANDS ARCH_LINUX "try `sudo pacman -S eigen`")
        set(EIGEN3_VERSION_STRING ${Eigen3_VERSION} CACHE STRING "Eigen3 version" FORCE)
    endif ()
    set_target_properties(Eigen3::Eigen PROPERTIES SYSTEM ON)

    erl_find_package(
            PACKAGE OpenCV
            REQUIRED GLOBAL COMPONENTS core imgproc highgui
            COMMANDS APPLE "run scripts/install_opencv.bash"
            COMMANDS UBUNTU_LINUX "try `sudo apt install libopencv-dev`"
            COMMANDS ARCH_LINUX "try `sudo pacman -S opencv`")
    if (EXISTS /usr/lib/libOpenGL.so)
        set(OpenGL_GL_PREFERENCE "GLVND")
    else ()
        set(OpenGL_GL_PREFERENCE "LEGACY")
    endif ()
    erl_find_package(
            PACKAGE yaml-cpp
            REQUIRED GLOBAL
            COMMANDS UBUNTU_LINUX "try `sudo apt install libyaml-cpp-dev`"
            COMMANDS ARCH_LINUX "try `sudo pacman -S yaml-cpp`")

    if (ROS_ACTIVATED)
        if (ROS_VERSION STREQUAL "1")
            set(OpenMP_INCLUDE_DIRS /usr/include)
            get_target_property(OpenMP_LIBRARIES OpenMP::OpenMP_CXX INTERFACE_LINK_LIBRARIES)
            message(STATUS "OpenMP_INCLUDE_DIRS: ${OpenMP_INCLUDE_DIRS}")
            message(STATUS "OpenMP_LIBRARIES: ${OpenMP_LIBRARIES}")
            get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
            message(STATUS "Eigen3_INCLUDE_DIRS: ${Eigen3_INCLUDE_DIRS}")
            if (YAML_CPP_INCLUDE_DIR OR YAML_CPP_LIBRARIES)
                set(yaml-cpp_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR} CACHE PATH "yaml-cpp include directories" FORCE)
                set(yaml-cpp_LIBRARIES ${YAML_CPP_LIBRARIES} CACHE STRING "yaml-cpp libraries" FORCE)
            else ()  # noconfig version of yaml-cpp: yaml-cpp is installed without setting CMAKE_BUILD_TYPE
                get_target_property(yaml-cpp_LIBRARIES yaml-cpp IMPORTED_LOCATION_NOCONFIG)
                get_filename_component(yaml-cpp_INCLUDE_DIRS ${yaml-cpp_LIBRARIES} DIRECTORY)
                get_filename_component(yaml-cpp_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS}/../include ABSOLUTE)
                set(yaml-cpp_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS} CACHE PATH "yaml-cpp include directories" FORCE)
                set(yaml-cpp_LIBRARIES ${yaml-cpp_LIBRARIES} CACHE STRING "yaml-cpp libraries" FORCE)
            endif ()
            message(STATUS "yaml-cpp_INCLUDE_DIRS: ${yaml-cpp_INCLUDE_DIRS}")
            message(STATUS "yaml-cpp_LIBRARIES: ${yaml-cpp_LIBRARIES}")
            # get_target_property(Matplot++_INCLUDE_DIRS Matplot++::matplot INTERFACE_INCLUDE_DIRECTORIES)
            # get_target_property(Matplot++_LIBRARIES Matplot++::matplot LOCATION)
            # message(STATUS "Matplot++_INCLUDE_DIRS: ${Matplot++_INCLUDE_DIRS}")
            # message(STATUS "Matplot++_LIBRARIES: ${Matplot++_LIBRARIES}")
        endif ()
    endif ()
endmacro()

#######################################################################################################################
# erl_setup_python
#######################################################################################################################
macro(erl_setup_python)
    option(ERL_BUILD_PYTHON "Build Python binding" ON)
    if (ERL_BUILD_PYTHON)
        erl_find_package(
                PACKAGE Python3
                REQUIRED COMPONENTS Interpreter Development
                COMMANDS APPLE "try `brew install python3`"
                COMMANDS UBUNTU_LINUX "try `sudo apt install python3-dev`"
                COMMANDS ARCH_LINUX "try `sudo pacman -S python`")
        set_target_properties(Python3::Python PROPERTIES SYSTEM ON)
        message(STATUS "Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
        erl_find_package(
                PACKAGE pybind11
                REQUIRED
                COMMANDS APPLE "try `brew install pybind11`"
                COMMANDS UBUNTU_LINUX "try `sudo apt install pybind11-dev`"
                COMMANDS ARCH_LINUX "try `sudo pacman -S pybind11`")
        foreach (item IN ITEMS python_link_helper python_headers headers module embed windows_extras thin_lto lto opt_size)
            set_target_properties(pybind11::${item} PROPERTIES SYSTEM ON)
        endforeach ()
    endif ()
endmacro()

#######################################################################################################################
# erl_setup_test
#######################################################################################################################
macro(erl_setup_test)
    option(ERL_BUILD_TEST "Build executables for test" ON)
    if (ERL_BUILD_TEST)
        add_definitions(-DERL_BUILD_TEST)
        enable_testing()
        if (NOT ROS_ACTIVATED)  # GTest is configured by ROS if ROS is activated
            # we need to use GTest::GTest and GTest::Main in other subprojects
            erl_find_package(
                    PACKAGE GTest
                    REQUIRED GLOBAL
                    COMMANDS UBUNTU_LINUX "try `sudo apt install libgtest-dev`"
                    COMMANDS ARCH_LINUX "try `sudo pacman -S gtest`")
            include(GoogleTest)
        endif ()
    endif ()
    if (NOT DEFINED ERL_BUILD_TEST_${PROJECT_NAME})
        set(ERL_BUILD_TEST_${PROJECT_NAME} ${ERL_BUILD_TEST})
    endif ()
endmacro()

#######################################################################################################################
# erl_setup_ros
#######################################################################################################################
macro(erl_setup_ros)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs MSG_DEPENDENCIES MSG_FILES SRV_FILES ACTION_FILES)
    cmake_parse_arguments(${PROJECT_NAME} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (ROS_ACTIVATED)
        if (ROS_VERSION STREQUAL "1")
            if (NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/package.xml)
                message(FATAL_ERROR "No package.xml found in ${CMAKE_CURRENT_LIST_DIR}")
            endif ()
            erl_find_package(
                    PACKAGE catkin
                    NO_RECORD
                    REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
                    COMMANDS UBUNTU_LINUX "try `sudo apt install ros-${ROS_DISTRO}-catkin python3-catkin-pkg python3-empy python3-nose python3-setuptools`"
                    COMMANDS ARCH_LINUX "try `paru -S python-catkin_tools ros-${ROS_DISTRO}-catkin`")
            if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/setup.py)
                catkin_python_setup()
                set(${PROJECT_NAME}_CATKIN_PYTHON_SETUP TRUE CACHE BOOL "TRUE if catkin_python_setup() was called" FORCE)
            else ()
                set(${PROJECT_NAME}_CATKIN_PYTHON_SETUP FALSE CACHE BOOL "TRUE if catkin_python_setup() was called" FORCE)
            endif ()

            if (${PROJECT_NAME}_MSG_FILES)
                add_message_files(FILES ${${PROJECT_NAME}_MSG_FILES})
            endif ()
            if (${PROJECT_NAME}_SRV_FILES)
                add_service_files(FILES ${${PROJECT_NAME}_SRV_FILES})
            endif ()
            if (${PROJECT_NAME}_ACTION_FILES)
                add_action_files(FILES ${${PROJECT_NAME}_ACTION_FILES})
            endif ()
            if (${PROJECT_NAME}_MSG_DEPENDENCIES)
                generate_messages(DEPENDENCIES ${${PROJECT_NAME}_MSG_DEPENDENCIES})
            endif ()
        else ()
            message(FATAL_ERROR "ROS2 is not supported yet")
        endif ()
    endif ()
endmacro()

#######################################################################################################################
# erl_catkin_package
#######################################################################################################################
macro(erl_catkin_package)
    if (ROS_ACTIVATED AND ROS_VERSION STREQUAL "1")
        catkin_package(${ARGV})
        # filter out Eigen3 installed at `/usr/include/eigen3` from catkin_INCLUDE_DIRS
        # if `/usr/local/include/eigen3` is also in catkin_INCLUDE_DIRS
        set(EIGEN3_AT_USR_INCLUDE_EIGEN3 FALSE)
        set(EIGEN3_AT_USR_LOCAL_INCLUDE_EIGEN3 FALSE)
        foreach (inc ${catkin_INCLUDE_DIRS})
            if (inc STREQUAL "/usr/include/eigen3")
                set(EIGEN3_AT_USR_INCLUDE_EIGEN3 TRUE)
            else ()
                list(APPEND filtered_catkin_INCLUDE_DIRS ${inc})
            endif ()
            if (inc STREQUAL "/usr/local/include/eigen3")
                set(EIGEN3_AT_USR_LOCAL_INCLUDE_EIGEN3 TRUE)
            endif ()
        endforeach ()
        if (NOT EIGEN3_AT_USR_LOCAL_INCLUDE_EIGEN3 AND EIGEN3_AT_USR_INCLUDE_EIGEN3)
            list(APPEND filtered_catkin_INCLUDE_DIRS "/usr/include/eigen3")
        endif ()
        set(catkin_INCLUDE_DIRS ${filtered_catkin_INCLUDE_DIRS})
        unset(filtered_catkin_INCLUDE_DIRS)

        # filter out MODULE_LIBRARY from catkin_LIBRARIES
        foreach (lib ${catkin_LIBRARIES})
            get_filename_component(lib_dir ${lib} DIRECTORY)
            list(APPEND catkin_LIBRARY_DIRS ${lib_dir})
            if (TARGET ${lib})
                get_target_property(lib_type ${lib} TYPE)
                if (NOT lib_type STREQUAL "MODULE_LIBRARY")  # MODULE_LIBRARY, e.g. pybind11 lib, cannot be linked
                    list(APPEND filtered_catkin_LIBRARIES ${lib})
                endif ()
            else ()
                list(APPEND filtered_catkin_LIBRARIES ${lib})
            endif ()
        endforeach ()
        set(catkin_LIBRARIES ${filtered_catkin_LIBRARIES})
        unset(filtered_catkin_LIBRARIES)
    endif ()

    erl_set_project_paths()

    if (ROS_ACTIVATED AND ROS_VERSION STREQUAL "1")
        set(CATKIN_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
        get_filename_component(CATKIN_WORKSPACE_DIR ${CATKIN_INSTALL_DIR} DIRECTORY)
        set(CATKIN_DEVEL_DIR ${CATKIN_WORKSPACE_DIR}/devel)
        set(CATKIN_DEVEL_LIB_DIR ${CATKIN_DEVEL_DIR}/lib)
        set(CATKIN_INSTALL_LIB_DIR ${CATKIN_INSTALL_DIR}/lib)
        set(CATKIN_INSTALL_PYTHON_DIR ${CATKIN_INSTALL_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
    endif ()
endmacro()

#######################################################################################################################
# erl_project_setup
#######################################################################################################################
macro(erl_project_setup _name)
    set(options ENABLE_CUDA)
    set(oneValueArgs)
    set(multiValueArgs ERL_PACKAGES CATKIN_COMPONENTS CATKIN_DEPENDS)
    cmake_parse_arguments(${_name} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (${_name}_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "Some arguments are not recognized by erl_project_setup")
    endif ()

    if (${_name}_ENABLE_CUDA)
        erl_enable_cuda()
    endif ()

    foreach (erl_package ${${_name}_ERL_PACKAGES})
        if (NOT TARGET ${erl_package})
            erl_find_package(
                    PACKAGE ${erl_package}
                    REQUIRED
                    COMMANDS GENERAL "build and install ${erl_package} at first")
        endif ()
    endforeach ()

    # check if is top-level
    if (CMAKE_VERSION VERSION_LESS 3.21)
        get_property(NOT_TOP_LEVEL DIRECTORY PROPERTY PARENT_DIRECTORY)
        if (NOT NOT_TOP_LEVEL)
            set(PROJECT_IS_TOP_LEVEL ON)
        else ()
            set(PROJECT_IS_TOP_LEVEL OFF)
        endif ()
    endif ()

    erl_setup_compiler()
    erl_detect_ros()

    if (ROS_ACTIVATED)
        if (ROS_VERSION STREQUAL "1")
            list(APPEND ${_name}_CATKIN_COMPONENTS ${${PROJECT_NAME}_ERL_PACKAGES})
            list(REMOVE_DUPLICATES ${_name}_CATKIN_COMPONENTS)
            list(APPEND ${_name}_CATKIN_DEPENDS ${${PROJECT_NAME}_ERL_PACKAGES})
            list(REMOVE_DUPLICATES ${_name}_CATKIN_DEPENDS)
        endif ()
    endif ()

    if (NOT ERL_PROJECT_SETUP_DONE OR ROS_ACTIVATED)
        erl_setup_lapack()
        erl_setup_common_packages()
        erl_setup_python()
        if (NOT ROS_ACTIVATED)  # if ROS is activated, there is no PARENT_SCOPE when erl_project_setup is called
            if (PROJECT_IS_TOP_LEVEL)
                set(ERL_PROJECT_SETUP_DONE TRUE)
            else ()
                set(ERL_PROJECT_SETUP_DONE TRUE PARENT_SCOPE)
            endif ()
        endif ()
    endif ()
    erl_setup_test()
endmacro()

#######################################################################################################################
# erl_add_python_package
#######################################################################################################################
macro(erl_add_python_package)
    set(options)
    set(oneValueArgs PYTHON_PKG_DIR PYBIND_MODULE_NAME)
    set(multiValueArgs DEPENDS_PYTHON_PKGS)
    cmake_parse_arguments(${PROJECT_NAME} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if (NOT DEFINED ERL_BUILD_PYTHON_${PROJECT_NAME})
        set(ERL_BUILD_PYTHON_${PROJECT_NAME} ${ERL_BUILD_PYTHON})
    endif ()
    if (NOT ERL_BUILD_PYTHON)
        set(ERL_BUILD_PYTHON_${PROJECT_NAME} OFF)
    endif ()

    # ${PROJECT_NAME}_PYTHON_DIR: <project_dir>/python
    # ${PROJECT_NAME}_PYTHON_PKG_DIR: <project_dir>/python/<py_package_name>
    # ${PROJECT_NAME}_PYTHON_BINDING_DIR: <project_dir>/python/binding
    # ${PROJECT_NAME}_BUILD_DIR: <project_build_dir>
    # ${PROJECT_NAME}_BUILD_PYTHON_DIR: <project_build_dir>/python
    # ${PROJECT_NAME}_BUILD_PYTHON_PKG_DIR: <project_build_dir>/python/<py_package_name>

    if (ERL_BUILD_PYTHON_${PROJECT_NAME})
        erl_setup_python()
        # get package name
        get_filename_component(${PROJECT_NAME}_PY_PACKAGE_NAME ${${PROJECT_NAME}_PYTHON_PKG_DIR} NAME)

        # ${PROJECT_NAME}_BUILD_PYTHON_PKG_DIR: <project_build_dir>/python/<py_package_name>
        set(${PROJECT_NAME}_BUILD_PYTHON_PKG_DIR ${${PROJECT_NAME}_BUILD_PYTHON_DIR}/${${PROJECT_NAME}_PY_PACKAGE_NAME})

        # add a binding library for this package
        file(GLOB_RECURSE SRC_FILES "${${PROJECT_NAME}_PYTHON_BINDING_DIR}/*.cpp")
        if (SRC_FILES)  # if there is any pybind11_*.cpp file
            if (NOT DEFINED ${PROJECT_NAME}_PYBIND_MODULE_NAME)
                set(${PROJECT_NAME}_PYBIND_MODULE_NAME py${PROJECT_NAME})
                message(STATUS "PYBIND_MODULE_NAME not set, using default value ${${PROJECT_NAME}_PYBIND_MODULE_NAME}")
            endif ()
            # pybind runtime lib
            pybind11_add_module(${${PROJECT_NAME}_PYBIND_MODULE_NAME} ${SRC_FILES})
            ## ref: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
            ## Use separate rpaths during build and install phases
            # set(CMAKE_SKIP_BUILD_RPATH  FALSE)
            ## Don't use the install-rpath during the build phase
            # set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
            # set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
            ## Automatically add all linked folders that are NOT in the build directory to the rpath (per library?)
            # set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
            if (APPLE)
                set(CMAKE_MACOSX_RPATH ON)
                set(_rpath_portable_origin "@loader_path")
            else ()
                set(_rpath_portable_origin $ORIGIN)
            endif ()
            set(_rpath "${_rpath_portable_origin}:${_rpath_portable_origin}/lib/${PROJECT_NAME}")
            # how to check rpath of a library file:
            # objdump -x <library_name>.so | grep 'R.*PATH'
            # output: RUNPATH     $ORIGIN:$ORIGIN/lib/<project_name>
            # how to check shared library dependencies of a library file:
            # ldd <library_name>.so
            set_target_properties(${${PROJECT_NAME}_PYBIND_MODULE_NAME} PROPERTIES
                    SKIP_BUILD_RPATH FALSE # Use separate rpaths during build and install phases
                    BUILD_WITH_INSTALL_RPATH FALSE # Don't use the install-rpath during the build phase
                    INSTALL_RPATH "${_rpath}" # search in the same directory, or in lib/<project_name>
                    INSTALL_RPATH_USE_LINK_PATH TRUE)
            target_compile_definitions(${${PROJECT_NAME}_PYBIND_MODULE_NAME}
                    PRIVATE PYBIND_MODULE_NAME=${${PROJECT_NAME}_PYBIND_MODULE_NAME})
            target_include_directories(${${PROJECT_NAME}_PYBIND_MODULE_NAME} SYSTEM PRIVATE ${Python3_INCLUDE_DIRS})
            target_link_libraries(${${PROJECT_NAME}_PYBIND_MODULE_NAME} PRIVATE ${PROJECT_NAME})
            # put the library in the source python package directory, such that setup.py can find it
            # set_target_properties(${${PROJECT_NAME}_PYBIND_MODULE_NAME} PROPERTIES
            #         LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_PYTHON_PKG_DIR})
            if (ROS_ACTIVATED AND ROS_VERSION STREQUAL "1")
                # copy file to a regular library name so that catkin does not throw an error, but this file may not
                # work with Python3 because the filename is lib<name>.so, which does not match the module name.
                set(LIB_NAME lib${${PROJECT_NAME}_PYBIND_MODULE_NAME}.so)
                set(DEVEL_LIB_PATH ${ERL_CATKIN_DEVEL_LIB_DIR}/${LIB_NAME})
                set(INSTALL_LIB_PATH ${ERL_CATKIN_INSTALL_LIB_DIR}/${LIB_NAME})

                add_custom_command(TARGET ${${PROJECT_NAME}_PYBIND_MODULE_NAME}
                        POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${${PROJECT_NAME}_PYBIND_MODULE_NAME}> ${DEVEL_LIB_PATH}
                        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${${PROJECT_NAME}_PYBIND_MODULE_NAME}> ${INSTALL_LIB_PATH}
                        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${${PROJECT_NAME}_PYBIND_MODULE_NAME}> ${${PROJECT_NAME}_PYTHON_PKG_DIR})  # copy to source directory for devel
            endif ()
        endif ()

        if (EXISTS ${${PROJECT_NAME}_ROOT_DIR}/setup.py)
            option(ERL_PYTHON_INSTALL_USER "Install Python package to user directory" OFF)
            set(erl_pip_install_args "--verbose")
            if (ERL_PYTHON_INSTALL_USER)
                set(erl_pip_install_args "${erl_pip_install_args} --user")
            endif ()
            add_custom_target(${PROJECT_NAME}_py_wheel
                    COMMAND ${Python3_EXECUTABLE} setup.py bdist_wheel
                    WORKING_DIRECTORY ${${PROJECT_NAME}_ROOT_DIR}
                    COMMENT "Building Python wheel for ${PROJECT_NAME}")
            add_custom_target(${PROJECT_NAME}_py_develop
                    COMMAND ${Python3_EXECUTABLE} -m pip install -e . ${erl_pip_install_args}
                    WORKING_DIRECTORY ${${PROJECT_NAME}_ROOT_DIR}
                    DEPENDS ${${PROJECT_NAME}_PYBIND_MODULE_NAME}
                    COMMENT "Installing Python package ${PROJECT_NAME} in develop mode")
            add_custom_target(${PROJECT_NAME}_py_install
                    COMMAND ${Python3_EXECUTABLE} -m pip install . ${erl_pip_install_args}
                    WORKING_DIRECTORY ${${PROJECT_NAME}_ROOT_DIR}
                    COMMENT "Installing Python package ${PROJECT_NAME} in install mode")
            get_filename_component(stubgen_path ${Python3_EXECUTABLE} DIRECTORY)
            set(stubgen_path ${stubgen_path}/stubgen)
            if (EXISTS ${stubgen_path})
                add_custom_target(${PROJECT_NAME}_py_stub
                        COMMAND ${stubgen_path} -o ${CMAKE_CURRENT_BINARY_DIR}/python/stubs -p ${${PROJECT_NAME}_PY_PACKAGE_NAME}.${${PROJECT_NAME}_PYBIND_MODULE_NAME} --verbose
                        DEPENDS ${PROJECT_NAME}_py_install
                        WORKING_DIRECTORY ${${PROJECT_NAME}_ROOT_DIR}
                        COMMENT "Generating stub files for Python package ${PROJECT_NAME}")
            endif ()
            unset(erl_pip_install_args)
        endif ()

    endif ()
endmacro()

#######################################################################################################################
# erl_install
#######################################################################################################################
macro(erl_install)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs EXECUTABLES LIBRARIES PYBIND_MODULES CATKIN_PYTHON_PROGRAMS OTHER_FILES)
    cmake_parse_arguments(${PROJECT_NAME}_INSTALL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Create a package to distribute the software
    # reference 1: https://dominikberner.ch/cmake-interface-lib/ (not complete)
    # reference 2: https://gitlab.com/libeigen/eigen/-/blob/master/CMakeLists.txt
    # include(GNUInstallDirs)  # called by erl_set_project_paths
    # include(CMakePackageConfigHelpers)  # called by erl_set_project_paths

    # Install the executable files
    if (${PROJECT_NAME}_INSTALL_EXECUTABLES)
        foreach (executable ${${PROJECT_NAME}_INSTALL_EXECUTABLES})
            message(STATUS "Generate install rule for executable ${executable}")
        endforeach ()
        # Export targets defined in this file and define the installation location
        # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
        install(TARGETS ${${PROJECT_NAME}_INSTALL_EXECUTABLES}  # add the targets here
                RUNTIME DESTINATION ${${PROJECT_NAME}_INSTALL_BINARY_DIR})
    endif ()

    # Install the library files
    if (${PROJECT_NAME}_INSTALL_LIBRARIES)
        foreach (library ${${PROJECT_NAME}_INSTALL_LIBRARIES})
            message(STATUS "Generate install rule for library ${library}")
        endforeach ()
        # Export targets defined in this file and define the installation location
        install(TARGETS ${${PROJECT_NAME}_INSTALL_LIBRARIES}  # add the targets here
                EXPORT ${PROJECT_NAME}_Targets
                ARCHIVE DESTINATION ${${PROJECT_NAME}_INSTALL_LIBRARY_DIR}
                LIBRARY DESTINATION ${${PROJECT_NAME}_INSTALL_LIBRARY_DIR}
                RUNTIME DESTINATION ${${PROJECT_NAME}_INSTALL_BINARY_DIR})
    endif ()

    # Install the header files
    if (EXISTS ${${PROJECT_NAME}_INCLUDE_DIR}/${PROJECT_NAME})
        install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIR}/${PROJECT_NAME}
                DESTINATION ${${PROJECT_NAME}_INSTALL_INCLUDE_DIR})  # ${PROJECT_NAME} is added automatically
    endif ()
    # Install other files
    if (${PROJECT_NAME}_INSTALL_OTHER_FILES)
        install(FILES ${${PROJECT_NAME}_INSTALL_OTHER_FILES}
                DESTINATION ${${PROJECT_NAME}_INSTALL_SHARE_DIR})
    endif ()
    # Install cmake files, only for erl_common
    if (PROJECT_NAME STREQUAL "erl_common")
        file(GLOB CMAKE_FILES ${ERL_CMAKE_DIR}/*.cmake)
        install(FILES ${CMAKE_FILES}
                DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKE_DIR})
    endif ()

    # ROS Support
    if (ROS_ACTIVATED AND ROS_VERSION STREQUAL "1")
        # Install the pybind module
        if (${PROJECT_NAME}_INSTALL_PYBIND_MODULES)
            foreach (module ${${PROJECT_NAME}_INSTALL_PYBIND_MODULES})
                message(STATUS "Generate install rule for pybind module ${module}")
            endforeach ()
            install(TARGETS ${${PROJECT_NAME}_INSTALL_PYBIND_MODULES}
                    ARCHIVE DESTINATION ${${PROJECT_NAME}_INSTALL_PYTHON_DIR}
                    LIBRARY DESTINATION ${${PROJECT_NAME}_INSTALL_PYTHON_DIR}
                    RUNTIME DESTINATION ${${PROJECT_NAME}_INSTALL_PYTHON_DIR})
        endif ()

        # Config file generation is taken care of by catkin_package()

        # Mark executable scripts (Python etc.) for installation in contrast to setup.py
        if (DEFINED ${PROJECT_NAME}_INSTALL_CATKIN_PYTHON_PROGRAMS)
            catkin_install_python(PROGRAMS
                    ${${PROJECT_NAME}_INSTALL_CATKIN_PYTHON_PROGRAMS}
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        endif ()
        # Install the python package
        if (${${PROJECT_NAME}_CATKIN_PYTHON_SETUP} AND DEFINED ${PROJECT_NAME}_BUILD_PYTHON_PKG_DIR)
            # install files generated from files in python/${PROJECT_NAME}
            install(DIRECTORY ${${PROJECT_NAME}_BUILD_PYTHON_PKG_DIR}
                    DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
        endif ()
        # TODO: install other files, e.g. launch files, config files, etc.
    elseif (BUILD_FOR_ROS2)
        message(FATAL_ERROR "Not implemented yet")
    else ()
        # Install the pybind module if pip install is used
        if (${PROJECT_NAME}_INSTALL_PYBIND_MODULES AND DEFINED PIP_LIB_DIR)
            foreach (module ${${PROJECT_NAME}_INSTALL_PYBIND_MODULES})
                message(STATUS "Generate install rule for pybind module ${module}")
            endforeach ()
            install(TARGETS ${${PROJECT_NAME}_INSTALL_PYBIND_MODULES}
                    ARCHIVE DESTINATION ${PIP_LIB_DIR}
                    LIBRARY DESTINATION ${PIP_LIB_DIR}
                    RUNTIME DESTINATION ${PIP_LIB_DIR})
        endif ()

        foreach (TARGET IN LISTS ${PROJECT_NAME}_Targets)
            message(STATUS "Generate the rule to export ${TARGET} from ${PROJECT_NAME}")
        endforeach ()

        # Do version comparison
        write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake
                VERSION ${PROJECT_VERSION}
                COMPATIBILITY SameMajorVersion)
        # Generate the configuration file which CMake uses for using an installed package
        configure_package_config_file(
                "${${PROJECT_NAME}_CMAKE_DIR}/${PROJECT_NAME}Config.cmake.in"
                "${${PROJECT_NAME}_BUILD_DIR}/${PROJECT_NAME}Config.cmake"
                INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKE_DIR})

        # Select targets and files to install
        # Generate and Install Targets.cmake file that defines the targets offered by this project
        # ${PROJECT_NAME} target will be located in the ${PROJECT_NAME} namespace.
        # Other CMake targets can refer to it using ${PROJECT_NAME}::${PROJECT_NAME}
        install(EXPORT ${PROJECT_NAME}_Targets
                FILE ${PROJECT_NAME}Targets.cmake
                DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKE_DIR})

        # reference: https://gitlab.com/libeigen/eigen/-/blob/master/CMakeLists.txt#L662
        # to make find_package consider the directory while searching for ${PROJECT_NAME}
        export(PACKAGE ${PROJECT_NAME})
        # Install the build configuration and information about version compatibility
        install(FILES "${${PROJECT_NAME}_BUILD_DIR}/${PROJECT_NAME}Config.cmake"
                "${${PROJECT_NAME}_BUILD_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
                DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKE_DIR})
    endif ()

endmacro()

#######################################################################################################################
# erl_mark_project_found
#######################################################################################################################
macro(erl_mark_project_found _name)
    set(${_name}_FOUND
            TRUE
            CACHE BOOL "TRUE if ${_name} and all required components found on the system" FORCE)
    message(STATUS "${_name} is found")
endmacro()
