cmake_minimum_required(VERSION 3.0)

set(VERSION_MAJOR  0)
set(VERSION_MINOR  9)
set(VERSION_BUGFIX 2)
set(VERSION_STAGE  -Alpha)
set(VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BUGFIX}${VERSION_STAGE}")
configure_file( doc/ble_mesh_version.txt.in doc/ble_mesh_version.txt )

set(NRF51_SOFTDEVICE "S130" CACHE STRING "nRF51 SoftDevice version (S110 or S130)")
set(NRF52_SOFTDEVICE "S132" CACHE STRING "nRF52 SoftDevice version (S132)")

set(TARGET_PLATFORM "51" CACHE STRING "Target platform (51, 52 or x86)")
set(DEBUG_LEVEL 3 CACHE STRING "Debug verbosity if built in debug mode (0, 1, 2 or 3)")

#####################
# Toolchain options #
#####################

set(BUILD_TOOLCHAIN "" CACHE STRING "Which toolchain to use (ARM, GCC)")
option(BUILD_HOST          "Compile for Host?"                                OFF)
option(BUILD_EXAMPLES      "Build the example projects?"                      ON)

if (NOT BUILD_HOST)
    option(PERSISTENT_STORAGE  "Enable persistent storage (i.e. flash_manager)"   ON)
    if (BUILD_TOOLCHAIN STREQUAL "")
        if (CMAKE_HOST_WIN32)
            set(BUILD_TOOLCHAIN "ARM")
        else ()
            set(BUILD_TOOLCHAIN "GCC")
        endif()
    endif()

    if (TARGET_PLATFORM MATCHES "51")
        list( APPEND HWID "51")
    endif()
    if (TARGET_PLATFORM MATCHES "52")
        list( APPEND  HWID "52")
    endif()
else()
    set(PERSISTENT_STORAGE ON)
    set(TARGET_PLATFORM "x86")
    list( APPEND  HWID "ut")
endif()

message(STATUS "nRF51 SD = ${NRF51_SOFTDEVICE}")
message(STATUS "nRF52 SD = ${NRF52_SOFTDEVICE}")
message(STATUS "HWID = ${HWID}")
################################
# Global configuration options #
###############################
option(UECC_ENABLE "Enables compiling the micro-ecc ECDH library" ON)

#########################################
# Setup paths for external dependencies #
#########################################

set (MBTLE_SOURCE_DIR ${CMAKE_SOURCE_DIR})

include(${CMAKE_SOURCE_DIR}/CMake/dependency_utils.cmake)

find_dependency(CMOCK_ROOT "Root folder of CMock test library" "${MBTLE_SOURCE_DIR}/../cmock" "src/cmock.c")
find_dependency(UNITY_ROOT "Root folder of Unity test library" "${CMOCK_ROOT}/vendor/unity" "src/unity.c")

# Setup PC Lint tool
include(${CMAKE_SOURCE_DIR}/CMake/pc_lint.cmake)


# Set a default build type if none was specified
# From https://blog.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "RelWithDebInfo")
if (EXISTS "${CMAKE_SOURCE_DIR}/.git")
    set(default_build_type "Debug")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if (BUILD_HOST)
    # Compiling for x86
    check_dependency("CMOCK_ROOT;UNITY_ROOT")
    add_definitions("-DHOST")
    include(${CMAKE_SOURCE_DIR}/CMake/configure_x86.cmake)
else ()
    # Compiling for nRF5x target
    include(${CMAKE_SOURCE_DIR}/CMake/configure_arm.cmake)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

project(MBTLE_CORE C ASM)
if (BUILD_HOST)
    include(${CMAKE_SOURCE_DIR}/CMake/configure_ubsan.cmake)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT BUILD_HOST)
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
endif()

if(NRF_MESH_STACK_DEPTH)
    add_definitions(-DNRF_MESH_STACK_DEPTH)
endif()
macro(append_suffix out suffix)
    unset(_tmp)
    foreach(arg ${ARGN})
        list( APPEND _tmp ${arg}_${suffix})
    endforeach()
    set(${out} ${_tmp})
endmacro()


set(MBTLE_DEFINITIONS "")
# Fill SRC list variables
include(${MBTLE_SOURCE_DIR}/mesh/cmake_defines.cmake)
include(${MBTLE_SOURCE_DIR}/mesh/mbtle_src_definition.cmake)

add_subdirectory("doc")
add_subdirectory("mesh")
add_subdirectory("lib")
add_subdirectory("tools")

# Add it AFTER mesh to avoid duplicating the flag
if (PERSISTENT_STORAGE)
    set(MBTLE_DEFINITIONS
        ${MBTLE_DEFINITIONS}
        "-DPERSISTENT_STORAGE=1")
else()
    set(MBTLE_DEFINITIONS
        ${MBTLE_DEFINITIONS}
        "-DPERSISTENT_STORAGE=0")
endif()

if ((NOT BUILD_HOST) OR (NOT BUILD_EXAMPLES))
    # Only compile Examples for the first hardware platform specified in HWID
    # MBTLE-1279    Add multi-platform support for example projects.
    list(GET HWID 0 HWID)
    message(STATUS "HWID = ${HWID}")
    add_subdirectory("examples")
endif ()
