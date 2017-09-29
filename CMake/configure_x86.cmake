# Setup for compiling on host (x86)
include("CMake/configure_hw.cmake")

if (EXISTS "${UNITY_ROOT}/src/unity.c")
    message(STATUS "Using Unity from ${UNITY_ROOT}")
else ()
    message(FATAL_ERROR "${UNITY_ROOT} not a valid path to the Unity library...")
endif ()

# Not needed on host.
set(BUILD_TOOLCHAIN "")

set(CMAKE_C_FLAGS
    "-std=gnu99"
    "-pthread"
    # Compile in 32-bit mode to mimic ARM and also to get 32-bit pointers
    "-m32"
    "-Wall"
    "-Wextra"
    "-Winline"
    "-Wno-unused-parameter"           # Generates lots and lots of warnings from the SoftDevice headers
    "-Wno-missing-field-initializers" # Generates unnecessary warnings about uninitialized internal use fields
    "-Werror=implicit-function-declaration"
    "-Wno-format"
    "-fno-strict-aliasing"
    "--coverage"
    )

#make SVC-calls run inline:
add_definitions(
    "-DSVCALL_AS_NORMAL_FUNCTION"
    )

option(ENABLE_PROFILING "Enable profiling of the stack" OFF)
option(ENABLE_COVERAGE "Enable coverage analysis" OFF)

# Enable profiling if requested:
if(ENABLE_PROFILING)
    message(STATUS "Profiling enabled")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
    set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -pg")
endif(ENABLE_PROFILING)

if (CMAKE_HOST_WIN32)
    # Include toolchain settings
    include("CMake/toolchain_mingw.cmake")

    if(ENABLE_COVERAGE)
        message(STATUS "Coverage enabled")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    endif(ENABLE_COVERAGE)

    message(STATUS "Compiling for ${CMAKE_BUILD_TYPE} WIN32 Host with flags: ${CMAKE_C_FLAGS}")

elseif(CMAKE_HOST_UNIX)
    include_directories(
        ${SDK_ROOT}/components/toolchain/gcc
        )

    option(ENABLE_INSTRUMENTATION "Enable instrumentation of functions" OFF)

    # Enable instrumentation of functions if requested:
    if(ENABLE_INSTRUMENTATION)
        message(STATUS "Instrumentation of functions enabled.")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -finstrument-functions")
        set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -finstrument-functions")
    endif(ENABLE_INSTRUMENTATION)

    # Enable coverage
    # NOTE: requires gcov and lcov (sudo apt-get install gcov lcov)
    # lcov generates output to ${CMAKE_BINARY_DIR}/coverage
    if(ENABLE_COVERAGE)
        message(STATUS "Coverage enabled")
        set(CMAKE_COMPILER_IS_GNUCXX ON)
        include("CMake/CodeCoverage.cmake")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")

        # NOTE: the '|| true' is there to force zero exit code for CTest
        setup_target_for_coverage(coverage "ctest || true" "coverage")
    endif(ENABLE_COVERAGE)

    message("Compiling for ${CMAKE_BUILD_TYPE} on Linux Host with flags: ${CMAKE_C_FLAGS}")

    function(set_properties_x86 target)
      if (NOT BUILD_HOST)
        message(FATAL_ERROR "Including x86 toolchain file for non-unit test build")
      endif (NOT BUILD_HOST)

      target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_S130_HEADERS})
      target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})
    endfunction(set_properties_x86)
else ()
    message(FATAL_ERROR "Platform not supported.")
endif ()

include_directories("${UNITY_ROOT}/src")
set(UNITY_SRC "${UNITY_ROOT}/src/unity.c")

enable_testing()

string(REGEX REPLACE ";" " " CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_C_FLAGS                "${CMAKE_C_FLAGS}" CACHE STRING "")
