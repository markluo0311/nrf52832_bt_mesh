include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)

CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)

set(CMAKE_C_FLAGS
    ${CMAKE_C_FLAGS}
    "-std=gnu99"
    "-Wall"
    "-Wextra"
    "-Wno-inline"
    "-Wno-unused-parameter"           # Generates lots and lots of warnings from the SoftDevice headers
    "-Wno-missing-field-initializers" # Generates unnecessary warnings about uninitialized internal use fields
    "-Werror=implicit-function-declaration"
    "-Wno-format"
    "-ffunction-sections"
    "-fdata-sections"
    "-fno-strict-aliasing"
    "-fshort-enums"
    "-fno-builtin"
    "-O" # Enable optimization by default to fit DFU example
    )
# When we break up long strings in CMake we get semicolon
# separated lists, undo this here...
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "")

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-Og -g")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g")

##  NRF51 && NRF52 SUPPORT
function(set_properties_nrf51 target)
    if(NOT BUILD_HOST)
        set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
        set_property(TARGET ${target} APPEND PROPERTY SOURCES
            "${MBTLE_SOURCE_DIR}/toolchain/gcc/gcc_startup_nrf51.S"
            "${MBTLE_SOURCE_DIR}/toolchain/system_nrf51.c"
            )
        set_property(TARGET ${target} APPEND PROPERTY COMPILE_DEFINITIONS ${NRF51_C_FLAGS})

        target_include_directories(${target} PRIVATE "${MBTLE_NRF51_SD_HEADERS}")
        set(M0_LINK_FLAGS
            ${M0_C_FLAGS}
            "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${target}.map"
            "-L${MBTLE_SOURCE_DIR}/toolchain/gcc"
            "-T${MBTLE_NRF51_LINKER_FILE}"
            "-Wl,--gc-sections"
            "--specs=nano.specs"
            "-mthumb"
            "-mcpu=cortex-m0"
            "-mabi=aapcs"
            )
        string(REGEX REPLACE ";" " " M0_LINK_FLAGS "${M0_LINK_FLAGS}")

        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS ${M0_LINK_FLAGS})

        set(M0_C_FLAGS
            "-mthumb"
            "-mcpu=cortex-m0"
            "-mfloat-abi=soft"
            "-mabi=aapcs"
            )

        target_compile_options(${target} PRIVATE ${M0_C_FLAGS})
        target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})
    endif()
endfunction()

function(set_properties_nrf52 target)
    if(NOT BUILD_HOST)

        set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
        set_property(TARGET ${target} APPEND PROPERTY SOURCES
            "${MBTLE_SOURCE_DIR}/toolchain/gcc/gcc_startup_nrf52.S"
            "${MBTLE_SOURCE_DIR}/toolchain/system_nrf52.c"
            )
        set_property(TARGET ${target} APPEND PROPERTY COMPILE_DEFINITIONS ${NRF52_C_FLAGS})

        set(M4_LINK_FLAGS
            "-Xlinker -Map=${CMAKE_CURRENT_BINARY_DIR}/${target}.map"
            "-L${MBTLE_SOURCE_DIR}/toolchain/gcc"
            "-T${MBTLE_NRF52_LINKER_FILE}"
            "-Wl,--gc-sections"
            "--specs=nano.specs"
            "-mthumb"
            "-mcpu=cortex-m4"
            "-mabi=aapcs"
            "-mfloat-abi=hard"
            "-mfpu=fpv4-sp-d16"
            )
        string(REGEX REPLACE ";" " " M4_LINK_FLAGS "${M4_LINK_FLAGS}")
        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS ${M4_LINK_FLAGS})

        set(M4_C_FLAGS
            "-mthumb"
            "-mcpu=cortex-m4"
            "-mfloat-abi=hard"
            "-mfpu=fpv4-sp-d16"
            "-mabi=aapcs"
            )
        target_compile_options(${target} PRIVATE ${M4_C_FLAGS})

        target_include_directories(${target} PRIVATE ${MBTLE_NRF52_SD_HEADERS})
        target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})

    endif()
endfunction()

set(BUILD_SHARED_LIBS OFF)
