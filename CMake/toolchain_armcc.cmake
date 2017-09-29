# Generic Mesh CMake functionality
set(CMAKE_SYSTEM_NAME Generic)

# CMake does not provide a macro for forcing the assembler
macro(CMAKE_FORCE_ASM_COMPILER compiler id)
    set(CMAKE_ASM_COMPILER "${compiler}")
    set(CMAKE_ASM_COMPILER_ID_RUN TRUE)
    set(CMAKE_ASM_COMPILER_ID ${id})
    set(CMAKE_ASM_COMPILER_FORCED TRUE)
endmacro()

# Path to our own compiler modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/Modules)

# Toolchain paths
if (WIN32 OR CYGWIN)
    set(KEIL_PATH           C:/keil_v5)
    set(TOOLCHAIN_SYSROOT   ${KEIL_PATH}/ARM/ARMCC/BIN)
else()
    set(TOOLCHAIN_SYSROOT   "")
endif()

FIND_PROGRAM(ARMCC_COMPILER armcc PATHS ${TOOLCHAIN_SYSROOT} ENV PATH NO_DEFAULT_PATH)
FIND_PROGRAM(ARMCC_LINKER armasm PATHS ${TOOLCHAIN_SYSROOT} ENV PATH NO_DEFAULT_PATH)
FIND_PROGRAM(ARMCC_FROMELF fromelf PATHS ${TOOLCHAIN_SYSROOT} ENV PATH NO_DEFAULT_PATH)

if (NOT ARMCC_COMPILER)
    message(FATAL_ERROR "Could not find 'armcc' compiler.")
endif (NOT ARMCC_COMPILER)
if (NOT ARMCC_LINKER)
    message(FATAL_ERROR "Could not find 'armasm' linker.")
endif (NOT ARMCC_LINKER)
if (NOT ARMCC_FROMELF)
    message(WARNING "Could not find 'fromelf' program, no hex files will be generated.")
endif (NOT ARMCC_FROMELF)

# Force the ARMCC compiler (not part of the CMake built-in compiler set)
# Note: replace this with CMAKE_C_COMPILER once our Modules/Compiler files are upstream
include(CMakeForceCompiler)
CMAKE_FORCE_C_COMPILER     (${ARMCC_COMPILER} ARMCC)
# Force the ARMCC assembler (no macro available)
# Note: replace this with CMAKE_ASM_COMPILER once our Modules/Compiler files are upstream
CMAKE_FORCE_ASM_COMPILER   (${ARMCC_LINKER} ARMCC)

MESSAGE( STATUS "CMAKE_C_COMPILER_ID:         " ${CMAKE_C_COMPILER_ID} )
MESSAGE( STATUS "CMAKE_ASM_COMPILER_ID:       " ${CMAKE_ASM_COMPILER_ID} )

set(CMAKE_CROSSCOMPILING true)
set(CMAKE_SKIP_INSTALL_RULES true)

set(CMAKE_ASM_FLAGS       "--cpreproc --apcs=interwork --cpreproc_opts=\"-DASSEMBLER_PREPROC__\"")
set(CMAKE_C_FLAGS         "--c99")
set(CMAKE_C_FLAGS_RELEASE "-O3 --no_debug")
set(CMAKE_C_FLAGS_DEBUG "-O1 --debug")
set(CMAKE_C_FLAGS_MINSIZEREL "-Ospace --no_debug")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 --debug")

set(CMAKE_DEPFILE_FLAGS_C "--depend=<DEPFILE> --depend_single_line --no_depend_system_headers")
set(CMAKE_C_LINK_FLAGS    "--strict --summary_stderr --info summarysizes --map --xref --info=stack --callgraph --symbols --info sizes --info totals --info unused --info veneers" CACHE STRING "")

##  NRF51 && NRF52 SUPPORT
function(set_properties_nrf51 target)
    if (BUILD_HOST)
        message(FATAL_ERROR "Including ARMCC toolchain file for unit test build")
    endif (BUILD_HOST)

    set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
    set_property(TARGET ${target} APPEND PROPERTY SOURCES
        "${MBTLE_SOURCE_DIR}/toolchain/arm/arm_startup_nrf51.s"
        "${MBTLE_SOURCE_DIR}/toolchain/system_nrf51.c"
        )
    set_property(TARGET ${target} APPEND PROPERTY COMPILE_DEFINITIONS ${NRF51_C_FLAGS})
    set_property(TARGET ${target} APPEND PROPERTY LINK_FLAGS "--scatter ${MBTLE_NRF51_SCATTER_FILE}")
    target_include_directories(${target} PRIVATE ${MBTLE_NRF51_SD_HEADERS})

    set(M0_C_FLAGS "--cpu=cortex-m0")
    target_compile_options(${target} PRIVATE ${M0_C_FLAGS})
    target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})
endfunction()

function(set_properties_nrf52 target)
    if (BUILD_HOST)
        message(FATAL_ERROR "Including ARMCC toolchain file for unit test build")
    endif (BUILD_HOST)

    set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
    set_property(TARGET ${target} APPEND PROPERTY SOURCES
        "${MBTLE_SOURCE_DIR}/toolchain/arm/arm_startup_nrf52.s"
        "${MBTLE_SOURCE_DIR}/toolchain/system_nrf52.c"
        )
    set_property(TARGET ${target} APPEND PROPERTY COMPILE_DEFINITIONS ${NRF52_C_FLAGS})
    set_property(TARGET ${target} APPEND PROPERTY LINK_FLAGS "--scatter ${MBTLE_NRF52_SCATTER_FILE}")

    set(M4_C_FLAGS "--cpu=cortex-m4")
    target_compile_options(${target} PRIVATE ${M4_C_FLAGS})
    target_include_directories(${target} PRIVATE ${MBTLE_NRF52_SD_HEADERS})
    target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})
endfunction()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "")
