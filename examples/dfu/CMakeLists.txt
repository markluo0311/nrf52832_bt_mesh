get_testname(targetname)

set(SRCS
    "src/main.c"
    "${MBTLE_SOURCE_DIR}/examples/nrf_mesh_sdk.c"
    "${MBTLE_SRCS_ACCESS}"
    "${MBTLE_SRCS_SERIAL}"
    "${MBTLE_SRCS_DFU}"
    )

include_directories(
    include
    "${MBTLE_LIB_HEADERS}"
    "${MBTLE_SRC_HEADERS}"
    "${MBTLE_SOURCE_DIR}/examples"
    )

add_definitions(
    ${MBTLE_DEFINITIONS}
    )

if (HWID STREQUAL "51")
    add_definitions("-DBOARD_PCA10031")
elseif (HWID STREQUAL "52")
    add_definitions("-DBOARD_PCA10040")
else()
    message(FATAL_ERROR "${HWID}")
endif ()

set(exec_name "${targetname}")
add_executable_platforms("${exec_name}" ${SRCS})

# Libraries are Linked in order of increasing dependency.
target_link_libraries_platforms(${exec_name}
    MBTLE_PB_ADV_EE
    MBTLE_PB_ADV_ER
    MBTLE_CORE
    RTT_LIB
    UECC_LIB
    )

set_platform_compile_properties(${exec_name})
create_hex_platform(${exec_name})
add_flash_target(${exec_name})
