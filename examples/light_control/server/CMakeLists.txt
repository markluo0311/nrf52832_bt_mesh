set(exec_name "light_control_server")

set(srcs
    "src/main.c"
    "${MBTLE_SRCS_PROV}"
    "${MBTLE_SRCS_ACCESS}"
    "${MBTLE_SOURCE_DIR}/examples/nrf_mesh_sdk.c"
    "${MBTLE_SOURCE_DIR}/examples/hal/src/simple_hal.c"
    "${MBTLE_SOURCE_DIR}/examples/models/simple_on_off/src/simple_on_off_server.c")

include_directories(
    "include"
    "${MBTLE_SOURCE_DIR}/examples/models/simple_on_off/include"
    "${MBTLE_SOURCE_DIR}/examples/hal/include"
    "${MBTLE_SOURCE_DIR}/mesh/models/config/include"
    "${MBTLE_LIB_HEADERS}"
    "${MBTLE_SRC_HEADERS}"
    "${MBTLE_SOURCE_DIR}/examples")

add_definitions(
    ${MBTLE_DEFINITIONS})

# MBTLE-1279 Add multi-platform support for example projects
if (HWID STREQUAL "51")
    add_definitions("-DBOARD_PCA10031")
elseif (HWID STREQUAL "52")
    add_definitions("-DBOARD_PCA10040")
else()
    message(FATAL_ERROR "${HWID}")
endif ()

add_executable_platforms("${exec_name}" ${srcs})

target_link_libraries_platforms(${exec_name}
    MBTLE_PB_ADV_EE
    MBTLE_CORE
    UECC_LIB
    RTT_LIB)

set_platform_compile_properties(${exec_name})
create_hex_platform(${exec_name})
add_flash_target(${exec_name})
