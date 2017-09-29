## SOURCES section
set(WEAK
    ${MBTLE_SOURCE_DIR}/mesh/src/nrf_mesh_weak.c
    )

if (BUILD_HOST)
    if(CMAKE_HOST_UNIX)
        set(MBTLE_SRCS_CORE
            ${MBTLE_SOURCE_DIR}/mesh/src/host/aes_soft.c
            ${MBTLE_SOURCE_DIR}/mesh/src/host/packet_mgr_linux.c
            )
    elseif(CMAKE_HOST_WIN32)
        set(MBTLE_SRCS_CORE
            ${MBTLE_SOURCE_DIR}/mesh/src/host/aes_soft.c
            )
    endif()
else()
    set(MBTLE_SRCS_CORE
        ${MBTLE_SRCS_CORE}
        ${MBTLE_SOURCE_DIR}/mesh/src/core/aes.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/bearer_adv.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/fifo.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/queue.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/msqueue.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/packet_mgr.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/radio.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/timer.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/timer_scheduler.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/timeslot.c
        )
endif()

set(MBTLE_SRCS_CORE
    ${MBTLE_SRCS_CORE}
    ${MBTLE_SOURCE_DIR}/mesh/src/core/aes_cmac.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/beacon.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/bearer.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/bearer_event.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/cache.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_mesh_configure.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/ccm_soft.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/enc.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/event.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/hal.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/log.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/msg_cache.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/network.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/net_beacon.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/net_state.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_mesh.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_mesh_keygen.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_mesh_utils.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_mesh_opt.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/radio_config.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/packet.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/packet_buffer.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/rand.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/replay_cache.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/ticker.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/toolchain.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/transport.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/internal_event.c
    )

set(MBTLE_SRCS_FLASH
    ${MBTLE_SOURCE_DIR}/mesh/src/core/mesh_flash.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_flash.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/flash_manager.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/flash_manager_defrag.c
    ${MBTLE_SOURCE_DIR}/mesh/src/core/flash_manager_internal.c)

set(MBTLE_SRCS_PROV
    ${MBTLE_SRCS_PROV}
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/provisioning.c
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/nrf_mesh_prov.c
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/prov_bearer_adv.c
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/prov_utils.c
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/prov_beacon.c
    )

set(MBTLE_SRCS_CONFIG_SERVER
    "${MBTLE_SOURCE_DIR}/mesh/models/config/src/config_server.c"
    "${MBTLE_SOURCE_DIR}/mesh/models/config/src/packed_index_list.c"
    "${MBTLE_SOURCE_DIR}/mesh/models/config/src/composition_data.c"
    )

set(MBTLE_SRCS_ACCESS
    "${MBTLE_SOURCE_DIR}/mesh/src/access/access.c"
    "${MBTLE_SOURCE_DIR}/mesh/src/access/access_publish.c"
    "${MBTLE_SOURCE_DIR}/mesh/src/access/access_reliable.c"
    "${MBTLE_SOURCE_DIR}/mesh/src/access/device_state_manager.c"
    "${MBTLE_SOURCE_DIR}/mesh/src/access/nrf_mesh_node_config.c"
    "${MBTLE_SRCS_CONFIG_SERVER}"
    )

set(MBTLE_SRCS_PB_ADV_EE
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/prov_provisionee.c
    )

set(MBTLE_SRCS_PB_ADV_ER
    ${MBTLE_SOURCE_DIR}/mesh/src/prov/prov_provisioner.c
    )

set(MBTLE_SRCS_PB_REMOTE_SV
    ${MBTLE_SOURCE_DIR}/mesh/models/pb_remote/src/pb_remote_server.c
    )

set(MBTLE_SRCS_PB_REMOTE_CL
    ${MBTLE_SOURCE_DIR}/mesh/models/pb_remote/src/pb_remote_client.c
    )

set(MBTLE_SRCS_SERIAL
    # ${MBTLE_SOURCE_DIR}/mesh/models/pb_remote/src/serial_pb_remote_client.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/nrf_mesh_serial.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_bearer.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_common.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_access.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_app.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_config.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_device.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_dfu.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_mesh.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_models.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_openmesh.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_handler_prov.c
    ${MBTLE_SOURCE_DIR}/mesh/src/serial/serial_uart.c
    )

set(MBTLE_SRCS_DFU
    ${MBTLE_SOURCE_DIR}/mesh/src/dfu/nrf_mesh_dfu.c
    )

if (NOT PERSISTENT_STORAGE)
    set(MBTLE_SRCS_DFU
        ${MBTLE_SRCS_DFU}
        ${MBTLE_SOURCE_DIR}/mesh/src/core/nrf_flash.c
        ${MBTLE_SOURCE_DIR}/mesh/src/core/mesh_flash.c
        )
endif()
# end SOURCES

## HEADERS section

set(MBTLE_LOCAL_S110_HEADERS
    "${MBTLE_SOURCE_DIR}/lib/softdevice/s110/headers"
    )

set(MBTLE_LOCAL_S130_HEADERS
    "${MBTLE_SOURCE_DIR}/lib/softdevice/s130/headers"
    )

set(MBTLE_LOCAL_S132_HEADERS
    "${MBTLE_SOURCE_DIR}/lib/softdevice/s132/headers"
    )

set(MBTLE_LOCAL_TOOLCHAIN_HEADERS
    "${MBTLE_SOURCE_DIR}/toolchain"
    "${MBTLE_SOURCE_DIR}/toolchain/device"
    "${MBTLE_SOURCE_DIR}/toolchain/gcc"
    "${MBTLE_SOURCE_DIR}/toolchain/cmsis/include"
    )

set(MBTLE_LIB_HEADERS
    "${MBTLE_SOURCE_DIR}/lib/micro-ecc"
    "${MBTLE_SOURCE_DIR}/lib/rtt/include"
    )

set(MBTLE_SRC_HEADERS
    "${MBTLE_SOURCE_DIR}/mesh/api"
    "${MBTLE_SOURCE_DIR}/mesh/include/core"
    "${MBTLE_SOURCE_DIR}/mesh/include/prov"
    "${MBTLE_SOURCE_DIR}/mesh/include/dfu"
    "${MBTLE_SOURCE_DIR}/mesh/include/serial"
    "${MBTLE_SOURCE_DIR}/mesh/include/access"
    "${MBTLE_SOURCE_DIR}/mesh/models/pb_remote/include"
    "${MBTLE_SOURCE_DIR}/mesh/models/config/include"
    "${MBTLE_SOURCE_DIR}/examples/hal/include"
    )

set(MBTLE_HOST_HEADERS
    "${MBTLE_SOURCE_DIR}/mesh/include/host"
    )

if (NRF51_SOFTDEVICE STREQUAL "S110")
    set(MBTLE_NRF51_SD_HEADERS   "${MBTLE_LOCAL_S110_HEADERS}")
    set(MBTLE_NRF51_LINKER_FILE  "${MBTLE_SOURCE_DIR}/toolchain/s110_nrf51422xxAC.ld")
    set(MBTLE_NRF51_SCATTER_FILE "${MBTLE_SOURCE_DIR}/toolchain/s110_nrf51422xxAC.sct")
elseif (NRF51_SOFTDEVICE STREQUAL "S130")
    set(MBTLE_NRF51_SD_HEADERS   "${MBTLE_LOCAL_S130_HEADERS}")
    set(MBTLE_NRF51_LINKER_FILE  "${MBTLE_SOURCE_DIR}/toolchain/s130_nrf51422xxAC.ld")
    set(MBTLE_NRF51_SCATTER_FILE "${MBTLE_SOURCE_DIR}/toolchain/s130_nrf51422xxAC.sct")
else ()
    message(FATAL_ERROR "Unsupported SoftDevice version for nRF51: ${NRF51_SOFTDEVICE}")
endif()

if (NRF52_SOFTDEVICE STREQUAL "S132")
    set(MBTLE_NRF52_SD_HEADERS   "${MBTLE_LOCAL_S132_HEADERS}")
    set(MBTLE_NRF52_LINKER_FILE  "${MBTLE_SOURCE_DIR}/toolchain/s132_nrf52832xxAA.ld")
    set(MBTLE_NRF52_SCATTER_FILE "${MBTLE_SOURCE_DIR}/toolchain/s132_nrf52832xxAA.sct")
else()
    message(FATAL_ERROR "Unsupported SoftDevice version for nRF52: ${NRF52_SOFTDEVICE}")
endif()

## end HEADERS

## DEFINITIONS section
set(MBTLE_DEFINITIONS
    ${MBTLE_DEFINITIONS}
    "-DBLE_STACK_SUPPORT_REQD"
    "-DSOFTDEVICE_PRESENT"
    "-DNRF_MESH_SERIAL_ENABLE"
    )

## end DEFINITIONS
