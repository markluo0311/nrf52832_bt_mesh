
# find_package(Ruby) complains about ruby libraries. We only want the executable.
if (WIN32)
    set(RUBY_EXECUTABLE "ruby.exe")
else ()
    set(RUBY_EXECUTABLE "ruby")
endif ()

set(CMOCK_BIN ${MBTLE_SOURCE_DIR}/mesh/test/mocks)
file(MAKE_DIRECTORY ${CMOCK_BIN})

file(GLOB MOCK_SRC
    "${MBTLE_SOURCE_DIR}/mesh/api/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/include/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/include/core/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/include/prov/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/include/dfu/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/include/serial/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/models/config/include/*.h"
    "${MBTLE_SOURCE_DIR}/mesh/models/pb_remote/include/*.h"
    "${MBTLE_SOURCE_DIR}/lib/micro-ecc/*.h"
    )

# Generate targets for mock files for all headers in ${MOCK_SRC}:
foreach(headerpath IN ITEMS ${MOCK_SRC})
    get_filename_component(header ${headerpath} NAME_WE)

    add_custom_command(OUTPUT ${CMOCK_BIN}/${header}_mock.c ${CMOCK_BIN}/${header}_mock.h
        COMMAND ${RUBY_EXECUTABLE} ${CMOCK_ROOT}/lib/cmock.rb -o${CMAKE_CURRENT_SOURCE_DIR}/mocks.yml "${headerpath}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${headerpath}
        COMMENT "Generating mock for ${header}.h..."
        )
endforeach(headerpath)

include_directories(BEFORE
    "include"
    "include/device_wrappers"
    "mocks"
    "../models/pb_remote/include"
    "${CMOCK_ROOT}/src/"
    )

add_definitions(${MBTLE_DEFINITIONS})

set(CMOCK_SRC ${CMOCK_ROOT}/src/cmock.c)

# Adds a unit test, with automatic test runner generation.
function(add_unit_test testname sources)
    add_executable(ut_${testname}
        ${sources}
        ${UNITY_SRC}
        ${CMOCK_SRC}
        ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test_assert.c)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        COMMAND ${RUBY_EXECUTABLE} ${UNITY_ROOT}/auto/generate_test_runner.rb ${CMAKE_CURRENT_SOURCE_DIR}/src/ut_${testname}.c
        ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        DEPENDS ${sources}
        )

    target_compile_options(ut_${testname} PRIVATE "-g") # Always add debugging flag for unit tests
    target_compile_definitions(ut_${testname} PUBLIC
        "-DNRF_MESH_LOG_ENABLE=1"
        "-DLOG_CALLBACK_DEFAULT=log_callback_stdout"
        "-DUNIT_TEST=1")

    set_platform_compile_properties(${testname})
    add_test(${testname} ut_${testname})
    if(ARGV2)
        target_compile_definitions(ut_${testname} PUBLIC ${ARGV2})
    endif(ARGV2)

    set(lint_sources)
    foreach(source ${sources})
        if(NOT ${source} MATCHES ".*_mock.c")
            set(lint_sources ${lint_sources} ${source})
        endif()
    endforeach()

    add_pc_lint(ut_${testname} ${lint_sources})

endfunction(add_unit_test)

# Adds a unit test, with automatic test runner generation.
function(add_mtt_test testname sources)
    add_executable(ut_${testname} ${sources} )
    target_include_directories(ut_${testname} PRIVATE mttest)

    # Add debugging flag for unit tests always.
    set(mtt_test_flags
        "-g"
        "-DNRF_MESH_LOG_ENABLE=1"
        "-DLOG_CALLBACK_DEFAULT=log_callback_stdout"
        "-DMTT_TEST=1"
        )
    target_compile_options(ut_${testname} PRIVATE ${mtt_test_flags})

    set_platform_compile_properties(${testname})
    target_link_libraries_platforms(${testname} mttest)
    add_test(${testname} ut_${testname})
    if(ARGV2)
        target_compile_definitions(ut_${testname} PUBLIC ${ARGV2})
    endif(ARGV2)
endfunction(add_mtt_test)

add_subdirectory(mttest)

set(packet_mgr_mtt_srcs
    src/mtt_packet_mgr.c
    ../src/core/packet_mgr.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_mtt_test(mtt_packet_mgr "${packet_mgr_mtt_srcs}")

# Transport Layer - transport
set(transport_test_srcs
    src/ut_transport.c
    ../src/core/transport.c
    ../src/core/packet.c
    ../src/host/packet_mgr_linux.c
    ../src/core/enc.c
    ../src/host/aes_soft.c
    ../src/core/ccm_soft.c
    ../src/core/rand.c
    ../src/core/aes_cmac.c
    ../src/core/toolchain.c
    ../src/core/replay_cache.c
    ../src/core/log.c
    ../src/core/nrf_mesh_utils.c
    mocks/nrf_mesh_dfu_mock.c
    mocks/bearer_event_mock.c
    mocks/net_state_mock.c
    )
add_unit_test(transport "${transport_test_srcs}")

# Network Layer - network
set(network_test_srcs
    src/ut_network.c
    ../src/core/fifo.c
    ../src/core/network.c
    ../src/core/packet.c
    ../src/core/enc.c
    ../src/host/aes_soft.c
    ../src/core/ccm_soft.c
    ../src/core/aes_cmac.c
    ../src/core/rand.c
    ../src/core/toolchain.c
    ../src/core/log.c
    ../src/host/packet_mgr_linux.c
    ../src/core/replay_cache.c
    ../src/core/nrf_mesh_utils.c
    mocks/bearer_mock.c
    mocks/msg_cache_mock.c
    mocks/transport_mock.c
    mocks/event_mock.c
    mocks/net_beacon_mock.c
    mocks/net_state_mock.c
    )
add_unit_test(network "${network_test_srcs}")

# Bearer - bearer
set(bearer_test_srcs
    src/ut_bearer.c
    ../src/core/bearer.c
    ../src/core/fifo.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(bearer "${bearer_test_srcs}" "-DBEARER_TYPES=0xffffffff")

# Timer - timer
set(timer_test_srcs
    src/ut_timer.c
    ../src/core/timer.c
    ../src/core/toolchain.c
    )
add_unit_test(timer "${timer_test_srcs}" "-DNRF51")

# Message Cache - msg_cache
set(msg_cache_test_srcs
    src/ut_msg_cache.c
    ../src/core/msg_cache.c
    ../src/core/toolchain.c
    )
add_unit_test(msg_cache "${msg_cache_test_srcs}")

# Packet Module - packet
set(packet_test_srcs
    src/ut_packet.c
    ../src/core/packet.c
    ../src/core/toolchain.c
    )
add_unit_test(packet "${packet_test_srcs}")

# Radio - radio config
set(radio_config_test_srcs
    src/ut_radio_config.c
    ../src/core/radio_config.c
    )
add_unit_test(radio_config "${radio_config_test_srcs}" "-DNRF51")

# Radio - radio
set(radio_test_srcs
    src/ut_radio.c
    ../src/core/radio.c
    ../src/core/radio_config.c
    ../src/core/fifo.c
    ../src/core/toolchain.c
    )
add_unit_test(radio "${radio_test_srcs}" "-DNRF51")

# Timer scheduler
set(timer_sch_test_srcs
    src/ut_timer_scheduler.c
    ../src/core/timer_scheduler.c
    ../src/core/fifo.c
    ../src/core/toolchain.c
    )
add_unit_test(timer_scheduler "${timer_sch_test_srcs}")

# Advertiser Bearer - bearer_adv
set(bearer_adv_test_srcs
    src/ut_bearer_adv.c
    ../src/core/bearer_adv.c
    ../src/core/fifo.c
    ../src/core/packet.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(bearer_adv "${bearer_adv_test_srcs}" "-DSOFTDEVICE_PRESENT -DNRF51")

# Packet Manager - packet_mgr
set(packet_mgr_test_srcs
    src/ut_packet_mgr.c
    ../src/core/packet_mgr.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(packet_mgr "${packet_mgr_test_srcs}" "-DPACKET_MGR_DEBUG_MODE=1")

# Packet Buffer - packet_buffer
set(packet_buffer_test_srcs
    src/ut_packet_buffer.c
    ../src/core/packet_buffer.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(packet_buffer "${packet_buffer_test_srcs}" "-DPACKET_BUFFER_DEBUG_MODE=1")

# CCM Software implementation - ccm_soft
set(ccm_soft_test_srcs
    src/ut_ccm_soft.c
    ../src/host/aes_soft.c
    ../src/core/ccm_soft.c
    ../src/core/log.c
    )
add_unit_test(ccm_soft "${ccm_soft_test_srcs}")

# AES-CMAC - aes_cmac
set(aes_cmac_test_srcs
    src/ut_aes_cmac.c
    ../src/core/aes_cmac.c
    ../src/host/aes_soft.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(aes_cmac "${aes_cmac_test_srcs}")

# Timeslot
set(timeslot_test_srcs
    src/ut_timeslot.c
    ../src/core/timeslot.c
    ../src/core/toolchain.c
    )
add_unit_test(timeslot "${timeslot_test_srcs}" "-DNRF51")

# Encryption
set(enc_test_srcs
    src/ut_enc.c
    ../src/core/enc.c
    ../src/core/rand.c
    ../src/host/aes_soft.c
    ../src/core/aes_cmac.c
    ../src/core/ccm_soft.c
    ../src/core/toolchain.c
    ../src/core/log.c
    )
add_unit_test(enc "${enc_test_srcs}")

# Keygen
set(keygen_srcs
    src/ut_keygen.c
    ../src/core/nrf_mesh_keygen.c
    ../src/core/enc.c
    ../src/core/rand.c
    ../src/host/aes_soft.c
    ../src/core/ccm_soft.c
    ../src/core/aes_cmac.c
    ../src/core/log.c
    )
add_unit_test(keygen "${keygen_srcs}")

# Beacon
set(beacon_test_srcs
    src/ut_beacon.c
    ../src/core/beacon.c
    mocks/packet_mgr_mock.c
    mocks/bearer_adv_mock.c
    mocks/net_beacon_mock.c
    mocks/prov_beacon_mock.c
    ../src/core/log.c
    )
add_unit_test(beacon "${beacon_test_srcs}")

# fifo
set(fifo_srcs
    src/ut_fifo.c
    ../src/core/fifo.c
    )
add_unit_test(fifo "${fifo_srcs}")

# CCM with additional data
set(ccm_ad_srcs
    src/ut_ccm_ad.c
    ../src/host/aes_soft.c
    ../src/core/ccm_soft.c
    ../src/core/log.c
    )
add_unit_test(ccm_ad "${ccm_ad_srcs}" "-DUSE_ADDITIONAL_DATA=1")

# Provisioning bearer - ADV
set(prov_bearer_adv_test_srcs
    src/ut_prov_bearer_adv.c
    ../src/prov/prov_bearer_adv.c
    ../src/core/log.c
    mocks/timer_mock.c
    mocks/timer_scheduler_mock.c
    mocks/bearer_adv_mock.c
    mocks/provisioning_mock.c
    mocks/packet_mgr_mock.c
    mocks/rand_mock.c
    mocks/prov_beacon_mock.c
    mocks/nrf_mesh_mock.c
    mocks/nrf_mesh_configure_mock.c
    )
add_unit_test(prov_bearer_adv "${prov_bearer_adv_test_srcs}")

# The Great Provisioning Test - Provisioner vs Provisionee - two in one!
set(prov_provisioning_test_srcs
    src/ut_prov_provisioning.c
    )
add_unit_test(prov_provisioning "${prov_provisioning_test_srcs}")
target_link_libraries(ut_prov_provisioning ut_UECC_LIB)

# Linked list:
set(linked_list_srcs
    src/ut_linked_list.c
    )
add_unit_test(linked_list "${linked_list_srcs}")

# Replay Cache:
set(replay_cache_srcs
    src/ut_replay_cache.c
    ../src/core/replay_cache.c
    )
add_unit_test(replay_cache "${replay_cache_srcs}")

set(serial_packet_srcs
    src/ut_serial_packet.c
    )
add_unit_test(serial_packet "${serial_packet_srcs}")

# Serial handler access module unit test
set(serial_handler_access_srcs
    src/ut_serial_handler_access.c
    ../src/serial/serial_handler_access.c
    ../src/serial/serial_handler_common.c
    mocks/serial_mock.c
    mocks/access_config_mock.c
    mocks/access_mock.c
    )
add_unit_test(serial_handler_access "${serial_handler_access_srcs}")

# Serial handler models module unit test
set(serial_handler_models_srcs
    src/ut_serial_handler_models.c
    ../src/serial/serial_handler_models.c
    ../src/serial/serial_handler_common.c
    mocks/serial_mock.c
    mocks/access_config_mock.c
    )
add_unit_test(serial_handler_models "${serial_handler_models_srcs}" "-DACCESS_MODEL_COUNT=2")

# Serial handler for the pb remote client
set(serial_pb_remote_client_srcs
    src/ut_serial_pb_remote_client.c
    ../models/pb_remote/src/serial_pb_remote_client.c
    mocks/serial_handler_models_mock.c
    mocks/serial_mock.c
    mocks/pb_remote_client_mock.c
    mocks/access_config_mock.c
    )
add_unit_test(serial_pb_remote_client "${serial_pb_remote_client_srcs}" "-DSERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT=2")

# Serial handler mesh
set(serial_handler_mesh_srcs
    src/ut_serial_handler_mesh.c
    ../src/serial/serial_handler_mesh.c
    ../src/serial/serial_handler_common.c
    ../src/core/log.c
    mocks/nrf_mesh_mock.c
    mocks/nrf_mesh_events_mock.c
    mocks/serial_mock.c
    mocks/access_mock.c
    mocks/net_state_mock.c
    mocks/device_state_manager_mock.c
    )
add_unit_test(serial_handler_mesh "${serial_handler_mesh_srcs}")

# Serial handler device
set(serial_handler_device_srcs
    src/ut_serial_handler_device.c
    ../src/serial/serial_handler_device.c
    ../src/serial/serial_handler_common.c
    ../src/core/log.c
    mocks/hal_mock.c
    mocks/serial_mock.c
    mocks/radio_mock.c
    mocks/bearer_adv_mock.c
    mocks/packet_mgr_mock.c
    )
add_unit_test(serial_handler_device "${serial_handler_device_srcs}" "-DNRF_MESH_SERIAL_BEACON_SLOTS=3")

# Serial handler DFU
set(serial_handler_dfu_srcs
    src/ut_serial_handler_dfu.c
    ../src/serial/serial_handler_dfu.c
    mocks/serial_mock.c
    mocks/nrf_mesh_dfu_mock.c
    mocks/nrf_mesh_events_mock.c
    )
add_unit_test(serial_handler_dfu "${serial_handler_dfu_srcs}" "-DNRF_MESH_DFU_ENABLE")

# Application serial handler
set(serial_handler_app_srcs
    src/ut_serial_handler_app.c
    ../src/serial/serial_handler_app.c
    mocks/serial_mock.c
    )
add_unit_test(serial_handler_app "${serial_handler_app_srcs}")

# Config serial handler
set(serial_handler_config_srcs
    src/ut_serial_handler_config.c
    ../src/serial/serial_handler_config.c
    mocks/bearer_adv_mock.c
    mocks/serial_mock.c
    mocks/nrf_mesh_configure_mock.c
    )
add_unit_test(serial_handler_config "${serial_handler_config_srcs}")

# OpenMesh serial handler
set(serial_handler_openmesh_srcs
    src/ut_serial_handler_openmesh.c
    ../src/serial/serial_handler_openmesh.c
    mocks/serial_mock.c
    mocks/nrf_mesh_dfu_mock.c
    mocks/hal_mock.c
    )
add_unit_test(serial_handler_openmesh "${serial_handler_openmesh_srcs}" "-DNRF_MESH_DFU_ENABLE")

# Flash module
set(mesh_flash_srcs
    src/ut_mesh_flash.c
    ../src/core/mesh_flash.c
    ../src/core/msqueue.c
    mocks/nrf_flash_mock.c
    mocks/timer_mock.c
    )
add_unit_test(mesh_flash "${mesh_flash_srcs}")

# nRF Mesh API
set(nrf_mesh_srcs
    src/ut_nrf_mesh.c
    ../src/core/nrf_mesh.c
    ../src/core/log.c
    ../src/core/packet_mgr.c
    ../src/core/nrf_mesh_utils.c
    mocks/toolchain_mock.c
    mocks/timer_scheduler_mock.c
    mocks/transport_mock.c
    mocks/network_mock.c
    mocks/msg_cache_mock.c
    mocks/bearer_mock.c
    mocks/enc_mock.c
    mocks/rand_mock.c
    mocks/nrf_mesh_dfu_mock.c
    mocks/nrf_mesh_configure_mock.c
    mocks/bearer_event_mock.c
    mocks/beacon_mock.c
    mocks/event_mock.c
    mocks/prov_bearer_adv_mock.c
    mocks/ticker_mock.c
    mocks/mesh_flash_mock.c
    mocks/flash_manager_mock.c
    )
add_unit_test(nrf_mesh "${nrf_mesh_srcs}")

set(serial_srcs
    src/ut_serial.c
    ../src/serial/serial.c
    ../src/core/log.c
    mocks/serial_handler_access_mock.c
    mocks/serial_handler_models_mock.c
    mocks/bearer_event_mock.c
    mocks/serial_bearer_mock.c
    mocks/serial_handler_app_mock.c
    mocks/serial_handler_config_mock.c
    mocks/serial_handler_dfu_mock.c
    mocks/serial_handler_mesh_mock.c
    mocks/serial_handler_openmesh_mock.c
    mocks/serial_handler_device_mock.c
    mocks/serial_handler_prov_mock.c
    mocks/serial_handler_dfu_mock.c
    )
add_unit_test(serial "${serial_srcs}" "-DNRF51")

# Serial bearer unit test - serial_bearer_slip_enc
set(serial_bearer_slip_enc_srcs
    src/ut_serial_bearer_slip_enc.c
    src/ut_serial_bearer_common.c
    ../src/serial/serial_bearer.c
    mocks/serial_mock.c
    mocks/serial_uart_mock.c
    mocks/packet_buffer_mock.c
    mocks/bearer_event_mock.c
    )
add_unit_test(serial_bearer_slip_enc "${serial_bearer_slip_enc_srcs}" "-DNRF51;-DSERIAL_SLIP_ENCODING")

# Serial bearer unit test - serial_bearer_simple
set(serial_bearer_simple_srcs
    src/ut_serial_bearer_simple.c
    src/ut_serial_bearer_common.c
    ../src/serial/serial_bearer.c
    mocks/serial_mock.c
    mocks/serial_uart_mock.c
    mocks/packet_buffer_mock.c
    mocks/bearer_event_mock.c
    )
add_unit_test(serial_bearer_simple "${serial_bearer_simple_srcs}" "-DNRF51")

set(ticker_srcs
    src/ut_ticker.c
    ../src/core/ticker.c
    mocks/timer_mock.c
    mocks/timer_scheduler_mock.c
    )
add_unit_test(ticker "${ticker_srcs}" "-DNRF51")

set(queue_srcs
  src/ut_queue.c
  ../src/core/queue.c
  )
add_unit_test(queue "${queue_srcs}")

set(net_beacon_srcs
    src/ut_net_beacon.c
    ../src/core/net_beacon.c
    mocks/timer_mock.c
    mocks/timer_scheduler_mock.c
    mocks/beacon_mock.c
    mocks/enc_mock.c
    mocks/rand_mock.c
    mocks/net_state_mock.c
    ../src/core/log.c
    )
add_unit_test(net_beacon "${net_beacon_srcs}" "-DNRF51")

set(prov_beacon_srcs
    src/ut_prov_beacon.c
    ../src/prov/prov_beacon.c
    mocks/beacon_mock.c
    mocks/nrf_mesh_configure_mock.c
    mocks/event_mock.c
    mocks/enc_mock.c
    ../src/core/log.c
    )
add_unit_test(prov_beacon "${prov_beacon_srcs}" "-DNRF51")

set(provisioning_srcs
    src/ut_provisioning.c
    ../src/prov/provisioning.c
    mocks/nrf_mesh_mock.c
    mocks/prov_bearer_adv_mock.c
    mocks/event_mock.c
    mocks/timer_scheduler_mock.c
    mocks/enc_mock.c
    mocks/timer_mock.c
    ../src/core/log.c
    )
add_unit_test(provisioning "${provisioning_srcs}")

set(prov_utils_srcs
    src/ut_prov_utils.c
    ../src/prov/prov_utils.c
    ../src/core/log.c
    mocks/enc_mock.c
    mocks/uECC_mock.c
    mocks/rand_mock.c
    )
add_unit_test(prov_utils "${prov_utils_srcs}")

set(access_srcs
    src/ut_access.c
    ../src/access/access.c
    ../src/core/log.c
    ../src/core/fifo.c
    mocks/device_state_manager_mock.c
    mocks/flash_manager_mock.c
    mocks/nrf_mesh_mock.c
    mocks/nrf_mesh_events_mock.c
    mocks/access_publish_mock.c
    mocks/access_reliable_mock.c
    mocks/bearer_event_mock.c
    mocks/event_mock.c
    )
set(access_defines
    -DACCESS_ELEMENT_COUNT=2
    -DACCESS_MODEL_COUNT=16
    -DACCESS_SUBSCRIPTION_LIST_COUNT=15    # One less than the number of models
    -DDSM_NONVIRTUAL_ADDR_MAX=30)
add_unit_test(access "${access_srcs}" "${access_defines}")

set(access_reliable_srcs
    src/ut_access_reliable.c
    mocks/access_mock.c
    mocks/access_config_mock.c
    mocks/timer_mock.c
    mocks/timer_scheduler_mock.c
    mocks/bearer_event_mock.c
    ../src/access/access_reliable.c)
set(access_reliable_defines
    -DACCESS_MODEL_COUNT=16
    -DACCESS_RELIABLE_TRANSFER_COUNT=8)
add_unit_test(access_reliable "${access_reliable_srcs}" "${access_reliable_defines}")

set(access_publish_srcs
    src/ut_access_publish.c
    mocks/bearer_event_mock.c
    mocks/timer_scheduler_mock.c
    ../src/access/access_publish.c
    )
add_unit_test(access_publish "${access_publish_srcs}")

set(config_client_srcs
    src/ut_config_client.c
    mocks/access_mock.c
    mocks/access_reliable_mock.c
    mocks/access_config_mock.c
    mocks/packet_mgr_mock.c
    ../models/config/src/config_client.c
    ../models/config/src/packed_index_list.c
    )
add_unit_test(config_client "${config_client_srcs}")

set(config_server_srcs
    src/ut_config_server.c
    ../models/config/src/config_server.c
    ../models/config/src/packed_index_list.c
    ../src/core/nrf_mesh_utils.c
    mocks/composition_data_mock.c
    mocks/access_mock.c
    mocks/access_config_mock.c
    mocks/device_state_manager_mock.c
    mocks/net_beacon_mock.c
    mocks/nrf_mesh_opt_mock.c
    mocks/nrf_mesh_keygen_mock.c
    mocks/rand_mock.c
    )
set(config_server_defines
    -DACCESS_ELEMENT_COUNT=1024
    -DACCESS_SUBSCRIPTION_LIST_COUNT=3)
add_unit_test(config_server "${config_server_srcs}" "${config_server_defines}")

set(composition_data_srcs
    src/ut_composition_data.c
    mocks/access_config_mock.c
    ../models/config/src/composition_data.c
    )
set(composition_data_defines
    -DDEVICE_COMPANY_ID=0x000C
    -DDEVICE_PRODUCT_ID=0x001A
    -DDEVICE_VENDOR_ID=0x0001
    -DREPLAY_CACHE_ENTRIES=0x0008
    -DDEVICE_FEATURES=0x0003
    -DACCESS_ELEMENT_COUNT=1
    -DACCESS_MODEL_COUNT=7
    )
add_unit_test(composition_data "${composition_data_srcs}" "${composition_data_defines}")

set(device_state_manager_srcs
    src/ut_device_state_manager.c
    ../src/access/device_state_manager.c
    ../src/core/nrf_mesh_utils.c
    mocks/rand_mock.c
    mocks/nrf_mesh_mock.c
    mocks/nrf_mesh_keygen_mock.c
    mocks/net_state_mock.c
    mocks/flash_manager_mock.c
    mocks/event_mock.c
    mocks/bearer_event_mock.c
    )
add_unit_test(device_state_manager "${device_state_manager_srcs}")

set(net_state_srcs
    src/ut_net_state.c
    ../src/core/net_state.c
    mocks/event_mock.c
    mocks/flash_manager_mock.c
    )
add_unit_test(net_state "${net_state_srcs}")

set(bitfield_srcs
    src/ut_bitfield.c
    )
add_unit_test(bitfield "${bitfield_srcs}")

set(nrf_mesh_configure_srcs
    src/ut_nrf_mesh_configure.c
    ../src/core/nrf_mesh_configure.c
    )
add_unit_test(nrf_mesh_configure "${nrf_mesh_configure_srcs}" "-DNRF51")

set(bearer_event_srcs
    src/ut_bearer_event.c
    ../src/core/bearer_event.c
    ../src/core/fifo.c
    )
add_unit_test(bearer_event "${bearer_event_srcs}" "-DNRF51")

set(nrf_mesh_node_config_srcs
    src/ut_nrf_mesh_node_config.c
    ../src/access/nrf_mesh_node_config.c
    ../src/core/log.c
    mocks/config_server_mock.c
    mocks/device_state_manager_mock.c
    mocks/nrf_mesh_mock.c
    mocks/nrf_mesh_events_mock.c
    mocks/nrf_mesh_prov_mock.c
    mocks/net_state_mock.c
    mocks/access_mock.c
    mocks/access_config_mock.c
    mocks/flash_manager_mock.c
    )
add_unit_test(nrf_mesh_node_config "${nrf_mesh_node_config_srcs}" "-DNRF51")

set(flash_manager_srcs
    src/ut_flash_manager.c
    src/flash_manager_test_util.c
    ../src/core/flash_manager.c
    ../src/core/flash_manager_internal.c
    ../src/core/packet_buffer.c
    ../src/core/fifo.c
    ../src/core/queue.c
    mocks/flash_manager_defrag_mock.c
    )
add_unit_test(flash_manager "${flash_manager_srcs}")

set(flash_manager_defrag_srcs
    src/ut_flash_manager_defrag.c
    src/flash_manager_test_util.c
    ../src/core/flash_manager_defrag.c
    ../src/core/fifo.c
    ../src/core/flash_manager_internal.c
    mocks/flash_manager_mock.c
    )
add_unit_test(flash_manager_defrag "${flash_manager_defrag_srcs}" "-DNRF51")

set(msqueue_srcs
    src/ut_msqueue.c
    ../src/core/msqueue.c
    mocks/mesh_flash_mock.c
    )
add_unit_test(msqueue "${msqueue_srcs}")
