/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unity.h>
#include <cmock.h>

#include "serial_handler_mesh.h"

#include "serial_status.h"

#include "serial_mock.h"
#include "net_state_mock.h"
#include "access_mock.h"
#include "device_state_manager_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_assert.h"

#define CMD_LENGTH_CHECK(_opcode, _intended_length)                                                   \
    do {                                                                                              \
        serial_packet_t _cmd;                                                                         \
        _cmd.opcode = _opcode;                                                                        \
        _cmd.length = _intended_length + 1;                                                           \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0, 0); \
        serial_handler_mesh_rx(&_cmd);                                                                \
        _cmd.length = _intended_length + 1;                                                           \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0, 0); \
        serial_handler_mesh_rx(&_cmd);                                                                \
    } while (0)

static nrf_mesh_tx_params_t m_expected_tx_params;
static uint32_t m_expected_packet_send;
static uint32_t m_packet_send_return;
static nrf_mesh_evt_handler_t * mp_evt_handler;
static serial_packet_t m_expected_tx_packet;
static uint32_t m_expected_serial_tx;

static uint32_t nrf_mesh_packet_send_cb(const nrf_mesh_tx_params_t * p_params, uint32_t * p_packet_ref, int count)
{
    TEST_ASSERT_TRUE(m_expected_packet_send > 0);
    TEST_ASSERT_EQUAL(m_expected_tx_params.dst.type, p_params->dst.type);
    TEST_ASSERT_EQUAL(m_expected_tx_params.dst.value, p_params->dst.value);
    TEST_ASSERT_EQUAL(m_expected_tx_params.dst.p_virtual_uuid, p_params->dst.p_virtual_uuid);
    TEST_ASSERT_EQUAL(m_expected_tx_params.ttl, p_params->ttl);
    TEST_ASSERT_EQUAL(m_expected_tx_params.reliable, p_params->reliable);
    TEST_ASSERT_EQUAL(m_expected_tx_params.src, p_params->src);
    TEST_ASSERT_EQUAL(m_expected_tx_params.security_material.p_net, p_params->security_material.p_net);
    TEST_ASSERT_EQUAL(m_expected_tx_params.security_material.p_app, p_params->security_material.p_app);
    TEST_ASSERT_EQUAL(m_expected_tx_params.data_len, p_params->data_len);
    TEST_ASSERT_EQUAL(m_expected_tx_params.p_data, p_params->p_data);
    m_expected_packet_send--;
    return m_packet_send_return;
}

static void nrf_mesh_event_handler_add_cb(nrf_mesh_evt_handler_t * p_handler, int count)
{
    mp_evt_handler = p_handler;
}

static void serial_tx_cb(const serial_packet_t * p_packet, int count)
{
    TEST_ASSERT_TRUE(m_expected_serial_tx > 0);
    m_expected_serial_tx--;
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t *) &m_expected_tx_packet, (uint8_t *) p_packet, m_expected_tx_packet.length + 1);
}

void setUp(void)
{
    device_state_manager_mock_Init();
    serial_mock_Init();
    nrf_mesh_mock_Init();
    nrf_mesh_events_mock_Init();
    net_state_mock_Init();
    access_mock_Init();
    m_expected_packet_send = 0;
    m_packet_send_return = 0;
    memset(&m_expected_tx_params, 0, sizeof(m_expected_tx_params));
    memset(&m_expected_tx_packet, 0, sizeof(m_expected_tx_packet));
}

void tearDown(void)
{
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    serial_mock_Verify();
    serial_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    access_mock_Verify();
    access_mock_Destroy();
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_invalid_cmd(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_RANGE_MESH_END;
    cmd.length = 1;
    serial_cmd_rsp_send_Expect(cmd.opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
    serial_handler_mesh_rx(&cmd);
}

void test_enable_disable(void)
{
    serial_packet_t cmd;

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ENABLE;
    cmd.length = 1;
    nrf_mesh_enable_ExpectAndReturn(0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DISABLE;
    cmd.length = 1;
    nrf_mesh_disable_ExpectAndReturn(0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
}

void test_stateclear()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_STATE_CLEAR;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    dsm_clear_Expect();
    access_clear_Expect();
    net_state_reset_Expect();
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MESH_STATE_CLEAR, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_mesh_rx(&cmd);
}

void test_subnet(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_ADD;
    cmd.length = 19;
    cmd.payload.cmd.mesh.subnet_add.net_key_index = 0x1234;
    dsm_handle_t handle = 0x5678;
    dsm_subnet_add_ExpectAndReturn(0x1234, cmd.payload.cmd.mesh.subnet_add.key, NULL, NRF_SUCCESS);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_subnet_add_ReturnThruPtr_p_subnet_handle(&handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_subnet_add_ExpectAndReturn(0x1234, cmd.payload.cmd.mesh.subnet_add.key, NULL, 0x12345678);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_subnet_add_ReturnThruPtr_p_subnet_handle(&handle);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_UPDATE;
    cmd.length = 19;
    cmd.payload.cmd.mesh.subnet_update.subnet_handle = 0x5678;
    dsm_subnet_update_ExpectAndReturn(0x5678, cmd.payload.cmd.mesh.subnet_update.key, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_subnet_update_ExpectAndReturn(0x5678, cmd.payload.cmd.mesh.subnet_update.key, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.subnet_delete.subnet_handle = 0x5678;
    dsm_subnet_delete_ExpectAndReturn(0x5678, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_subnet_delete_ExpectAndReturn(0x5678, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_GET_ALL;
    cmd.length = 1;
    dsm_handle_t handle_list[DSM_SUBNET_MAX];
    for (uint32_t i = 0; i < DSM_SUBNET_MAX; i++)
    {
        handle_list[i] = 0x12A0 + i;
    }
    uint32_t count = 5;
    TEST_ASSERT_TRUE_MESSAGE(count < DSM_SUBNET_MAX, "Count should be lower than SUBNET_MAX to avoid segfault");
    dsm_subnet_get_all_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
    dsm_subnet_get_all_IgnoreArg_p_key_list();
    dsm_subnet_get_all_IgnoreArg_p_count();
    dsm_subnet_get_all_ReturnMemThruPtr_p_key_list(handle_list, sizeof(handle_list[0]) * count);
    dsm_subnet_get_all_ReturnMemThruPtr_p_count(&count, sizeof(count));
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) handle_list, sizeof(handle_list[0]) * count, sizeof(handle_list[0]) * count);
    serial_handler_mesh_rx(&cmd);
    dsm_subnet_get_all_ExpectAndReturn(NULL, NULL, 0x12345678);
    dsm_subnet_get_all_IgnoreArg_p_key_list();
    dsm_subnet_get_all_IgnoreArg_p_count();
    dsm_subnet_get_all_ReturnArrayThruPtr_p_key_list(handle_list, count);
    dsm_subnet_get_all_ReturnMemThruPtr_p_count(&count, sizeof(count));
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    count = 0;
    dsm_subnet_get_all_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
    dsm_subnet_get_all_IgnoreArg_p_key_list();
    dsm_subnet_get_all_IgnoreArg_p_count();
    dsm_subnet_get_all_ReturnArrayThruPtr_p_key_list(handle_list, count);
    dsm_subnet_get_all_ReturnMemThruPtr_p_count(&count, sizeof(count));
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_COUNT_MAX_GET;
    cmd.length = 1;
    uint16_t list_size = DSM_SUBNET_MAX;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &list_size, sizeof(list_size), sizeof(list_size));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_appkey(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_ADD;
    cmd.length = 21;
    cmd.payload.cmd.mesh.appkey_add.app_key_index = 0x1234;
    cmd.payload.cmd.mesh.appkey_add.subnet_handle = 0xABCD;
    dsm_handle_t handle = 0x5678;
    dsm_appkey_add_ExpectAndReturn(0x1234, 0xABCD, cmd.payload.cmd.mesh.appkey_add.key, NULL, NRF_SUCCESS);
    dsm_appkey_add_IgnoreArg_p_app_handle();
    dsm_appkey_add_ReturnThruPtr_p_app_handle(&handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_appkey_add_ExpectAndReturn(0x1234, 0xABCD, cmd.payload.cmd.mesh.appkey_add.key, NULL, 0x12345678);
    dsm_appkey_add_IgnoreArg_p_app_handle();
    dsm_appkey_add_ReturnThruPtr_p_app_handle(&handle);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_UPDATE;
    cmd.length = 19;
    cmd.payload.cmd.mesh.appkey_update.appkey_handle = 0x5678;
    dsm_appkey_update_ExpectAndReturn(0x5678, cmd.payload.cmd.mesh.appkey_update.key, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_appkey_update_ExpectAndReturn(0x5678, cmd.payload.cmd.mesh.appkey_update.key, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.appkey_delete.appkey_handle = 0x5678;
    dsm_appkey_delete_ExpectAndReturn(0x5678, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_appkey_delete_ExpectAndReturn(0x5678, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_GET_ALL;
    cmd.length = 3;
    cmd.payload.cmd.mesh.appkey_get_all.subnet_handle = 0x5678;
    dsm_handle_t handle_list[DSM_SUBNET_MAX];
    handle_list[0] = 0x5678;
    for (uint32_t i = 1; i < DSM_SUBNET_MAX; i++)
    {
        handle_list[i] = 0x1200 + i;
    }

    uint32_t count = 5;
    dsm_appkey_get_all_ExpectAndReturn(0x5678, NULL, NULL, NRF_SUCCESS);
    dsm_appkey_get_all_IgnoreArg_p_key_list();
    dsm_appkey_get_all_IgnoreArg_p_count();
    dsm_appkey_get_all_ReturnArrayThruPtr_p_key_list(&handle_list[1], count);
    dsm_appkey_get_all_ReturnThruPtr_p_count(&count);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS,
            (uint8_t *) handle_list,
            sizeof(handle_list[0]) * (count + 1),
            sizeof(handle_list[0]) * (count + 1));
    serial_handler_mesh_rx(&cmd);
    dsm_appkey_get_all_ExpectAndReturn(0x5678, NULL, NULL, 0x12345678);
    dsm_appkey_get_all_IgnoreArg_p_key_list();
    dsm_appkey_get_all_IgnoreArg_p_count();
    dsm_appkey_get_all_ReturnArrayThruPtr_p_key_list(&handle_list[1], count);
    dsm_appkey_get_all_ReturnMemThruPtr_p_count(&count, sizeof(count));
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);

    count = 0;
    dsm_appkey_get_all_ExpectAndReturn(0x5678, NULL, NULL, NRF_SUCCESS);
    dsm_appkey_get_all_IgnoreArg_p_key_list();
    dsm_appkey_get_all_IgnoreArg_p_count();
    dsm_appkey_get_all_ReturnArrayThruPtr_p_key_list(handle_list, count);
    dsm_appkey_get_all_ReturnMemThruPtr_p_count(&count, sizeof(count));
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_COUNT_MAX_GET;
    cmd.length = 1;
    uint16_t list_size = DSM_APP_MAX;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &list_size, sizeof(list_size), sizeof(list_size));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_devkey(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DEVKEY_ADD;
    cmd.length = 21;
    cmd.payload.cmd.mesh.devkey_add.owner_addr = 0x1234;
    cmd.payload.cmd.mesh.devkey_add.subnet_handle = 0xABCD;
    dsm_handle_t handle = 0x5678;
    dsm_devkey_add_ExpectAndReturn(0x1234, 0xABCD, cmd.payload.cmd.mesh.devkey_add.key, NULL, NRF_SUCCESS);
    dsm_devkey_add_IgnoreArg_p_devkey_handle();
    dsm_devkey_add_ReturnThruPtr_p_devkey_handle(&handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_devkey_add_ExpectAndReturn(0x1234, 0xABCD, cmd.payload.cmd.mesh.devkey_add.key, NULL, 0x12345678);
    dsm_devkey_add_IgnoreArg_p_devkey_handle();
    dsm_devkey_add_ReturnThruPtr_p_devkey_handle(&handle);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DEVKEY_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.devkey_delete.devkey_handle = 0x5678;
    dsm_devkey_delete_ExpectAndReturn(0x5678, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &handle, sizeof(handle), sizeof(handle));
    serial_handler_mesh_rx(&cmd);
    dsm_devkey_delete_ExpectAndReturn(0x5678, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DEVKEY_COUNT_MAX_GET;
    cmd.length = 1;
    uint16_t list_size = DSM_DEVICE_MAX;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &list_size, sizeof(list_size), sizeof(list_size));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_local_unicast(void)
{
    serial_packet_t cmd;

    /* Get before this thing has been set, shouldn't be prohibited by the serial. */
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_GET;
    cmd.length = 1;
    dsm_local_unicast_address_t addr;
    addr.address_start = NRF_MESH_ADDR_UNASSIGNED;
    addr.count = 0;
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnMemThruPtr_p_address(&addr, sizeof(addr));
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr, sizeof(addr), sizeof(addr));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_SET;
    cmd.length = 5;
    cmd.payload.cmd.mesh.local_unicast_addr_set.start_address = 0x1234;
    cmd.payload.cmd.mesh.local_unicast_addr_set.count = 0x5678;
    addr.address_start = 0x1234;
    addr.count = 0x5678;
    dsm_local_unicast_addresses_set_ExpectWithArrayAndReturn(&addr, 1, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_addresses(void)
{
    serial_packet_t cmd;
    dsm_handle_t addr_handle;

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_GET;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_get.address_handle = 0x1234;
    addr_handle = 0x1234;
    nrf_mesh_address_t addr;
    addr.value = 0xABCD;
    addr.type = NRF_MESH_ADDRESS_TYPE_GROUP;
    addr.p_virtual_uuid = NULL;
    dsm_address_get_ExpectAndReturn(0x1234, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&addr);
    dsm_address_subscription_get_ExpectAndReturn(0x1234, true);
    serial_evt_cmd_rsp_data_raw_addr_t raw_addr_rsp;
    raw_addr_rsp.addr_type = addr.type;
    raw_addr_rsp.subscribed = true;
    raw_addr_rsp.raw_short_addr = addr.value;
    raw_addr_rsp.address_handle = addr_handle;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &raw_addr_rsp, sizeof(raw_addr_rsp) - NRF_MESH_UUID_SIZE, sizeof(raw_addr_rsp) - NRF_MESH_UUID_SIZE);
    serial_handler_mesh_rx(&cmd);
    /* fail getter */
    dsm_address_get_ExpectAndReturn(0x1234, NULL, 0x12345678);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&addr);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    /* get virtual */
    uint8_t virtual_uuid[NRF_MESH_UUID_SIZE];
    for (uint32_t i = 0; i < NRF_MESH_UUID_SIZE; i++)
    {
        virtual_uuid[i] = i;
    }
    addr.value = 0xFF01;
    addr.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    addr.p_virtual_uuid = virtual_uuid;
    dsm_address_get_ExpectAndReturn(0x1234, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&addr);
    dsm_address_subscription_get_ExpectAndReturn(0x1234, false);
    raw_addr_rsp.addr_type = addr.type;
    raw_addr_rsp.subscribed = false;
    raw_addr_rsp.raw_short_addr = addr.value;
    memcpy(raw_addr_rsp.virtual_uuid, virtual_uuid, NRF_MESH_UUID_SIZE);
    raw_addr_rsp.address_handle = addr_handle;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &raw_addr_rsp, sizeof(raw_addr_rsp), sizeof(raw_addr_rsp));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_GET_ALL;
    cmd.length = 1;
    dsm_handle_t addr_list[] =
    {
        0x0001,
        0x1234,
        0xABCD,
        0x5435,
        0x0001 /* shouldn't care about duplicates. */
    };
    uint32_t count = sizeof(addr_list) / sizeof(addr_list[0]);
    dsm_address_get_all_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
    dsm_address_get_all_IgnoreArg_p_count();
    dsm_address_get_all_IgnoreArg_p_address_handle_list();
    dsm_address_get_all_ReturnArrayThruPtr_p_address_handle_list(addr_list, count);
    dsm_address_get_all_ReturnThruPtr_p_count(&count);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) addr_list, sizeof(addr_list[0]) * count, sizeof(addr_list[0]) * count);
    serial_handler_mesh_rx(&cmd);
    /* fail getter */
    dsm_address_get_all_ExpectAndReturn(NULL, NULL, 0x12345678);
    dsm_address_get_all_IgnoreArg_p_count();
    dsm_address_get_all_IgnoreArg_p_address_handle_list();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, 0xAA, NULL, 0, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_NONVIRTUAL_COUNT_MAX_GET;
    cmd.length = 1;
    uint16_t max_count = DSM_NONVIRTUAL_ADDR_MAX;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &max_count, sizeof(max_count), sizeof(max_count));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_VIRTUAL_COUNT_MAX_GET;
    cmd.length = 1;
    max_count = DSM_VIRTUAL_ADDR_MAX;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &max_count, sizeof(max_count), sizeof(max_count));
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

}

void test_addr_subscription(void)
{
    serial_packet_t cmd;
    dsm_handle_t addr_handle;

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD;
    cmd.length = 3;
    addr_handle = 0x1234;
    cmd.payload.cmd.mesh.addr_subscription_add.address = 0x1234;
    dsm_address_subscription_add_ExpectAndReturn(0x1234, NULL, NRF_SUCCESS);
    dsm_address_subscription_add_IgnoreArg_p_address_handle();
    dsm_address_subscription_add_ReturnThruPtr_p_address_handle(&addr_handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_subscription_add_ExpectAndReturn(0x1234, NULL, 0x12345678);
    dsm_address_subscription_add_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    const uint8_t test_uuid[NRF_MESH_UUID_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD_VIRTUAL;
    cmd.length = 17;
    memcpy(cmd.payload.cmd.mesh.addr_subscription_add_virtual.uuid, test_uuid, sizeof(test_uuid));
    dsm_address_subscription_virtual_add_ExpectWithArrayAndReturn(test_uuid, sizeof(test_uuid), NULL, 0, NRF_SUCCESS);
    dsm_address_subscription_virtual_add_IgnoreArg_p_address_handle();
    dsm_address_subscription_virtual_add_ReturnThruPtr_p_address_handle(&addr_handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_subscription_virtual_add_ExpectWithArrayAndReturn(test_uuid, sizeof(test_uuid), NULL, 0, 0x12345678);
    dsm_address_subscription_virtual_add_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_REMOVE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_subscription_remove.address_handle = addr_handle;
    dsm_address_subscription_remove_ExpectAndReturn(addr_handle, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_subscription_remove_ExpectAndReturn(addr_handle, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_addr_publication(void)
{
    serial_packet_t cmd;
    dsm_handle_t addr_handle;

    addr_handle = 0x1234;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_publication_add.address = 0x1234;
    dsm_address_publish_add_ExpectAndReturn(0x1234, NULL, NRF_SUCCESS);
    dsm_address_publish_add_IgnoreArg_p_address_handle();
    dsm_address_publish_add_ReturnThruPtr_p_address_handle(&addr_handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_publish_add_ExpectAndReturn(0x1234, NULL, 0x12345678);
    dsm_address_publish_add_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    const uint8_t test_uuid[NRF_MESH_UUID_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD_VIRTUAL;
    cmd.length = 17;
    memcpy(cmd.payload.cmd.mesh.addr_publication_add_virtual.uuid, test_uuid, sizeof(test_uuid));
    dsm_address_publish_virtual_add_ExpectWithArrayAndReturn(test_uuid, sizeof(test_uuid), NULL, 0, NRF_SUCCESS);
    dsm_address_publish_virtual_add_IgnoreArg_p_address_handle();
    dsm_address_publish_virtual_add_ReturnThruPtr_p_address_handle(&addr_handle);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_publish_virtual_add_ExpectWithArrayAndReturn(test_uuid, sizeof(test_uuid), NULL, 0, 0x12345678);
    dsm_address_publish_virtual_add_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_REMOVE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_publication_remove.address_handle = addr_handle;
    dsm_address_publish_remove_ExpectAndReturn(addr_handle, NRF_SUCCESS);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(cmd.opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &addr_handle, sizeof(addr_handle), sizeof(addr_handle));
    serial_handler_mesh_rx(&cmd);

    dsm_address_publish_remove_ExpectAndReturn(addr_handle, 0x12345678);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}

void test_packet_send(void)
{
    serial_packet_t cmd;

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_PACKET_SEND;
    cmd.length = 9; // no payload
    cmd.payload.cmd.mesh.packet_send.appkey_handle = 0x5678;
    cmd.payload.cmd.mesh.packet_send.reliable = true;
    cmd.payload.cmd.mesh.packet_send.ttl = 0x32;
    cmd.payload.cmd.mesh.packet_send.src_addr = 0xABCD;
    cmd.payload.cmd.mesh.packet_send.dst_addr_handle = 0x0102;
    for (uint32_t i = 0; i < sizeof(cmd.payload.cmd.mesh.packet_send.data); i++)
    {
        cmd.payload.cmd.mesh.packet_send.data[i] = i;
    }
    nrf_mesh_network_secmat_t net;
    nrf_mesh_application_secmat_t app;
    nrf_mesh_secmat_t secmat;
    secmat.p_net = &net; secmat.p_app = &app;
    m_expected_packet_send = 1;
    m_packet_send_return = 0x12345678;
    m_expected_tx_params.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    m_expected_tx_params.dst.value = 0x1234;
    m_expected_tx_params.dst.p_virtual_uuid = NULL;
    m_expected_tx_params.ttl = 0x32;
    m_expected_tx_params.reliable = true;
    m_expected_tx_params.src = 0xABCD;
    m_expected_tx_params.security_material.p_net = &net;
    m_expected_tx_params.security_material.p_app = &app;
    m_expected_tx_params.data_len = 0;
    m_expected_tx_params.p_data = NULL;
    dsm_local_unicast_address_t local_addr;
    local_addr.address_start = 0xABC0;
    local_addr.count = 0x10;
    dsm_address_get_ExpectAndReturn(0x0102, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&m_expected_tx_params.dst);
    dsm_tx_secmat_get_ExpectAndReturn(0x5678, NULL, NRF_SUCCESS);
    dsm_tx_secmat_get_IgnoreArg_p_secmat();
    dsm_tx_secmat_get_ReturnMemThruPtr_p_secmat(&secmat, sizeof(secmat));
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addr);
    nrf_mesh_packet_send_StubWithCallback(nrf_mesh_packet_send_cb);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    TEST_ASSERT_EQUAL(0, m_expected_packet_send);

    cmd.length = 8; // one too short
    serial_cmd_rsp_send_Expect(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
    serial_handler_mesh_rx(&cmd);

    cmd.length = 98; //maxlen
    m_expected_packet_send = 1;
    m_expected_tx_params.data_len = 89;
    m_expected_tx_params.p_data = cmd.payload.cmd.mesh.packet_send.data;
    dsm_address_get_ExpectAndReturn(0x0102, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&m_expected_tx_params.dst);
    dsm_tx_secmat_get_ExpectAndReturn(0x5678, NULL, NRF_SUCCESS);
    dsm_tx_secmat_get_IgnoreArg_p_secmat();
    dsm_tx_secmat_get_ReturnMemThruPtr_p_secmat(&secmat, sizeof(secmat));
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addr);
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    TEST_ASSERT_EQUAL(0, m_expected_packet_send);

    cmd.length = 99; // one too long
    serial_cmd_rsp_send_Expect(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
    serial_handler_mesh_rx(&cmd);

    /* give a source address that isn't a local unicast address */
    cmd.length = 98; //maxlen
    local_addr.address_start = 0x0103;
    dsm_address_get_ExpectAndReturn(0x0102, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&m_expected_tx_params.dst);
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addr);
    serial_translate_error_ExpectAndReturn(NRF_ERROR_INVALID_ADDR, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    local_addr.address_start = 0x0100;
    local_addr.count = 2; // only includes 0x0100 and 0x0101, not 0x0102.
    dsm_address_get_ExpectAndReturn(0x0102, NULL, NRF_SUCCESS);
    dsm_address_get_IgnoreArg_p_address();
    dsm_address_get_ReturnThruPtr_p_address(&m_expected_tx_params.dst);
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addr);
    serial_translate_error_ExpectAndReturn(NRF_ERROR_INVALID_ADDR, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    /* Unknown target address */
    dsm_address_get_ExpectAndReturn(0x0102, NULL, 0x12345678);
    dsm_address_get_IgnoreArg_p_address();
    serial_translate_error_ExpectAndReturn(0x12345678, 0xAA);
    serial_cmd_rsp_send_Expect(cmd.opcode, 0xAA, NULL, 0);
    serial_handler_mesh_rx(&cmd);
    TEST_ASSERT_EQUAL(0, m_expected_packet_send);
}

void test_events(void)
{
    const uint32_t SEAL = 0x5EA15EA1;
    uint8_t packet[1 + NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_PAYLOAD_MAXLEN + sizeof(SEAL)];
    memcpy(&packet[sizeof(packet) - sizeof(SEAL)], &SEAL, sizeof(SEAL));
    serial_packet_t * p_packet = (serial_packet_t *) packet;

    serial_tx_StubWithCallback(serial_tx_cb);
    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_event_handler_add_cb);
    serial_handler_mesh_init();
    TEST_ASSERT_NOT_NULL(mp_evt_handler);
    dsm_handle_t app_handle = 0xABCD;
    dsm_handle_t dst_handle = 0xA0B0;
    dsm_handle_t subnet_handle = 0xF1E1;
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_MESSAGE_RECEIVED;
    evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_GROUP;
    evt.params.message.dst.value = 0x8ABC;
    evt.params.message.dst.p_virtual_uuid = NULL;
    evt.params.message.src.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    evt.params.message.src.value = 0x1234;
    evt.params.message.src.p_virtual_uuid = NULL;
    for (uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        evt.params.message.adv_addr.addr[i] = i;
    }
    uint8_t data[9];
    for (uint32_t i = 0; i < 9; i++)
    {
        data[i] = i;
    }
    nrf_mesh_network_secmat_t net;
    nrf_mesh_application_secmat_t app;
    for (uint32_t i = 0; i < NRF_MESH_KEY_SIZE; i++)
    {
        net.encryption_key[i] = i + 0x10;
        net.privacy_key[i] = i + 0x20;
        app.key[i] = i + 0x30;
    }
    net.nid = 0x43;
    app.is_device_key = false;
    app.aid = 0x12;
    evt.params.message.adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;
    evt.params.message.secmat.p_net = &net;
    evt.params.message.secmat.p_app = &app;
    evt.params.message.ttl = 0x84;
    evt.params.message.rssi = -5;
    evt.params.message.length = sizeof(data);
    evt.params.message.p_buffer = data;
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 20 + sizeof(data);
    m_expected_tx_packet.opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_SUBSCRIPTION;
    m_expected_tx_packet.payload.evt.mesh.message_received.src = 0x1234;
    m_expected_tx_packet.payload.evt.mesh.message_received.adv_addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;
    m_expected_tx_packet.payload.evt.mesh.message_received.appkey_handle = app_handle;
    m_expected_tx_packet.payload.evt.mesh.message_received.dst = dst_handle;
    m_expected_tx_packet.payload.evt.mesh.message_received.rssi = -5;
    m_expected_tx_packet.payload.evt.mesh.message_received.ttl = 0x84;
    m_expected_tx_packet.payload.evt.mesh.message_received.subnet_handle = subnet_handle;
    m_expected_tx_packet.payload.evt.mesh.message_received.actual_length = sizeof(data);
    memcpy(m_expected_tx_packet.payload.evt.mesh.message_received.data, data, sizeof(data));
    memcpy(m_expected_tx_packet.payload.evt.mesh.message_received.adv_addr, evt.params.message.adv_addr.addr, BLE_GAP_ADDR_LEN);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    dsm_address_handle_get_ExpectAndReturn(&evt.params.message.dst, NULL, NRF_SUCCESS);
    dsm_address_handle_get_IgnoreArg_p_address_handle();
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst_handle);
    dsm_subnet_handle_get_ExpectAndReturn(evt.params.message.secmat.p_net, subnet_handle);
    dsm_appkey_handle_get_ExpectAndReturn(evt.params.message.secmat.p_app, app_handle);
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));
    /* virtual */
    m_expected_serial_tx = 1;
    uint8_t virtual_uuid[NRF_MESH_UUID_SIZE];
    evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    evt.params.message.dst.p_virtual_uuid = virtual_uuid;
    dsm_address_handle_get_ExpectAndReturn(&evt.params.message.dst, NULL, NRF_SUCCESS);
    dsm_address_handle_get_IgnoreArg_p_address_handle();
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst_handle);
    dsm_subnet_handle_get_ExpectAndReturn(evt.params.message.secmat.p_net, subnet_handle);
    dsm_appkey_handle_get_ExpectAndReturn(evt.params.message.secmat.p_app, app_handle);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));
    /* Assert if the address suddenly can't be found. This shouldn't happen, as
     * the packet wouldn't have made it to the event handler if it didn't exist
     * in the dsm. */
    dsm_address_handle_get_ExpectAndReturn(&evt.params.message.dst, NULL, NRF_ERROR_NOT_FOUND);
    dsm_address_handle_get_IgnoreArg_p_address_handle();
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst_handle);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    TEST_NRF_MESH_ASSERT_EXPECT(mp_evt_handler->evt_cb(&evt));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));
    /* No payload */
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 20;
    evt.params.message.length = 0;
    evt.params.message.p_buffer = NULL;
    m_expected_tx_packet.payload.evt.mesh.message_received.actual_length = 0;
    dsm_address_handle_get_ExpectAndReturn(&evt.params.message.dst, NULL, NRF_SUCCESS);
    dsm_address_handle_get_IgnoreArg_p_address_handle();
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst_handle);
    dsm_subnet_handle_get_ExpectAndReturn(evt.params.message.secmat.p_net, subnet_handle);
    dsm_appkey_handle_get_ExpectAndReturn(evt.params.message.secmat.p_app, app_handle);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));
    /* Max payload */
    uint8_t long_data[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    for (uint32_t i = 0; i < NRF_MESH_SEG_PAYLOAD_SIZE_MAX; i++)
    {
        long_data[i] = i;
    }
    memcpy(m_expected_tx_packet.payload.evt.mesh.message_received.data,
            long_data,
            SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN);
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 98; //TODO: MBTLE-1525: Don't cut this short here...
    m_expected_tx_packet.payload.evt.mesh.message_received.actual_length = NRF_MESH_SEG_PAYLOAD_SIZE_MAX;
    evt.params.message.length = NRF_MESH_SEG_PAYLOAD_SIZE_MAX;
    evt.params.message.p_buffer = long_data;
    dsm_address_handle_get_ExpectAndReturn(&evt.params.message.dst, NULL, NRF_SUCCESS);
    dsm_address_handle_get_IgnoreArg_p_address_handle();
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst_handle);
    dsm_subnet_handle_get_ExpectAndReturn(evt.params.message.secmat.p_net, subnet_handle);
    dsm_appkey_handle_get_ExpectAndReturn(evt.params.message.secmat.p_app, app_handle);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));
    /* unicast */
    m_expected_serial_tx = 1;
    m_expected_tx_packet.opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_UNICAST;
    m_expected_tx_packet.payload.evt.mesh.message_received.dst = evt.params.message.dst.value;
    evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    dsm_subnet_handle_get_ExpectAndReturn(evt.params.message.secmat.p_net, subnet_handle);
    dsm_appkey_handle_get_ExpectAndReturn(evt.params.message.secmat.p_app, app_handle);
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
    memset(&evt, 0, sizeof(evt));
    memset(&m_expected_tx_packet, 0, sizeof(m_expected_tx_packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&SEAL, &packet[sizeof(packet) - sizeof(SEAL)], sizeof(SEAL));


    evt.type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION;
    evt.params.iv_update.state = (net_state_iv_update_t) 0x95;
    evt.params.iv_update.iv_index = 0x12345678;
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 5;
    m_expected_tx_packet.opcode = SERIAL_OPCODE_EVT_MESH_IV_UPDATE_NOTIFICATION;
    m_expected_tx_packet.payload.evt.mesh.iv_update.iv_index = 0x12345678;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);

    evt.type = NRF_MESH_EVT_KEY_REFRESH_START;
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 1;
    m_expected_tx_packet.opcode = SERIAL_OPCODE_EVT_MESH_KEY_REFRESH_START;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);

    evt.type = NRF_MESH_EVT_KEY_REFRESH_END;
    m_expected_serial_tx = 1;
    m_expected_tx_packet.length = 1;
    m_expected_tx_packet.opcode = SERIAL_OPCODE_EVT_MESH_KEY_REFRESH_END;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_tx_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_tx_packet.length;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);

    /* Give it some random event it doesn't care about, should exit silently,
     * without side-effects. */
    evt.type = NRF_MESH_EVT_DFU_BANK_AVAILABLE;
    mp_evt_handler->evt_cb(&evt);
    TEST_ASSERT_EQUAL(0, m_expected_serial_tx);
}
