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

#include "serial_handler_device.h"
#include "nrf_mesh_config_serial.h"
#include "serial_mock.h"
#include "serial_status.h"
#include "radio_mock.h"
#include "bearer_adv_mock.h"
#include "packet_mgr_mock.h"
#include "hal_mock.h"

#define CMD_LENGTH_CHECK(_opcode, _intended_length)                                                   \
    do {                                                                                              \
        serial_packet_t _cmd;                                                                         \
        _cmd.opcode = _opcode;                                                                        \
        _cmd.length = _intended_length + 1;                                                           \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0, 0); \
        serial_handler_device_rx(&_cmd);                                                              \
        _cmd.length = _intended_length + 1;                                                           \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0, 0); \
        serial_handler_device_rx(&_cmd);                                                              \
    } while (0)

#define EXPECT_PACKET()                                 \
    do {                                                \
        m_tx_cb_count++;                                \
        serial_tx_StubWithCallback(serial_tx_cb);       \
    } while (0)

#define EXPECT_ACK(_opcode, _error_code)                                \
    do {                                                                \
        serial_translate_error_ExpectAndReturn(_error_code, 0xAA);      \
        serial_cmd_rsp_send_Expect(_opcode, 0xAA, NULL, 0); \
    } while (0)

#define EXPECT_ACK_NO_TRANSLATE(_opcode, _serial_status)                            \
    do {                                                                \
        serial_cmd_rsp_send_Expect(_opcode, _serial_status, NULL, 0); \
    } while (0)

#define EXPECT_ACK_WITH_PAYLOAD(_opcode, _data, _len)                   \
    do { \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, 0, (uint8_t *) (_data), _len, _len); \
    } while (0)


nrf_mesh_assertion_handler_t                  m_assertion_handler;
static serial_packet_t                        m_expected_packet;
static uint32_t                               m_tx_cb_count;
static uint32_t                               m_tx_cb_count_actual;

static void callbacks_verify(void)
{
    TEST_ASSERT_EQUAL_MESSAGE(m_tx_cb_count, m_tx_cb_count_actual, "Serial TX called too few times.");
}


static void assertion_handler(uint32_t pc)
{
    TEST_FAIL_MESSAGE("ASSERT");
}

static void serial_tx_cb(const serial_packet_t* p_packet, int cmock_num_calls)
{
    m_tx_cb_count_actual = cmock_num_calls + 1;
    if (m_tx_cb_count != (uint32_t) cmock_num_calls + 1)
    {
        TEST_ASSERT_EQUAL(m_tx_cb_count, cmock_num_calls + 1);
    }
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, p_packet->length, "Sent packet has length 0");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, m_expected_packet.length, "Reference packet has length 0");
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t *) &m_expected_packet,
            (uint8_t *) p_packet,
            p_packet->length + SERIAL_PACKET_LENGTH_OVERHEAD);
}


void setUp(void)
{
    serial_mock_Init();
    hal_mock_Init();
    radio_mock_Init();
    bearer_adv_mock_Init();
    packet_mgr_mock_Init();

    m_assertion_handler = assertion_handler;
    m_tx_cb_count = 0;
    m_tx_cb_count_actual = 0;
}

void tearDown(void)
{
    serial_mock_Verify();
    serial_mock_Destroy();
    hal_mock_Verify();
    hal_mock_Destroy();
    radio_mock_Verify();
    radio_mock_Destroy();
    bearer_adv_mock_Verify();
    bearer_adv_mock_Destroy();
    packet_mgr_mock_Verify();
    packet_mgr_mock_Destroy();
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_device_rx(void)
{
    uint8_t packet[NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_PAYLOAD_MAXLEN];
    serial_packet_t * p_packet = (serial_packet_t *) packet;

    serial_packet_t cmd;
    char echo_string[] = "Test echo string";
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + strlen(echo_string);
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_ECHO;
    strcpy((char*) cmd.payload.cmd.device.echo.data, echo_string);
    memcpy(&m_expected_packet, &cmd, cmd.length + SERIAL_PACKET_LENGTH_OVERHEAD);
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    serial_handler_device_rx(&cmd);
    serial_mock_Verify();
    /* 0-length */
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memcpy(&m_expected_packet, &cmd, cmd.length + SERIAL_PACKET_LENGTH_OVERHEAD);
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    serial_handler_device_rx(&cmd);
    serial_mock_Verify();

    /* get serial version */
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_SERIAL_VERSION_GET;
    uint16_t api_version = SERIAL_API_VERSION;
    EXPECT_ACK_WITH_PAYLOAD(SERIAL_OPCODE_CMD_DEVICE_SERIAL_VERSION_GET, &api_version, 2);
    serial_handler_device_rx(&cmd);
    serial_mock_Verify();

    /* get serial version invalid length */
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + 1;
    EXPECT_ACK_NO_TRANSLATE(SERIAL_OPCODE_CMD_DEVICE_SERIAL_VERSION_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_device_rx(&cmd);
    serial_mock_Verify();
    callbacks_verify();

    /* reset device */
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_RADIO_RESET;
    hal_device_reset_Expect(1);
    EXPECT_ACK_NO_TRANSLATE(SERIAL_OPCODE_CMD_DEVICE_RADIO_RESET, SERIAL_STATUS_ERROR_REJECTED);
    serial_handler_device_rx(&cmd);
    serial_mock_Verify();
}

void test_beacon(void)
{
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        bearer_adv_advertiser_init_Expect(NULL);
        bearer_adv_advertiser_init_IgnoreArg_p_adv();
    }
    serial_handler_device_init();

    serial_packet_t cmd;

    /* Get default parameters of all advertisers */
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_GET;
    cmd.length = 2;
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_params_get.beacon_slot = i;

        radio_tx_power_get_ExpectAndReturn((radio_tx_power_t) 0x43);

        serial_evt_cmd_rsp_data_beacon_params_t rsp;
        rsp.beacon_slot = i;
        rsp.tx_power = 0x43;
        rsp.interval_ms = BEARER_ADV_INT_MIN_MS_DEFAULT;
        rsp.channel_map = NRF_MESH_ADV_CHAN_DEFAULT;
        EXPECT_ACK_WITH_PAYLOAD(cmd.opcode, &rsp, sizeof(rsp));
        serial_handler_device_rx(&cmd);
    }
    /* out-of-bounds beacon slot */
    cmd.payload.cmd.device.beacon_params_get.beacon_slot = NRF_MESH_SERIAL_BEACON_SLOTS;
    serial_cmd_rsp_send_Expect(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_PARAMETER, NULL, 0);
    serial_handler_device_rx(&cmd);

    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    /* Set parameters of all advertisers */
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_SET;
    cmd.length = 8;
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_params_set.beacon_slot = i;
        cmd.payload.cmd.device.beacon_params_set.tx_power = 0x12;
        cmd.payload.cmd.device.beacon_params_set.interval_ms = 100 * (i + 1);
        cmd.payload.cmd.device.beacon_params_set.channel_map = 0x05;
        bearer_adv_interval_reset_ExpectAndReturn(NULL, NRF_SUCCESS);
        bearer_adv_interval_reset_IgnoreArg_p_adv();
        EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
        serial_handler_device_rx(&cmd);
    }
    /* out-of-bounds beacon slot */
    cmd.payload.cmd.device.beacon_params_set.beacon_slot = NRF_MESH_SERIAL_BEACON_SLOTS;
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_PARAM);
    serial_handler_device_rx(&cmd);

    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);

    /* Get changed parameters of all advertisers */
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_GET;
    cmd.length = 2;
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_params_get.beacon_slot = i;

        radio_tx_power_get_ExpectAndReturn((radio_tx_power_t) 0x43);

        serial_evt_cmd_rsp_data_beacon_params_t rsp;
        rsp.beacon_slot = i;
        rsp.tx_power = 0x43;
        rsp.interval_ms = 100 * (i + 1);
        rsp.channel_map = 0x05;
        EXPECT_ACK_WITH_PAYLOAD(cmd.opcode, &rsp, sizeof(rsp));
        serial_handler_device_rx(&cmd);
    }

    /* Start beacons */
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_BEACON_START;
    /* Repeat for several different payloads */
    uint8_t payloads[] = {0, 2, 31, 0, 6};
    for (uint32_t j = 0; j < sizeof(payloads); j++)
    {
        uint32_t payload = payloads[j];
        cmd.length = 2 + payload;
        for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
        {
            char err_msg[128];
            sprintf(err_msg, "Payload size: %u, Beacon slot: %u", payload, i);
            cmd.payload.cmd.device.beacon_start.beacon_slot = i;
            memset(cmd.payload.cmd.device.beacon_start.data, i, payload);
            packet_t packet;
            memset(&packet, 0, sizeof(packet));
            packet_generic_t * p_packet = (packet_generic_t *) &packet;
            packet_mgr_alloc_ExpectAndReturn(NULL, 3 + 6 + payload, NRF_SUCCESS); /* size = header + addr + payload */
            packet_mgr_alloc_IgnoreArg_pp_buffer();
            packet_mgr_alloc_ReturnThruPtr_pp_buffer(&p_packet);
            bearer_adv_tx_ExpectAndReturn(NULL, &packet, BEARER_ADV_REPEAT_INFINITE, NRF_SUCCESS);
            bearer_adv_tx_IgnoreArg_p_adv();
            EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
            serial_handler_device_rx(&cmd);
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(6 + payload, packet.header.length, err_msg); /* length=addr + payload */
            if (payload != 0)
            {
                TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(cmd.payload.cmd.device.beacon_start.data,
                        packet.payload,
                        payload,
                        err_msg);
            }
        }
    }
    cmd.payload.cmd.device.beacon_start.beacon_slot = NRF_MESH_SERIAL_BEACON_SLOTS;
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_PARAM);
    serial_handler_device_rx(&cmd);
    cmd.payload.cmd.device.beacon_start.beacon_slot = 0;
    cmd.length = 34;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_device_rx(&cmd);
    cmd.length = 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_device_rx(&cmd);
    /* fail packet allocation */
    packet_t packet;
    cmd.length = 33;
    packet_generic_t * p_packet = NULL;
    packet_mgr_alloc_ExpectAndReturn(NULL, 3 + 6 + 31, 0x12345678); /* size = header + addr + payload */
    packet_mgr_alloc_IgnoreArg_pp_buffer();
    packet_mgr_alloc_ReturnThruPtr_pp_buffer(&p_packet);
    EXPECT_ACK(cmd.opcode, 0x12345678);
    serial_handler_device_rx(&cmd);
    /* fail packet tx */
    memset(&packet, 0, sizeof(packet));
    p_packet = &packet;
    packet_mgr_alloc_ExpectAndReturn(NULL, 3 + 6 + 31, NRF_SUCCESS); /* size = header + addr + payload */
    packet_mgr_alloc_IgnoreArg_pp_buffer();
    packet_mgr_alloc_ReturnThruPtr_pp_buffer(&p_packet);
    bearer_adv_tx_ExpectAndReturn(NULL, &packet, BEARER_ADV_REPEAT_INFINITE, 0x12345678);
    bearer_adv_tx_IgnoreArg_p_adv();
    packet_mgr_free_Expect(p_packet);
    EXPECT_ACK(cmd.opcode, 0x12345678);
    serial_handler_device_rx(&cmd);

    /* Stop the beacons, but fail. */
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_BEACON_STOP;
    cmd.length = 2;
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_stop.beacon_slot = i;
        bearer_adv_adv_stop_ExpectAndReturn(NULL, 0x12345678);
        bearer_adv_adv_stop_IgnoreArg_p_adv();
        EXPECT_ACK(cmd.opcode, 0x12345678);
        serial_handler_device_rx(&cmd);
    }
    /* Successfully stop the beacons */
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_stop.beacon_slot = i;
        bearer_adv_adv_stop_ExpectAndReturn(NULL, NRF_SUCCESS);
        bearer_adv_adv_stop_IgnoreArg_p_adv();
        EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
        serial_handler_device_rx(&cmd);
    }
    /* Second time we stop them, it shouldn't allow us. */
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        cmd.payload.cmd.device.beacon_stop.beacon_slot = i;
        EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
        serial_handler_device_rx(&cmd);
    }
    cmd.payload.cmd.device.beacon_stop.beacon_slot = NRF_MESH_SERIAL_BEACON_SLOTS;
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_PARAM);
    serial_handler_device_rx(&cmd);

    CMD_LENGTH_CHECK(cmd.opcode, cmd.length);
}
