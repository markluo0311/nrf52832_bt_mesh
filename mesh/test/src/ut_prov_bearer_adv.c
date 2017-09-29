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

#include "prov_bearer_adv.h"
#include "nrf_mesh_prov.h"
#include "log.h"

#include <cmock.h>
#include <unity.h>

#include "bearer_adv_mock.h"
#include "provisioning_mock.h"
#include "timer_scheduler_mock.h"
#include "packet_mgr_mock.h"
#include "rand_mock.h"
#include "prov_beacon_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_configure_mock.h"
#include "timer_mock.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* Transaction start values for each role (from mesh core spec d09r19, section 5.2.1) */
#define PROVISIONER_TRANSACTION_START_VALUE 0
#define PROVISIONEE_TRANSACTION_START_VALUE 0x80

#define BLE_ADV_OVERHEAD (BLE_GAP_ADDR_LEN + sizeof(ble_ad_data_t) /* length field and AD Type*/)
#define PROV_ADV_OVERHEAD (4 /*link ID*/ + 1 /* transaction no*/)
/** The largest provisioning PDU length (copied from mesh core spec d09r19, table 5.3: 64 byte payload + 1 for pdu type). */
#define PROV_PAYLOAD_MAX_LENGTH  65
#define PROV_LINK_OPEN_DATA_SIZE 17
#define PROV_LINK_ACK_DATA_SIZE 1
#define PROV_LINK_CLOSE_DATA_SIZE 2
#define PROV_TRANS_ACK_DATA_SIZE 1
/** The largest payload length in a single provisioning data packet see Table 5.2 in mesh core spec (d09r19) */
#define GENERIC_PROV_PDU_MAX_LEN 24
/** Max see Figure 5.3 in mesh core spec (d09r19) */
#define PROV_START_PDU_HEADER_SIZE (1 /*SegN | GPCF */ + 2 /*Total length */ + 1 /* FCS */)
#define PROV_START_PDU_PAYLOAD_MAX_LEN (GENERIC_PROV_PDU_MAX_LEN - PROV_START_PDU_HEADER_SIZE)
/** Max see Figure 5.5 in mesh core spec (d09r19) */
#define PROV_CONTINUE_PDU_HEADER_SIZE (1 /*SegN | GPCF */)
#define PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN (GENERIC_PROV_PDU_MAX_LEN - PROV_CONTINUE_PDU_HEADER_SIZE)

#define PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD   (3)
#define PROV_LINK_ACK_OPCODE_WITH_GPCF_FIELD    ((1<<2) | 3)
#define PROV_LINK_CLOSE_OPCODE_WITH_GPCF_FIELD  ((2<<2) | 3)
#define PROV_TRANS_ACK_OPCODE_WITH_GPCF_FIELD   (1)
#define PROV_TRANS_START_OPCODE_WITH_GPCF_FIELD (0)
#define PROV_TRANS_CONTINUE_OPCODE_WITH_GPCF_FIELD  (2)

#define PROV_BEARER_ADV_UNACKED_REPEAT_COUNT    (4)

static packet_t m_packet;
static packet_generic_t * mp_packet;

#define ALLOC_AND_TX(P_ADV, ALLOC_SIZE, ALLOC_RETURN, TX_REPEAT, TX_RETURN)               \
do                                                                                        \
{                                                                                         \
    mp_packet = &m_packet;                                                                \
    packet_mgr_alloc_ExpectAndReturn(NULL, ALLOC_SIZE, ALLOC_RETURN);                     \
    packet_mgr_alloc_IgnoreArg_pp_buffer();                                               \
    packet_mgr_alloc_ReturnMemThruPtr_pp_buffer(&mp_packet, sizeof(packet_generic_t **)); \
    if (ALLOC_RETURN == NRF_SUCCESS)                                                      \
    {                                                                                     \
        bearer_adv_tx_ExpectAndReturn(P_ADV, mp_packet, TX_REPEAT, TX_RETURN);            \
    }                                                                                     \
}                                                                                         \
while (0)

static uint8_t data[PROV_PAYLOAD_MAX_LENGTH+1];
static uint8_t uuid1[NRF_MESH_UUID_SIZE] = {0,1,2,3};
static uint8_t uuid2[NRF_MESH_UUID_SIZE] = {9,10,11,12};
static uint8_t link_open_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE];
static uint8_t link_ack_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE];
static uint8_t link_close_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE];
static uint8_t trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + GENERIC_PROV_PDU_MAX_LEN];
static uint8_t trans_ack_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE];
static uint8_t minimal_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + 1];

static prov_bearer_if_tx_t           prov_bearer_adv_tx;
static prov_bearer_if_listen_start_t prov_bearer_adv_listen;
static prov_bearer_if_listen_stop_t  prov_bearer_adv_listen_stop;
static prov_bearer_if_link_open_t    prov_bearer_adv_link_open;
static prov_bearer_if_link_close_t   prov_bearer_adv_link_close;

nrf_mesh_assertion_handler_t m_assertion_handler;

void nrf_mesh_assertion_handler(uint32_t pc)
{
    TEST_FAIL_MESSAGE("Mesh assertion triggered");
}

/********** Local Mock Functions **********/

/********** Test Setup **********/

void setUp(void)
{
    __LOG_INIT((LOG_SRC_PROV | LOG_SRC_TEST), LOG_LEVEL_ERROR, LOG_CALLBACK_DEFAULT);

    bearer_adv_mock_Init();
    provisioning_mock_Init();
    timer_scheduler_mock_Init();
    packet_mgr_mock_Init();
    rand_mock_Init();
    prov_beacon_mock_Init();
    nrf_mesh_mock_Init();
    timer_mock_Init();

    const prov_bearer_interface_t * p_bearer = prov_bearer_adv_interface_get();
    prov_bearer_adv_tx = p_bearer->tx;
    prov_bearer_adv_listen = p_bearer->listen_start;
    prov_bearer_adv_listen_stop = p_bearer->listen_stop;
    prov_bearer_adv_link_open = p_bearer->link_open;
    prov_bearer_adv_link_close = p_bearer->link_close;

    for (int i=0;i<PROV_PAYLOAD_MAX_LENGTH+1;i++)
    {
        data[i] = i;
    }
    m_assertion_handler = nrf_mesh_assertion_handler;
}

void tearDown(void)
{
    bearer_adv_mock_Verify();
    provisioning_mock_Verify();
    timer_scheduler_mock_Verify();
    packet_mgr_mock_Verify();
    rand_mock_Verify();
    prov_beacon_mock_Verify();
    nrf_mesh_mock_Verify();
    timer_mock_Verify();
    bearer_adv_mock_Verify();
    bearer_adv_mock_Destroy();
    provisioning_mock_Verify();
    provisioning_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    packet_mgr_mock_Verify();
    packet_mgr_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    prov_beacon_mock_Verify();
    prov_beacon_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

/* In order to use the prov_bearer_adv function. */
extern uint8_t calculate_3GPP_CRC(const uint8_t * p_input, uint16_t size);

/********** Test functions **********/
static ble_ad_data_t * get_transaction_start_packet(uint8_t * p_data, uint16_t data_size, uint8_t transaction)
{
    uint8_t start_payload_size = data_size > PROV_START_PDU_PAYLOAD_MAX_LEN ? PROV_START_PDU_PAYLOAD_MAX_LEN : data_size;
    uint8_t segN = ((data_size - start_payload_size) > 0) * (data_size - start_payload_size)/PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN; /*lint !e514 Boolean used in arithmetic */
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD - 1] = transaction;
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = (segN << 2) | PROV_TRANS_START_OPCODE_WITH_GPCF_FIELD;
    /* Total Length*/
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + 2] = data_size & 0xFF;
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + 1] = data_size >> 8;
    /* FCS*/
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + 3] = calculate_3GPP_CRC(p_data, data_size);
    memcpy(&trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_START_PDU_HEADER_SIZE ], p_data, start_payload_size);
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_data_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_START_PDU_HEADER_SIZE + start_payload_size;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_transaction_continue_packet(uint8_t * p_data, uint16_t data_size, uint8_t transaction, uint8_t segment)
{
    uint8_t continue_payload_size = data_size > PROV_START_PDU_PAYLOAD_MAX_LEN ? (data_size - PROV_START_PDU_PAYLOAD_MAX_LEN) : 0;
    if (segment == 0 || continue_payload_size <= (segment - 1 ) * PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN)
    {
        return NULL;
    }

    continue_payload_size -= (segment - 1 ) * PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN;
    continue_payload_size = continue_payload_size > PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN ? PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN: continue_payload_size;
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD - 1] = transaction;
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = (segment << 2) | PROV_TRANS_CONTINUE_OPCODE_WITH_GPCF_FIELD;
    memcpy(&trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + PROV_CONTINUE_PDU_HEADER_SIZE ], p_data + PROV_START_PDU_PAYLOAD_MAX_LEN + (segment - 1)* PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN, continue_payload_size);
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_data_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_CONTINUE_PDU_HEADER_SIZE + continue_payload_size;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_transaction_ack_packet(uint8_t transaction)
{
    trans_ack_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD - 1] = transaction;
    trans_ack_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = PROV_TRANS_ACK_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_ack_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_open_packet(uint8_t * p_uuid)
{
    link_open_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD;
    memcpy(&link_open_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD+1], p_uuid, NRF_MESH_UUID_SIZE);

    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_open_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_ack_packet(void)
{
    link_ack_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = PROV_LINK_ACK_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_ack_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_close_packet(nrf_mesh_prov_link_close_reason_t close_reason)
{
    link_close_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = PROV_LINK_CLOSE_OPCODE_WITH_GPCF_FIELD;
    link_close_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD+1] = (uint8_t)close_reason;

    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_close_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}
static void rx_link_open(prov_bearer_adv_t * p_bearer, uint8_t * p_uuid, uint32_t link_id, bool accept)
{
    ble_ad_data_t *  p_ad_data  = get_link_open_packet(p_uuid);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        bearer_adv_interval_reset_ExpectAndReturn(&p_bearer->advertiser, NRF_SUCCESS);
        ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
        prov_cb_link_opened_Expect(prov_bearer_adv_parent_get(p_bearer));
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 1000 + p_bearer->link_timeout);
    }
    prov_bearer_adv_pkt_in(p_ad_data);
}

static void rx_link_ack(prov_bearer_adv_t * p_bearer, uint32_t link_id, bool accept)
{
    ble_ad_data_t * p_ad_data  = get_link_ack_packet();
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        bearer_adv_flush_tx_Expect(&p_bearer->advertiser);
        prov_cb_link_opened_Expect(prov_bearer_adv_parent_get(p_bearer));
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 1000 + p_bearer->link_timeout);
    }
    prov_bearer_adv_pkt_in(p_ad_data);
}

static void rx_link_close(prov_bearer_adv_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason, uint32_t link_id,  bool accept)
{
    ble_ad_data_t * p_ad_data  = get_link_close_packet(close_reason);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        timer_sch_abort_Expect(&p_bearer->link_timeout_event);
        prov_cb_link_closed_Expect(prov_bearer_adv_parent_get(p_bearer), close_reason);
        bearer_adv_flush_tx_Expect(&p_bearer->advertiser);
    }
    prov_bearer_adv_pkt_in(p_ad_data);
}

static void tx_link_open(prov_bearer_adv_t * p_bearer, uint8_t * p_uuid, uint32_t link_id)
{
    p_bearer->link_id = link_id;
    p_bearer->link_timeout = 1000;
    p_bearer->p_next = NULL;

    ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_SUCCESS);

    bearer_adv_advertiser_init_Expect(&p_bearer->advertiser);
    rand_hw_rng_get_Expect((uint8_t*) &p_bearer->link_id, sizeof(p_bearer->link_id));
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 1000 + PROV_PROVISIONING_LINK_TIMEOUT_MIN_US);
    bearer_adv_adv_start_Expect(&p_bearer->advertiser);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_link_open(prov_bearer_adv_parent_get(p_bearer), p_uuid, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));
}

static void tx_link_close(prov_bearer_adv_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_SUCCESS);
    prov_bearer_adv_link_close(prov_bearer_adv_parent_get(p_bearer), close_reason);

    /* Send queue empty in order to move on to link closed state */
    bearer_adv_flush_tx_Expect(&p_bearer->advertiser);
    prov_cb_link_closed_Expect(prov_bearer_adv_parent_get(p_bearer), close_reason);
    timer_sch_abort_Expect(&p_bearer->link_timeout_event);
    p_bearer->advertiser.queue_empty_cb(&p_bearer->advertiser);
}

static void listen_start(prov_bearer_adv_t * p_bearer)
{
    packet_t packet;
    packet_generic_t * p_packet = &packet;
    bearer_adv_advertiser_init_Expect(&p_bearer->advertiser);
    prov_beacon_unprov_build_ExpectAndReturn(NULL, 0, p_packet);
    bearer_adv_tx_ExpectAndReturn(&p_bearer->advertiser, p_packet, BEARER_ADV_REPEAT_INFINITE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_listen(prov_bearer_adv_parent_get(p_bearer), NULL, 0, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));
}

static void listen_stop(prov_bearer_adv_t * p_bearer)
{
    bearer_adv_flush_tx_Expect(&p_bearer->advertiser);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_listen_stop(prov_bearer_adv_parent_get(p_bearer)));
}

static void receive_data_ack(prov_bearer_adv_t * p_bearer, uint32_t link_id, bool accept)
{
    ble_ad_data_t * p_ad_data  = get_transaction_ack_packet(p_bearer->transaction_out);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 1000 + p_bearer->link_timeout);
        timer_sch_abort_Expect(&p_bearer->timeout_event);
        bearer_adv_flush_tx_Expect(&p_bearer->advertiser);
        prov_cb_ack_in_Expect(prov_bearer_adv_parent_get(p_bearer));
    }
    prov_bearer_adv_pkt_in(p_ad_data);
}

static uint8_t send_data_packet_internals(prov_bearer_adv_t * p_bearer, uint8_t * p_data, uint8_t data_length)
{
    uint8_t no_segments = 1;
    uint8_t remaninig_data_length = data_length > PROV_START_PDU_PAYLOAD_MAX_LEN ? PROV_START_PDU_PAYLOAD_MAX_LEN : data_length;
    /* Start packet:*/
    ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_START_PDU_HEADER_SIZE + remaninig_data_length, NRF_SUCCESS, 1, NRF_SUCCESS);
    remaninig_data_length = data_length - remaninig_data_length;
    while (remaninig_data_length > 0)
    {
        no_segments++;
        uint8_t current_chunk_data_length = remaninig_data_length > PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN ? PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN: remaninig_data_length;
        ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_CONTINUE_PDU_HEADER_SIZE + current_chunk_data_length, NRF_SUCCESS, 1, NRF_SUCCESS);
        remaninig_data_length -= current_chunk_data_length;
    }
    return no_segments;
}

static uint8_t send_data_packet(prov_bearer_adv_t * p_bearer, uint8_t * p_data, uint8_t data_length)
{
    uint8_t no_segments = send_data_packet_internals(p_bearer, p_data, data_length);
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(0);
    timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 0);
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    timer_sch_reschedule_Expect(&p_bearer->timeout_event, 0);
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    (void) prov_bearer_adv_tx(prov_bearer_adv_parent_get(p_bearer), p_data, data_length);
    return no_segments;
}

static void receive_data(prov_bearer_adv_t * p_bearer, uint8_t * p_data, uint16_t data_length, uint8_t transcation, uint32_t link_id)
{
    /* Can't receive data until a link is established*/
    ble_ad_data_t * p_ad_data  = get_transaction_start_packet(p_data, data_length, transcation);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;

    ALLOC_AND_TX(&p_bearer->advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    prov_cb_pkt_in_Expect(prov_bearer_adv_parent_get(p_bearer), NULL, data_length);
    prov_cb_pkt_in_IgnoreArg_p_data();
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&p_bearer->link_timeout_event, 1000 + p_bearer->link_timeout);
    prov_bearer_adv_pkt_in(p_ad_data);

    uint8_t segment = 1;
    p_ad_data = get_transaction_continue_packet(p_data, data_length, transcation, segment);
    while (NULL != p_ad_data)
    {
        p_ad_data->data[3] = link_id & 0xFF;
        p_ad_data->data[2] = (link_id >> 8) & 0xFF;
        p_ad_data->data[1] = (link_id >> 16) & 0xFF;
        p_ad_data->data[0] = (link_id >> 24) & 0xFF;
        prov_bearer_adv_pkt_in(p_ad_data);
        segment++;
        p_ad_data = get_transaction_continue_packet(p_data, data_length, transcation, segment);
    }

}

void test_link_establish_active(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0xFEDCBA98;

    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);

    /* Invalid link open rx and tx*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer, uuid1, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));

    /* Close link */
    tx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS);
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* RX a packet with LINK_CLOSE command, expect a callback*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback. */
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Try to open a link and we fail due to pacman, the state should not change */
    bearer.bearer.pb_adv.link_id = link_id;
    bearer_adv_advertiser_init_Expect(&bearer.bearer.pb_adv.advertiser);
    rand_hw_rng_get_Expect((uint8_t*) &bearer.bearer.pb_adv.link_id, sizeof(bearer.bearer.pb_adv.link_id));
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE, NRF_ERROR_NO_MEM, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, prov_bearer_adv_link_open(&bearer, uuid1, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));

    /* Try to open a link and we fail due to bearer, the state should not change */
    bearer.bearer.pb_adv.link_id = link_id;
    bearer_adv_advertiser_init_Expect(&bearer.bearer.pb_adv.advertiser);
    rand_hw_rng_get_Expect((uint8_t*) &bearer.bearer.pb_adv.link_id, sizeof(bearer.bearer.pb_adv.link_id));
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_ERROR_NO_MEM);
    packet_mgr_free_Expect(mp_packet);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, prov_bearer_adv_link_open(&bearer, uuid1, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));

    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer.bearer.pb_adv, link_id, true);
    /* Invalid link open tx*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer, uuid1, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));
    /* Invalid listen stop call*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen_stop(&bearer));
    /* Invalid Link ACK */
    rx_link_ack(&bearer.bearer.pb_adv, link_id, false);
    /* Valid Link Close packet but different link id*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT, link_id + 1, false);
    /* Link CLOSE */
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT, link_id, true);
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* A call to the prov_bearer_adv_link_close shall not fail, so if one of the external modules lets us down
       we close without sending and immediately do a callback to the module above.
       *** Technically, this is illegal behaviour, perhaps we need a timer to retry closing? ***/
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, NRF_ERROR_NO_MEM, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_SUCCESS);
    bearer_adv_flush_tx_Expect(&bearer.bearer.pb_adv.advertiser);
    prov_cb_link_closed_Expect(&bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    timer_sch_abort_Expect(&bearer.bearer.pb_adv.link_timeout_event);
    prov_bearer_adv_link_close(&bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);

    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_ERROR_NO_MEM);
    bearer_adv_flush_tx_Expect(&bearer.bearer.pb_adv.advertiser);
    packet_mgr_free_Expect(mp_packet);
    prov_cb_link_closed_Expect(&bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    timer_sch_abort_Expect(&bearer.bearer.pb_adv.link_timeout_event);
    prov_bearer_adv_link_close(&bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);

    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* Close link */
    tx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* Close link */
    tx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* RX a packet with LINK_CLOSE command, expect a callback. */
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR, link_id, true);
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* RX of unknown close command is acceptable, and this should be forwarded. */
    rx_link_close(&bearer.bearer.pb_adv, (nrf_mesh_prov_link_close_reason_t) 0xff, link_id, true);
    /* Close link with unknown close command is not acceptable, assert */
    TEST_NRF_MESH_ASSERT_EXPECT(prov_bearer_adv_link_close(&bearer, (nrf_mesh_prov_link_close_reason_t) 0xff));
}

void test_link_establish_passive(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0xFFFFFFFF;

    /* Start listenining. */
    listen_start(&bearer.bearer.pb_adv);
    /* Ignore if by chance we match a close link request to the dummy link_id of 0. */
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, 0, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen(&bearer, NULL, 0, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));
    listen_stop(&bearer.bearer.pb_adv);
    listen_start(&bearer.bearer.pb_adv);

    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Send a packet with the LINK_OPEN opcode but no uuid, should be ignored. */
    minimal_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) minimal_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD + 1;
    p_ad_data->type = AD_TYPE_PB_ADV;
    prov_bearer_adv_pkt_in(p_ad_data);

    /* Send a control packet with the unknown opcode, should be ignored:*/
    minimal_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD] = 0xFF;
    p_ad_data = (ble_ad_data_t *) minimal_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD + 1;
    p_ad_data->type = AD_TYPE_PB_ADV;
    prov_bearer_adv_pkt_in(p_ad_data);

    /* RX a packet with LINK_OPEN command using wrong uuid*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid2, link_id, false);

    /* Fail RX of a packet with LINK_OPEN command due to PACKET MANAGER*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    bearer_adv_interval_reset_ExpectAndReturn(&bearer.bearer.pb_adv.advertiser, NRF_SUCCESS);
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, NRF_ERROR_NO_MEM, 1, NRF_SUCCESS);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);

    /* Fail RX of a packet with LINK_OPEN command due to BEARER ADV */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    bearer_adv_interval_reset_ExpectAndReturn(&bearer.bearer.pb_adv.advertiser, NRF_SUCCESS);
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_ERROR_NO_MEM);
    packet_mgr_free_Expect(mp_packet);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);

    /* State shouldn't have changed due to the failures, so a successful RX should be possible: */

    /* RX a packet with LINK_OPEN command*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, true);

    /* Sending another link open should produce an ack without the callback to higher layers */
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen(&bearer, NULL, 0, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer, uuid1, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US));

    /* RX a packet with LINK_CLOSE command, expect a callback*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);

    /* RX a packet with LINK_OPEN command, do not expect a link opened callback*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);
}

void test_multiple_bearer(void)
{
    prov_bearer_t bearer1 = {0};
    prov_bearer_t bearer2 = {0};
    prov_bearer_t bearer3 = {0};
    prov_bearer_t bearer4 = {0};
    uint32_t link_id1 = 0x1;
    uint32_t link_id2 = 0x10;
    uint32_t link_id3 = 0x100;
    uint32_t link_id4 = 0x1000;

    /** Test out of order connection establishment (both as provisioner and provisionee). */
    /* Start listenining on bearer 1. */
    listen_start(&bearer1.bearer.pb_adv);
    /* Open link 2. */
    tx_link_open(&bearer2.bearer.pb_adv, uuid2, link_id2);
    /* Open link 3. */
    tx_link_open(&bearer3.bearer.pb_adv, uuid1, link_id3);
    /* Stop listening on bearer1 before listening on bearer4 (out of order listening is not allowed)*/
    /* Stopping bearer 1 after taking action with bearer 2 and 3 in order to test remove_active_bearer
       function in prov_bearer_adv.c */
    listen_stop(&bearer1.bearer.pb_adv);
    /* Start listenining on bearer 4. */
    listen_start(&bearer4.bearer.pb_adv);
    /* Send link open on bearer1: doing this after starting to listen on bearer 4 in order to test
       get_bearer_from_state  function in prov_bearer_adv.c */
    tx_link_open(&bearer1.bearer.pb_adv, uuid2, link_id1);

    /* LINK_OPEN for bearer4*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer4.bearer.pb_adv, uuid1, link_id4, true);
    /* Link ACK for bearer 1, 2 and 3*/
    rx_link_ack(&bearer3.bearer.pb_adv, link_id3, true);
    /* Link ACK */
    rx_link_ack(&bearer1.bearer.pb_adv, link_id1, true);
    /* Link ACK */
    rx_link_ack(&bearer2.bearer.pb_adv, link_id2, true);

    rx_link_close(&bearer1.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id1, true);
    rx_link_close(&bearer2.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id2, true);
    rx_link_close(&bearer3.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id3, true);
    rx_link_close(&bearer4.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id4, true);
}

void test_packet_send(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0;
    uint8_t curr_transcation = PROVISIONER_TRANSACTION_START_VALUE;
    /* Can't send data until a link is established*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_tx(&bearer, data, PROV_PAYLOAD_MAX_LENGTH));

    /** Establish a connection **/
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer.bearer.pb_adv, link_id, true);

    /* Can't send data larger than PROV_PAYLOAD_MAX_LENGTH */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, prov_bearer_adv_tx(&bearer, data, PROV_PAYLOAD_MAX_LENGTH+1));

    /* Send 1 byte long data */
    uint8_t no_segments = send_data_packet(&bearer.bearer.pb_adv, data, 1);
    /* transcation no starts with 0 when in provisioner role (section 5.2.1 in mesh core spec d09r19) */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Can't send data without receiving an ack or timeout for the previous one*/
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, prov_bearer_adv_tx(&bearer, data, 1));
    /* Send in an ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;

    /* Send the largest packet that can fit into a single segment. */
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;

    /* Send a large enough packet to split it in two with only 1 byte in the continue packet. */
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(2, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;

    /* Send a large packet that barely fits two packets*/
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(2, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;

    /* Send a large enough packet to split it in three with only 1 byte in the last continue packet. */
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN + 1);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;

    /* Send the largest possible data packet */
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_PAYLOAD_MAX_LENGTH);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* No ack, send a timeout */
    (void) send_data_packet_internals(&bearer.bearer.pb_adv, data, PROV_PAYLOAD_MAX_LENGTH);
    bearer.bearer.pb_adv.timeout_event.cb(0, &bearer.bearer.pb_adv);
    /* Transcation no must remain the same */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* Send in an ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);

    /* Close link*/
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
}

void test_packet_send_abnormal(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0x12345678;
    uint8_t curr_transcation = PROVISIONEE_TRANSACTION_START_VALUE;
    /* Can't send data until a link is established*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_tx(&bearer, data, PROV_PAYLOAD_MAX_LENGTH));

    /* Start listenining. */
    listen_start(&bearer.bearer.pb_adv);
    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, true);

    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    uint8_t no_segments = send_data_packet(&bearer.bearer.pb_adv, data, 1);
    TEST_ASSERT_EQUAL(1, no_segments);

    /* Send in ack with wrong transcation value, it should be ignored */
    prov_bearer_adv_pkt_in(get_transaction_ack_packet(curr_transcation-1));
    /* LINK_OPEN and LINK_ACK control messages should also be ignored */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, false);
    rx_link_ack(&bearer.bearer.pb_adv, link_id, false);
    /* Send in an ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, true);
    curr_transcation++;
    /* Ignore if not expecting an ack */
    receive_data_ack(&bearer.bearer.pb_adv, link_id, false);

    /* Send the largest possible data packet */
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, PROV_PAYLOAD_MAX_LENGTH);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* No ACK */
    /* Time out cb when the link time out has passed: this should produce a LINK_CLOSE message*/
    /* First attempt to TX fails*/
    timestamp_t timeout_timestamp = bearer.bearer.pb_adv.sar_timeout + 1;
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_ERROR_NO_MEM);
    packet_mgr_free_Expect(mp_packet);
    bearer.bearer.pb_adv.timeout_event.cb(bearer.bearer.pb_adv.sar_timeout + 1, &bearer.bearer.pb_adv);

    /* timeout interval should be larger than 0 */
    TEST_ASSERT(bearer.bearer.pb_adv.timeout_event.interval > 0);
    /* Timeout value should be updated to NOW to ensure a timeout trigger for the next retry cb. */
    TEST_ASSERT_EQUAL(timeout_timestamp, bearer.bearer.pb_adv.sar_timeout);
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, NRF_SUCCESS, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, NRF_SUCCESS);
    bearer.bearer.pb_adv.timeout_event.cb(bearer.bearer.pb_adv.sar_timeout+1, &bearer.bearer.pb_adv);

    /* Send queue empty in order to move on to link closed state */
    timer_sch_abort_Expect(&bearer.bearer.pb_adv.timeout_event);
    bearer_adv_flush_tx_Expect(&bearer.bearer.pb_adv.advertiser);
    prov_cb_link_closed_Expect(prov_bearer_adv_parent_get(&bearer.bearer.pb_adv), NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    timer_sch_abort_Expect(&bearer.bearer.pb_adv.link_timeout_event);
    bearer.bearer.pb_adv.advertiser.queue_empty_cb(&bearer.bearer.pb_adv.advertiser);

    /* Receive a link close when in TX state */
    /* Start listenining. */
    listen_start(&bearer.bearer.pb_adv);
    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, true);
    no_segments = send_data_packet(&bearer.bearer.pb_adv, data, 1);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Close link*/
    timer_sch_abort_Expect(&bearer.bearer.pb_adv.timeout_event);
    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
}

void test_packet_receive(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0xDAFA;
    uint8_t curr_transcation = PROVISIONEE_TRANSACTION_START_VALUE;
    /** Establish a connection **/
    /* Open link. */
    tx_link_open(&bearer.bearer.pb_adv, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer.bearer.pb_adv, link_id, true);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send the largest packet that can fit into a single segment. */
    receive_data(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send a large enough packet to split it in two with only 1 byte in the continue packet. */
    receive_data(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send a large packet that barely fits two packets*/
    receive_data(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send a large enough packet to split it in three with only 1 byte in the last continue packet. */
    receive_data(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send the largest possible data packet */
    receive_data(&bearer.bearer.pb_adv, data, PROV_PAYLOAD_MAX_LENGTH, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Bump up the transaction number to a near wrap_around*/
    bearer.bearer.pb_adv.transaction_in = 0xFF;
    curr_transcation = 0xFF;
    receive_data(&bearer.bearer.pb_adv, data, PROV_START_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);
    /* Re send the old packet after the wrap around and expect a response but no callback!*/
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    prov_cb_pkt_in_Expect(prov_bearer_adv_parent_get(&bearer.bearer.pb_adv), NULL, 1);
    prov_cb_pkt_in_IgnoreArg_p_data();
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&bearer.bearer.pb_adv.link_timeout_event, 1000 + bearer.bearer.pb_adv.link_timeout);
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data, 1, curr_transcation));
    TEST_ASSERT_EQUAL(curr_transcation+1, bearer.bearer.pb_adv.transaction_in);

    rx_link_close(&bearer.bearer.pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);

    /* Send unknown control packet. Nothing should happen*/
    ble_ad_data_t * p_ad_data = get_link_open_packet(uuid1);
    p_ad_data->data[PROV_ADV_OVERHEAD] = 0xFF;
    prov_bearer_adv_pkt_in(p_ad_data);

    /* Start listenining. */
    listen_start(&bearer.bearer.pb_adv);
    /* Send unknown control packet. Nothing should happen*/
    p_ad_data = get_link_open_packet(uuid1);
    p_ad_data->data[PROV_ADV_OVERHEAD] = 0xFF;
    prov_bearer_adv_pkt_in(p_ad_data);

    listen_stop(&bearer.bearer.pb_adv);
}

void test_packet_receive_abnormal(void)
{
    prov_bearer_t bearer = {0};
    uint32_t link_id = 0xBACA;
    uint8_t curr_transcation = PROVISIONER_TRANSACTION_START_VALUE;
    /* Can't receive data until a link is established*/
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data,1,0));

    /* Start listenining. */
    listen_start(&bearer.bearer.pb_adv);

    /* Can't receive data until a link is established*/
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data,1,0));

    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer.bearer.pb_adv, uuid1, link_id, true);

    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);
    /* Send 1 byte long data. */
    receive_data(&bearer.bearer.pb_adv, data, 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send the same data. */
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data, 1, curr_transcation-1));
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Re-send different data with the same transcation value, it won't care and still send an ack and ignore contents. */
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    prov_bearer_adv_pkt_in(get_transaction_start_packet(&data[10], 10, curr_transcation-1));
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Increment transaction number with a large number, should be OK, and the next expected
     * transaction number should be incremented. */
    curr_transcation += 10;
    receive_data(&bearer.bearer.pb_adv, data, 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /* Send new data with wrong fcs (CRC)*/
    ble_ad_data_t * p_ad_data = get_transaction_start_packet(&data[10], 10, curr_transcation);
    trans_data_payload[sizeof(ble_ad_data_t) + PROV_ADV_OVERHEAD + 3]++;
    prov_bearer_adv_pkt_in(p_ad_data);
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);

    /***** Send a new segmented packet and in between send an old packet *****/
    /* First segment of the new packet: */
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data, PROV_START_PDU_PAYLOAD_MAX_LEN +1, curr_transcation));
    /* transaction has not been updated yet*/
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);
    /* Re send an old packet but do not expect a response*/
    prov_bearer_adv_pkt_in(get_transaction_start_packet(data, 1, curr_transcation-1));
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);
    /* Send the rest of the new packet and expect an ack */
    ALLOC_AND_TX(&bearer.bearer.pb_adv.advertiser, BLE_ADV_OVERHEAD + PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, NRF_SUCCESS, 1, NRF_SUCCESS);
    prov_cb_pkt_in_Expect(prov_bearer_adv_parent_get(&bearer.bearer.pb_adv), NULL, PROV_START_PDU_PAYLOAD_MAX_LEN + 1);
    prov_cb_pkt_in_IgnoreArg_p_data();
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&bearer.bearer.pb_adv.link_timeout_event, 1000 + bearer.bearer.pb_adv.link_timeout);
    prov_bearer_adv_pkt_in(get_transaction_continue_packet(data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, 1));
    /* transaction number is now updated */
    TEST_ASSERT_EQUAL(curr_transcation, bearer.bearer.pb_adv.transaction_in);
}

void test_interface_return(void)
{
    const prov_bearer_interface_t * p_interface = prov_bearer_adv_interface_get();
    TEST_ASSERT_NOT_NULL(p_interface);
    TEST_ASSERT_NOT_NULL(p_interface->tx);
    TEST_ASSERT_NOT_NULL(p_interface->listen_start);
    TEST_ASSERT_NOT_NULL(p_interface->listen_stop);
    TEST_ASSERT_NOT_NULL(p_interface->link_open);
    TEST_ASSERT_NOT_NULL(p_interface->link_close);
}

void test_packet_fcs_sample_check(void)
{
    const uint8_t sample_data_8_7_3[] = {0x00, 0x00};
    TEST_ASSERT_EQUAL(0x14, calculate_3GPP_CRC(sample_data_8_7_3, sizeof(sample_data_8_7_3)));

    const uint8_t sample_data_8_7_4[] = {0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(0xD6, calculate_3GPP_CRC(sample_data_8_7_4, sizeof(sample_data_8_7_4)));

    const uint8_t sample_data_8_7_5[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(0x64, calculate_3GPP_CRC(sample_data_8_7_5, sizeof(sample_data_8_7_5)));

    const uint8_t sample_data_8_7_6[] =
    {
        0x03, 0x2c, 0x31, 0xa4, 0x7b, 0x57, 0x79, 0x80, 0x9e, 0xf4, 0x4c, 0xb5, 0xea, 0xaf, 0x5c, 0x3e,
        0x43, 0xd5, 0xf8, 0xfa, 0xad, 0x4a, 0x87, 0x94, 0xcb, 0x98, 0x7e, 0x9b, 0x03, 0x74, 0x5c, 0x78,
        0xdd, 0x91, 0x95, 0x12, 0x18, 0x38, 0x98, 0xdf, 0xbe, 0xcd, 0x52, 0xe2, 0x40, 0x8e, 0x43, 0x87,
        0x1f, 0xd0, 0x21, 0x10, 0x91, 0x17, 0xbd, 0x3e, 0xd4, 0xea, 0xf8, 0x43, 0x77, 0x43, 0x71, 0x5d,
        0x4f
    };
    TEST_ASSERT_EQUAL(0xD1, calculate_3GPP_CRC(sample_data_8_7_6, sizeof(sample_data_8_7_6)));

    const uint8_t sample_data_8_7_7[] =
    {
        0x03, 0xf4, 0x65, 0xe4, 0x3f, 0xf2, 0x3d, 0x3f, 0x1b, 0x9d, 0xc7, 0xdf, 0xc0, 0x4d, 0xa8, 0x75,
        0x81, 0x84, 0xdb, 0xc9, 0x66, 0x20, 0x47, 0x96, 0xec, 0xcf, 0x0d, 0x6c, 0xf5, 0xe1, 0x65, 0x00,
        0xcc, 0x02, 0x01, 0xd0, 0x48, 0xbc, 0xbb, 0xd8, 0x99, 0xee, 0xef, 0xc4, 0x24, 0x16, 0x4e, 0x33,
        0xc2, 0x01, 0xc2, 0xb0, 0x10, 0xca, 0x6b, 0x4d, 0x43, 0xa8, 0xa1, 0x55, 0xca, 0xd8, 0xec, 0xb2,
        0x79
    };
    TEST_ASSERT_EQUAL(0x10, calculate_3GPP_CRC(sample_data_8_7_7, sizeof(sample_data_8_7_7)));

    const uint8_t sample_data_8_7_8[] = {0x05, 0xb3, 0x8a, 0x11, 0x4d, 0xfd, 0xca, 0x1f, 0xe1, 0x53, 0xbd, 0x2c, 0x1e, 0x0d, 0xc4, 0x6a, 0xc2};
     TEST_ASSERT_EQUAL(0xD1, calculate_3GPP_CRC(sample_data_8_7_8, sizeof(sample_data_8_7_8)));

    const uint8_t sample_data_8_7_9[] = {0x05, 0xee, 0xba, 0x52, 0x1c, 0x19, 0x6b, 0x52, 0xcc, 0x2e, 0x37, 0xaa, 0x40, 0x32, 0x9f, 0x55, 0x4e};
     TEST_ASSERT_EQUAL(0xEC, calculate_3GPP_CRC(sample_data_8_7_9, sizeof(sample_data_8_7_9)));

    const uint8_t sample_data_8_7_10[] = {0x06, 0x8b, 0x19, 0xac, 0x31, 0xd5, 0x8b, 0x12, 0x4c, 0x94, 0x62, 0x09, 0xb5, 0xdb, 0x10, 0x21, 0xb9};
    TEST_ASSERT_EQUAL(0xD3, calculate_3GPP_CRC(sample_data_8_7_10, sizeof(sample_data_8_7_10)));

    const uint8_t sample_data_8_7_11[] = {0x06, 0x55, 0xa2, 0xa2, 0xbc, 0xa0, 0x4c, 0xd3, 0x2f, 0xf6, 0xf3, 0x46, 0xbd, 0x0a, 0x0c, 0x1a, 0x3a};
    TEST_ASSERT_EQUAL(0x59, calculate_3GPP_CRC(sample_data_8_7_11, sizeof(sample_data_8_7_11)));

    const uint8_t sample_data_8_7_12[] =
    {
        0x07, 0xd0, 0xbd, 0x7f, 0x4a, 0x89, 0xa2, 0xff, 0x62, 0x22, 0xaf, 0x59, 0xa9, 0x0a, 0x60, 0xad,
        0x58, 0xac, 0xfe, 0x31, 0x23, 0x35, 0x6f, 0x5c, 0xec, 0x29, 0x73, 0xe0, 0xec, 0x50, 0x78, 0x3b,
        0x10, 0xc7
    };
    TEST_ASSERT_EQUAL(0x8B, calculate_3GPP_CRC(sample_data_8_7_12, sizeof(sample_data_8_7_12)));

    const uint8_t sample_data_8_7_13[] = {0x08};
    TEST_ASSERT_EQUAL(0x3E, calculate_3GPP_CRC(sample_data_8_7_13, sizeof(sample_data_8_7_13)));
}


