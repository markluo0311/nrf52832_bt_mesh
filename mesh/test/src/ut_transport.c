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

#include <cmock.h>
#include <unity.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>

#include "enc.h"
#include "transport.h"
#include "nrf_error.h"
#include "event.h"
#include "replay_cache.h"
#include "timer_scheduler.h"
#include "nrf_mesh.h"
#include "log.h"
#include "packet_mesh.h"

#include "bearer_event_mock.h"
#include "net_state_mock.h"

#define UNUSED_ADDR    0x0000
#define UNICAST_ADDR   0x1201
#define UNICAST_DST    0x0607
#define GROUP_ADDR     0xF00D
#define BROADCAST_ADDR 0xFFFF

#define NETWORK_KEY     {0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6}
#define APPLICATION_KEY {0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48}
#define DEVICE_KEY      {0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48}
#define APPLICATION_ID  0x26

#define IV_INDEX 0x12345678
#define MSG_SEQ_NO 0x000007

/* Test message #18 from the specification: */
#define PACKET_TV0_UNENCRYPTED {0x18, AD_TYPE_MESH, 0x68, 0x03, 0x00, 0x00, 0x07, 0x12, 0x01, 0xff, 0xff, 0x66, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define PACKET_TV0_TRANSPORT   {0x18, AD_TYPE_MESH, 0x68, 0x03, 0x00, 0x00, 0x07, 0x12, 0x01, 0xff, 0xff, 0x66, 0x5a, 0x8b, 0xde, 0x6d, 0x91, 0x06, 0xea, 0x07, 0x8a, 0x00, 0x00, 0x00, 0x00}

#define NUM_PACKETS 1

nrf_mesh_assertion_handler_t m_assertion_handler;

static const uint8_t m_net_key[NRF_MESH_KEY_SIZE] = NETWORK_KEY;
static const uint8_t m_app_key[NRF_MESH_KEY_SIZE] = APPLICATION_KEY;
static const uint8_t m_dev_key[NRF_MESH_KEY_SIZE] = DEVICE_KEY;

static const uint8_t m_unenc_pkts[NUM_PACKETS][28] = {PACKET_TV0_UNENCRYPTED};
static const uint8_t m_enc_pkts[NUM_PACKETS][28] = {PACKET_TV0_TRANSPORT};

static const uint8_t m_sar_rx_pkts[2][31] =
{
    {0x1e, AD_TYPE_MESH, 0x68, 0x04, 0x31, 0x29, 0xab, 0x00, 0x03, 0x12, 0x01, 0x80, 0x26, 0xac, 0x01, 0xec, 0xe8, 0x88, 0xaa,
        0x21, 0x69, 0x32, 0x6d, 0x23, 0xf3, 0xaf, 0xdf, 0x00, 0x00, 0x00, 0x00},
    {0x1e, AD_TYPE_MESH, 0x68, 0x04, 0x31, 0x29, 0xac, 0x00, 0x03, 0x12, 0x01, 0x80, 0x26, 0xac, 0x21, 0xcf, 0xdc, 0x18, 0xc5,
        0x2f, 0xde, 0xf7, 0x72, 0x69, 0xff, 0x44, 0x33, 0x00, 0x00, 0x00, 0x00}
};


static packet_net_t * mp_net_pkt_out;
static packet_t * mp_packet;
static uint32_t m_network_pkt_out_return = NRF_SUCCESS;

static nrf_mesh_init_params_t * mp_mesh_init_params;

static nrf_mesh_application_secmat_t m_application;
static nrf_mesh_network_secmat_t     m_network;
static nrf_mesh_application_secmat_t m_device_key;

static nrf_mesh_address_t m_addr;

static uint32_t m_seqnum;
static uint32_t m_iv_index;

static unsigned m_event_push_count;

static unsigned m_timers_added;
static timestamp_t m_time_now;

typedef enum
{
    TIMER_UNINITIALIZED = 0,
    TIMER_SCHEDULED,
    TIMER_ABORTED
} timer_state_t;

typedef struct
{
    timer_event_t * p_timer;
    timer_state_t   state;
} transport_timer_t;

static transport_timer_t m_timer_event_buffer[4];

/********************/
/* Mockup functions */
/********************/

timestamp_t timer_now()
{
    return m_time_now;
}

void timer_sch_reschedule(timer_event_t * p_timer, timestamp_t time)
{
    if (p_timer != NULL)
    {
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "(Re)-scheduling timer!\n");
        for (int i = 0; i < 4; ++i)
        {
            if (m_timer_event_buffer[i].p_timer == p_timer)
            {
                m_timer_event_buffer[i].state = TIMER_SCHEDULED;
                break;
            }

            if (m_timer_event_buffer[i].state == TIMER_UNINITIALIZED)
            {
                m_timer_event_buffer[i].p_timer = p_timer;
                m_timer_event_buffer[i].state = TIMER_SCHEDULED;
                m_timers_added++;
                break;
            }
        }
    }
}

void timer_sch_abort(timer_event_t * p_timer)
{
    if (p_timer)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (m_timer_event_buffer[i].p_timer == p_timer)
            {
                m_timer_event_buffer[i].state = TIMER_ABORTED;
                break;
            }

            if (m_timer_event_buffer[i].state == TIMER_UNINITIALIZED)
            {
                m_timer_event_buffer[i].p_timer = p_timer;
                m_timer_event_buffer[i].state = TIMER_ABORTED;
                break;
            }
        }
    }
}

uint32_t network_make_derived_keys(nrf_mesh_network_secmat_t * p_net_secmat)
{
    return NRF_SUCCESS;
}

uint32_t network_pkt_out(packet_t * p_packet, const nrf_mesh_network_secmat_t * const p_net_secmat, bool local_packet)
{
    /* __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, ) */
    if (mp_net_pkt_out != NULL)
    {
        free(mp_net_pkt_out);
    }

    mp_net_pkt_out = malloc(p_packet->payload[0] + 1);
    TEST_ASSERT_NOT_NULL(mp_net_pkt_out);
    memcpy(mp_net_pkt_out, p_packet->payload, p_packet->payload[0] + 1);
    __LOG_XB(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "network_pkt_out: ", (const uint8_t *) mp_net_pkt_out, p_packet->payload[0] + 1);
    return m_network_pkt_out_return;
}

uint32_t network_pkt_relay(packet_net_t * p_net_packet, const nrf_mesh_network_secmat_t * const p_net_secmat)
{
    return NRF_SUCCESS;
}

uint32_t network_iv_update_acknowledge(nrf_mesh_network_secmat_t * p_net_secmat)
{
    return NRF_SUCCESS;
}

void event_handle(nrf_mesh_evt_t * p_event)
{
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Pushing event: %u.\n", p_event->type);
    m_event_push_count ++;
}

static uint32_t net_state_seqnum_alloc_mock(uint32_t * p_seqnum, int count)
{
    *p_seqnum = m_seqnum++;
    return NRF_SUCCESS;
}

static uint32_t net_state_rx_iv_index_get_mock(uint8_t ivi, int count)
{
    TEST_ASSERT_EQUAL(ivi, ivi & 0x01);
    if (ivi == (m_iv_index & 0x01))
    {
        return m_iv_index;
    }
    else
    {
        return m_iv_index - 1;
    }
}

static uint32_t net_state_tx_iv_index_get_mock(int count)
{
    return m_iv_index;
}

static void net_state_iv_index_lock_mock(bool lock, int count)
{
    /* Does nothing */
}

void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat, uint8_t aid, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    TEST_ASSERT_EQUAL_PTR(&m_network, p_network_secmat);
    if (*pp_app_secmat == NULL && aid == m_application.aid)
    {
        *pp_app_secmat = &m_application;
        return;
    }
    *pp_app_secmat = NULL;
}

void nrf_mesh_devkey_secmat_get(uint16_t unicast_addr, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    if (*pp_app_secmat == NULL)
    {
        *pp_app_secmat = &m_device_key;
        return;
    }
}

bool nrf_mesh_rx_address_get(uint16_t raw_address, nrf_mesh_address_t * p_address)
{
    if (raw_address == m_addr.value)
    {
        memcpy(p_address, &m_addr, sizeof(m_addr));
        return true;
    }
    else if (raw_address == 0xFFFF)
    {
        p_address->type = NRF_MESH_ADDRESS_TYPE_GROUP;
        p_address->value = 0xFFFF;
        p_address->p_virtual_uuid = NULL;
        return true;
    }
    return false;
}
/*********************/
/* Utility functions */
/*********************/

static void provision(const uint8_t * app_key, const uint8_t * device_key, uint32_t iv_index, uint32_t msg_seq_no)
{
    memset(&m_network, 0, sizeof(nrf_mesh_network_secmat_t));
    memcpy(m_network.encryption_key, m_net_key, NRF_MESH_KEY_SIZE);
    m_seqnum = msg_seq_no;
    m_iv_index = iv_index;

    uint32_t status = network_make_derived_keys(&m_network);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, status);

    memset(&m_application, 0, sizeof(nrf_mesh_application_secmat_t));
    memcpy(m_application.key, app_key, NRF_MESH_KEY_SIZE);
    m_application.aid = APPLICATION_ID;

    memset(&m_device_key, 0, sizeof(nrf_mesh_application_secmat_t));
    memcpy(m_device_key.key, device_key, NRF_MESH_KEY_SIZE);
    m_device_key.aid = 0;
}

/************************/
/* Test setup functions */
/************************/


void setUp(void)
{

    __LOG_INIT((LOG_SRC_ENC | LOG_SRC_TRANSPORT | LOG_SRC_CCM |LOG_SRC_TEST), 4, LOG_CALLBACK_DEFAULT);

    packet_mgr_init(mp_mesh_init_params);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_mgr_alloc((packet_generic_t **) &mp_packet, sizeof(packet_t)));

    transport_init(NULL);

    m_event_push_count = 0;
    m_network_pkt_out_return = NRF_SUCCESS;

    memset(m_timer_event_buffer, 0, sizeof(m_timer_event_buffer));
    m_timers_added = 0;
    m_time_now = 0;

    bearer_event_mock_Init();
    net_state_mock_Init();

    net_state_seqnum_alloc_StubWithCallback(net_state_seqnum_alloc_mock);
    net_state_rx_iv_index_get_StubWithCallback(net_state_rx_iv_index_get_mock);
    net_state_tx_iv_index_get_StubWithCallback(net_state_tx_iv_index_get_mock);
    net_state_iv_index_lock_StubWithCallback(net_state_iv_index_lock_mock);
}

void tearDown(void)
{
    replay_cache_clear();
    packet_mgr_free(mp_packet);

    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
}

void setup_test_packet(const uint8_t * network_packet)
{
    uint32_t size = (network_packet[0] + 1) + sizeof(ble_packet_hdr_t) + BLE_GAP_ADDR_LEN;
    printf("Size: %u %u\n", size, network_packet[0]);
    TEST_ASSERT(size <= BLE_ADV_PACKET_MAX_LENGTH);

    /* Allocate and construct an incoming packet: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_mgr_alloc((packet_generic_t **) &mp_packet, size));
    memset(mp_packet, 0, size);

    packet_payload_size_set(mp_packet, network_packet[0] + 1);
    memcpy(mp_packet->payload, network_packet, network_packet[0] + 1);
}

/******************/
/* Test functions */
/******************/

void test_transport_init(void)
{
    transport_init(mp_mesh_init_params);
}

void test_transport_pkt_in(void)
{
    uint32_t status;
    packet_meta_t meta = {0};
    uint8_t meta_addr[6];
    meta.p_addr = meta_addr;

    nrf_mesh_address_t unicast_addr = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = UNICAST_DST, .p_virtual_uuid = NULL};
    memcpy(&m_addr, &unicast_addr, sizeof(unicast_addr));

    /* Add keys: */
    provision(m_app_key, m_dev_key, IV_INDEX, MSG_SEQ_NO);

    for (int i = 0; i < NUM_PACKETS; ++i)
    {
        replay_cache_clear();
        printf("Packet %d\n", i);
        setup_test_packet(m_enc_pkts[i]);
        packet_net_t * p_net_pkt = packet_net_packet_get(mp_packet);
        packet_net_t * p_out     = (packet_net_t *) m_unenc_pkts[i];
        packet_mesh_t * p_mesh_packet = (packet_mesh_t *) p_net_pkt;

        printf("ivi: 0x%02x\n"
               "nid: 0x%02x\n"
               "ctl: 0x%02x\n"
               "ttl: 0x%02x\n"
               "seq: 0x%08x\n"
               "src: 0x%04x\n"
               "dst: 0x%04x\n"
               "seg: 0x%02x\n",
               packet_mesh_net_ivi_get(p_mesh_packet),
               packet_mesh_net_nid_get(p_mesh_packet),
               packet_mesh_net_ctl_get(p_mesh_packet),
               packet_mesh_net_ttl_get(p_mesh_packet),
               packet_mesh_net_seq_get(p_mesh_packet),
               packet_mesh_net_src_get(p_mesh_packet),
               packet_mesh_net_dst_get(p_mesh_packet),
               packet_mesh_trs_seg_get(p_mesh_packet)
            );

        p_net_pkt->header.dst = 0;

        /* Test function. */
        status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, status);

        p_net_pkt->header.dst = 0xffff;

        status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

        /* Packet should have been pushed to app.  */
        TEST_ASSERT_EQUAL(i + 1, m_event_push_count);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(p_out->payload, p_net_pkt->payload, packet_net_payload_size_get(p_net_pkt) - 8);
    }
}

void test_transport_reject_invalid_app_key(void)
{
    uint32_t status;
    packet_meta_t meta = {0};
    uint8_t meta_addr[6];
    meta.p_addr = meta_addr;

    nrf_mesh_address_t address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = UNICAST_ADDR, .p_virtual_uuid = NULL};
    memcpy(&m_addr, &address, sizeof(m_addr));
    setup_test_packet(m_enc_pkts[0]);

    packet_net_t * p_net_pkt = packet_net_packet_get(mp_packet);

    uint8_t invalid_app_key[16];

    memcpy(invalid_app_key, m_app_key, sizeof(m_app_key));
    invalid_app_key[3] = 0x00;

    /* Add key and address. */
    provision(invalid_app_key, m_dev_key, IV_INDEX, MSG_SEQ_NO);

    p_net_pkt->header.dst = 0;
    /* Test function. */
    status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, status);

    p_net_pkt->header.dst = LE2BE16(UNICAST_ADDR);
    status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, status);

    /* Packet should be silently ignored */
    TEST_ASSERT_EQUAL(0, m_event_push_count);

}

void test_transport_replay_cache(void)
{
    packet_meta_t meta = {0};
    uint32_t status;
    uint8_t meta_addr[6];
    meta.p_addr = meta_addr;
    nrf_mesh_address_t address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x0607, .p_virtual_uuid = NULL};
    memcpy(&m_addr, &address, sizeof(m_addr));
    setup_test_packet(m_enc_pkts[0]);

    packet_net_t * p_net_pkt = packet_net_packet_get(mp_packet);
    packet_net_t * p_out_enc = (packet_net_t *) m_enc_pkts[0];
    packet_net_t * p_out_unenc = (packet_net_t *) m_unenc_pkts[0];

    /* Add keys: */
    provision(m_app_key, m_dev_key, IV_INDEX, MSG_SEQ_NO);

    /* Check that packet is not added to cache if crypto fails */
    /* Mess up MIC */
    p_net_pkt->payload[0] -= 1;
    status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, status);

    setup_test_packet(m_enc_pkts[0]);

    /* Test function. */
    status = transport_pkt_in(packet_net_packet_get(mp_packet), &meta, &m_network, m_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    /* Setup same packet again. */
    setup_test_packet(m_enc_pkts[0]);

    status = transport_pkt_in(packet_net_packet_get(mp_packet), &meta, &m_network, m_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    /* Should have been caught by replay protection and not decrypted */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_out_enc, packet_net_packet_get(mp_packet), packet_net_payload_size_get(packet_net_packet_get(mp_packet)) - 8);

    replay_cache_clear();

    /* Setup same packet again. */
    setup_test_packet(m_enc_pkts[0]);
    p_net_pkt->payload[0] += 1;

    status = transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_out_unenc, p_net_pkt, packet_net_payload_size_get(p_net_pkt) - 8);
}

void test_virtual_addressing(void)
{
    TEST_IGNORE_MESSAGE("Not implemented");
}

void test_trs_sar_rx(void)
{
    const uint8_t devkey[] = {0x9d, 0x6d, 0xd0, 0xe9, 0x6e, 0xb2, 0x5d, 0xc1, 0x9a, 0x40, 0xed, 0x99, 0x14, 0xf8, 0xf0, 0x3f};
    provision(m_app_key, devkey, 0x12345678, 0x3129ab);

    nrf_mesh_address_t address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x1201, .p_virtual_uuid = NULL};
    memcpy(&m_addr, &address, sizeof(m_addr));

    uint32_t status = NRF_SUCCESS;
    packet_meta_t meta = {0};
    uint8_t meta_addr[6];
    meta.p_addr = meta_addr;

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();

    for (int i = 0; i < 2; ++i)
    {
        setup_test_packet(&m_sar_rx_pkts[i][0]);
        status = transport_pkt_in(packet_net_packet_get(mp_packet), &meta, &m_network, m_iv_index);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    }

    TEST_ASSERT_EQUAL(2, m_timers_added);

    status = transport_sar_process();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    TEST_ASSERT_EQUAL(1, m_event_push_count);
}

void test_trs_sar_rx_giveup(void)
{
    /**
     * This test only sends the first segment in a SAR transaction and fires the
     * session timeout timer. The transport SAR should report back with a
     * SAR_SESSION_CANCELLED event.
     */
    uint8_t appkey[] = {0xf1, 0xa2, 0x4a, 0xbe, 0xa9, 0xb8, 0x6c, 0xd3,
                        0x33, 0x80, 0xa2, 0x4c, 0x4d, 0xfb, 0xe7, 0x43};

    provision(appkey, m_dev_key, 0x01020304, 0x08090a);

    nrf_mesh_address_t address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = UNICAST_ADDR, .p_virtual_uuid = NULL};
    memcpy(&m_addr, &address, sizeof(m_addr));

    packet_meta_t meta = {0};
    uint8_t meta_addr[6];
    meta.p_addr = meta_addr;

    setup_test_packet(m_sar_rx_pkts[0]);
    packet_net_t * p_net_pkt = packet_net_packet_get(mp_packet);
    TEST_ASSERT_NOT_NULL(p_net_pkt);

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_pkt_in(p_net_pkt, &meta, &m_network, m_iv_index));

    /* On RX two timers are added: the session timer and the segment acknowledgement timer. */
    TEST_ASSERT_EQUAL(2, m_timers_added);

    /* First timer added is the session timeout timer. */
    m_timer_event_buffer[0].p_timer->cb(0, m_timer_event_buffer[0].p_timer->p_context);

    TEST_ASSERT_EQUAL(1, m_event_push_count);
}


void test_trs_sar_rx_bigboy(void)
{
    TEST_IGNORE();
}

void test_trs_sar_multiple_session_tx(void)
{
    /**
     * This tests SAR TX without receiving an ACK before the session timeout
     * callback is called.
     */
    uint8_t appkey[] = {0xf1, 0xa2, 0x4a, 0xbe, 0xa9, 0xb8, 0x6c, 0xd3,
                        0x33, 0x80, 0xa2, 0x4c, 0x4d, 0xfb, 0xe7, 0x43};

    uint8_t data[] = {0x80, 0x11, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x08, 0x09, 0x0a};

    provision(appkey, m_dev_key, 0x01020304, 0x08090a);

    nrf_mesh_address_t unicast_addr1 = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x0d0e};

    nrf_mesh_tx_params_t params =
        {
            .p_data = &data[0],
            .data_len = sizeof(data),
            .src = 0x0b0c,
            .security_material.p_app = &m_application,
            .security_material.p_net = &m_network,
            .ttl = 0x12
        };
    memcpy(&params.dst, &unicast_addr1, sizeof(params.dst));
    uint32_t ref;

    for (uint32_t i=0;i<TRANSPORT_SAR_TX_SESSIONS_MAX;i++)
    {
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, transport_tx_sar(&params, &ref));
}

void test_trs_sar_opt_retries_tx(void)
{
    /**
     * This tests SAR TX without receiving an ACK before the session timeout
     * callback is called.
     */
    uint8_t appkey[] = {0xf1, 0xa2, 0x4a, 0xbe, 0xa9, 0xb8, 0x6c, 0xd3,
                        0x33, 0x80, 0xa2, 0x4c, 0x4d, 0xfb, 0xe7, 0x43};

    uint8_t data[] = {0x80, 0x11, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x08, 0x09, 0x0a};

    provision(appkey, m_dev_key, 0x01020304, 0x08090a);

    nrf_mesh_address_t unicast_addr = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x0d0e};
    nrf_mesh_address_t group_addr   = {.type = NRF_MESH_ADDRESS_TYPE_GROUP, .value = GROUP_ADDR};

    nrf_mesh_tx_params_t params =
        {
            .p_data = &data[0],
            .data_len = sizeof(data),
            .src = 0x0b0c,
            .security_material.p_app = &m_application,
            .security_material.p_net = &m_network,
            .ttl = 0x12
        };
    memcpy(&params.dst, &unicast_addr, sizeof(params.dst));
    uint32_t ref;

    /* Default retry value */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    memcpy(&params.dst, &group_addr, sizeof(params.dst));
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    /* Re-init */
    setUp();
    /* Zero retry value */
      nrf_mesh_opt_t opt = {.opt.val = 0};
    memcpy(&params.dst, &unicast_addr, sizeof(params.dst));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SAR_TX_RETRIES, &opt));

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    memcpy(&params.dst, &group_addr, sizeof(params.dst));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SAR_TX_RETRIES, &opt));

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    /* Re-init */
    setUp();
    /* MAX retry value*/
    opt.opt.val = TRANSPORT_SAR_TX_RETRIES_MAX;
    memcpy(&params.dst, &unicast_addr, sizeof(params.dst));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SAR_TX_RETRIES, &opt));

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    memcpy(&params.dst, &group_addr, sizeof(params.dst));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SAR_TX_RETRIES, &opt));

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    /* INVALID retry value*/
    opt.opt.val = TRANSPORT_SAR_TX_RETRIES_MAX + 1;
    memcpy(&params.dst, &unicast_addr, sizeof(params.dst));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, transport_opt_set(NRF_MESH_OPT_TRS_SAR_TX_RETRIES, &opt));
}

void test_trs_sar_tx_giveup(void)
{
    /**
     * This tests SAR TX without receiving an ACK before the session timeout
     * callback is called.
     */
    uint8_t appkey[] = {0xf1, 0xa2, 0x4a, 0xbe, 0xa9, 0xb8, 0x6c, 0xd3,
                        0x33, 0x80, 0xa2, 0x4c, 0x4d, 0xfb, 0xe7, 0x43};

    uint8_t data[] = {0x80, 0x11, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x08, 0x09, 0x0a};

    provision(appkey, m_dev_key, 0x01020304, 0x08090a);

    nrf_mesh_address_t address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x0d0e};

    nrf_mesh_tx_params_t params =
        {
            .p_data = &data[0],
            .data_len = sizeof(data),
            .src = 0x0b0c,
            .security_material.p_app = &m_application,
            .security_material.p_net = &m_network,
            .ttl = 0
        };
    memcpy(&params.dst, &address, sizeof(params.dst));
    uint32_t ref;
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx_sar(&params, &ref));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_sar_process());

    /* On TX two timers are added: The session timer and the TX retry timer. */
    TEST_ASSERT_EQUAL(2, m_timers_added);

    /* The first timer added is the session timeout timer. */
    m_timer_event_buffer[0].p_timer->cb(0, m_timer_event_buffer[0].p_timer->p_context);

    TEST_ASSERT_EQUAL(1, m_event_push_count);
}

void test_trs_maxlen_get(void)
{
    nrf_mesh_opt_t opt = {.len = 0, .opt.val = 0};

    /* Default: 32bit TMIC + 32bit NMIC */
    TEST_ASSERT_EQUAL(11, transport_unseg_maxlen_get());
    TEST_ASSERT_EQUAL(380, transport_seg_maxlen_get());

    opt.opt.val = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SZMIC, &opt));

    /* 64bit MIC + 32bit NMIC */
    TEST_ASSERT_EQUAL(376, transport_seg_maxlen_get());

    opt.opt.val = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SZMIC, &opt));
}

void test_big_mic(void)
{
    TEST_IGNORE_MESSAGE("Test not implemented");
}
