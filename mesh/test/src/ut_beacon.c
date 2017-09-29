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

#include "beacon.h"
#include "packet_mgr_mock.h"
#include "bearer_adv_mock.h"
#include "net_beacon_mock.h"
#include "prov_beacon_mock.h"
#include "nrf_mesh_assert.h"

void setUp(void)
{
    packet_mgr_mock_Init();
    bearer_adv_mock_Init();
    net_beacon_mock_Init();
    prov_beacon_mock_Init();
}

void tearDown(void)
{
    packet_mgr_mock_Verify();
    packet_mgr_mock_Destroy();
    bearer_adv_mock_Verify();
    bearer_adv_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    prov_beacon_mock_Verify();
    prov_beacon_mock_Destroy();
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_beacon_create()
{
    uint8_t dummy_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    packet_t packet;
    packet_generic_t* p_packet = &packet;
    packet_mgr_alloc_ExpectAndReturn(NULL,
            sizeof(dummy_data) + 3 /* header */ + 6 /* adv addr */ + 2 /* AD-header */ + 1 /* beacon type */, NRF_SUCCESS);
    packet_mgr_alloc_IgnoreArg_pp_buffer();
    packet_mgr_alloc_ReturnThruPtr_pp_buffer(&p_packet);
    p_packet = beacon_create(0x43, dummy_data, sizeof(dummy_data));
    TEST_ASSERT_EQUAL_PTR(&packet, p_packet);
    TEST_ASSERT_EQUAL_HEX8(6 /* adv addr */ + sizeof(dummy_data) + 1 /* AD len */ + 1 /* AD type */ + 1 /* beacon type */, packet.header.length);
    TEST_ASSERT_EQUAL_HEX8(sizeof(dummy_data) + 1 /* ad type */ + 1 /* beacon type */, packet.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(AD_TYPE_BEACON, packet.payload[1]);
    TEST_ASSERT_EQUAL_HEX8(0x43, packet.payload[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(dummy_data, &packet.payload[3], sizeof(dummy_data));

    /* Test illegal params */
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(0x43, NULL, sizeof(dummy_data)));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(0x43, dummy_data, 0));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(BEACON_TYPE_INVALID, dummy_data, sizeof(dummy_data)));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(0x43, dummy_data, 100));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(0x43, dummy_data, BEACON_DATA_MAXLEN + 1));
}

void test_beacon_pkt_in()
{
    uint8_t beacon_data[] = {0x06, AD_TYPE_BEACON, BEACON_TYPE_UNPROV, 0x01, 0x02, 0x03, 0x04};
    packet_meta_t meta = {}; // don't really care about contents
    prov_beacon_unprov_pkt_in_Expect(&beacon_data[3], 4, &meta);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, beacon_pkt_in((ble_ad_data_t*) beacon_data, &meta));

    beacon_data[2] = BEACON_TYPE_SEC_NET_BCAST;
    net_beacon_pkt_in_Expect(&beacon_data[3], 4, &meta);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, beacon_pkt_in((ble_ad_data_t*) beacon_data, &meta));

    beacon_data[2] = 0x43;
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_DATA, beacon_pkt_in((ble_ad_data_t*) beacon_data, &meta));
    beacon_data[2] = BEACON_TYPE_SEC_NET_BCAST;
    beacon_data[0] = 0x01;
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_LENGTH, beacon_pkt_in((ble_ad_data_t*) beacon_data, &meta));
    beacon_data[0] = 0x06;
    beacon_data[1] = 0x16; /* invalid AD type */
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_DATA, beacon_pkt_in((ble_ad_data_t*) beacon_data, &meta));

    beacon_data[1] = AD_TYPE_BEACON;
    net_beacon_pkt_in_Expect(&beacon_data[3], 4, NULL);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, beacon_pkt_in((ble_ad_data_t*) beacon_data, NULL));

    TEST_NRF_MESH_ASSERT_EXPECT(beacon_pkt_in((ble_ad_data_t*) NULL, NULL));
}

