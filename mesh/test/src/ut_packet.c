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

#include <string.h>
#include "unity.h"
#include "packet.h"

#define MESH_NET_PACKET_MIN_LENGTH 5
#define MESH_NET_PACKET_MAX_LENGTH 16

void setUp(void)
{

}

void tearDown(void)
{

}

/***************************************/

void test_packet_size(void)
{
    packet_t packet;
    packet_net_t* p_net_packet = (packet_net_t*) &packet.payload[0];
    p_net_packet->ad_type = AD_TYPE_MESH;

    /* test struct defintions and packing */
    /* TODO: should be compiled with all toolchains, as this could be different between them */
    TEST_ASSERT_EQUAL(3, sizeof(ble_packet_hdr_t));
    TEST_ASSERT_EQUAL(40, sizeof(packet_t));
    TEST_ASSERT_EQUAL(2, sizeof(ble_ad_data_t));
    TEST_ASSERT_EQUAL_PTR(((uint8_t*) &p_net_packet->header) + 5, &p_net_packet->header.src);
    TEST_ASSERT_EQUAL_PTR(((uint8_t*) &p_net_packet->header) + 7, &p_net_packet->header.dst);
    TEST_ASSERT_EQUAL(9, sizeof(packet_net_hdr_t));
    TEST_ASSERT_EQUAL(11, sizeof(packet_net_t));

    /* test helper functions */
    packet_net_payload_size_set(p_net_packet, 8);
    TEST_ASSERT_EQUAL(sizeof(packet_net_t) - sizeof(p_net_packet->length) + 8, p_net_packet->length);
    TEST_ASSERT_EQUAL(8, packet_net_payload_size_get(p_net_packet));

    uint8_t buffer_size = packet_net_buffer_size_get(p_net_packet);
    TEST_ASSERT_EQUAL(packet_net_payload_size_get(p_net_packet) + sizeof(packet_net_t), buffer_size);

    packet_payload_size_set(&packet, sizeof(p_net_packet) + packet_net_payload_size_get(p_net_packet));
    TEST_ASSERT_EQUAL(sizeof(p_net_packet) + packet_net_payload_size_get(p_net_packet) + BLE_GAP_ADDR_LEN, packet.header.length);
    buffer_size = packet_buffer_size_get(&packet);
    TEST_ASSERT_EQUAL(packet_payload_size_get(&packet) + BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_HEADER_LENGTH, buffer_size);
}

void test_packet_type_packet_get(void)
{
    packet_t test_packet;

    /* Basic test for whether the function can find an AD struct based on field type: */
    memset(&test_packet, 0, sizeof(packet_t));
    packet_payload_size_set(&test_packet, 2);
    test_packet.payload[0] = 0x01;
    test_packet.payload[1] = 0x10;
    TEST_ASSERT_EQUAL(&test_packet.payload[0], packet_type_packet_get(&test_packet, 0x10));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x01));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x00));

    /* Test if it can find an AD field that is not at the beginning of the packet: */
    memset(&test_packet, 0, sizeof(packet_t));
    uint8_t test_vector_multifield[] = {
        /* 2 byte AD field, type 0x10: */ 0x02, 0x10, 0x01,
        /* 3 byte AD field, type 0x11: */ 0x03, 0x11, 0x01, 0x02,
        /* 2 bute AD field, type 0x12: */ 0x02, 0x12, 0x01 };
    memcpy(test_packet.payload, test_vector_multifield, sizeof(test_vector_multifield));
    packet_payload_size_set(&test_packet, sizeof(test_vector_multifield));

    TEST_ASSERT_EQUAL(&test_packet.payload[0], packet_type_packet_get(&test_packet, 0x10));
    TEST_ASSERT_EQUAL(&test_packet.payload[3], packet_type_packet_get(&test_packet, 0x11));
    TEST_ASSERT_EQUAL(&test_packet.payload[7], packet_type_packet_get(&test_packet, 0x12));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x01));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x02));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x03));

    /* Test if the function handles 0-length AD fields: */
    memset(&test_packet, 0, sizeof(packet_t));
    uint8_t test_vector_zerolen[] = { 0x00, 0x01, 0x10 };
    memcpy(test_packet.payload, test_vector_zerolen, sizeof(test_vector_zerolen));
    packet_payload_size_set(&test_packet, sizeof(test_vector_zerolen));

    TEST_ASSERT_EQUAL(&test_packet.payload[1], packet_type_packet_get(&test_packet, 0x10));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x01));
    TEST_ASSERT_NULL(packet_type_packet_get(&test_packet, 0x00));
}

void test_packet_net(void)
{
    packet_t packet;
    packet.header.length = BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH;

    packet_ad_type_set(&packet, AD_TYPE_MESH);
    packet.payload[0] = BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH - 1;

    TEST_ASSERT_EQUAL(AD_TYPE_MESH, packet.payload[1]);
    TEST_ASSERT_EQUAL_PTR(&packet.payload[0], packet_net_packet_get(&packet));

    /* find mesh packet after other, valid ad-data: */
    memset(&packet, 0, sizeof(packet));
    packet.header.length = BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH;
    ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &packet.payload[0];
    p_ad_data->type = 0xFF;
    p_ad_data->length = 4;
    p_ad_data = (ble_ad_data_t *) &packet.payload[5];
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = MESH_NET_PACKET_MAX_LENGTH - 6;

    printf("%d\n", packet.header.length);
    TEST_ASSERT_EQUAL_PTR(p_ad_data, packet_net_packet_get(&packet));
    /* don't find it if the BLE header is too short */
    packet.header.length = BLE_GAP_ADDR_LEN + 2;
    printf("%d\n", packet.header.length);
    TEST_ASSERT_EQUAL_PTR(NULL, packet_net_packet_get(&packet));
    packet.header.length = BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH;
    printf("%d\n", packet.header.length);

    /* don't find mesh packet after other invalid ad-data: */
    p_ad_data = (ble_ad_data_t*) &packet.payload[0];
    p_ad_data->length = 100;

    TEST_ASSERT_EQUAL_PTR(NULL, packet_net_packet_get(&packet));

    /* find mesh packet after 0-byte ad data */
    p_ad_data->length = 0;
    p_ad_data = (ble_ad_data_t *) (((uint8_t *) p_ad_data) + 1);
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = MESH_NET_PACKET_MIN_LENGTH;

    TEST_ASSERT_EQUAL_PTR(p_ad_data, packet_net_packet_get(&packet));

    /* find mesh packet with other data after */
    p_ad_data = (ble_ad_data_t*) &packet.payload[0];
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = MESH_NET_PACKET_MIN_LENGTH;
    p_ad_data = (ble_ad_data_t*) (((uint8_t*) p_ad_data) + 1 + MESH_NET_PACKET_MIN_LENGTH);
    p_ad_data->type = 0xFF;

    TEST_ASSERT_EQUAL_PTR(&packet.payload[0], packet_net_packet_get(&packet));
}

void test_packet_mic(void)
{
    packet_t packet;
    packet.header.length = BLE_ADV_PACKET_MAX_LENGTH;
    memset(packet.payload, 0, BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);
    packet_net_t* p_net = (packet_net_t*) &packet.payload[0];
    p_net->ad_type = AD_TYPE_MESH;
    packet_net_payload_size_set(p_net, 11);

    packet_net_mic_set(p_net, 0xAABBCCDD);
    TEST_ASSERT_EQUAL_HEX8(0xDD, packet.payload[18]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, packet.payload[19]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, packet.payload[20]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, packet.payload[21]);
    TEST_ASSERT_EQUAL_HEX32(0xAABBCCDD, packet_net_mic_get(p_net));

    memset(p_net->payload, 0, 10);
    packet_net_payload_size_set(p_net, 5);

    packet_net_mic_set(p_net, 0xAABBCCDD);
    TEST_ASSERT_EQUAL_HEX8(0xDD, packet.payload[12]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, packet.payload[13]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, packet.payload[14]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, packet.payload[15]);
    TEST_ASSERT_EQUAL_HEX32(0xAABBCCDD, packet_net_mic_get(p_net));
}

