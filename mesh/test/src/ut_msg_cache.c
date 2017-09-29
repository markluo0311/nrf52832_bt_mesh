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
#include <stdbool.h>
#include "unity.h"
#include "msg_cache.h"
#include "transport.h"

#define MESH_NET_PACKET_PAYLOAD_MIN_LENGTH (sizeof(packet_net_t) + 4)

static uint8_t packet_buf[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];

void setUp(void)
{
    msg_cache_init();
}

void tearDown(void)
{

}


/********************************************/

void test_msg_cache_entry_add_new(void)
{
    memset(packet_buf, 0, sizeof(packet_buf));

    packet_net_t* p_packet = (packet_net_t*) &packet_buf[0];
    packet_net_payload_size_set(p_packet, MESH_NET_PACKET_PAYLOAD_MIN_LENGTH);
    p_packet->ad_type = AD_TYPE_MESH;

    /* sneaky "empty entry" entry */
    p_packet->header.src = NRF_MESH_ADDR_UNASSIGNED;
    p_packet->header.seq = 0;
    packet_net_payload_size_set(p_packet, MESH_NET_PACKET_PAYLOAD_MIN_LENGTH);
    packet_net_mic_set(p_packet, 0);
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(0, 0);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));

    /* invalid length entry */
    p_packet->header.src = 0x1234;
    p_packet->header.seq = 0xAAAA00;
    p_packet->length = 1;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, p_packet->header.seq);
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    packet_net_payload_size_set(p_packet, MESH_NET_PACKET_PAYLOAD_MIN_LENGTH);

    /* valid entry */
    p_packet->header.src = 0x1234;
    p_packet->header.seq = 0xAAAA00;
    packet_net_mic_set(p_packet, 0xAAAAAA00); /* packet mic changes if we change the header */
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));

    /* same address */
    p_packet->header.src = 0x1234;
    p_packet->header.seq = 0xAAAA01;
    packet_net_mic_set(p_packet, 0xAAAAAA01);/* packet mic changes if we change the header */
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));

    /* same seq */
    p_packet->header.src = 0xAAAA;
    p_packet->header.seq = 0xAAAA01;
    packet_net_mic_set(p_packet, 0xAAAAAA02);/* packet mic changes if we change the header */
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));

    /* lower seq */
    p_packet->header.src = 0xAAAA;
    p_packet->header.seq = 0x000001;
    packet_net_mic_set(p_packet, 0xAAAAAA03);/* packet mic changes if we change the header */
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));

    /* check if old entries are still there */
    if (MSG_CACHE_ENTRY_COUNT > 3)
    {
        p_packet->header.src = 0x1234;
        p_packet->header.seq = 0xAAAA00;
        packet_net_mic_set(p_packet, 0xAAAAAA00); /* packet mic changes if we change the header */
        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    }

    if (MSG_CACHE_ENTRY_COUNT > 2)
    {
        p_packet->header.src = 0x1234;
        p_packet->header.seq = 0xAAAA01;
        packet_net_mic_set(p_packet, 0xAAAAAA01);/* packet mic changes if we change the header */
        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    }

    if (MSG_CACHE_ENTRY_COUNT > 1)
    {
        p_packet->header.src = 0xAAAA;
        p_packet->header.seq = 0xAAAA01;
        packet_net_mic_set(p_packet, 0xAAAAAA02);/* packet mic changes if we change the header */
        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
    }
}

void test_msg_cache_entry_overflow(void)
{
    memset(packet_buf, 0, sizeof(packet_buf));
    packet_net_t* p_packet = (packet_net_t*) &packet_buf[0];
    p_packet->ad_type = AD_TYPE_MESH;
    uint32_t mic = 0xAA000000;
    p_packet->header.src = 0x1000;
    p_packet->header.seq = 0xAAAA00;
    packet_net_payload_size_set(p_packet, MESH_NET_PACKET_PAYLOAD_MIN_LENGTH);

    /* fill all */
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        p_packet->header.src++;
        p_packet->header.seq = LE2BE24(BE2LE24(p_packet->header.seq) + 1);
        packet_net_mic_set(p_packet, mic++);
        TEST_ASSERT_FALSE(msg_cache_entry_exists(p_packet));
        msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
        TEST_ASSERT_TRUE(msg_cache_entry_exists(p_packet));
    }

    /* check that all entries are still there */
    p_packet->header.src = 0x1000;
    p_packet->header.seq = 0xAAAA00;
    mic = 0xAA000000;
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        p_packet->header.src++;
        p_packet->header.seq = LE2BE24(BE2LE24(p_packet->header.seq) + 1);
        packet_net_mic_set(p_packet, mic++);
        TEST_ASSERT_TRUE(msg_cache_entry_exists(p_packet));
    }

    /* overflow */
    p_packet->header.src++;
    p_packet->header.seq = LE2BE24(BE2LE24(p_packet->header.seq) + 1);
    packet_net_mic_set(p_packet, mic++);
    TEST_ASSERT_FALSE(msg_cache_entry_exists(p_packet));
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_TRUE(msg_cache_entry_exists(p_packet));

    /* first entry should no longer be valid */
    p_packet->header.src = 0x1001;
    p_packet->header.seq = 0xAAAA01;
    packet_net_mic_set(p_packet, 0xAA000000);
    TEST_ASSERT_FALSE(msg_cache_entry_exists(p_packet));
}

void test_msg_cache_entry_add_invalid_length(void)
{
    memset(packet_buf, 0, sizeof(packet_buf));
    packet_net_t* p_packet = (packet_net_t*) &packet_buf[0];
    packet_net_payload_size_set(p_packet, 1);
    p_packet->ad_type = AD_TYPE_MESH;
    p_packet->header.src = 0x1000;
    p_packet->header.seq = 0xAAAA00;

    msg_cache_entry_add(p_packet->header.src, p_packet->header.seq);
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
}

void test_clear(void)
{
    memset(packet_buf, 0, sizeof(packet_buf));
    packet_net_t* p_packet = (packet_net_t*) &packet_buf[0];
    packet_net_payload_size_set(p_packet, MESH_NET_PACKET_PAYLOAD_MIN_LENGTH);
    p_packet->ad_type = AD_TYPE_MESH;

    p_packet->header.src = 0x1234;
    p_packet->header.seq = 0xAAAA00;
    msg_cache_entry_add(p_packet->header.src, BE2LE24(p_packet->header.seq));
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(p_packet));
    msg_cache_clear();
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(p_packet));
}
