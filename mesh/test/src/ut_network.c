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

#include <stdlib.h>

#include <unity.h>
#include <cmock.h>

#include "nrf_mesh.h"

#include "beacon.h"
#include "log.h"
#include "network.h"
#include "packet_mgr.h"
#include "timer.h"
#include "toolchain.h"
#include "utils.h"

#include "bearer_mock.h"
#include "event_mock.h"
#include "msg_cache_mock.h"
#include "transport_mock.h"
#include "net_beacon_mock.h"
#include "net_state_mock.h"

/*************** Static Test Parameters ***************/

#define TEST_APPLICATION_KEY { 0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48 }
#define TEST_NETWORK_KEY     { 0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6 }
#define TEST_DEVICE_KEY      { 0x9d, 0x6d, 0xd0, 0xe9, 0x6e, 0xb2, 0x5d, 0xc1, 0x9a, 0x40, 0xed, 0x99, 0x14, 0xf8, 0xf0, 0x3f }

/* Parameters for the encryption test: */
#define TEST_PRIVACY_KEY     { 0x8b, 0x84, 0xee, 0xde, 0xc1, 0x00, 0x06, 0x7d, 0x67, 0x09, 0x71, 0xdd, 0x2a, 0xa7, 0x00, 0xcf }
#define TEST_ENCRYPTION_KEY  { 0x09, 0x53, 0xfa, 0x93, 0xe7, 0xca, 0xac, 0x96, 0x38, 0xf5, 0x88, 0x20, 0x22, 0x0a, 0x39, 0x8e }
#define TEST_NETWORK_ID      { 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70 }
#define TEST_NID               0x68
#define TEST_IV_INDEX          0x12345678

/* Beacon test packets */
#define TEST_BEACON_NORMAL              { 0x00, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x78, 0x8e, 0xa2, 0x61, 0x58, 0x2f, 0x36, 0x4f, 0x6f }
#define TEST_BEACON_IV_UPDATE           { 0x02, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x79, 0xc2, 0xaf, 0x80, 0xad, 0x07, 0x2a, 0x13, 0x5c }
#define TEST_BEACON_IV_UPDATE_COMPLETE  { 0x00, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x79, 0xc6, 0x2f, 0x09, 0xe4, 0xc9, 0x57, 0xf5, 0x9d }

/*************** Test Vectors ***************/

/* Macro for specifying the complete, encrypted network packet. In the spec, this data is labelled "NetworkPDU". */
#define TEST_PACKET_ENCRYPTED(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_encrypted_packet = buffer; \
                p_vector->lengths.encrypted = sizeof(buffer); \
            }
/* Macro for specifying the transport packet. In the spec, this data is labelled "TransportPDU". */
#define TEST_PACKET_TRANSPORT(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_transport_packet = buffer; \
                p_vector->lengths.transport = sizeof(buffer); \
            }
/*
 * Macro for specifying the unencrypted header data. This is a concatenation of the fields "IVI NID", "CTL TTL",
 * SEQ, SRC and DST in the specification sample data.
 */
#define TEST_PACKET_HEADER(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_header_data = buffer; \
                p_vector->lengths.header = sizeof(buffer); \
            }
/* Macro for specifying metadata about a test packet. */
#define TEST_PACKET_PARAMS(p_vector, iv, sequence, src, dst, ttl_, ctl_, miclen_) \
            { \
                p_vector->params.iv_index = iv; \
                p_vector->params.seqnum = sequence; \
                p_vector->params.source = src; \
                p_vector->params.dest   = dst; \
                p_vector->params.ttl    = ttl_; \
                p_vector->params.ctl    = ctl_; \
                p_vector->params.miclen = miclen_; \
            }

typedef struct
{
    uint32_t iv_index;
    uint32_t seqnum : 24;
    uint16_t source;
    uint16_t dest;
    uint8_t  ttl    : 7;
    uint8_t  ctl    : 1;
    uint8_t  miclen;
} test_packet_params_t;

typedef struct
{
    const uint8_t * p_encrypted_packet; /* Complete, encrypted packet. */
    const uint8_t * p_transport_packet; /* Transport packet (encrypted) passed from network to transport. */
    const uint8_t * p_header_data;      /* Unobfuscated packet header. */

    test_packet_params_t params;        /* Metadata describing the packet. */

    struct
    {
        uint8_t encrypted;
        uint8_t transport;
        uint8_t header;
    } lengths;
} test_vector_t;

/* Specifies all available test vectors; corresponds to sample data in the spec, see below: */
#define TEST_VECTORS_ALL                   { 1, 2, 3, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22, 23, 24}

/* Specifies which sets of test vectors to use for specific tests: */
#define NETWORK_PKT_IN_TEST_VECTORS        TEST_VECTORS_ALL
#define SELF_RECEIVE_TEST_VECTORS          TEST_VECTORS_ALL
#define NETWORK_PKT_OUT_TEST_VECTORS       { 1, 2, 3, 6, 7, 8, 9, 16, 17, 18, 19 /* Skip cases with other IV indices */ }
#define INVALID_NETKEY_TEST_VECTORS        TEST_VECTORS_ALL
#define NETWORK_PKT_RELAY_TEST_VECTORS     TEST_VECTORS_ALL

/*
 * Retrieves a test vector. These are indexed by the number used in the specification. Some test vectors are skipped,
 * as they use friendship credentials or other unsupported features.
 */
static void get_test_vector(unsigned int vector, test_vector_t * p_vector)
{
    switch (vector)
    {
        case 1:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xec, 0xa4, 0x87, 0x51, 0x67, 0x65, 0xb5, 0xe5, 0xbf, 0xda, 0xcb,
                    0xaf, 0x6c, 0xb7, 0xfb, 0x6b, 0xff, 0x87, 0x1f, 0x03, 0x54, 0x44, 0xce, 0x83, 0xa6, 0x70, 0xdf);
            TEST_PACKET_TRANSPORT(p_vector, 0x03, 0x4b, 0x50, 0x05, 0x7e, 0x40, 0x00, 0x00, 0x01, 0x00, 0x00);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x00, 0x00, 0x01, 0x12, 0x01, 0xff, 0xfd);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000001, 0x1201, 0xfffd, 0, 1, 8);
            break;
        }
        case 2:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xd4, 0xc8, 0x26, 0x29, 0x6d, 0x79, 0x79, 0xd7, 0xdb, 0xc0, 0xc9, 0xb4,
                0xd4, 0x3e, 0xeb, 0xec, 0x12, 0x9d, 0x20, 0xa6, 0x20, 0xd0, 0x1e);
            TEST_PACKET_TRANSPORT(p_vector, 0x04, 0x32, 0x03, 0x08, 0xba, 0x07, 0x2f);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x01, 0x48, 0x20, 0x23, 0x45, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014820, 0x2345, 0x1201, 0, 1, 8);
            break;
        }
        case 3:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xda, 0x06, 0x2b, 0xc9, 0x6d, 0xf2, 0x53, 0x27, 0x30, 0x86, 0xb8, 0xc5,
                0xee, 0x00, 0xbd, 0xd9, 0xcf, 0xcc, 0x62, 0xa2, 0xdd, 0xf5, 0x72);
            TEST_PACKET_TRANSPORT(p_vector, 0x04, 0xfa, 0x02, 0x05, 0xa6, 0x00, 0x0a);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x2b, 0x38, 0x32, 0x2f, 0xe3, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x2b3832, 0x2fe3, 0x1201, 0, 1, 8);
            break;
        }
        case 6:
        {
            /* Test message #6 contains two messages, only the first one is included here. */
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xca, 0xb5, 0xc5, 0x34, 0x8a, 0x23, 0x0a, 0xfb, 0xa8, 0xc6, 0x3d, 0x4e,
                    0x68, 0x63, 0x64, 0x97, 0x9d, 0xea, 0xf4, 0xfd, 0x40, 0x96, 0x11, 0x45, 0x93, 0x9c, 0xda, 0x0e);
            TEST_PACKET_TRANSPORT(p_vector, 0x80, 0x26, 0xac, 0x01, 0xee, 0x9d, 0xdd, 0xfd, 0x21, 0x69, 0x32, 0x6d, 0x23, 0xf3, 0xaf, 0xdf);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x04, 0x31, 0x29, 0xab, 0x00, 0x03, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x3129ab, 0x0003, 0x1201, 4, 0, 4);
            break;
        }
        case 7:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xe4, 0x76, 0xb5, 0x57, 0x9c, 0x98, 0x0d, 0x0d, 0x73, 0x0f, 0x94, 0xd7,
                    0xf3, 0x50, 0x9d, 0xf9, 0x87, 0xbb, 0x41, 0x7e, 0xb7, 0xc0, 0x5f);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0xa6, 0xac, 0x00, 0x00, 0x00, 0x02);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x8b, 0x01, 0x48, 0x35, 0x23, 0x45, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014835, 0x2345, 0x0003, 0x0b, 1, 8);
            break;
        }
        case 8:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x4d, 0xaa, 0x62, 0x67, 0xc2, 0xcf, 0x0e, 0x2f, 0x91, 0xad, 0xd6, 0xf0,
                    0x6e, 0x66, 0x00, 0x68, 0x44, 0xce, 0xc9, 0x7f, 0x97, 0x31, 0x05, 0xae, 0x25, 0x34, 0xf9, 0x58);
            TEST_PACKET_TRANSPORT(p_vector, 0x80, 0x26, 0xac, 0x01, 0xee, 0x9d, 0xdd, 0xfd, 0x21, 0x69, 0x32, 0x6d, 0x23, 0xf3, 0xaf, 0xdf);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x04, 0x31, 0x29, 0xad, 0x00, 0x03, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x3129ad, 0x0003, 0x1201, 0x4, 0, 4);
            break;
        }
        case 9:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xae, 0xc4, 0x67, 0xed, 0x49, 0x01, 0xd8, 0x5d, 0x80, 0x6b, 0xbe, 0xd2,
                    0x48, 0x61, 0x4f, 0x93, 0x80, 0x67, 0xb0, 0xd9, 0x83, 0xbb, 0x7b);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0xa6, 0xac, 0x00, 0x00, 0x00, 0x03);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x8b, 0x01, 0x48, 0x36, 0x23, 0x45, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014836, 0x2345, 0x0003, 0x0b, 1, 8);
            break;
        }
        case 16:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xe8, 0x0e, 0x5d, 0xa5, 0xaf, 0x0e, 0x6b, 0x9b, 0xe7, 0xf5, 0xa6, 0x42,
                    0xf2, 0xf9, 0x86, 0x80, 0xe6, 0x1c, 0x3a, 0x8b, 0x47, 0xf2, 0x28);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0x89, 0x51, 0x1b, 0xf1, 0xd1, 0xa8, 0x1c, 0x11, 0xdc, 0xef);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x0b, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000006, 0x1201, 0x0003, 0x0b, 0, 4);
            break;
        }
        case 17:
        {
            /* Values from errata 9693 */
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xb2, 0xbd, 0x2c, 0x1e, 0x1b, 0x6f, 0x2a, 0x80, 0xd3, 0x81, 0xb9, 0x1f,
                    0x82, 0x4d, 0xd4, 0xf0, 0xa3, 0xcd, 0x54, 0xce, 0xa2, 0x3b, 0x7a);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0x89, 0x51, 0x1b, 0xf1, 0xd1, 0xa8, 0x1c, 0x11, 0xdc, 0xef);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x0a, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000006, 0x1201, 0x0003, 0x0a, 0, 4);
            break;
        }
        case 18:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x48, 0xcb, 0xa4, 0x37, 0x86, 0x0e, 0x56, 0x73, 0x72, 0x8a, 0x62, 0x7f,
                    0xb9, 0x38, 0x53, 0x55, 0x08, 0xe2, 0x1a, 0x6b, 0xaf, 0x57);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x5a, 0x8b, 0xde, 0x6d, 0x91, 0x06, 0xea, 0x07, 0x8a);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x03, 0x00, 0x00, 0x07, 0x12, 0x01, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000007, 0x1201, 0xffff, 0x03, 0, 4);
            break;
        }
        case 19:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x11, 0x0e, 0xde, 0xec, 0xd8, 0x3c, 0x30, 0x10, 0xa0, 0x5e, 0x1b, 0x23,
                    0xa9, 0x26, 0x02, 0x3d, 0xa7, 0x5d, 0x25, 0xba, 0x91, 0x79, 0x37, 0x36);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0xca, 0x6c, 0xd8, 0x8e, 0x69, 0x8d, 0x12, 0x65, 0xf4, 0x3f, 0xc5);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x03, 0x00, 0x00, 0x09, 0x12, 0x01, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000009, 0x1201, 0xffff, 0x03, 0, 4);
            break;
        }
        case 20:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x5c, 0xca, 0x51, 0xe2, 0xe8, 0x99, 0x8c, 0x3d, 0xc8, 0x73, 0x44, 0xa1,
                    0x6c, 0x78, 0x7f, 0x6b, 0x08, 0xcc, 0x89, 0x7c, 0x94, 0x1a, 0x53, 0x68);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x9c, 0x98, 0x03, 0xe1, 0x10, 0xfe, 0xa9, 0x29, 0xe9, 0x54, 0x2d);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x09, 0x12, 0x34, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x070809, 0x1234, 0xffff, 0x03, 0, 4);
            break;
        }
        case 21:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0xb1, 0x05, 0x1f, 0x5e, 0x94, 0x5a, 0xe4, 0xd6, 0x11, 0x35, 0x8e, 0xaf,
                    0x17, 0x79, 0x6a, 0x6c, 0x98, 0x97, 0x7f, 0x69, 0xe5, 0x87, 0x2c, 0x46, 0x20);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x2f, 0xa7, 0x30, 0xfd, 0x98, 0xf6, 0xe4, 0xbd, 0x12, 0x0e, 0xa9, 0xd6);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0a, 0x12, 0x34, 0x81, 0x05);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080a, 0x1234, 0x8105, 0x03, 0, 4);
            break;
        }
        case 22:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0xd8, 0x5c, 0xae, 0xce, 0xf1, 0xe3, 0xed, 0x31, 0xf3, 0xfd, 0xcf, 0x88,
                    0xa4, 0x11, 0x13, 0x5f, 0xea, 0x55, 0xdf, 0x73, 0x0b, 0x6b, 0x28, 0xe2, 0x55);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x38, 0x71, 0xb9, 0x04, 0xd4, 0x31, 0x52, 0x63, 0x16, 0xca, 0x48, 0xa0);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0b, 0x12, 0x34, 0xb5, 0x29);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080b, 0x1234, 0xb529, 0x03, 0, 4);
            break;
        }
        case 23:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x77, 0xa4, 0x8d, 0xd5, 0xfe, 0x2d, 0x7a, 0x9d, 0x69, 0x6d, 0x3d, 0xd1,
                    0x6a, 0x75, 0x48, 0x96, 0x96, 0xf0, 0xb7, 0x0c, 0x71, 0x1b, 0x88, 0x13, 0x85);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x24, 0x56, 0xdb, 0x5e, 0x31, 0x00, 0xee, 0xf6, 0x5d, 0xaa, 0x7a, 0x38);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0c, 0x12, 0x34, 0x97, 0x36);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080c, 0x1234, 0x9736, 0x03, 0, 4);
            break;
        }
        case 24:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x34, 0x58, 0x6b, 0xab, 0xde, 0xf3, 0x94, 0xe9, 0x98, 0xb4, 0x08, 0x1f,
                    0x5a, 0x73, 0x08, 0xce, 0x3e, 0xdb, 0xb3, 0xb0, 0x6c, 0xde, 0xcd, 0x02, 0x8e, 0x30, 0x7f, 0x1c);
            TEST_PACKET_TRANSPORT(p_vector, 0xe6, 0xa0, 0x34, 0x01, 0xde, 0x15, 0x47, 0x11, 0x84, 0x63, 0x12, 0x3e, 0x5f, 0x6a, 0x17, 0xb9);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0d, 0x12, 0x34, 0x97, 0x36);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080d, 0x1234, 0x9736, 0x03, 0, 4);
            break;
        }
        default:
            TEST_FAIL_MESSAGE("Unknown test vector requested!");
    }
}

/*************** Mesh Assertion Handling  ***************/

nrf_mesh_assertion_handler_t m_assertion_handler;
void nrf_mesh_assertion_handler(uint32_t pc)
{
    TEST_FAIL_MESSAGE("Mesh assertion triggered!");
}

/*************** Static Variables and Stuff ***************/

static nrf_mesh_network_secmat_t test_network;
static uint32_t m_net_secmat_get_calls_expect;
static nrf_mesh_network_secmat_t * mp_net_secmats;
static uint32_t m_net_secmat_count;
static uint16_t m_rx_address;
static uint32_t m_relay_expected;

static uint16_t m_relay_src;
static uint16_t m_relay_dst;
static uint8_t m_relay_ttl;
static uint32_t m_relay_return;

static uint32_t m_bearer_tx_return;

/*************** Additional Mock Functions ***************/

timestamp_t timer_now(void)
{
    return 0;
}

static packet_net_t * p_transport_pkt_in_previous;

uint32_t transport_pkt_in_mock_cb(packet_net_t * p_packet, const packet_meta_t * p_packet_meta, const nrf_mesh_network_secmat_t * const p_network, uint32_t iv_index, int cmock_num_calls)
{
    if (p_transport_pkt_in_previous != NULL)
    {
        free(p_transport_pkt_in_previous);
    }

    p_transport_pkt_in_previous = malloc(p_packet->length + 1 /* Add one to the length to account for the length field itself */);
    TEST_ASSERT_NOT_NULL(p_transport_pkt_in_previous);
    memcpy(p_transport_pkt_in_previous, p_packet, p_packet->length + 1);

    return NRF_SUCCESS;
}

static void transport_pkt_in_mock_reset(void)
{
    if (p_transport_pkt_in_previous != NULL)
    {
        free(p_transport_pkt_in_previous);
        p_transport_pkt_in_previous = NULL;
    }
}

static packet_t * p_bearer_tx_previous;

uint32_t bearer_tx_mock_cb(packet_t * p_packet, bearer_t bearers, uint8_t retransmit_count, int cmock_num_calls)
{
    if (p_bearer_tx_previous != NULL)
    {
        free(p_bearer_tx_previous);
    }

    p_bearer_tx_previous = malloc(sizeof(packet_t));
    TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);
    memcpy(p_bearer_tx_previous, p_packet, packet_buffer_size_get(p_packet));

    return m_bearer_tx_return;
}

static void bearer_tx_mock_reset(void)
{
    if (p_bearer_tx_previous != NULL)
    {
        free(p_bearer_tx_previous);
        p_bearer_tx_previous = NULL;
    }
}

/*************** Static Helper Functions and Types ***************/

/*
 * Provisions the test network. Will use test vector keys if indicated, otherwise, it'll be dummies.
 */
static void provision(bool use_real_keys)
{
    if (use_real_keys)
    {
        uint8_t privacy[] = TEST_PRIVACY_KEY;
        uint8_t encryption[] = TEST_ENCRYPTION_KEY;
        test_network.nid = TEST_NID;
        memcpy(test_network.privacy_key, privacy, NRF_MESH_KEY_SIZE);
        memcpy(test_network.encryption_key, encryption, NRF_MESH_KEY_SIZE);
    }
    else
    {
        memset(test_network.privacy_key, 0x11, NRF_MESH_KEY_SIZE);
        memset(test_network.encryption_key, 0x22, NRF_MESH_KEY_SIZE);
        test_network.nid = 0x12;
    }
}

/* Constructs a complete BLE packet from the input payload. */
static packet_net_t * construct_in_packet(packet_t ** pp_packet, const uint8_t * p_payload, uint8_t payload_size)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_mgr_alloc((packet_generic_t **) pp_packet, sizeof(packet_t)));
    TEST_ASSERT_NOT_NULL(*pp_packet);

    packet_payload_size_set(*pp_packet, sizeof(packet_net_t) + payload_size);
    packet_ad_type_set(*pp_packet, AD_TYPE_MESH);
    packet_ad_length_set(*pp_packet, payload_size + BLE_AD_DATA_OVERHEAD);

    packet_net_t * p_net_packet = packet_net_packet_get(*pp_packet);
    TEST_ASSERT_NOT_NULL(*pp_packet);
    packet_net_payload_size_set(p_net_packet, payload_size - sizeof(p_net_packet->header));

    memcpy(&p_net_packet->header, p_payload, payload_size);

    return p_net_packet;
}

/* Constructs a packet containing the fields normally filled out by the transport layer. */
static packet_net_t * construct_out_packet(packet_t ** pp_packet, const uint8_t * p_payload, uint8_t payload_size, const test_packet_params_t * params)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_mgr_alloc((packet_generic_t **) pp_packet, sizeof(packet_t)));
    TEST_ASSERT_NOT_NULL(*pp_packet);

    packet_payload_size_set(*pp_packet, sizeof(packet_net_t) + payload_size + params->miclen);
    packet_ad_type_set(*pp_packet, AD_TYPE_MESH);
    packet_ad_length_set(*pp_packet, payload_size + BLE_AD_DATA_OVERHEAD);

    packet_net_t * p_net_packet = packet_net_packet_get(*pp_packet);
    packet_net_payload_size_set(p_net_packet, payload_size + params->miclen);
    memcpy(p_net_packet->payload, p_payload, payload_size);

    p_net_packet->header.seq = LE2BE24(params->seqnum);
    p_net_packet->header.src = LE2BE16(params->source);
    p_net_packet->header.dst = LE2BE16(params->dest);
    p_net_packet->header.ttl = params->ttl;
    p_net_packet->header.ctl = params->ctl;
    p_net_packet->header.ivi = params->iv_index & 0x01;
    p_net_packet->header.nid = test_network.nid;

    return p_net_packet;
}

void nrf_mesh_net_secmat_next_get(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat)
{
    TEST_ASSERT_TRUE(m_net_secmat_get_calls_expect > 0);
    TEST_ASSERT_NOT_NULL(mp_net_secmats);
    TEST_ASSERT_NOT_EQUAL(0, m_net_secmat_count);
    m_net_secmat_get_calls_expect--;

    if (*pp_secmat == NULL)
    {
        *pp_secmat = &mp_net_secmats[0];
    }
    else if ((uint32_t)(*pp_secmat - &mp_net_secmats[0]) >= m_net_secmat_count - 1)
    {
        *pp_secmat = NULL;
    }
    else
    {
        (*pp_secmat)++; /* get next */
    }
}

bool nrf_mesh_rx_address_get(uint16_t address, nrf_mesh_address_t * p_address)
{
    return (m_rx_address == address);
}

static bool relay_cb(uint16_t src, uint16_t dst, uint8_t ttl)
{
    TEST_ASSERT_TRUE(m_relay_expected > 0);
    TEST_ASSERT_EQUAL(src, m_relay_src);
    TEST_ASSERT_EQUAL(dst, m_relay_dst);
    TEST_ASSERT_EQUAL(ttl, m_relay_ttl);
    m_relay_expected--;
    return m_relay_return;
}
/*************** Test Initialization and Finalization ***************/

void setUp(void)
{
    bearer_mock_Init();
    msg_cache_mock_Init();
    transport_mock_Init();
    event_mock_Init();
    net_state_mock_Init();
    net_beacon_mock_Init();

    m_net_secmat_get_calls_expect = 0;
    mp_net_secmats = NULL;
    m_net_secmat_count = 0;
    m_rx_address = 0;
    m_bearer_tx_return = NRF_SUCCESS;

    __LOG_INIT((LOG_SRC_NETWORK | LOG_SRC_ENC | LOG_SRC_TEST), 3, LOG_CALLBACK_DEFAULT);

    m_assertion_handler = nrf_mesh_assertion_handler;
    nrf_mesh_init_params_t init_params =
    {
        .assertion_handler = nrf_mesh_assertion_handler,
    };

    net_state_init_Expect();
    net_beacon_init_Expect();
    net_state_recover_from_flash_Expect();
    network_init(&init_params);
    packet_mgr_init(&init_params);
}

void tearDown(void)
{
    transport_pkt_in_mock_reset();
    bearer_tx_mock_reset();

    bearer_mock_Verify();
    bearer_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
}

/*************** Test Cases ***************/

void test_packet_in(void)
{
    const uint8_t run_testvectors[] = NETWORK_PKT_IN_TEST_VECTORS;
    provision(true);

    transport_pkt_in_StubWithCallback(transport_pkt_in_mock_cb);

    mp_net_secmats = &test_network;
    m_net_secmat_count = 1;

    /* Processes all the test vectors as incoming packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        packet_net_t * p_net_packet = construct_in_packet(&p_packet, test_vector.p_encrypted_packet, test_vector.lengths.encrypted);

        transport_pkt_in_mock_reset();

        msg_cache_entry_exists_IgnoreAndReturn(false);
        msg_cache_entry_add_Expect(LE2BE16(test_vector.params.source), test_vector.params.seqnum);

        m_net_secmat_get_calls_expect = 1;

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_in(p_net_packet, NULL));
        TEST_ASSERT_NOT_NULL(p_transport_pkt_in_previous);

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_header_data,     &p_transport_pkt_in_previous->header, test_vector.lengths.header);
        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_transport_packet, p_transport_pkt_in_previous->payload, test_vector.lengths.transport);

        packet_mgr_free(p_packet);
    }

    /* Add more networks, make sure we hit the right one for all test vectors. */
    nrf_mesh_network_secmat_t secmats[4];
    memset(&secmats[0], 0x12, sizeof(nrf_mesh_network_secmat_t));
    memset(&secmats[1], 0x34, sizeof(nrf_mesh_network_secmat_t));
    memcpy(&secmats[2], &test_network, sizeof(nrf_mesh_network_secmat_t));
    memset(&secmats[3], 0x56, sizeof(nrf_mesh_network_secmat_t));
    mp_net_secmats = secmats;
    m_net_secmat_count = 4;

    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        packet_net_t * p_net_packet = construct_in_packet(&p_packet, test_vector.p_encrypted_packet, test_vector.lengths.encrypted);

        transport_pkt_in_mock_reset();

        msg_cache_entry_exists_IgnoreAndReturn(false);
        msg_cache_entry_add_Expect(LE2BE16(test_vector.params.source), test_vector.params.seqnum);

        m_net_secmat_get_calls_expect = 3;

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_in(p_net_packet, NULL));
        TEST_ASSERT_NOT_NULL(p_transport_pkt_in_previous);

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_header_data,     &p_transport_pkt_in_previous->header, test_vector.lengths.header);
        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_transport_packet, p_transport_pkt_in_previous->payload, test_vector.lengths.transport);

        packet_mgr_free(p_packet);
    }
}

void test_self_receive(void)
{
    const uint8_t run_testvectors[] = SELF_RECEIVE_TEST_VECTORS;
    provision(true);

    mp_net_secmats = &test_network;
    m_net_secmat_count = 1;

    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);

        packet_t * p_packet = NULL;
        packet_net_t* p_net_packet = construct_in_packet(&p_packet, test_vector.p_encrypted_packet, test_vector.lengths.encrypted);

        m_rx_address = test_vector.params.source;
        m_net_secmat_get_calls_expect = 1;

        msg_cache_entry_exists_IgnoreAndReturn(false);
        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_in(p_net_packet, NULL));

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        packet_mgr_free(p_packet);
    }
}

void test_packet_out(void)
{
    const uint8_t run_testvectors[] = NETWORK_PKT_OUT_TEST_VECTORS;
    provision(true);
    m_bearer_tx_return = NRF_SUCCESS;
    bearer_tx_StubWithCallback(bearer_tx_mock_cb);

    /* Processes all the test vectors as outbound packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        (void) construct_out_packet(&p_packet, test_vector.p_transport_packet, test_vector.lengths.transport, &test_vector.params);

        bearer_tx_mock_reset();

        net_state_tx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_out(p_packet, &test_network, true));
        TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);

        packet_net_t * p_transport_pkt = packet_net_packet_get(p_bearer_tx_previous);

        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_encrypted_packet, &p_transport_pkt->header, test_vector.lengths.encrypted);
    }

    /* Fail sending */
    m_bearer_tx_return = 0x12345678;
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        (void) construct_out_packet(&p_packet, test_vector.p_transport_packet, test_vector.lengths.transport, &test_vector.params);

        bearer_tx_mock_reset();

        net_state_tx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index);

        TEST_ASSERT_EQUAL(0x12345678, network_pkt_out(p_packet, &test_network, true));
        TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);

        packet_net_t * p_transport_pkt = packet_net_packet_get(p_bearer_tx_previous);
        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_encrypted_packet, &p_transport_pkt->header, test_vector.lengths.encrypted);
    }
}

void test_invalid_net_key_in(void)
{
    const uint8_t run_testvectors[] = INVALID_NETKEY_TEST_VECTORS;

    provision(false);

    transport_pkt_in_StubWithCallback(transport_pkt_in_mock_cb);

    mp_net_secmats = &test_network;
    m_net_secmat_count = 1;

    /* Processes all the test vectors as incoming packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);

        packet_t * p_packet = NULL;
        packet_net_t* p_net_packet = construct_in_packet(&p_packet, test_vector.p_encrypted_packet, test_vector.lengths.encrypted);

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);

        m_net_secmat_get_calls_expect = 2;

        transport_pkt_in_mock_reset();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_in(p_net_packet, NULL));
        TEST_ASSERT_NULL(p_transport_pkt_in_previous);

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        packet_mgr_free(p_packet);
    }
}

void test_relay(void)
{
    const uint8_t run_testvectors[] = NETWORK_PKT_RELAY_TEST_VECTORS;
    provision(true);
    m_bearer_tx_return = NRF_SUCCESS;

    bearer_tx_StubWithCallback(bearer_tx_mock_cb);

    /* Processes all the test vectors as outbound packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        test_vector.params.ttl++; // to get match on encrypted packet
        packet_net_t * p_net_packet = construct_out_packet(&p_packet, test_vector.p_transport_packet, test_vector.lengths.transport, &test_vector.params);

        if (test_vector.params.ttl >= 2)
        {
            net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);
        }

        bearer_tx_mock_reset();

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_relay(p_net_packet, &test_network));

        if (test_vector.params.ttl >= 2)
        {
            /* Check that packets with TTL >= 2 is relayed. */
            TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);
            TEST_ASSERT_EQUAL_MEMORY(test_vector.p_encrypted_packet, &((packet_net_t *) p_bearer_tx_previous->payload)->header, test_vector.lengths.header);
        }
        else
        {
            /* Check that packets with TTL <= 1 is not relayed. */
            TEST_ASSERT_NULL(p_bearer_tx_previous);
        }
    }
    /* Now fail TX */
    m_bearer_tx_return = 0x12345678;
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        test_vector.params.ttl++; // to get match on encrypted packet
        packet_net_t * p_net_packet = construct_out_packet(&p_packet, test_vector.p_transport_packet, test_vector.lengths.transport, &test_vector.params);

        bearer_tx_mock_reset();

        if (test_vector.params.ttl >= 2)
        {
            net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);
            TEST_ASSERT_EQUAL(m_bearer_tx_return, network_pkt_relay(p_net_packet, &test_network));

            /* Check that packets with TTL >= 2 is relayed. */
            TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);
            TEST_ASSERT_EQUAL_MEMORY(test_vector.p_encrypted_packet, &((packet_net_t *) p_bearer_tx_previous->payload)->header, test_vector.lengths.header);
        }
        else
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_relay(p_net_packet, &test_network));

            /* Check that packets with TTL <= 1 is not relayed. */
            TEST_ASSERT_NULL(p_bearer_tx_previous);
        }
    }
}

void test_app_cb_relay(void)
{
    nrf_mesh_init_params_t init_params;
    init_params.relay_cb = relay_cb;
    net_state_init_Expect();
    net_beacon_init_Expect();
    net_state_recover_from_flash_Expect();
    network_init(&init_params);

    const uint8_t run_testvectors[] = NETWORK_PKT_RELAY_TEST_VECTORS;
    provision(true);

    bearer_tx_StubWithCallback(bearer_tx_mock_cb);

    /* Processes all the test vectors as outbound packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        packet_t * p_packet = NULL;
        packet_net_t * p_net_packet = construct_out_packet(&p_packet, test_vector.p_transport_packet, test_vector.lengths.transport, &test_vector.params);

        if (test_vector.params.ttl >= 2)
        {
            m_relay_expected = 1;
            m_relay_dst = test_vector.params.dest;
            m_relay_src = test_vector.params.source;
            m_relay_ttl = test_vector.params.ttl;
            m_relay_return = i % 2; // only relay half of the packets.
            if (m_relay_return)
            {
                net_state_rx_iv_index_get_ExpectAndReturn(test_vector.params.iv_index & 0x01, test_vector.params.iv_index);
            }
        }
        else
        {
            m_relay_expected = 0;
        }

        bearer_tx_mock_reset();

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_pkt_relay(p_net_packet, &test_network));

        TEST_ASSERT_EQUAL(0, m_relay_expected);

        if (test_vector.params.ttl >= 2 && m_relay_return)
        {
            /* Check that packets with TTL >= 2 is relayed. */
            TEST_ASSERT_NOT_NULL(p_bearer_tx_previous);
            // Contents not important
        }
        else
        {
            /* Check that packets with TTL <= 1 is not relayed. */
            TEST_ASSERT_NULL(p_bearer_tx_previous);
        }
    }
}

