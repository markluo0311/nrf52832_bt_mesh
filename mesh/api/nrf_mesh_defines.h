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

#ifndef NRF_MESH_DEFINES_H__
#define NRF_MESH_DEFINES_H__

/**
 * @defgroup NRF_MESH_DEFINES Mesh common definitions
 * @ingroup MESH_API_GROUP_CORE
 * The values in this header file are (eventually) considered immutable.
 * @{
 */

/**
 * @defgroup MESH_DEFINES_API API level definitions
 * @{
 */

/** Maximum possible segmented payload size. */
#define NRF_MESH_SEG_PAYLOAD_SIZE_MAX (380)

/** Maximum possible single segment payload size. */
#define NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX (11)

/** Size of a single segment in a segmented transfer. */
#define NRF_MESH_SEG_SIZE (12)

/** Advertisement channel 37. */
#define NRF_MESH_ADV_CHAN_0 (1 << 0)

/** Advertisement channel 38. */
#define NRF_MESH_ADV_CHAN_1 (1 << 1)

/** Advertisement channel 39. */
#define NRF_MESH_ADV_CHAN_2 (1 << 2)

/** All advertisement channels */
#define NRF_MESH_ADV_CHAN_ALL (NRF_MESH_ADV_CHAN_0 | NRF_MESH_ADV_CHAN_1 | NRF_MESH_ADV_CHAN_2)

/** Size (in octets) of an encryption key.*/
#define NRF_MESH_KEY_SIZE  (16)

/** Size (in octets) of the network ID. */
#define NRF_MESH_NETID_SIZE (8)

/** Size (in octets) of a UUID. */
#define NRF_MESH_UUID_SIZE (16)

/** Number of bits available for the TTL field. */
#define NRF_MESH_TTL_BIT_COUNT (7)

/** Maximum TTL value. */
#define NRF_MESH_TTL_MAX ((1 << NRF_MESH_TTL_BIT_COUNT) - 1)

/** 32-bit MIC size. */
#define MIC_SIZE_32BIT (4)

/** 64-bit MIC size. */
#define MIC_SIZE_64BIT (8)

/** Maximum global key index allowed, according to specification, Section 3.8.6.4. */
#define NRF_MESH_GLOBAL_KEY_INDEX_MAX  (0xFFF)

/** Size (in octets) of the unprovisioned beacon URI hash. */
#define NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE    (4)

/** Size of an ECDH public key. */
#define NRF_MESH_ECDH_PUBLIC_KEY_SIZE    (64)

/** Size of an ECDH private key. */
#define NRF_MESH_ECDH_PRIVATE_KEY_SIZE   (32)

/** Size of an ECDH shared secret. */
#define NRF_MESH_ECDH_SHARED_SECRET_SIZE    (32)

/** Unassigned address. */
#define NRF_MESH_ADDR_UNASSIGNED                  (0x0000)

/** Offset of bits determining the address type. */
#define NRF_MESH_ADDR_TYPE_BITS_OFFSET            (14)
/** Mask of bits determining the address type. */
#define NRF_MESH_ADDR_TYPE_BITS_MASK              (0xC000)

/** @} end of MESH_DEFINES_API */

/**
 * @defgroup MESH_DEFINES_UTILS Utility definitions
 * @{
 */

/**
 * Converts hours to seconds.
 * @param t The number of hours.
 * @return  The number of seconds corresponding to the specified number of hours.
 */
#define HOURS_TO_SECONDS(t) ((t) * 60 * 60)

/**
 * Converts seconds to microseconds.
 * @param t The number of seconds.
 * @return  The number of microseconds corresponding to the specified number of seconds.
 */
#define SEC_TO_US(t) ((t) * 1000000)

/**
 * Converts milliseconds to microseconds.
 * @param t The number of milliseconds.
 * @return  The number of microseconds corresponding to the specified number of milliseconds.
 */
#define MS_TO_US(t) ((t) * 1000)

/** @} end of MESH_DEFINES_UTILS */

/**
 * @defgroup MESH_DEFINES_BEARER Bearer level definitions
 * @{
 */

/** Bearer type radio advertisement. */
#define BEARER_ADV_RADIO (0x01)

/** @} end of MESH_DEFINES_BEARER */

/**
 * @defgroup MESH_DEFINES_NETWORK Network layer definitions
 * @{
 */

/** Number of bits in the sequence number. */
#define NETWORK_SEQNUM_BITS         24

/** Maximum allowed sequence number. */
#define NETWORK_SEQNUM_MAX          ((1 << NETWORK_SEQNUM_BITS) - 1)

/** Maximum allowed number of retransmissions for relayed packets. */
#define NETWORK_RELAY_RETRANSMITS_MAX   ((1 << 3) - 1)

/** Maximum allowed number of 10 ms steps for the interval between relayed packet retranmissions. */
#define NETWORK_RELAY_INTERVAL_STEPS_MAX ((1 << 5) - 1)

/** @} end of MESH_DEFINES_NETWORK*/

/**
 * @defgroup MESH_DEFINES_TRANSPORT Transport layer definitions
 * @{
 */

/** RX timeout lower limit. */
#define TRANSPORT_SAR_RX_TIMEOUT_MIN SEC_TO_US(60)
/** RX timeout upper limit. */
#define TRANSPORT_SAR_RX_TIMEOUT_MAX SEC_TO_US(120)

/** RX acknowledgement timeout lower limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_MIN MS_TO_US(150)
/** RX acknowledgement timeout upper limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_MAX SEC_TO_US(10)

/** TX retry timeout lower limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_MIN MS_TO_US(150)
/** TX retry timeout upper limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_MAX SEC_TO_US(10)

/** TX timeout lower limit. */
#define TRANSPORT_SAR_TX_TIMEOUT_MIN MS_TO_US(150)
/** TX timeout upper limit. */
#define TRANSPORT_SAR_TX_TIMEOUT_MAX SEC_TO_US(60)

/** TX retries upper limit (UINT8_MAX). */
#define TRANSPORT_SAR_TX_RETRIES_MAX (255)

/** Default value for segment acknowledgement messages. */
#define TRANSPORT_SAR_SEGACK_TTL_DEFAULT (63)

/** @} end of MESH_DEFINES_TRANSPORT */

/**
 * @defgroup MESH_DEFINES_LOG Log API definitions
 * @{
 */

/** Receive logs from the bearer. */
#define LOG_SRC_BEARER (1 << 0)

/** Receive logs from the network layer. */
#define LOG_SRC_NETWORK (1 << 1)

/** Receive logs from the transport layer. */
#define LOG_SRC_TRANSPORT (1 << 2)

/** Receive logs from the provisioning module. */
#define LOG_SRC_PROV (1 << 3)

/** Receive logs from the packet manager. */
#define LOG_SRC_PACMAN (1 << 4)

/** Receive logs from the internal event module. */
#define LOG_SRC_INTERNAL (1 << 5)

/** Receive logs from the nRF Mesh API. */
#define LOG_SRC_API (1 << 6)

/** Receive logs from the DFU module. */
#define LOG_SRC_DFU (1 << 7)

/** Receive logs from the beacon module. */
#define LOG_SRC_BEACON (1 << 8)

/** Receive logs from the test framework. */
#define LOG_SRC_TEST (1 << 9)

/** Receive logs from the encryption module. */
#define LOG_SRC_ENC (1 << 10)

/** Receive logs from the timer scheduler. */
#define LOG_SRC_TIMER_SCHEDULER (1 << 11)

/** Receive logs from the CCM block. */
#define LOG_SRC_CCM (1 << 12)

/** Receive logs from the access layer. */
#define LOG_SRC_ACCESS (1 << 13)

/** Receive logs from the application. */
#define LOG_SRC_APP (1 << 14)

/** Receive logs from the serial handler. */
#define LOG_SRC_SERIAL (1 << 15)

/** Default group for receiving logs from the core stack. */
#define LOG_GROUP_STACK (LOG_SRC_BEARER | LOG_SRC_NETWORK | LOG_SRC_TRANSPORT)

/** Event level data. */
#define EVT_LEVEL_DATA   (11)

/** Event level info. */
#define EVT_LEVEL_INFO   (10)

/** Event level error. */
#define EVT_LEVEL_ERROR  ( 9)

/** Event level base. */
#define EVT_LEVEL_BASE   ( 8)

/** Log level debug (3). */
#define LOG_LEVEL_DBG3   ( 7)

/** Log level debug (2). */
#define LOG_LEVEL_DBG2   ( 6)

/** Log level debug (1). */
#define LOG_LEVEL_DBG1   ( 5)

/** Log level info. */
#define LOG_LEVEL_INFO   ( 4)

/** Log level report. */
#define LOG_LEVEL_REPORT ( 3)

/** Log level warning. */
#define LOG_LEVEL_WARN   ( 2)

/** Log level error. */
#define LOG_LEVEL_ERROR  ( 1)

/** Log level assert. */
#define LOG_LEVEL_ASSERT ( 0)

/** @} end of MESH_DEFINES_LOG */

/** @} */
#endif  /* NRF_MESH_DEFINES_H__ */
