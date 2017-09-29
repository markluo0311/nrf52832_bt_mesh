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

#ifndef NRF_MESH_OPT_H_
#define NRF_MESH_OPT_H_

#include <stdint.h>
/**
 * @defgroup NRF_MESH_OPT Mesh options API
 * @ingroup MESH_API_GROUP_CORE
 * @{
 */
/** Start of radio parameters. */
#define NRF_MESH_OPT_RADIO_START    1
/** Start of provisioning parameters. */
#define NRF_MESH_OPT_PROV_START     200
/** Start of transport layer parameters. */
#define NRF_MESH_OPT_TRS_START      300
/** Start of network layer parameters. */
#define NRF_MESH_OPT_NET_START      400

/**
 * Option ID type.
 *
 * @note All corresponding option values are assumed unsigned 32-bit integers
 *       unless stated otherwise.
 */
typedef enum
{
    /** Radio mode (@c nrf_mesh_radio_mode_t). */
    NRF_MESH_OPT_RADIO_MODE = NRF_MESH_OPT_RADIO_START,
    /** Access address (uint32_t). */
    NRF_MESH_OPT_RADIO_ACCESS_ADDR,
    /** Channel map (0-39) (@c nrf_mesh_chmap_t). */
    NRF_MESH_OPT_RADIO_CHMAP,
    /** Time between the start of two subsequent scans (in milliseconds). */
    NRF_MESH_OPT_RADIO_SCAN_INT_MS,
    /** Duration of a single scan (in milliseconds). */
    NRF_MESH_OPT_RADIO_SCAN_WINDOW_MS,
    /** Radio TX power. */
    NRF_MESH_OPT_RADIO_TX_POWER,
    /** Enable/Disable AD type filtering (uses opt.val: 0 to disable, 1 to enable). */
    NRF_MESH_OPT_RADIO_AD_TYPE_FILTERING,
    /** Add an AD type to the list of accepted AD types (uses the first 8 bits of opt.val). */
    NRF_MESH_OPT_RADIO_AD_TYPE_ADD,
    /** Remove an AD type from the list of accepted AD types (uses the first 8 bits of opt.val). */
    NRF_MESH_OPT_RADIO_AD_TYPE_REMOVE,
    /** Enable (1) / disable (0) ECDH offloading in provisioning. */
    NRF_MESH_OPT_PROV_ECDH_OFFLOADING = NRF_MESH_OPT_PROV_START,
    /** Transport SAR RX timeout. */
    NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT = NRF_MESH_OPT_TRS_START,
    /** Transport SAR RX acknowledgement timeout. */
    NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT,
    /** Timeout for cancelling the SAR TX session. */
    NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT,
    /** Timeout for retransmitting a SAR packet. */
    NRF_MESH_OPT_TRS_SAR_TX_TIMEOUT,
    /** Number of retries before cancelling a SAR session. */
    NRF_MESH_OPT_TRS_SAR_TX_RETRIES,
    /** Default TTL value for segment acknowledgement messages. */
    NRF_MESH_OPT_TRS_SAR_SEGACK_TTL,
    /** 32-bit (0) or 64-bit (1) MIC size for transport layer. */
    NRF_MESH_OPT_TRS_SZMIC,
    /** Packet relaying enabled (1) or disabled (0). */
    NRF_MESH_OPT_NET_RELAY_ENABLE = NRF_MESH_OPT_NET_START,
    /** Number of retransmits per relayed packet. */
    NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT,
    /** Number of 10 ms intervals between retransmits of relayed packets. */
    NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_STEPS,
} nrf_mesh_opt_id_t;

/**
 * Radio channel map.
 *
 * @warning Using the extended channel map is not recommended as it breaks Bluetooth 
 *          Mesh compatibility. In addition, the extended channels (0-36) are
 *          very prone to WiFi noise.
 */
typedef struct __attribute((packed))
{
    /** Extended channel map. (Not available) */
    uint64_t ext_chmap : 37;
    /** Advertising channel map. */
    uint8_t  adv_chmap : 3;
} nrf_mesh_chmap_t;

/**
 * Radio mode configuration.
 *
 * @note Directly mapped to hardware registers
 * @note @c NRF_MESH_RADIO_MODE_250Kbit is deprecated for nRF52.
 *
 * @warning To be on-air compatible with other devices, @c
 *          NRF_MESH_RADIO_MODE_BLE_1MBIT is required.
 */
typedef enum
{
    /** 1 Mbps mode. */
    NRF_MESH_RADIO_MODE_1MBIT,
    /** 2 Mbps mode. */
    NRF_MESH_RADIO_MODE_2MBIT,
    /** 250 Kbps mode (deprecated for nRF52). */
    NRF_MESH_RADIO_MODE_250KBIT,
    /** 1 Mbps BLE compatible mode. */
    NRF_MESH_RADIO_MODE_BLE_1MBIT
} nrf_mesh_radio_mode_t;

/**
 * Options structure.
 */
typedef struct
{
    /** Length of opt field (for future compatibility). */
    uint32_t len;
    /** Option to set/get. */
    union
    {
        /** Unsigned 32-bit value. */
        uint32_t val;
        /** Byte array. */
        uint8_t * p_array;
        /** Channel map. */
        nrf_mesh_chmap_t chmap;
        /** Radio mode. */
        nrf_mesh_radio_mode_t radio_mode;
    } opt;
} nrf_mesh_opt_t;

/**
 * Function for setting various nRF Mesh options.
 *
 * @param[in] id    Identifier for option to set. See @c nrf_mesh_opt_id_t.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS Successfully set option.
 */
uint32_t nrf_mesh_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Function for getting various nRF Mesh options.
 *
 * @param[in]  id    Identifier for option to get. See @c nrf_mesh_opt_id_t.
 * @param[out] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS Successfully retrieved option.
 */
uint32_t nrf_mesh_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/** @} end of NRF_MESH_OPT */
#endif
