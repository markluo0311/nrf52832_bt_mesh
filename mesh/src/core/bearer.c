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
#include <stddef.h>

#include "bearer.h"
#include "bearer_adv.h"
#include "internal_event.h"
#include "bearer_event.h"
#include "fifo.h"
#include "packet_mgr.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "nrf_error.h"

#if (BEARER_TYPES & BEARER_ADV_RADIO)
#include "radio.h"
#endif

/** Default scan interval */
#define BEARER_ADV_SCAN_INT_MS_DEFAULT (100)
/** Default scan window */
#define BEARER_ADV_SCAN_WINDOW_MS_DEFAULT   (100)
/** Bearer RX info for a single packet */

/* Constants used for AD type filtering */
#define FILTER_SIZE (256/32) /* AD type is 8 bits , so we need 256 bits to encode all possible values as a bit field, we will store them as 32 bit numbers (words), thus we need 256/32 words */
#define ADTYPE_FILTER_INDEX_BITMASK 0xE0 /* Used for determining the index of the word in m_adtype_filter array when looking for a value's bitfield */
#define ADTYPE_FILTER_INDEX_BITMASK_LOCATION 5 /* Number of bits to represent a word (32: 1<<5) */
#define ADTYPE_FILTER_VALUE_MASK 31 /* Number of bits in each word */
#define ADTYPE_FILTER_INDEX(type) (((type) & ADTYPE_FILTER_INDEX_BITMASK) >> ADTYPE_FILTER_INDEX_BITMASK_LOCATION)
#define ADTYPE_FILTER_VALUE(type) (1UL << ((type) & ADTYPE_FILTER_VALUE_MASK))

typedef struct
{
    packet_t*        p_packet; /**< Packet pointer for packet that was received. */
    packet_meta_t    rx_meta;  /**< RX packet meta data. */
    bearer_t         bearer;   /**< The bearer type from which the packet originated. */
} bearer_rx_t;

/** Types of filters for addresses */
typedef enum
{
    ADDR_FILTER_TYPE_NONE,      /**< No filter set. */
    ADDR_FILTER_TYPE_WHITELIST, /**< Whitelist filter, address must be in list to be accepted. */
    ADDR_FILTER_TYPE_BLACKLIST, /**< Blacklist filter, addresses in the list will be rejected. */
    ADDR_FILTER_TYPE_RANGE      /**< Range filter, address must be in range between two addresses. */
} addr_filter_type_t;
typedef struct
{
    const ble_gap_addr_t* p_gap_addr_list;
    uint16_t count;
    addr_filter_type_t type;
} gap_addr_filter_t;

#if (BEARER_TYPES & BEARER_ADV_RADIO)
static advertiser_t m_regular_advertiser;
#endif

static fifo_t m_rx_fifo;

static bearer_rx_t m_rx_fifo_buffer[BEARER_RX_QUEUE_LENGTH];

static uint32_t packets_dropped_invalid_length = 0;
static uint32_t packets_dropped_invalid_adtype = 0;
static uint32_t packets_dropped_invalid_addr = 0;
static uint32_t packets_accepted = 0;

/** Filter for GAP addresses */
static gap_addr_filter_t m_addr_filter;

static uint32_t m_adtype_filter[FILTER_SIZE] = {0};
static bool m_adtype_filtering = false;

#ifdef NRF_MESH_TEST_BUILD
static uint32_t packets_dropped_invalid_rssi = 0;
static uint8_t m_rssi_filter_val = 0;
#endif

static inline bool m_adtype_valid(uint8_t type)
{
    bool adtype_valid = false;
    if (m_adtype_filtering)
    {
        uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
        uint32_t filter_value = ADTYPE_FILTER_VALUE(type);

        if ( filter_value & m_adtype_filter[filter_index] )
        {
            adtype_valid = true;
        }
    }
    else
    {
        adtype_valid = true;
    }
    return adtype_valid;
}

/**
 * Check whether the given address filter will accept the given packet.
 *
 * @param[in] p_filter The filter to check against.
 * @param[in] p_packet The packet pointer to check for.
 *
 * @returns Whether the packet is accepted by the filter.
 */
static inline bool gap_addr_filter_accept(const gap_addr_filter_t* p_filter, const packet_t* p_packet)
{
    if (p_filter->type == ADDR_FILTER_TYPE_NONE)
    {
        return true; /* No filter set, accept all */
    }

    NRF_MESH_ASSERT(p_filter->p_gap_addr_list != NULL);

    bool in_list = false;
    switch (p_filter->type)
    {
        case ADDR_FILTER_TYPE_WHITELIST:
        case ADDR_FILTER_TYPE_BLACKLIST:
            for (uint32_t i = 0; i < p_filter->count; ++i)
            {
                if (memcmp(p_filter->p_gap_addr_list[i].addr, p_packet->addr, BLE_GAP_ADDR_LEN) == 0 &&
                    p_packet->header.addr_type == p_filter->p_gap_addr_list[i].addr_type)
                {
                    in_list = true;
                    break;
                }
            }
            break;
        case ADDR_FILTER_TYPE_RANGE:
            if ((p_packet->header.addr_type == p_filter->p_gap_addr_list[0].addr_type) &&
                (memcmp(p_filter->p_gap_addr_list[0].addr, p_packet->addr, BLE_GAP_ADDR_LEN) <= 0) &&
                (memcmp(p_filter->p_gap_addr_list[1].addr, p_packet->addr, BLE_GAP_ADDR_LEN) > 0))
                {
                    in_list = true;
                }
            break;
        default:
            NRF_MESH_ASSERT(false);
            return true;
    }

    switch (p_filter->type)
    {
        case ADDR_FILTER_TYPE_WHITELIST:
            return in_list;
        case ADDR_FILTER_TYPE_BLACKLIST:
            return !in_list;
        case ADDR_FILTER_TYPE_RANGE:
            return in_list;
        default:
            NRF_MESH_ASSERT(false);
            return true; /* Something went wrong, don't filter packets. */
    }
}

static uint32_t gap_addr_filter_set(const ble_gap_addr_t* const p_addrs,
        uint16_t addr_count, addr_filter_type_t type)
{
    if (m_addr_filter.type != ADDR_FILTER_TYPE_NONE &&
        m_addr_filter.type != type)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_addrs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (addr_count == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    bearer_event_critical_section_begin();
    m_addr_filter.type            = type;
    m_addr_filter.p_gap_addr_list = p_addrs;
    m_addr_filter.count           = addr_count;
    bearer_event_critical_section_end();
    return NRF_SUCCESS;
}

static void bearer_rx_cb(packet_t* p_packet, bearer_t bearer, const packet_meta_t * p_meta)
{
#ifdef NRF_MESH_TEST_BUILD
    if (m_rssi_filter_val > 0)
    {
        if (p_meta->rssi > m_rssi_filter_val)
        {
            packets_dropped_invalid_rssi++;
            packet_mgr_free(p_packet);
            return;
        }
    }
#endif

    if (!gap_addr_filter_accept(&m_addr_filter, p_packet))
    {
        packets_dropped_invalid_addr++;
        packet_mgr_free(p_packet);
        return;
    }

    bearer_rx_t rx_packet;
    rx_packet.p_packet = p_packet;
    rx_packet.bearer = bearer;

#if BEARER_ASSERT_ON_WRONG_RX_LENGTH
    NRF_MESH_ASSERT(p_packet->header.length <= 37);
#else
    if (p_packet->header.length > 37)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_INVALID_PACKET_LEN, sizeof(ble_packet_hdr_t), &p_packet->header );
        ++packets_dropped_invalid_length;
        packet_mgr_free(p_packet);
        return;
    }
#endif

    bool adtype_valid = false;
    for (int i = 0; i < packet_payload_size_get(p_packet);)
    {
        uint8_t length = p_packet->payload[i];
        uint8_t type = p_packet->payload[i + 1];

        adtype_valid = m_adtype_valid(type);
        if (adtype_valid)
        {
            if (length >= packet_payload_size_get(p_packet) - i)
            {
                ++packets_dropped_invalid_length;
                // Commenting out as this happens a lot
                //__INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_INVALID_PACKET_LEN, 1, &length);
                packet_mgr_free(p_packet);
                return;
            }
            break;
        }

        i += length + 1;
    }

    if (!adtype_valid)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_INVALID_ADTYPE, 0, 0);
        ++packets_dropped_invalid_adtype;
        packet_mgr_free(p_packet);
        return;
    }

    if (p_meta != NULL)
    {
        memcpy(&rx_packet.rx_meta, p_meta, sizeof(packet_meta_t));
    }

    if (fifo_push(&m_rx_fifo, &rx_packet) != NRF_SUCCESS)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_NO_MEM, 0, 0);
        packet_mgr_free(p_packet);
    }
    else
    {
        ++packets_accepted;
    }
}

uint32_t bearer_init(const nrf_mesh_init_params_t * p_init_params)
{
    uint32_t error_code;
    memset(m_rx_fifo_buffer, 0, sizeof(bearer_rx_t) * BEARER_RX_QUEUE_LENGTH);
    m_rx_fifo.elem_array = m_rx_fifo_buffer;
    m_rx_fifo.elem_size = sizeof(bearer_rx_t);
    m_rx_fifo.array_len = BEARER_RX_QUEUE_LENGTH;
    fifo_init(&m_rx_fifo);

#if BEARER_TYPES & BEARER_ADV_RADIO

    bearer_scan_config_t config = {
        .scan_channel_map = NRF_MESH_ADV_CHAN_DEFAULT,
        .scan_int_ms     = BEARER_ADV_SCAN_INT_MS_DEFAULT,
        .scan_window_ms  = BEARER_ADV_SCAN_WINDOW_MS_DEFAULT,
        .rx_cb           = bearer_rx_cb
    };

    error_code = bearer_adv_init(&config);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
#endif

#if (BEARER_TYPES & BEARER_ADV_RADIO)
    m_regular_advertiser.adv_channel_map = NRF_MESH_ADV_CHAN_DEFAULT;
    m_regular_advertiser.adv_int_max_ms = BEARER_ADV_INT_MAX_MS_DEFAULT;
    m_regular_advertiser.adv_int_min_ms = BEARER_ADV_INT_MIN_MS_DEFAULT;
    m_regular_advertiser.adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    m_regular_advertiser.queue_empty_cb = NULL;
    bearer_adv_advertiser_init(&m_regular_advertiser);
#endif

    return NRF_SUCCESS;
}

void bearer_adtype_filtering_set(bool filter)
{
    m_adtype_filtering = filter;
}

void bearer_adtype_add(uint8_t type)
{
    uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
    uint32_t filter_value = ADTYPE_FILTER_VALUE(type);
    m_adtype_filter[filter_index] |= filter_value;
}

void bearer_adtype_remove(uint8_t type)
{
    uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
    uint32_t filter_value = ADTYPE_FILTER_VALUE(type);
    m_adtype_filter[filter_index] &= ~filter_value;
}

uint32_t bearer_filter_gap_addr_whitelist_set(const ble_gap_addr_t* const p_addrs, uint16_t addr_count)
{
    return gap_addr_filter_set(p_addrs, addr_count, ADDR_FILTER_TYPE_WHITELIST);
}

uint32_t bearer_filter_gap_addr_blacklist_set(const ble_gap_addr_t* const p_addrs, uint16_t addr_count)
{
    return gap_addr_filter_set(p_addrs, addr_count, ADDR_FILTER_TYPE_BLACKLIST);
}

uint32_t bearer_filter_gap_addr_range_set(const ble_gap_addr_t* const p_addrs)
{
    if (p_addrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_addrs[0].addr_type != p_addrs[1].addr_type ||
        memcmp(p_addrs[0].addr, p_addrs[1].addr, BLE_GAP_ADDR_LEN) > 0)
    {
        return NRF_ERROR_INVALID_DATA;
    }
    return gap_addr_filter_set(p_addrs, 2, ADDR_FILTER_TYPE_RANGE);
}

uint32_t bearer_filter_gap_addr_clear(void)
{
    if (m_addr_filter.type == ADDR_FILTER_TYPE_NONE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    bearer_event_critical_section_begin();
    m_addr_filter.type            = ADDR_FILTER_TYPE_NONE;
    m_addr_filter.p_gap_addr_list = NULL;
    m_addr_filter.count           = 0;
    bearer_event_critical_section_end();
    return NRF_SUCCESS;
}

uint32_t bearer_enable(void)
{
#if (BEARER_TYPES & BEARER_ADV_RADIO)
    bearer_adv_scan_start();
#endif

    return NRF_SUCCESS;
}

uint32_t bearer_disable(void)
{
#if (BEARER_TYPES & BEARER_ADV_RADIO)
    bearer_adv_scan_stop();
    (void) bearer_adv_adv_stop(&m_regular_advertiser);
#endif
    return NRF_SUCCESS;
}

void bearer_stats_get(uint32_t * p_packets_dropped_invalid_length,
        uint32_t * p_packets_dropped_invalid_adtype,
        uint32_t * p_packets_dropped_invalid_addr,
        uint32_t * p_packets_accepted)
{
    *p_packets_dropped_invalid_length = packets_dropped_invalid_length;
    *p_packets_dropped_invalid_adtype = packets_dropped_invalid_adtype;
    *p_packets_dropped_invalid_addr   = packets_dropped_invalid_addr;
    *p_packets_accepted = packets_accepted;
}

uint32_t bearer_tx(packet_t* p_packet, bearer_t bearers, uint8_t transmit_count)
{
    NRF_MESH_ASSERT(p_packet->header.length <= 37);

    uint32_t error_code = NRF_SUCCESS;
    /* We're feeding the bearer from the event handler and main, prevent race condition */
    if ((bearers & (BEARER_ADV_RADIO)) != bearers || bearers == 0)
    {
        /* one of the given bearers is not configured. */
        error_code = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        bearer_event_critical_section_begin();
        error_code = bearer_adv_tx(&m_regular_advertiser, p_packet, transmit_count);
        bearer_event_critical_section_end();
    }
    return error_code;
}

uint32_t bearer_rx(packet_t** pp_packet, bearer_t* p_bearer, packet_meta_t * p_meta)
{
    bearer_rx_t rx_packet;
    uint32_t error_code = fifo_pop(&m_rx_fifo, &rx_packet);

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    if (p_bearer != NULL)
    {
        *p_bearer = rx_packet.bearer;
    }
    *pp_packet = rx_packet.p_packet;

    if (p_meta != NULL)
        memcpy(p_meta, &rx_packet.rx_meta, sizeof(packet_meta_t));

    return NRF_SUCCESS;
}

#ifdef NRF_MESH_TEST_BUILD
void bearer_rssi_filtering_set(uint8_t rssi)
{
    m_rssi_filter_val = rssi;
}

uint32_t bearer_packet_drop_rssi_get(void)
{
    return packets_dropped_invalid_rssi;
}

advertiser_t* get_p_advertiser(void)
{
    return &m_regular_advertiser;
}

#endif
