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

#include <stddef.h>
#include <string.h>

#include "bearer_adv.h"

#include "nrf_mesh_config_core.h"
#include "bearer.h"
#include "bearer_event.h"
#include "radio.h"
#include "timer_scheduler.h"
#include "packet.h"
#include "packet_mgr.h"
#include "fifo.h"
#include "rand.h"
#include "utils.h"
#include "toolchain.h"
#include "log.h"
#include "nrf_mesh_opt.h"
#include "event.h"
#include "bearer_event.h"

#include "nrf_mesh_assert.h"

#include "nrf.h"
#include "ble_gap.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define BLE_ADV_QUICK_START_OFFSET          (1000) /**< Earliest potential quick start time for a message */
#define BLE_ADV_CHANNEL_MAP_MASK            (0x07) /**< Channel map for advertisement channels. */
#define GAP_ADDR_SUM_MAX                    (0xFF * 5 + 0x3F) /**< Sum if all bits of random part is set in GAP address. */
/*****************************************************************************
* Local type definitions
*****************************************************************************/
/*****************************************************************************
* Static globals
*****************************************************************************/
/** Current scan channel */
static uint8_t m_scan_ch;
/** Scanning in progress */
static bool m_is_scanning = false;
/** Current configuration parameters */
static bearer_scan_config_t  m_scan_config;
/** Timer event for periodic scanning */
static timer_event_t m_scan_evt;
/** Timer event for periodic scan end */
static timer_event_t m_scan_window_end_evt;

/** Default Gap Address for Advertiser Init */
static ble_gap_addr_t m_default_gap_addr;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void adv_evt_timeout(timestamp_t timestamp, void * p_context);

static void next_adv_schedule(advertiser_t* p_adv)
{
    timestamp_t time_now = timer_now();

    /* If there's less than adv_int ms since the last advertisement, schedule
     * for last advertisement + adv_int */
    if (TIMER_OLDER_THAN(p_adv->internal.adv_evt.timestamp, time_now) &&
        TIMER_OLDER_THAN(time_now, p_adv->internal.adv_evt.timestamp + p_adv->adv_int_min_ms * 1000))
    {
        uint32_t rand_offset = rand_prng_get(&p_adv->internal.adv_event_prng)
                % ((p_adv->adv_int_max_ms - p_adv->adv_int_min_ms) * 1000);
        timer_sch_reschedule(&p_adv->internal.adv_evt,
                p_adv->internal.adv_evt.timestamp + p_adv->adv_int_min_ms * 1000 + rand_offset);
    }
    else
    {
        /* Order timer with quick, randomized offset. */
        uint32_t rand_offset = rand_prng_get(&p_adv->internal.adv_event_prng)
            % (p_adv->adv_int_min_ms * 1000 - BLE_ADV_QUICK_START_OFFSET);
        timer_sch_reschedule(&p_adv->internal.adv_evt,
                time_now + BLE_ADV_QUICK_START_OFFSET + rand_offset);
    }

}

/** Order a preemptable scan for other mesh packets. Channel must be a valid BLE channel. */
static void scan_order(void)
{
    radio_event_t rx_evt;
    rx_evt.event_type = RADIO_EVENT_TYPE_RX_PREEMPTABLE;

    if (packet_mgr_alloc((void**) &rx_evt.p_packet, BLE_ADV_PACKET_MAX_LENGTH) != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_BEARER, LOG_LEVEL_WARN, "scan_order(): cannot allocate memory for incoming packet\n");
        return;
    }

    rx_evt.channel = m_scan_ch;
    uint32_t count = 1;
    if (radio_order(&rx_evt, &count) == NRF_SUCCESS)
    {
        radio_invoke();
    }
    else
    {
        packet_mgr_free(rx_evt.p_packet);
    }
}

static void scan_channel_iterate(void)
{
    while (true)
    {
        if (m_scan_ch == 39)
        {
            m_scan_ch = 36;
        }
        else
        {
            ++m_scan_ch;
        }

        if ((1 << (m_scan_ch - 37)) & (m_scan_config.scan_channel_map))
        {
            break;
        }
    }
}

static void scan_begin_timer_cb(timestamp_t timestamp, void * p_context)
{
    m_is_scanning = true;
    scan_channel_iterate();
    scan_order();
}

static void scan_end_timer_cb(timestamp_t timestamp, void * p_context)
{
    m_is_scanning = false;
    radio_preemptable_cancel();
}

static void packet_fields_set(packet_t* p_packet, advertiser_t* p_adv)
{
    p_packet->header._rfu1 = 0;
    p_packet->header._rfu2 = 0;
    p_packet->header._rfu3 = 0;
    p_packet->header.type = p_adv->adv_packet_type;
    p_packet->header.addr_type = p_adv->internal.ble_adv_addr.addr_type;
    memcpy(p_packet->addr, p_adv->internal.ble_adv_addr.addr, BLE_GAP_ADDR_LEN);
}

static void tx_event_send(void* p_context)
{
    /* done transmitting, no longer need the reference */
    nrf_mesh_evt_t tx_complete =
        {
            .type = NRF_MESH_EVT_TX_COMPLETE,
            .params.tx_complete.packet_id = (uint32_t) p_context
        };
    event_handle(&tx_complete);
}

static bool is_valid_gap_addr(const ble_gap_addr_t* p_addr)
{
    bool is_valid = false;
    switch (p_addr->addr_type)
    {
        case BLE_GAP_ADDR_TYPE_PUBLIC:
        case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
            /* The two most significant bits of the address shall be equal to 1 */
            if ((p_addr->addr[5] & 0xC0) == 0xC0)
            {
                is_valid = true;
            }
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
            /* The two most significant bits of the address shall be equal to 0 */
            if ((p_addr->addr[5] & 0xC0) == 0x00)
            {
                is_valid = true;
            }
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
            /* The two most significant bits of the address shall be equal to 0b10 */
            if ((p_addr->addr[5] & 0xC0) == 0x80)
            {
                is_valid = true;
            }
            break;
        default:
            break;
    }

    /* All bits of the random part of the address shall not be equal to 1
       All bits of the random part of the address shall not be equal to 0 */
    uint32_t addr_sum = ((p_addr->addr[5] & 0x3F)
                         + p_addr->addr[4] + p_addr->addr[3] + p_addr->addr[2]
                         + p_addr->addr[1] + p_addr->addr[0]);
    if (!is_valid || (addr_sum == 0) || (addr_sum == GAP_ADDR_SUM_MAX))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*****************************************************************************
* Lower layer callback functions
*****************************************************************************/
/** Radio callback function for ending a reception event */
static void radio_rx_cb(uint8_t* p_data, bool success, uint32_t crc, int8_t rssi)
{
    if (!success)
    {
        packet_mgr_free(p_data);
        return;
    }
    packet_t* p_packet = (packet_t*) p_data;

    /*lint -save -e446 */
    packet_meta_t meta =
    {
        .rssi = rssi,
        .timestamp = timer_now(),
        .addr_type = p_packet->header.addr_type,
        .p_addr = p_packet->addr
    };
    /*lint -restore */

    m_scan_config.rx_cb(p_packet, BEARER_ADV_RADIO, &meta);
}

/** Radio callback function for ending a transmission event */
static void radio_tx_cb(uint8_t* p_data, bool free_packet)
{
    if (free_packet)
    {
        packet_mgr_free((packet_t*) p_data);
        NRF_MESH_ASSERT(bearer_event_generic_post(tx_event_send, (void*) p_data) == NRF_SUCCESS);
    }
}

/** Radio callback function for reporting an empty radio event queue */
static void radio_idle_cb(void)
{
    if (m_is_scanning)
    {
        scan_order();
    }
}

static void packet_tx(packet_t* p_packet, uint8_t* p_channel_map, bool free_on_end)
{
    radio_event_t radio_evts[3];

    uint32_t count = 0;
    for (uint32_t ch = 0; ch < 3; ++ch)
    {
        if (*p_channel_map & (1 << ch))
        {
            radio_evts[count].event_type = RADIO_EVENT_TYPE_TX;
            radio_evts[count].p_packet = (uint8_t*) p_packet;
            radio_evts[count].free_on_end = free_on_end;
            radio_evts[count].channel = ch + 37;
            count++;
        }
    }

    (void) radio_order(radio_evts, &count);
    radio_invoke();

    for (uint32_t ch = 0; ch < 3; ++ch)
    {
        if (*p_channel_map & (1 << ch))
        {
            if (count == 0)
            {
                *p_channel_map &= ~(1U << ch);
            }
            else
            {
                count--;
            }
        }
    }
}

/** Timer callback for advertisement event */
static void adv_evt_timeout(timestamp_t timestamp, void * p_context)
{
    advertiser_t* p_advertiser = (advertiser_t*) p_context;
    /* update state */
    p_advertiser->internal.advertisement_in_progress = false;
    const bool replace_infinite_beacon =
        (
             p_advertiser->internal.tx_evt.transmits == BEARER_ADV_REPEAT_INFINITE
             &&
             !fifo_is_empty(&p_advertiser->internal.tx_fifo)
        );


    /* fetch new packet */
    if (p_advertiser->internal.tx_evt.transmits == 0 || replace_infinite_beacon)
    {
        /* Replace the old packet - remove its reference. */
        if (p_advertiser->internal.tx_evt.p_packet != NULL)
        {
            packet_mgr_free(p_advertiser->internal.tx_evt.p_packet);
            p_advertiser->internal.tx_evt.p_packet = NULL;
        }
        uint32_t error_code = fifo_pop(&p_advertiser->internal.tx_fifo, &p_advertiser->internal.tx_evt);

        if (error_code != NRF_SUCCESS)
        {
            /* no more events in fifo, stop advertising */
            p_advertiser->internal.adv_evt.interval = 0;
            return;
        }
        /* set BLE fields */
        packet_fields_set(p_advertiser->internal.tx_evt.p_packet, p_advertiser);
    }


    NRF_MESH_ASSERT(p_advertiser->adv_channel_map != 0);

    uint8_t ch_map = p_advertiser->adv_channel_map;
    const bool free_on_end = (p_advertiser->internal.tx_evt.transmits == 1);

    packet_tx(p_advertiser->internal.tx_evt.p_packet, &ch_map, free_on_end);

    if (ch_map != 0 && p_advertiser->internal.tx_evt.transmits != BEARER_ADV_REPEAT_INFINITE)
    {
        p_advertiser->internal.tx_evt.transmits--;
        if (free_on_end)
        {
            /* The packet will be freed in the radio TX callback. */
            p_advertiser->internal.tx_evt.p_packet = NULL;
        }
    }

    /* schedule next adv event */
    if (p_advertiser->internal.tx_evt.transmits > 0 || !fifo_is_empty(&p_advertiser->internal.tx_fifo))
    {
        p_advertiser->internal.advertisement_in_progress = true;

        /* Schedule next event according to regular advertisement intervals. */
        uint32_t rand_offset = 0;

        if (p_advertiser->adv_int_max_ms > p_advertiser->adv_int_min_ms)
        {
            rand_offset = rand_prng_get(&p_advertiser->internal.adv_event_prng)
                % ((p_advertiser->adv_int_max_ms - p_advertiser->adv_int_min_ms) * 1000);
        }

        /* Set the interval, will make the timer event jump back into the queue with the given offset. */
        p_advertiser->internal.adv_evt.interval = p_advertiser->adv_int_min_ms * 1000 + rand_offset;
    }
    else if (fifo_is_empty(&p_advertiser->internal.tx_fifo))
    {
        /* Won't automatically go back into the queue if interval is 0: */
        p_advertiser->internal.adv_evt.interval = 0;

        if (p_advertiser->queue_empty_cb != NULL)
        {
            p_advertiser->queue_empty_cb(p_advertiser);
        }
    }
}
/*****************************************************************************
* Interface functions
*****************************************************************************/

uint32_t bearer_adv_init(bearer_scan_config_t* p_config)
{
    uint32_t error_code = bearer_adv_reconfig(p_config);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    m_scan_ch = 36;
    m_is_scanning = false;

    memset(&m_scan_window_end_evt, 0, sizeof(timer_event_t));
    memset(&m_scan_evt, 0, sizeof(timer_event_t));

    radio_init_params_t radio_params =
        {
            .radio_mode     = RADIO_MODE_BLE_1MBIT,
            .access_address = BEARER_ADV_ACCESS_ADDR,
            .rx_cb          = radio_rx_cb,
            .tx_cb          = radio_tx_cb,
            .idle_cb        = radio_idle_cb
        };
    __LOG(LOG_SRC_BEARER, LOG_LEVEL_DBG1, "BEARER_ADV_ACCESS_ADDR: %8X\n", BEARER_ADV_ACCESS_ADDR);
    radio_init(&radio_params);

#if !HOST
    if (!is_valid_gap_addr(&m_default_gap_addr))
    {
        ble_gap_addr_t ficr_gap_addr;
        bearer_adv_addr_get(&ficr_gap_addr);
        bearer_adv_gap_type_set(&ficr_gap_addr);
        (void) bearer_adv_addr_default_set(&ficr_gap_addr);
    }
#endif

    return NRF_SUCCESS;
}

void bearer_adv_advertiser_init(advertiser_t* p_adv)
{
    NRF_MESH_ASSERT(p_adv != NULL);
    NRF_MESH_ASSERT(p_adv->internal.advertisement_in_progress == false);
    NRF_MESH_ASSERT(fifo_get_len(&p_adv->internal.tx_fifo) == 0);

    p_adv->internal.tx_fifo.elem_size = sizeof(tx_packet_t);
    p_adv->internal.tx_fifo.elem_array = p_adv->internal.tx_fifo_buffer;
    p_adv->internal.tx_fifo.array_len = BEARER_TX_QUEUE_LENGTH;
    fifo_init(&p_adv->internal.tx_fifo);

    rand_prng_seed(&p_adv->internal.adv_event_prng);

    p_adv->internal.tx_evt.p_packet = NULL;
    p_adv->internal.tx_evt.transmits = 0;
    p_adv->internal.advertisement_in_progress = false;

    p_adv->internal.adv_evt.cb = adv_evt_timeout;
    p_adv->internal.adv_evt.p_context = p_adv;

    memcpy(&p_adv->internal.ble_adv_addr.addr[0], &m_default_gap_addr.addr[0], BLE_GAP_ADDR_LEN);
    p_adv->internal.ble_adv_addr.addr_type = m_default_gap_addr.addr_type;

    __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "Advertiser address: %.02x:%.02x:%.02x:%.02x:%.02x:%.02x\n",
        p_adv->internal.ble_adv_addr.addr[5], p_adv->internal.ble_adv_addr.addr[4], p_adv->internal.ble_adv_addr.addr[3],
        p_adv->internal.ble_adv_addr.addr[2], p_adv->internal.ble_adv_addr.addr[1], p_adv->internal.ble_adv_addr.addr[0]);
}

uint32_t bearer_adv_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt)
{
    switch (id)
    {
        case NRF_MESH_OPT_RADIO_MODE:
            radio_mode_set((uint8_t) p_opt->opt.radio_mode);
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_ACCESS_ADDR:
            radio_access_addr_set(p_opt->opt.val);
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_CHMAP:
            if (p_opt->opt.chmap.adv_chmap != 0x00)
            {
                m_scan_config.scan_channel_map = p_opt->opt.chmap.adv_chmap;
                return NRF_SUCCESS;
            }
            else
            {
                return NRF_ERROR_INVALID_PARAM;
            }

        case NRF_MESH_OPT_RADIO_SCAN_INT_MS:
            if ((p_opt->opt.val < BEARER_ADV_SCAN_INT_MIN) ||
                (p_opt->opt.val > BEARER_ADV_SCAN_INT_MAX))
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_scan_config.scan_int_ms = p_opt->opt.val;
            bearer_adv_scan_start();
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_SCAN_WINDOW_MS:
            if ((p_opt->opt.val < BEARER_ADV_SCAN_INT_MIN) ||
                (p_opt->opt.val > m_scan_config.scan_int_ms))
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_scan_config.scan_window_ms = p_opt->opt.val;
            bearer_adv_scan_start();
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_TX_POWER:
            radio_tx_power_set((radio_tx_power_t) p_opt->opt.val);
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_AD_TYPE_FILTERING:
            bearer_adtype_filtering_set( (bool) p_opt->opt.val);
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_AD_TYPE_ADD:
            bearer_adtype_add((uint8_t) p_opt->opt.val);
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_AD_TYPE_REMOVE:
            bearer_adtype_remove((uint8_t) p_opt->opt.val);
            return NRF_SUCCESS;

        default:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_ERROR, "Unknown option %d...", id); /* TODO: Assertable? */
            return NRF_ERROR_INVALID_PARAM;
    }
}

uint32_t bearer_adv_reconfig(bearer_scan_config_t* p_config)
{
    if (p_config == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((p_config->scan_channel_map & ~BLE_ADV_CHANNEL_MAP_MASK) ||
        (p_config->scan_channel_map == 0) ||
        (p_config->scan_int_ms < BEARER_ADV_SCAN_INT_MIN) ||
        (p_config->scan_int_ms > BEARER_ADV_SCAN_INT_MAX) ||
        (p_config->scan_window_ms < BEARER_ADV_SCAN_INT_MIN) ||
        (p_config->scan_window_ms > BEARER_ADV_SCAN_INT_MAX) ||
        (p_config->scan_window_ms > p_config->scan_int_ms)
        )
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    memcpy(&m_scan_config, p_config, sizeof(bearer_scan_config_t));

    return NRF_SUCCESS;
}

uint32_t bearer_adv_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt)
{
    switch (id)
    {
        case NRF_MESH_OPT_RADIO_MODE:
            p_opt->opt.radio_mode = (nrf_mesh_radio_mode_t) radio_mode_get();
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_ACCESS_ADDR:
            p_opt->opt.val = radio_access_addr_get();
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_CHMAP:
            p_opt->opt.chmap.adv_chmap = m_scan_config.scan_channel_map & 0x07;
            p_opt->opt.chmap.ext_chmap = 0;
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_SCAN_INT_MS:
            p_opt->opt.val = m_scan_config.scan_int_ms;
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_SCAN_WINDOW_MS:
            p_opt->opt.val = m_scan_config.scan_window_ms;
            return NRF_SUCCESS;

        case NRF_MESH_OPT_RADIO_TX_POWER:
            p_opt->opt.val = radio_tx_power_get();
            return NRF_SUCCESS;

        default:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_ERROR, "Unknown option %d...", id); /* TODO: Assertable? */
            return NRF_ERROR_INVALID_PARAM;
    }
}


bool bearer_adv_available(advertiser_t* p_adv)
{
    return !fifo_is_full(&p_adv->internal.tx_fifo);
}

uint32_t bearer_adv_tx(advertiser_t* p_adv, packet_t* p_packet, uint8_t transmits)
{
    uint32_t error_code;
    tx_packet_t tx_evt;
    tx_evt.p_packet = p_packet;
    tx_evt.transmits = transmits;

    NRF_MESH_ASSERT(p_packet->header.length <= 37);

    error_code = fifo_push(&p_adv->internal.tx_fifo, &tx_evt);
    if (error_code == NRF_SUCCESS && !p_adv->internal.advertisement_in_progress)
    {
        p_adv->internal.advertisement_in_progress = true;
        next_adv_schedule(p_adv);
    }

    return (error_code == NRF_ERROR_NO_MEM) ? NRF_ERROR_BUSY : error_code ;
}

uint32_t bearer_adv_tx_raw(packet_t* p_packet, uint8_t* p_channel_map)
{
    if (p_packet == NULL || p_channel_map == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((*p_channel_map & BLE_ADV_CHANNEL_MAP_MASK) == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    /* Transmit on all channels */
    packet_tx(p_packet, p_channel_map, true);
    if (*p_channel_map == 0)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        return NRF_SUCCESS;
    }
}

uint32_t bearer_adv_adv_stop(advertiser_t* p_adv)
{
    if (p_adv->internal.advertisement_in_progress)
    {
        timer_sch_abort(&p_adv->internal.adv_evt);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_STATE;
}

void bearer_adv_adv_start(advertiser_t* p_adv)
{
    if (!p_adv->internal.advertisement_in_progress && !fifo_is_empty(&p_adv->internal.tx_fifo))
    {
        p_adv->internal.advertisement_in_progress = true;
        next_adv_schedule(p_adv);
    }
}

void bearer_adv_flush_tx(advertiser_t* p_adv)
{
    (void) bearer_adv_adv_stop(p_adv);

    /* Prevent advertisement timer from triggering while we flush */
    bearer_event_critical_section_begin();

    p_adv->internal.advertisement_in_progress = false;

    /* If remaining transmits is 0, the tx event packet either isn't present, or
     * it's in the process of being transmitted. Either way, we shouldn't
     * decref it here. */
    if (p_adv->internal.tx_evt.p_packet && p_adv->internal.tx_evt.transmits > 0)
    {
        packet_t* p_packet = p_adv->internal.tx_evt.p_packet;
        p_adv->internal.tx_evt.p_packet = NULL;
        packet_mgr_free(p_packet);
        p_adv->internal.tx_evt.transmits = 0;
    }
    tx_packet_t tx_packet;
    while (fifo_pop(&p_adv->internal.tx_fifo, &tx_packet) == NRF_SUCCESS)
    {
        packet_mgr_free(tx_packet.p_packet);
    }
    bearer_event_critical_section_end();
}

uint32_t bearer_adv_interval_reset(advertiser_t * p_adv)
{
    if (p_adv == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (!p_adv->internal.advertisement_in_progress)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        timer_sch_reschedule(&p_adv->internal.adv_evt, timer_now() + p_adv->adv_int_min_ms * 1000);
        return NRF_SUCCESS;
    }
}

void bearer_adv_scan_start(void)
{
    m_is_scanning = false;
    timestamp_t start_time = timer_now() + 1000;
    m_scan_evt.interval = 1000 * m_scan_config.scan_int_ms;
    m_scan_evt.cb = scan_begin_timer_cb;
    timer_sch_reschedule(&m_scan_evt, start_time);

    if (m_scan_config.scan_int_ms != m_scan_config.scan_window_ms)
    {
        m_scan_window_end_evt.interval = 1000 * m_scan_config.scan_int_ms;
        m_scan_window_end_evt.cb = scan_end_timer_cb;
        timer_sch_reschedule(&m_scan_window_end_evt, start_time + m_scan_config.scan_window_ms * 1000);
    }
}

void bearer_adv_scan_stop(void)
{
    timer_sch_abort(&m_scan_evt);
    timer_sch_abort(&m_scan_window_end_evt);
    scan_end_timer_cb(0, NULL);
}

void bearer_adv_addr_get(ble_gap_addr_t* p_addr)
{
#if !HOST
    p_addr->addr_type = !!(NRF_FICR->DEVICEADDRTYPE);
    memcpy(p_addr->addr, (uint8_t*) NRF_FICR->DEVICEADDR, BLE_GAP_ADDR_LEN);
    bearer_adv_gap_type_set(p_addr);
#endif
}

void bearer_adv_gap_type_set(ble_gap_addr_t* p_addr)
{
    NRF_MESH_ASSERT(p_addr != NULL);

    switch (p_addr->addr_type)
    {
        case BLE_GAP_ADDR_TYPE_PUBLIC:
        case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
            /* The two most significant bits of the address shall be equal to 1 */
            p_addr->addr[5] = 0xC0 | p_addr->addr[5];
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
            /* The two most significant bits of the address shall be equal to 0 */
            p_addr->addr[5] = 0x3F & p_addr->addr[5];
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
            /* The two most significant bits of the address shall be equal to 0b10 */
            p_addr->addr[5] = 0xBF & p_addr->addr[5];
            p_addr->addr[5] = 0x40 | p_addr->addr[5];
            break;
        default:
            NRF_MESH_ASSERT(false);
    }
}

uint32_t bearer_adv_addr_default_set(const ble_gap_addr_t* p_addr)
{
    if (!p_addr)
    {
        return NRF_ERROR_NULL;
    }
    else if (!is_valid_gap_addr(p_addr))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    else
    {
        memcpy(m_default_gap_addr.addr, p_addr->addr, BLE_GAP_ADDR_LEN);
        m_default_gap_addr.addr_type = p_addr->addr_type;
        return NRF_SUCCESS;
    }
}

uint32_t bearer_adv_addr_set(advertiser_t* p_adv, const ble_gap_addr_t* p_addr)
{
    if (!p_adv || !p_addr)
    {
        return NRF_ERROR_NULL;
    }
    else if (!is_valid_gap_addr(p_addr))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    else
    {
        memcpy(p_adv->internal.ble_adv_addr.addr, p_addr->addr, BLE_GAP_ADDR_LEN);
        p_adv->internal.ble_adv_addr.addr_type = p_addr->addr_type;
        return NRF_SUCCESS;
    }
}

