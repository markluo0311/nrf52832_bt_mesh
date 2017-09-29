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
#include "radio.h"
#include "radio_config.h"
#include "fifo.h"

#include "nrf.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"

#include <stdbool.h>
#include <string.h>

/** Number of elements in radio event queue. */
#define RADIO_FIFO_QUEUE_SIZE           (8) /* must be power of two */
/** Max packet length field in radio */
#define RADIO_PACKET_MAXLEN             (37)
/** Byte offset of packet length field */
#define PACKET_FIELD_SIZE_OFFSET        (1)


/**
 * Internal enum denoting radio state.
 */
typedef enum
{
    RADIO_STATE_RX,         /**< RX event in progress. */
    RADIO_STATE_TX,         /**< TX event in progress. */
    RADIO_STATE_DISABLED,   /**< No radio event in progress. */
    RADIO_STATE_STOPPED,    /**< Radio operation has been stopped, and must be resumed before new event can be executed */
    RADIO_STATE_NEVER_USED  /**< Radio module has not been initialized (ever). */
} radio_state_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
/** Global radio state */
static radio_config_t m_radio_config;

static radio_state_t m_radio_state = RADIO_STATE_NEVER_USED;

/** Radio event FIFO */
static fifo_t m_radio_fifo;
/** Radio event FIFO buffer */
static radio_event_t m_radio_fifo_queue[RADIO_FIFO_QUEUE_SIZE];

/** Idle callback function pointer */
static radio_idle_cb_t m_idle_cb;
/** RX callback function pointer */
static radio_rx_cb_t m_rx_cb;
/** TX callback function pointer */
static radio_tx_cb_t m_tx_cb;

/*****************************************************************************
* Static functions
*****************************************************************************/
static bool purge_preemptable(void)
{
    radio_event_t current_evt;
    bool preemptable = (fifo_peek(&m_radio_fifo, &current_evt) == NRF_SUCCESS &&
                        current_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE);
    if (preemptable)
    {
        /* event is preemptable, stop it */
        NRF_MESH_ASSERT(fifo_pop(&m_radio_fifo, NULL) == NRF_SUCCESS);

        radio_stop();
        while (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled); /*lint !e722 Suspicious use of semi-colon */
        NRF_RADIO->EVENTS_END = 0;

        /* propagate failed rx event */
        m_rx_cb(current_evt.p_packet, false, 0xFFFFFFFF, 0);
    }
    return preemptable;
}

static void setup_event(radio_event_t* p_evt)
{
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk
                      | RADIO_SHORTS_END_DISABLE_Msk
                      | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    radio_config_config(&m_radio_config);
    radio_config_channel_set(p_evt->channel);
    NRF_RADIO->PACKETPTR = (uint32_t) p_evt->p_packet;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NRF_RADIO->EVENTS_END = 0;

    if (p_evt->event_type == RADIO_EVENT_TYPE_TX)
    {
        NRF_RADIO->TASKS_TXEN = 1;
        m_radio_state = RADIO_STATE_TX;
    }
    else
    {
        NRF_RADIO->TASKS_RXEN = 1;
        m_radio_state = RADIO_STATE_RX;
    }
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void radio_init(const radio_init_params_t * p_init_params)
{
    m_radio_fifo.array_len   = RADIO_FIFO_QUEUE_SIZE;
    m_radio_fifo.elem_array  = m_radio_fifo_queue;
    m_radio_fifo.elem_size   = sizeof(radio_event_t);
    fifo_init(&m_radio_fifo);

    m_radio_config.access_addr = p_init_params->access_address;
    m_radio_config.tx_power = RADIO_POWER_NRF_0DBM;
    m_radio_config.payload_maxlen = RADIO_PACKET_MAXLEN;
    m_radio_config.datarate = p_init_params->radio_mode;

    m_rx_cb       = p_init_params->rx_cb;
    m_tx_cb       = p_init_params->tx_cb;
    m_idle_cb     = p_init_params->idle_cb;

    m_radio_state = RADIO_STATE_NEVER_USED;
}

void radio_mode_set(uint8_t radio_mode)
{
    /* Default to BLE 1Mbit if parameter is invalid */
    m_radio_config.datarate = (radio_mode > 3) ? RADIO_MODE_BLE_1MBIT : (radio_mode_t)radio_mode;
}

void radio_access_addr_set(uint32_t access_address)
{
    m_radio_config.access_addr = access_address;
}

void radio_tx_power_set(radio_tx_power_t tx_power)
{
    m_radio_config.tx_power = tx_power;
}

uint8_t radio_mode_get(void)
{
    return m_radio_config.datarate;
}

uint32_t radio_access_addr_get(void)
{
    return m_radio_config.access_addr;
}

radio_tx_power_t radio_tx_power_get(void)
{
    return m_radio_config.tx_power;
}

uint32_t radio_order(radio_event_t* p_radio_events, uint32_t* p_count)
{
    if (p_radio_events == NULL || p_count == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (*p_count == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (m_radio_state == RADIO_STATE_NEVER_USED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    /* Reserve spots in the fifo, so we can know when we're pushing the last
     * event in the input array before it already happened. If a fifo_push
     * fails, it's already too late to tell the event before it that it is the
     * last event in the series. */
    static uint32_t s_fifo_reserved = 0;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    uint32_t status = NRF_SUCCESS;
    uint32_t count = *p_count;

    const uint32_t fifo_available = RADIO_FIFO_QUEUE_SIZE - fifo_get_len(&m_radio_fifo);
    if (fifo_available <= s_fifo_reserved)
    {
        count = 0;
        status = NRF_ERROR_NO_MEM;
    }

    if (count > fifo_available - s_fifo_reserved)
    {
        count = fifo_available - s_fifo_reserved;
    }
    const uint32_t reserved_slots = count;
    s_fifo_reserved += reserved_slots;
    _ENABLE_IRQS(was_masked);

    for (uint32_t i = 0; i < count; ++i)
    {
        if (p_radio_events[i].channel >= 40 ||
            p_radio_events[i].p_packet == NULL ||
            (p_radio_events[i].event_type != RADIO_EVENT_TYPE_RX &&
             p_radio_events[i].event_type != RADIO_EVENT_TYPE_TX &&
             p_radio_events[i].event_type != RADIO_EVENT_TYPE_RX_PREEMPTABLE))
        {
            status = NRF_ERROR_INVALID_PARAM;
            count = i;
            break;
        }

        p_radio_events[i]._last_in_series = (i == (count - 1));
        NRF_MESH_ASSERT(fifo_push(&m_radio_fifo, &p_radio_events[i]) == NRF_SUCCESS);
    }

    /* Free the reserved slots, since they've been transferred into the queue. */
    _DISABLE_IRQS(was_masked);
    s_fifo_reserved -= reserved_slots;
    _ENABLE_IRQS(was_masked);

    *p_count = count;
    return status;
}

void radio_invoke(void)
{
    /* trigger radio callback */
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (m_radio_state != RADIO_STATE_STOPPED)
    {
#if !HOST
        NVIC_SetPendingIRQ(RADIO_IRQn);
#endif
    }

    _ENABLE_IRQS(was_masked);
}

void radio_stop(void)
{
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->TASKS_DISABLE = 1;
    m_radio_state = RADIO_STATE_STOPPED;
}

void radio_resume(void)
{
    if (m_radio_state != RADIO_STATE_STOPPED)
    {
        return;
    }

    m_radio_state = RADIO_STATE_DISABLED;

    if (fifo_is_empty(&m_radio_fifo))
    {
        m_idle_cb();
    }
    else
    {
#if !HOST
        NVIC_SetPendingIRQ(RADIO_IRQn);
#endif
    }
}

void radio_preemptable_cancel(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (m_radio_state == RADIO_STATE_RX)
    {
        while (purge_preemptable()); /*lint !e722 Suspicious use of semicolon */
    }
    m_radio_state = RADIO_STATE_DISABLED;
    _ENABLE_IRQS(was_masked);
}

void radio_event_handler(void)
{
    if (NRF_RADIO->EVENTS_END)
    {
        bool crc_status = NRF_RADIO->CRCSTATUS;
        uint32_t crc = NRF_RADIO->RXCRC;
        uint8_t rssi = 100;

        if (NRF_RADIO->EVENTS_RSSIEND)
        {
            NRF_RADIO->EVENTS_RSSIEND = 0;
            rssi = NRF_RADIO->RSSISAMPLE & 0xff;
        }

        radio_event_t prev_evt;
        NRF_RADIO->EVENTS_END = 0;

        /* pop the event that just finished */
        if (fifo_pop(&m_radio_fifo, &prev_evt) != NRF_SUCCESS)
        {
            radio_stop();
            return;
        }

        /* send to super space */
        if (prev_evt.event_type == RADIO_EVENT_TYPE_RX ||
            prev_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            m_rx_cb(prev_evt.p_packet, crc_status, crc, rssi);
        }
        else
        {
            m_tx_cb(prev_evt.p_packet, prev_evt._last_in_series && prev_evt.free_on_end);
        }

        m_radio_state = RADIO_STATE_DISABLED;
    }
    else
    {
        while ((fifo_get_len(&m_radio_fifo) > 1) && purge_preemptable()) {}
    }

    if (m_radio_state == RADIO_STATE_DISABLED ||
        m_radio_state == RADIO_STATE_NEVER_USED ||
        m_radio_state == RADIO_STATE_STOPPED)
    {
        radio_event_t evt;
        if (fifo_peek(&m_radio_fifo, &evt) == NRF_SUCCESS)
        {
            setup_event(&evt);
        }
        else
        {
            m_idle_cb();
        }
    }
}

void radio_on_ts_begin(void)
{
    radio_config_reset();
    radio_config_config(&m_radio_config);

    m_radio_state = RADIO_STATE_DISABLED;

#if !HOST
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
#endif

    if (fifo_is_empty(&m_radio_fifo))
    {
        m_idle_cb();
    }
    else
    {
#if !HOST
        NVIC_SetPendingIRQ(RADIO_IRQn);
#endif
    }
}

void radio_on_ts_end(void)
{
    radio_stop();
}

