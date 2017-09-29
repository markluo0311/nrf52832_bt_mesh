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
#ifndef MESH_RADIO_H__
#define MESH_RADIO_H__
#include <stdint.h>
#include <stdbool.h>

#include "radio_config.h"

/**
 * @defgroup RADIO Radio hardware abstraction
 * @ingroup MESH_CORE
 * Provides an async control interface for the radio HW module.
 * @{
 */

/** Callback function type for completed radio rx-event */
typedef void (*radio_rx_cb_t)(uint8_t* p_data, bool successful, uint32_t crc, int8_t rssi);

/** Callback function type for completed radio tx-event */
typedef void (*radio_tx_cb_t)(uint8_t* p_data, bool free_packet);

/** Callback function type for when the radio is out of things to do */
typedef void (*radio_idle_cb_t)(void);

/**
 * Types of schedulable radio events.
 */
typedef enum
{
    RADIO_EVENT_TYPE_TX,            /**< Transmit a packet. */
    RADIO_EVENT_TYPE_RX,            /**< Receive a packet. */
    RADIO_EVENT_TYPE_RX_PREEMPTABLE /**< Receive a packet, but yield if a new event is queued. */
} radio_event_type_t;

/** Schedulable radio event type. */
typedef struct
{
    radio_event_type_t event_type;  /**< Event type to be scheduled. */
    uint8_t * p_packet;             /**< Packet pointer to transmit or receive into. */
    uint8_t channel;                /**< Channel to execute event on. */
    bool free_on_end;               /**< Only used for TX. If set, the radio will set the free_packet flag for the callback in the last callback in the series. */

    /** Internal parameter that will be ignored when ordering. Used to keep
     * track of which event that will trigger the last_tx flag in the TX
     * callback. Only used for TX events. */
    bool _last_in_series;
} radio_event_t;

/** Radio initialization parameter structure. */
typedef struct
{
    /** Radio mode (250kbit, 500kbit, 1Mbit or 2Mbit). */
    radio_mode_t    radio_mode;
    /** Access address to use for all transmissions and receptions. */
    uint32_t        access_address;
    /** Function pointer to a callback for when the radio has finished a receive. */
    radio_rx_cb_t   rx_cb;
    /** Function pointer to a callback for when the radio has finished a transmission. */
    radio_tx_cb_t   tx_cb;
    /** Function pointer to a callback for when the radio runs out of queued events. */
    radio_idle_cb_t idle_cb;
} radio_init_params_t;

/**
 * Initialize the radio hardware.
 *
 * @param[in] p_init_params Initialization parameter structure, see @c
 *                          radio_init_params_t.
 */
void radio_init(const radio_init_params_t * p_init_params);

/**
 * Set radio mode.
 *
 * @param[in] radio_mode Radio mode. Possible values:
 *                       - RADIO_MODE_MODE_Nrf_1Mbit
 *                       - RADIO_MODE_MODE_Nrf_2Mbit
 *                       - RADIO_MODE_MODE_Nrf_250Kbit
 *                       - RADIO_MODE_MODE_Ble_1Mbit
 */
void radio_mode_set(uint8_t radio_mode);

/**
 * Set access address.
 *
 * @param[in] access_address Access address.
 */
void radio_access_addr_set(uint32_t access_address);

/**
 * Set tx power.
 *
 * @param[in] tx_power tx power.
 */
void radio_tx_power_set(radio_tx_power_t tx_power);

/**
 * Schedule a radio event.
 *
 * @param[in] p_radio_events Array of user-created radio events to be queued.
 * The radio makes its own copy of the events, but the packet pointer remains
 * the same.
 * @param[in,out] p_count The number of events in the event array. Will be set
 * to reflect the number of events that actually got queued.
 *
 * @retval NRF_SUCCESS              The event was successfully queued.
 * @retval NRF_ERROR_INVALID_PARAM  The value of the @p p_count parameter was
 * 0.
 * @retval NRF_ERROR_NULL           One or more of the parameters were NULL.
 * @retval NRF_ERROR_NO_MEM         The radio queue is full.
 * @retval NRF_ERROR_INVALID_STATE  The radio has not been initialized.
 */
uint32_t radio_order(radio_event_t* p_radio_events, uint32_t* p_count);

/**
 * Invoke the radio interrupt, causing the radio to update its state, and
 * process any outstanding events.
 */
void radio_invoke(void);

/**
 * Get current radio mode.
 *
 * @return Current radio mode.
 */
uint8_t radio_mode_get(void);

/**
 * Get current access address.
 *
 * @return Current access address.
 */
uint32_t radio_access_addr_get(void);

/**
 * Get current tx power.
 *
 * @return Current tx power.
 */
radio_tx_power_t radio_tx_power_get(void);

/**
 * Disable the radio. Stops any ongoing rx or tx procedures, but does not pop the off the
 *        queue, and they will be attempted once the radio is resumed.
 */
void radio_stop(void);

/** Cancel any ongoing preemptable radio events */
void radio_preemptable_cancel(void);

/** Resume radio operation after a stop. */
void radio_resume(void);

/** Radio event handler, checks any relevant events generated by the radio, and acts accordingly. */
void radio_event_handler(void);

/** Callback for timeslot handler to initiate radio hardware when timeslot begins. */
void radio_on_ts_begin(void);

/** Callback for timeslot handler to disable radio hardware when timeslot ends. */
void radio_on_ts_end(void);

/** @} */

#endif /* RADIO_H__ */
