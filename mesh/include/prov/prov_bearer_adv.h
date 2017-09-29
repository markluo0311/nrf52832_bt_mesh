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

#ifndef PROV_BEARER_ADV_H__
#define PROV_BEARER_ADV_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bearer_adv.h"
#include "timer_scheduler.h"
#include "prov_bearer.h"
#include "prov_pdu.h"

/**
 * @defgroup PROV_BEARER_ADV Provisioning over Advertising Bearer (PB-ADV)
 * @ingroup MESH_API_GROUP_PROV
 * This module provides support for the provisioning over advertising (PB-ADV)
 * provisioning bearer. PB-ADV is used for provisioning devices within radio
 * range of the provisioner.
 * @{
 */

/** PB-ADV state. */
typedef enum
{
    PROV_BEARER_ADV_STATE_IDLE,         /**< Ready for a new session. */
    PROV_BEARER_ADV_STATE_LISTEN,       /**< Listen for incoming link open requests. */
    PROV_BEARER_ADV_STATE_LINK_OPENING, /**< Link open message sent, awaiting link ack. */
    PROV_BEARER_ADV_STATE_LINK_OPEN,    /**< Link is open, and ready for a provisioning process. */
    PROV_BEARER_ADV_STATE_LINK_CLOSING, /**< Link is being closed, waiting for close packets to finish sending. */
} prov_bearer_adv_state_t;

/** PB-ADV instance state. */
typedef enum
{
    PROV_BEARER_ADV_INSTANCE_RELEASED,     /**< Bearer has successfully closed a link. */
    PROV_BEARER_ADV_INSTANCE_INITIALIZED  /**< Bearer instance has been initialized. */
} prov_bearer_adv_instance_t;

/** PB-ADV data buffer state. */
typedef enum
{
    PROV_BEARER_ADV_BUF_STATE_UNUSED, /**< The buffer is currently unused. */
    PROV_BEARER_ADV_BUF_STATE_RX,     /**< The buffer is being used for receiving a packet. */
    PROV_BEARER_ADV_BUF_STATE_TX,     /**< The buffer is being used for transmitting a packet. */
} prov_bearer_adv_buf_state_t;

/** PB-ADV data buffer. */
typedef struct
{
    prov_bearer_adv_buf_state_t state;                        /**< State of this buffer. */
    uint8_t                     length;                       /**< Length of the payload. */
    uint8_t                     fcs;                          /**< FCS for the payload. */
    uint8_t                     finished_segments;            /**< Finished segments. Used as a counter on TX,
                                                                    and a bitfield on RX. */
    uint8_t                     payload[PROV_PDU_MAX_LENGTH]; /**< Payload of the buffer. */
} prov_bearer_adv_buffer_t;

/** PB-ADV link. */
typedef struct prov_bearer_adv_t
{
    advertiser_t               advertiser;          /**< Advertiser instance. */
    uint32_t                   link_id;             /**< PB-ADV link identifier. */
    prov_bearer_adv_instance_t instance_state;      /**< State identifier for the struct instance. */
    prov_bearer_adv_state_t    state;               /**< State identifier for the PB-ADV state machine. */
    uint8_t                    transaction_in;      /**< Transaction number for the incoming messages. */
    uint8_t                    transaction_out;     /**< Transaction number for the outgoing messages. */
    nrf_mesh_prov_link_close_reason_t close_reason; /**< Reason for closing the link. */
    timer_event_t              timeout_event;       /**< State structure for the timeout timer. */
    timer_event_t              link_timeout_event;  /**< State structure for the link timeout timer. */
    uint32_t                   sar_timeout;         /**< SAR timeout value. */
    uint32_t                   link_timeout;        /**< Link timeout value. */
    prov_bearer_adv_buffer_t   buffer;              /**< Buffer structure for ongoing transfer. */

    struct prov_bearer_adv_t * p_next;              /**< Pointer to the next active PB-ADV link. */
} prov_bearer_adv_t;

/*
 * Including this file here is an ugly workaround for CMock bug #85; CMock needs the size of
 * the prov_bearer_t type to generate some mock functions, and without this, prov_bearer_t
 * is an incomplete type.
 */
#include "provisioning.h"

/**
 * Gets the provisioning bearer interface functions for the PB-ADV bearer.
 * @return A pointer to the bearer interface for the PB-ADV bearer.
 */
const prov_bearer_interface_t * prov_bearer_adv_interface_get(void);

/**
 * Process an incoming advertisement packet.
 *
 * @param[in] p_incoming Pointer to the incoming packet, as an AD data structure.
 */
void prov_bearer_adv_pkt_in(const ble_ad_data_t * p_incoming);

/** @} */

#endif

