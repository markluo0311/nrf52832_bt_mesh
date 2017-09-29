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
#ifndef BEARER_ADV_H__
#define BEARER_ADV_H__

#include <stdint.h>
#include <stdbool.h>
#include "fifo.h"
#include "rand.h"
#include "bearer.h"
#include "packet.h"
#include "timer_scheduler.h"
#include "nrf_mesh_opt.h"
#include "ble_gap.h"
#include "nrf_mesh_config_core.h"

/**
 * @defgroup BEARER_ADV Advertisement bearer module
 * @ingroup MESH_CORE
 * Controls the radio module to transmit and receive bearer packets.
 * @{
 */

/** Scanner configuration parameters */
typedef struct
{
    uint8_t             scan_channel_map;   /**< Bitfield for channels to scan on, starting at 37 */
    uint16_t            scan_int_ms;        /**< Time between the start of two subsequent scans. */
    uint16_t            scan_window_ms;     /**< Duration of a single scan. */
    bearer_rx_cb_t      rx_cb;              /**< RX callback function. */
} bearer_scan_config_t;

/**
* TX packet structure, containing info about how a packet should be transmitted. For internal use
* only.
*/
typedef struct
{
    packet_t*           p_packet;   /**< Pointer to the network packet to transmit */
    uint8_t             transmits;  /**< Number of transmits this packet should do. */
} tx_packet_t;

/** Forward declaration. See @ref __advertiser_t. */
typedef struct __advertiser_t advertiser_t;

/**
 * Callback function for when the advertiser queue is empty.
 */
typedef void (*bearer_adv_queue_empty_cb_t)(advertiser_t * p_ctx);

/**
 * Advertiser instance. Internal parameters will be overwritten by module, and should not be
 * altered by the user.
 */
struct __advertiser_t
{
    /** Lowest advertisement interval in milliseconds. */
    uint32_t            adv_int_min_ms;
    /** Highest advertisement interval in milliseconds. */
    uint32_t            adv_int_max_ms;
    /** Packet type to use for outgoing packets. */
    ble_packet_type_t   adv_packet_type;
    /** Bitfield for channels to use, starting at 37. */
    uint8_t             adv_channel_map;

    /** Function to call if the advertiser queue is empty. */
    bearer_adv_queue_empty_cb_t queue_empty_cb;

    /** Internal params, will be overwritten by @ref bearer_adv_advertiser_init(), and should not
      be altered directly from user space.  */
    struct
    {
        /** Fifo for outgoing packets. */
        fifo_t              tx_fifo;
        /** Fifo buffer for outgoing packets. */
        tx_packet_t         tx_fifo_buffer[BEARER_TX_QUEUE_LENGTH];
        /** Current TX event. */
        tx_packet_t         tx_evt;
        /** PRNG instance for generating random timeoffsets for the advertisements. */
        prng_t              adv_event_prng;
        /** Timer event for periodic advertisement. */
        timer_event_t       adv_evt;
        /** Local advertisement address. */
        ble_gap_addr_t      ble_adv_addr;
        /** Whether the advertiser has never fired. */
        bool                awaiting_first_tx;
        /** Currently advertising. */
        bool                advertisement_in_progress;
    } internal;
};

/**
 * Initialize advertisement bearer module.
 *
 * @param[in] p_config Structure of scanner configuration parameters. See
 *            @ref bearer_scan_config_t.
 *
 * @retval NRF_SUCCESS The bearer was successfully initialized.
 * @retval NRF_ERROR_NULL The configuration struct is NULL.
 * @retval NRF_ERROR_INVALID_PARAM One or more of the parameters in the configuration structure does
 *         not meet their requirements.
 */
uint32_t bearer_adv_init(bearer_scan_config_t* p_config);

/**
 * Initialize an advertiser instance.
 *
 * @param[in,out] p_adv Advertiser structure to initialize. All fields will be initialized.
 */
void bearer_adv_advertiser_init(advertiser_t* p_adv);

/**
 * Reconfigure advertisement bearer module.
 *
 * @param[in] p_config Structure of configuration parameters for the scanner. See
 *            @ref bearer_scan_config_t.
 *
 * @retval NRF_SUCCESS The bearer was successfully initialized.
 * @retval NRF_ERROR_NULL The configuration struct is NULL.
 * @retval NRF_ERROR_INVALID_PARAM One or more of the parameters in the configuration structure does
 *         not meet their requirements.
 */
uint32_t bearer_adv_reconfig(bearer_scan_config_t* p_config);

/**
 * Set advertisement bearer options.
 *
 * @param[in] id    Identifier for option (@ref nrf_mesh_opt_id_t).
 * @param[in] p_opt Value of option to set.
 *
 * @retval NRF_SUCCESS Successfully set option.
 * @retval NRF_ERROR_NOT_SUPPORTED Setting the given parameter is not (yet)
 *                                 supported.
 * @retval NRF_ERROR_INVALID_PARAM The parameter provided was out of bounds.
 */
uint32_t bearer_adv_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Get advertisement bearer options.
 *
 * @param[in]  id    Identifier for option (@ref nrf_mesh_opt_id_t).
 * @param[out] p_opt Value of option to get.
 *
 * @retval NRF_SUCCESS Successfully retrieved option.
 * @retval NRF_ERROR_NOT_SUPPORTED Getting the given parameter is not (yet)
 *                                 supported.
 */
uint32_t bearer_adv_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/**
 * Check whether the adv bearer is ready to receive a packet for transmission.
 *
 * @param[in] p_adv Advertiser instance to check for.
 *
 * @return Whether the adv bearer has room for another packet, and the bearer may safely
 *         queue it.
 */
bool bearer_adv_available(advertiser_t* p_adv);

/**
 * Queue a packet for transmission on the serial adv-bearer. Implicitly starts advertising.
 *
 * @param[in] p_adv Advertiser instance to transmit the packet on.
 * @param[in] p_packet Pointer to a packet that is to be transmitted.
 * @param[in] transmits The number of times the packet should be transmitted, or
 *            @ref BEARER_ADV_REPEAT_INFINITE to advertise the given packet until a new packet is
 *            submitted.
 *
 * @retval NRF_SUCCESS The packet was successfully queued for transmission.
 * @retval NRF_ERROR_BUSY The tx queue was full, and the packet could not be scheduled for
 *         transmission.
 */
uint32_t bearer_adv_tx(advertiser_t* p_adv, packet_t* p_packet, uint8_t transmits);

/**
 * Transmit the given packet. The transmit is scheduled immediately, and will
 * be transmitted as soon as the radio is able to service it.
 *
 * @note The packet contents will not be altered in any way, and it's expected
 * that the user sets all fields before calling this function, including header
 * fields and advertisement address.
 *
 * @param[in] p_packet A pointer to the packet to send.
 * @param[in,out] p_channel_map A pointer to a channel map bitfield variable,
 * which will be used to set which channels the packet will be transmitted on,
 * starting at channel 37. The function will alter the variable to indicate
 * which channels it was able to schedule transmissions on. If the function
 * returns success, the output will be unaltered.
 *
 * @return NRF_SUCCESS The packet was successfully scheduled for sending on all
 * advertisement channels marked in the channel map.
 * @return NRF_ERROR_NULL One or both of the arguments are NULL.
 * @return NRF_ERROR_INVALID_PARAM The channel map does not have any valid
 * channels set.
 * @return NRF_ERROR_NO_MEM The radio was unable to schedule one or more packet
 * for transmission. The resulting channel map will indicate on which channels
 * the transmission was successful.
 */
uint32_t bearer_adv_tx_raw(packet_t* p_packet, uint8_t* p_channel_map);

/**
 * Start advertising accoring to config-parameters.
 *
 * @param[in,out] p_adv Advertiser instance to start.
 */
void bearer_adv_adv_start(advertiser_t* p_adv);

/**
 * Stop advertising. Does NOT flush TX queue.
 *
 * @param[in,out] p_adv Advertiser instance to stop. Must not be NULL or an invalid pointer.
 *
 * @retval NRF_ERROR_INVALID_STATE No advertisements in progress.
 * @retval NRF_SUCCESS The advertiser was successfully stopped.
 * @retval NRF_ERROR_NO_MEM The action could not be scheduled for processing. Allow the bearer event
 * module to process its queue.
 */
uint32_t bearer_adv_adv_stop(advertiser_t* p_adv);

/**
 * Flush all packets in the TX queue. Packets will be freed, and never
 * transmitted. Also applies to any packets currently in transmission.
 *
 * @param[in,out] p_adv Advertiser instance to flush TX queue of.
 */
void bearer_adv_flush_tx(advertiser_t* p_adv);

/**
 * Reset the advertisement interval, making the next advertisement transmit in
 * adv_int_min_ms ms from now.
 *
 * @param[in,out] p_adv Advertiser to reset interval of.
 *
 * @retval NRF_SUCCESS Successfully reset the advertisement interval.
 * @retval NRF_ERROR_NULL The @c p_adv pointer was @c NULL.
 * @retval NRF_ERROR_INVALID_STATE Tried to reset advertisement interval of an advertiser not
 * currently running. If advertising is stopped, the interval will be reset on the next TX.
 */
uint32_t bearer_adv_interval_reset(advertiser_t * p_adv);

/** Start periodic scanning according to config-parameters. */
void bearer_adv_scan_start(void);

/** Stop periodic scanning. */
void bearer_adv_scan_stop(void);

/**
 * Get hardware-stored BLE advertisement address.
 *
 * @param[out] p_addr Pointer to advertiser structure to store
 *  advertisement address into.
 */
void bearer_adv_addr_get(ble_gap_addr_t* p_addr);


/**
 * Sets the two most significant bits of the advertisement address according to the BLE address type.
 *
 * @param[in,out] p_addr BLE advertisement address to edit.
 */
void bearer_adv_gap_type_set(ble_gap_addr_t* p_addr);

/**
 * Sets the default advertising address.
 *
 * @param[in] p_addr BLE advertisement address structure to store for
 *                   the advertiser.
 *
 * @retval NRF_SUCCESS The address was successfully stored in static memory.
 * @retval NRF_ERROR_NULL Null pointer supplied to function.
 * @retval NRF_ERROR_INVALID_ADDR The address format was invalid.
 */
uint32_t bearer_adv_addr_default_set(const ble_gap_addr_t* p_addr);

/**
 * Sets the advertising address for an already initialized advertiser.
 *
 * @param[in,out] p_adv  Advertiser to set the advertisement address.
 * @param[in]     p_addr BLE advertisement address structure to store for
 *                       the advertiser.
 *
 * @retval NRF_SUCCESS The address was successfully changed in advertiser.
 * @retval NRF_ERROR_NULL Null pointer supplied to function.
 * @retval NRF_ERROR_INVALID_ADDR The address format was invalid.
 */
uint32_t bearer_adv_addr_set(advertiser_t* p_adv, const ble_gap_addr_t* p_addr);
/** @} */

#endif /* BEARER_ADV_H__ */
