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
#ifndef MESH_BEARER_H__
#define MESH_BEARER_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "nrf_mesh.h"

/**
 * @defgroup BEARER Bearer handler module
 * @ingroup MESH_CORE
 * Interfaces the different bearer types, and creates a generic interface for the layers above.
 * @{
 */

/** Forward declaration. See @ref __advertiser_t. */
typedef struct __advertiser_t advertiser_t;

/** Infinite repeated transmissions of one packet. */
#define BEARER_ADV_REPEAT_INFINITE (0xFF)

#ifndef BEARER_TYPES
    /** Specify which bearers are available to the mesh by default. */
    #define BEARER_TYPES            (BEARER_ADV_RADIO)
#endif


/** Bearer type, any combination of @ref BEARER_TYPES. */
typedef uint32_t bearer_t;

/** RX callback type for the various bearer implementations. */
typedef void (*bearer_rx_cb_t)(packet_t* p_packet, bearer_t bearer, const packet_meta_t * p_meta);

/**
 * Initialize the bearer module.
 *
 * @param p_init_params General Mesh initialization struct propagated from @c
 *                      nrf_mesh_init.
 *
 * @retval NRF_SUCCESS             The bearer was successfully initialized.
 */
uint32_t bearer_init(const nrf_mesh_init_params_t * p_init_params);


/**
 * Enable or disable AD type filtering. If enabled, only the
 * packets with the AD types added via @ref bearer_adtype_add()
 * will be kept and processed by the bearer.
 *
 * @param[in] filter AD type filtering state; set to true to enable
 *            AD Type filter, and false to disable
 *
 * @note If no AD type is added via @ref bearer_adtype_add()
 * then all advertisement packets received will be dropped.
 */
void bearer_adtype_filtering_set(bool filter);

/**
 * Remove the AD type from the list of accepted AD types.
 *
 * @param[in] type The AD type that will be removed from the filter.
 *            Data from packets with this AD type will be dropped if
 *            AD type filtering is enabled via @ref bearer_adtype_filtering_set
 *
 * @note This function has no effect if the AD type has not
 * been added.
 */
void bearer_adtype_remove(uint8_t type);

/**
 * Add the AD type to the list of accepted AD types.
 *
 * @param[in] type The AD type that will be added to the filter.
 *            Data from packets with this AD type will always
 *            be passed on to the upper layers.
 *
 * @note Ad type filtering must be enabled (via a call to
 * @ref bearer_adtype_filtering_set) for the added AD types to
 * have any effect.
 */
void bearer_adtype_add(uint8_t type);

/**
 * Set a whitelist for GAP addresses. Only packets with a GAP address entry in
 * the whitelist will be passed from the radio to the stack for processing.
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least @p addr_count length.
 *
 * @warning Changing the contents of the whitelist while it's in use may result
 * in unwanted packets being accepted. It is recommended to clear the list
 * before changing it.
 *
 * @param[in] p_addrs List of addresses to accept. Must be statically allocated.
 * @param[in] addr_count The number of addresses in the given list.
 *
 * @retval NRF_SUCCESS The whitelist was successfully set.
 * @retval NRF_ERROR_INVALID_STATE A blacklist is already in place. Clear the
 * filter before changing the type of accept criteria.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH The @p addr_count variable was 0.
 */
uint32_t bearer_filter_gap_addr_whitelist_set(const ble_gap_addr_t* const p_addrs, uint16_t addr_count);

/**
 * Set a blacklist for GAP addresses. Packets with a GAP address entry in the
 * blacklist will not be passed from the radio to the upper stack for
 * processing.
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least @p addr_count length.
 *
 * @warning Changing the contents of the blacklist while it's in use may result
 * in unwanted packets being accepted. It is recommended to clear the list
 * before changing it.
 *
 * @param[in] p_addrs List of addresses to accept. Must be statically allocated.
 * @param[in] addr_count The number of addresses in the given list.
 *
 * @retval NRF_SUCCESS The blacklist was successfully set.
 * @retval NRF_ERROR_INVALID_STATE A whitelist is already in place. Clear the
 * filter before changing the type of accept criteria.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH The @p addr_count variable was 0.
 */
uint32_t bearer_filter_gap_addr_blacklist_set(const ble_gap_addr_t* const p_addrs, uint16_t addr_count);

/**
 * Set a range for GAP addresses. Packets with a GAP address equal to or higher
 * than the first entry and a GAP address lower than the second entry will
 * pass, ie p_addrs[0] <= addr < p_addrs[1].
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least 2 entries. The addresses must have the same address
 * type.
 *
 * @warning Changing the range filter while it's in use may result in unwanted
 * packets being accepted. It is recommended to clear the filter before
 * changing it.
 *
 * @param[in] p_addrs Statically allocated array of two or more addresses, with
 * the same address type, where the first one is lower than or equal to the
 * second.
 *
 * @retval NRF_SUCCESS The blacklist was successfully set.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_DATA The addresses does not have the same address
 * type.
 * @retval NRF_ERROR_INVALID_STATE Another filter is already in place. Clear it
 * before changing the type of accept criteria.
 */
uint32_t bearer_filter_gap_addr_range_set(const ble_gap_addr_t* const p_addrs);

/**
 * Remove the currently assigned GAP address filter.
 *
 * @retval NRF_SUCCESS The current GAP address filter was removed.
 * @retval NRF_ERROR_INVALID_STATE No GAP address filter was in place.
 */
uint32_t bearer_filter_gap_addr_clear(void);

/**
 * Enable the bearer layer.
 *
 * Starts the scanner, timers, etc.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t bearer_enable(void);

/**
 * Disable the bearer layer.
 *
 * Stops scanner, timers, etc.
 *
 * @retval NRF_SUCCESS Bearer layer
 */
uint32_t bearer_disable(void);

/**
 * Retrieves statistics for the filter on incoming packets.
 * @param p_packets_dropped_invalid_length Pointer to a variable where the number of packets dropped
 *                                         due to AD fields of invalid length is stored.
 * @param p_packets_dropped_invalid_adtype Pointer to a variable where the number of packets dropped
 *                                         due to unrecognzied AD types is stored.
 * @param p_packets_dropped_invalid_addr   Pointer to a variable where the number of packets dropped
 *                                         due to filtered GAP addresses is stored.
 * @param p_packets_accepted               Pointer to a variable where the number of accepted packets
 *                                         is stored.
 */
void bearer_stats_get(uint32_t * p_packets_dropped_invalid_length,
        uint32_t * p_packets_dropped_invalid_adtype,
        uint32_t * p_packets_dropped_invalid_addr,
        uint32_t * p_packets_accepted);

/**
 * Queue a network-packet for transmission.
 *
 * @note If an error occurs while queueing the packet for one of the given bearers,
 *       none of the bearers will accept the packet, and a return code will be posted.
 *
 * @param[in] p_packet Pointer to a packet to be transmitted.
 * @param[in] bearers Bearer type bitfield, deciding which bearers should
 *            transmit the packet. Multiple bearers can be selected by
 *            bitwise-or'ing several bearer types.
 * @param[in] transmit_count Number of retransmits this packet should do, if
 *            applicable to the given bearers.
 *
 * @retval NRF_SUCCESS The packet was successfully queued at all the given bearers.
 * @retval NRF_ERROR_INVALID_PARAM One of the given bearers are permanently
 *         unavailable, and the packet will not be transmitted by any bearer.
 * @retval NRF_ERROR_BUSY The packet was rejected by at least one bearer,
 *         and will not be transmitted on any bearer.
 */
uint32_t bearer_tx(packet_t* p_packet,
                   bearer_t bearers,
                   uint8_t transmit_count);

/**
 * Fetch a network packet from a bearer.
 *
 * @param[out] pp_packet Pointer to a network packet pointer to place the reference
 *             to the packet in.
 * @param[out] p_bearer Pointer to a bearer variable to fill with the type of bearer the packet
 *             came from (may only be one). May be NULL if the information is irrelevant.
 * @param[out] p_meta Pointer to meta data structure to be filled by the type of
 *             bearer (radio).
 *
 * @retval NRF_SUCCESS A pointer to a packet was successfully popped from the queue, and placed
 *         in the pp_net_packet parameter.
 * @retval NRF_ERROR_NOT_FOUND There were no packets in the RX queue, and the parameters were
 *         left unaltered.
 */
uint32_t bearer_rx(packet_t** pp_packet, bearer_t* p_bearer, packet_meta_t * p_meta);


#ifdef NRF_MESH_TEST_BUILD
/**
 * Enable or disable RSSI based filtering. To disable, set the
 * rssi value to 0.
 *
 * @param[in] rssi The maximum acceptable rssi value: All the packets
 *            with higher RSSI values will be dropped.
 *
 * @note This function is for testing purposes only.
 */
void bearer_rssi_filtering_set(uint8_t rssi);

/**
 * Getter function for number of packets dropped due to high rssi.
 *
 * @return Count of dropped packets due to high rssi.
 *
 * @note This function is for testing purposes only.
 */
uint32_t bearer_packet_drop_rssi_get(void);

/*
 * Getter function for advertising context.
 *
 * @warning This function is for testing purposes only.
 *
 * @return Pointer to m_regular_advertiser context.
 */
advertiser_t* get_p_advertiser(void);

#endif
/** @} */

#endif /* BEARER_H__ */

