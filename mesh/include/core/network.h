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
#ifndef MESH_NETWORK_H__
#define MESH_NETWORK_H__

#include "packet.h"
#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_config_core.h"

/**
 * @defgroup NETWORK Network Layer
 * @ingroup MESH_CORE
 * Processes incoming messages and decrypts packet contents using a network key.
 * @{
 */

/**
 * Initializes the network layer.
 * @param[in] p_init_params Pointer to the initialization parameters structure.
 */
void network_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Sets a network layer option.
 * @param[in] id    Option ID.
 * @param[in] p_opt Pointer to a structure containing the new value of the option.
 * @retval NRF_SUCCESS          The value of the specified option was successfully changed.
 * @retval NRF_ERROR_NOT_FOUND  The specified option ID is not valid for this module.
 * @retval NRF_ERROR_INVALID_PARAM The value provided for the option was invalid.
 * @retval NRF_ERROR_NULL       The @c p_opt parameter was NULL.
 */
uint32_t network_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt);

/**
 * Retrieves the value of a network layer option.
 * @param[in]  id    Option ID.
 * @param[out] p_opt Pointer to where the retrieved option value will be stored.
 * @retval NRF_SUCCESS             The value of the specified option was successfully retrieved.
 * @retval NRF_ERROR_NOT_FOUND     The specified option ID is not valid for this module.
 * @retval NRF_ERROR_INVALID_PARAM The value provided for the option was invalid.
 * @retval NRF_ERROR_NULL          The @c p_opt parameter was NULL.
 */
uint32_t network_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt);

/**
 * Function for sending an outgoing packet through the network layer.
 *
 * This function is used by the transport layer when transmitting packets and by the
 * network layer when retransmitting packets.
 *
 * @param[in] p_packet     Packet buffer for the packet to send.
 * @param[in] p_net_secmat Network security material.
 * @param[in] local_packet Set to @c true if the packet originated locally, such as a packet being
 * transmitted from an application running on the current node. If the packet is a packet that is
 * being relayed by the node, this is set to @c false. This parameter affects the priority of the
 * packet when it is sent to lower layers.
 *
 * @retval NRF_SUCCESS The packet was successfully sent.
 * @retval NRF_ERROR_BUSY The bearer is busy.
 */
uint32_t network_pkt_out(packet_t * p_packet, const nrf_mesh_network_secmat_t * const p_net_secmat, bool local_packet);

/**
 * Function for processing incoming packets.
 *
 * @param[in] p_net_packet  Network packet to process.
 * @param[in] p_packet_meta Packet metadata (@see packet_meta_t).
 *
 * @retval NRF_SUCCESS The packet was successfully processed.
 * @retval NRF_ERROR_INVALID_ADDR The destination address is not valid.
 * @retval NRF_ERROR_NOT_FOUND    The packet could not be decrypted by the transport layer.
 */
uint32_t network_pkt_in(packet_net_t * p_net_packet, const packet_meta_t * p_packet_meta);

/**
 * Relays a network packet.
 *
 * Used by the transport layer to relay messages intended for more than one
 * receipient.
 *
 * @param[in] p_net_packet Network packet pointer.
 * @param[in] p_net_secmat Network security material.
 *
 * @retval NRF_SUCCESS      Successfully queued packet for relaying.
 * @retval NRF_ERROR_NULL   Invalid pointers supplied to the function.
 * @retval NRF_ERROR_NO_MEM Not enough memory to allocate new relay packet.
 */
uint32_t network_pkt_relay(packet_net_t * p_net_packet, const nrf_mesh_network_secmat_t * const p_net_secmat);

/** @} */

#endif

