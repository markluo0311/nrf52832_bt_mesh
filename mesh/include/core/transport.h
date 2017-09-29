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
#ifndef MESH_TRANSPORT_H__
#define MESH_TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet_mgr.h"
#include "timer.h"
#include "utils.h"

#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_defines.h"

#include "nrf_mesh_config_core.h"

/**
 * @defgroup TRANSPORT Transport Layer
 * @ingroup MESH_CORE
 * @{
 */

/** Default TX timeout. */
#define TRANSPORT_SAR_RX_TIMEOUT_DEFAULT       (60 * 1000 * 1000)

/** Default RX acknowledgement timeout. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_DEFAULT   (500 * 1000)

/** Default TX retry timeout. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_DEFAULT (2*TRANSPORT_SAR_RX_ACK_TIMEOUT_DEFAULT)

/** Default TX timeout. */
#define TRANSPORT_SAR_TX_TIMEOUT_DEFAULT       (10 * 1000 * 1000)

/** Maximum number of concurrent transport SAR RX sessions. */
#define TRANSPORT_SAR_RX_SESSIONS_MAX          (2)

/** Maximum number of concurrent transport SAR TX sessions. */
#define TRANSPORT_SAR_TX_SESSIONS_MAX          (2)

/** Default number of retries before cancelling SAR TX session. */
#define TRANSPORT_SAR_TX_RETRIES_DEFAULT       (4)


/**
 * Transport configuration struct.
 *
 * @note All timing values are given in microseconds.
 *
 * @warning Be very careful when modifying the timeout values. Too low values
 * _will_ clog the bearer TX queue and potentially halt the system.
 */
typedef struct
{
    timestamp_t rx_timeout;                          /**< Timeout for receiving a SAR message. */
    timestamp_t rx_ack_timeout;                      /**< RX acknowledgement timer timeout value. */
    timestamp_t tx_timeout;                          /**< Timeout before a packet is resent. */
    timestamp_t tx_retry_timeout;                    /**< Timeout for TX retries.*/
    uint8_t     tx_retries;                          /**< Number of retries before cancelling SAR session. */
    uint8_t     segack_ttl : NRF_MESH_TTL_BIT_COUNT; /**< Default TTL value for segment acknowledgement messages. */
    uint8_t     szmic : 1;                           /**< Use 32- or 64-bit MIC for application payload. */
} transport_config_t;

/**
 * Transport SAR session types.
 */
typedef enum
{
    TRS_SAR_SESSION_TX = 0,         /**< TX session. */
    TRS_SAR_SESSION_RX = 1          /**< RX session. */
} trs_sar_session_t;

/**
 * Transport SAR PDU structure.
 *
 * @note All multi-octet values are assumed *little* endian.
 */
typedef struct
{
    /** Network context structure pointer.  */
    const nrf_mesh_network_secmat_t * p_net_secmat;
    /** Transport SAR session header. */
    struct
    {
        uint32_t timeout;            /**< Timeout for session. */
        uint32_t seq_auth;           /**< Sequence number that encrypted the message. */
        uint32_t seq;                /**< Message sequence number of the last segment received. */
        uint32_t block_ack;          /**< Bit-field of the received messages. */
        uint16_t src;                /**< Source address. */
        uint16_t dst;                /**< Destination address. */
        uint16_t length;             /**< Total length of payload. */
        uint8_t  ivi;                /**< IV index bit. */
        uint8_t  akf;                /**< Application key flag. */
        uint8_t  szmic;              /**< Transport layer MIC size. */
        uint8_t  aid;                /**< Application key identifier. */
        uint8_t  seg_left;           /**< Remaining segments in transmission. */
        uint8_t  retries;            /**< Number of retries left (TX). */
        uint8_t  ttl;                /**< Time To Live value. */
        bool session_active;         /**< Set if the current session is active. */
        bool session_type;           /**< SAR session type. */

    } header;
    /**
     * Re-segmented SAR payload with 4 byte MIC at the end.
     */
    uint8_t * payload;
} trs_sar_ctx_t;

/**
 * Allocation function type for transport SAR buffer allocation. Matches
 * stdlib's malloc.
 *
 * @param[in] size Number of bytes to allocate.
 *
 * @returns A pointer to a valid buffer of at least size @p size, or NULL if
 * the allocation failed.
 */
typedef void* (*transport_sar_alloc_t)(size_t size);

/**
 * Release function type for transport SAR buffer deallocation. Matches
 * stdlib's free.
 *
 * @param[in] ptr A pointer to a previously allocated buffer, that is to be released.
 */
typedef void (*transport_sar_release_t)(void* ptr);

/**
 * Initializes the transport layer.
 *
 * @param[in] p_init_params  Generic initialization parameters pointer.
 */
void transport_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Set the SAR buffer allocation and release functions. Defaults to stdlib's
 * malloc and free. The transport layer has to allocate a temporary buffer
 * for transport packets that span multiple network packets, in order to put
 * them together (RX) or split them (TX). The transport module takes no
 * precautions to prevent overlapping memory regions for different buffers,
 * although this will cause undefined behavior.
 *
 * @note If both parameters are NULL, the function behaves like @ref
 * transport_sar_mem_funcs_reset. If only one of them is NULL, The function
 * does nothing and returns @c NRF_ERROR_NULL.
 *
 * @param[in] alloc_func Function pointer to the wanted allocation function.
 * @param[in] release_func Function pointer to the wanted release function.
 *
 * @retval NRF_SUCCESS The allocation and release functions were successfully
 * set.
 * @retval NRF_ERROR_NULL One of the given function pointers were NULL, but not both.
 */
uint32_t transport_sar_mem_funcs_set(transport_sar_alloc_t alloc_func, transport_sar_release_t release_func);

/**
 * Reset the SAR buffer allocation and release functions to malloc and free.
 */
void transport_sar_mem_funcs_reset(void);
/**
 * Function for passing packets from the network layer to the transport layer.
 *
 * @note The transport layer converts the destination and source addresses to
 * little endian before passing them on to the application.
 *
 * @param[in] p_net_packet      The network packet to process.
 * @param[in] p_packet_meta     Packet metadata (rssi, timestamp, ..) @see packet_meta_t
 * @param[in] p_net_secmat      The network security material @see nrf_mesh_network_secmat_t.
 * @param[in] iv_index          IV index the packet was decrypted with, in little endian.
 *
 * @retval NRF_ERROR_INVALID_ADDR The destination address is not valid.
 * @retval NRF_ERROR_NOT_FOUND    The packet could not be decryptet with any application key.
 * @retval NRF_ERROR_NULL         One or more of the input parameters was NULL.
 * @retval NRF_SUCCESS            The packet was successfully decrypted and sent up the stack.
 */
uint32_t transport_pkt_in(packet_net_t * p_net_packet,
                          const packet_meta_t * p_packet_meta,
                          const nrf_mesh_network_secmat_t * const p_net_secmat,
                          uint32_t iv_index);

/**
 * Give transport layer SAR context to process messages.
 *
 * @retval NRF_SUCCESS Successfully done processing.
 * @retval NRF_ERROR_NO_MEM No memory for doing operations on incoming packets.
 */
uint32_t transport_sar_process(void);

/**
 * Do decryption on a SAR packet.
 *
 * @param[in] p_sar_ctx Pointer to SAR context buffer.
 *
 * @retval NRF_SUCCESS         Successfully decrypted packet.
 * @retval NRF_ERROR_NOT_FOUND Not able to decrypt packet.
 * @retval NRF_ERROR_NO_MEM    Not able to allocate memory for decryption buffer.
 */
uint32_t trs_sar_pkt_decrypt(const trs_sar_ctx_t * p_sar_ctx);

/**
 * Convert a transport address context structure to a 16-bit short address.
 *
 * @param[in]  p_addr          Transport address context pointer.
 * @param[out] p_short_addr    16-bit little-endian short address.
 *
 * @retval NRF_SUCCESS            Successfully converted address.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address.
 */
uint32_t transport_addr_to_short(nrf_mesh_address_t * p_addr, uint16_t * p_short_addr);

/**
 * Transmit a single-segment message.
 *
 * The transport layer processes a message from the application, encrypts it and
 * passes it on to the network layer.
 *
 * @param[in]  p_params           Message parameters.
 * @param[out] p_packet_reference Reference to SAR buffer (for TX complete event).
 *
 * @retval NRF_SUCCESS            The packet was successfully queued for transmission.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address supplied.
 * @retval NRF_ERROR_NULL         Null-pointer supplied.
 * @retval NRF_ERROR_NO_MEM       Insufficient amount of available memory.
 */
uint32_t transport_tx(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference);

/**
 * Transmit a multisegment (SAR) message.
 *
 * @param[in]  p_params           Message parameters.
 * @param[out] p_packet_reference Reference to SAR buffer (for TX complete event).
 *
 * @retval NRF_SUCCESS            Successfully started SAR session.
 * @retval NRF_ERROR_BUSY         The network IV is about to change and new SAR transactions cannot be started
 *                                until this operation is complete.
 * @retval NRF_ERROR_NO_MEM       Not enough available memory.
 * @retval NRF_ERROR_NULL         NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address.
 */
uint32_t transport_tx_sar(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference);

/**
 * Set transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Get transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/**
 * Get the maximum segmented message payload length.
 *
 * The maximum length is dependent on the value of the `szmic` bit.
 *
 * @retval seg_maxlen The maximum segmented message length.
 */
uint32_t transport_seg_maxlen_get(void);

/**
 * Get the maximum unsegmented message payload length.
 *
 * The maximum length is dependent on the value of the `szmic` bit.
 *
 * @retval unseg_maxlen The maximum unsegmented message length.
 */
uint32_t transport_unseg_maxlen_get(void);

/** @} */

#endif
