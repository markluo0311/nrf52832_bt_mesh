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
#ifndef ACCESS_RELIABLE_H__
#define ACCESS_RELIABLE_H__

#include <stdint.h>
#include "access.h"

#include "nrf_mesh_defines.h"

/**
 * @defgroup ACCESS_RELIABLE nRF BLE Mesh Access layer reliable publish
 * @ingroup MESH_API_GROUP_ACCESS
 * @{
 * @defgroup ACCESS_RELIABLE_MSCS Message sequence charts
 * @{
 * @mscfile access_reliable_publish.msc "Reliable publish API usage"
 * @mscfile access_reliable_publish_fail.msc "Reliable publish API errors"
 * @}
 */

/**
 * Minimum timeout for a reliable message in microseconds.
 * @note BLE Mesh specified to be minimum 60s.
 */
#define ACCESS_RELIABLE_TIMEOUT_MIN  (SEC_TO_US(30))

/** Maximum timeout for a reliable message in microseconds. */
#define ACCESS_RELIABLE_TIMEOUT_MAX  (SEC_TO_US(60))

/** Penalty in microseconds for each hop for a reliable message. */
#define ACCESS_RELIABLE_HOP_PENALTY (MS_TO_US(BEARER_ADV_INT_MIN_MS_DEFAULT))

/** Penalty for each segment in a reliable transfer. */
#define ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY MS_TO_US(BEARER_ADV_INT_MIN_MS_DEFAULT)

/**
 * Base interval in microseconds for a reliable message.
 * I.e., the interval given TTL=0 and an unsegmented message.
 */
#define ACCESS_RELIABLE_INTERVAL_DEFAULT (MS_TO_US(BEARER_ADV_INT_MIN_MS_DEFAULT) * 10)

/** Back-off factor used to increase the interval for each retry. */
#define ACCESS_RELIABLE_BACK_OFF_FACTOR (2)

/** Margin in microseconds for which two timeout events are fired "at the same time". */
#define ACCESS_RELIABLE_TIMEOUT_MARGIN (MS_TO_US(1))

/** Time in microseconds to wait before retrying a publish if the stack reports @c NRF_ERROR_NO_MEM. */
#define ACCESS_RELIABLE_RETRY_DELAY (MS_TO_US(BEARER_ADV_INT_MIN_MS_DEFAULT) * 2)

/** Access reliable transfer status codes. */
typedef enum
{
    /**
     * The reliable transfer was completed successfully.
     * @note This status is only used for @ref NRF_MESH_ADDRESS_TYPE_UNICAST addresses.
     */
    ACCESS_RELIABLE_TRANSFER_SUCCESS,
    /** The reliable transfer reached its timeout. */
    ACCESS_RELIABLE_TRANSFER_TIMEOUT
} access_reliable_status_t;

/**
 * Access layer reliable message callback type.
 * Used to indicate a successful or unsuccessful reliable transfer.
 *
 * @param[in] model_handle Access layer model handle.
 * @param[in] p_args       Generic argument pointer.
 * @param[in] status       Access reliable transfer status code.
 */
typedef void (*access_reliable_cb_t)(access_model_handle_t model_handle,
                                     void * p_args,
                                     access_reliable_status_t status);

/**
 * Access layer reliable publish parameter structure.
 * @todo MBTLE-1540: Use internal general purpose memory manager to store retain the data for the user.
 */
typedef struct
{
    /** Access layer model handle. */
    access_model_handle_t model_handle;
    /**
     * Access layer message.
     * @note The data pointed to must be retained for the entire reliable
     * transfer.
     */
    access_message_tx_t message;
    /** Opcode of the expected reply. */
    access_opcode_t reply_opcode;
    /**
     * Relative reliable message timeout.
     * I.e., the time from access_model_reliable_publish() is called to the message is timed out.
     */
    uint32_t timeout;
    /** Callback to call after the reliable transfer has ended. */
    access_reliable_cb_t status_cb;
} access_reliable_t;

/**
 * Initializes the reliable publication framework.
 */
void access_reliable_init(void);

/**
 * Cancels all ongoing transfers.
 *
 * @warning This will _not_ notify the about the canceled transfer. Thus calling this function in
 * the middle of a reliable transfer may cause unexpected behavior for the affected models.
 */
void access_reliable_cancel_all(void);

/**
 * Starts publishing a reliable message.
 *
 * @param[in] p_reliable Reliable message parameter structure pointer.
 *
 * @retval NRF_SUCCESS              Successfully started the reliable message publication.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 */
uint32_t access_model_reliable_publish(const access_reliable_t * p_reliable);

/**
 * Cancels an ongoing reliable message.
 *
 * @param[in] model_handle Access layer model handle that owns the transfer.
 *
 * @retval NRF_SUCCESS         Successfully canceled the reliable message.
 * @retval NRF_ERROR_NOT_FOUND No active reliable message for given handle.
 * @retval NRF_ERROR_NOT_FOUND Invalid model handle or model not bound to element.
 */
uint32_t access_model_reliable_cancel(access_model_handle_t model_handle);

/**
 * Callback called by access layer when a model receives a message.
 *
 * @param[in] model_handle Model handle that received the message
 * @param[in] p_message    Message pointer.
 * @param[in] p_args       Generic argument pointer for the model.
 */
void access_reliable_message_rx_cb(access_model_handle_t model_handle, const access_message_rx_t * p_message, void * p_args);

/** @} */
#endif /* ACCESS_RELIABLE_H__ */
