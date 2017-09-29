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

#ifndef ACCESS_H__
#define ACCESS_H__

#include <stdint.h>
#include "device_state_manager.h"

/**
 * @defgroup ACCESS Access layer API
 * @ingroup MESH_API_GROUP_ACCESS
 * @{
 * @defgroup ACCESS_MSCS Access layer API MSCs
 * @{
 * @mscfile model.msc "Basic access layer usage"
 * @mscfile message_rx.msc "Receiving an access layer message"
 * @mscfile periodic.msc "Periodic publishing"
 * @}
 */

/** Invalid access model handle value. */
#define ACCESS_HANDLE_INVALID (0xFFFF)
/** Company ID value for Bluetooth SIG opcodes or models. */
#define ACCESS_COMPANY_ID_NONE (0xFFFF)
/** Comapny ID value for Nordic Semiconductor. */
#define ACCESS_COMPANY_ID_NORDIC (0x0059)

/** Value used for TTL parameters in order to set the TTL to the default value. */
#define ACCESS_TTL_USE_DEFAULT  (0xFF)

/**
 * Maximum payload length for an access layer message.
 *
 * @note Payloads greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX will be segmented.
 */
#define ACCESS_MESSAGE_LENGTH_MAX (NRF_MESH_SEG_PAYLOAD_SIZE_MAX)

/*lint -align_max(push) -align_max(1) */

/** Access layer model ID. */
typedef struct __attribute((packed))
{
    /** Model ID. */
    uint16_t model_id;
    /** Company ID. Bluetooth SIG models shall set this to @ref ACCESS_COMPANY_ID_NONE. */
    uint16_t company_id;
} access_model_id_t;

/*lint -align_max(pop) */

/** Access layer handle type. */
typedef uint16_t access_model_handle_t;

/**
 * Access layer publish timeout event callback.
 *
 * @param[in] handle Access layer model handle.
 * @param[in] p_args Optional generic argument pointer.
 */
typedef void (*access_publish_timeout_cb_t)(access_model_handle_t handle, void * p_args);

/**
 * Access layer opcode type.
 *
 * The format of the opcodes is given in the table below:
 * Table 3.43 in the Mesh Profile Specification (v1.0)
 *
 * | Byte 0     | Byte 1     | Byte 2     | Description                                                     |
 * | ---------- | ---------- | ---------- | --------------------------------------------------------------- |
 * | `0xxxxxxx` |            |            | 1-octet Bluetooth SIG Opcodes (excluding 01111111)              |
 * | `01111111` |            |            | Reserved for Future Use                                         |
 * | `10xxxxxx` | `xxxxxxxx` |            | 2-octet Bluetooth SIG Opcodes                                   |
 * | `11xxxxxx` | `zzzzzzzz` | `zzzzzzzz` | 3-octet Vendor Specific Opcodes. `z` denotes company identifier |
 *
 */
typedef struct
{
    /** 14-bit or 7-bit Bluetooth SIG defined opcode or 6-bit vendor specific opcode. */
    uint16_t opcode;
    /** Company ID. Set to @ref ACCESS_COMPANY_ID_NONE if it is a Bluetooth SIG defined opcode. */
    uint16_t company_id;
} access_opcode_t;

/** Access layer RX event structure. */
typedef struct
{
    /** Opcode of the message. */
    access_opcode_t opcode;
    /** Pointer to the first byte of message data (excludes the opcode). */
    const uint8_t * p_data;
    /** Length of @c p_data. */
    uint16_t length;
    /** Meta data for the message. */
    struct
    {
        /** Source address of the message. */
        nrf_mesh_address_t src;
        /** Destination address of the message. */
        nrf_mesh_address_t dst;
        /** Timestamp of when the (first part of the) message was received in microseconds. */
        uint32_t timestamp;
        /** Delta between when the first and the last segment received in microseconds. */
        uint32_t timestamp_delta;
        /** TTL value for the received message. */
        uint8_t ttl;
        /** Application key handle that decrypted the message. */
        dsm_handle_t appkey_handle;
    } meta_data;
} access_message_rx_t;

/** Access layer TX parameter structure. */
typedef struct
{
    /** Opcode for the message. */
    access_opcode_t opcode;
    /** Pointer to the message data. */
    const uint8_t * p_buffer;
    /** Length of the data (excluding the opcode). */
    uint16_t length;
} access_message_tx_t;

/**
 * Access layer opcode handler callback type.
 *
 * @param[in] handle    Access layer model handle.
 * @param[in] p_message Access RX message structure.
 * @param[in] p_args    Optional generic argument pointer.
 */
typedef void (*access_opcode_handler_cb_t)(access_model_handle_t handle,
                                           const access_message_rx_t * p_message,
                                           void * p_args);

/**
 * Opcode handler type.
 *
 * Each specific model implementation is assumed to statically define an array of "opcode handlers",
 * one handler for each of the expected opcodes of the given model.
 */
typedef struct
{
    /** The model opcode. */
    access_opcode_t opcode;
    /** The opcode handler callback for the given opcode. */
    access_opcode_handler_cb_t handler;
} access_opcode_handler_t;

/**
 * Access model allocation parameter structure.
 */
typedef struct
{
    /** SIG or Vendor Model ID. */
    access_model_id_t model_id;
    /** Element index to add the model to. */
    uint16_t element_index;
    /** Pointer to list of opcode handler callbacks. */
    const access_opcode_handler_t * p_opcode_handlers;
    /** Number of opcode handles. */
    uint32_t opcode_count;
    /**
     * Generic argument pointer. This pointer will be supplied as an argument in the callbacks from
     * the access layer, e.g., @ref access_opcode_handler_cb_t. May be set to @c NULL if unused.
     */
    void * p_args;
    /**
     * Timeout callback called when the publication timer expires. Set to @c NULL for models that
     * doesn't support periodic publishing.
     */
    access_publish_timeout_cb_t publish_timeout_cb;
} access_model_add_params_t;

/**
 * Initializes the access layer.
 */
void access_init(void);


/**
 * Clears the access layer states, and erases the persistent storage copy.
 */
void access_clear(void);

/**
 * Allocates, initializes and adds a model to the element at the given element index.
 *
 * @param[in]  p_model_params            Pointer to model initialization parameter structure.
 * @param[out] p_model_handle            Pointer to store allocated model handle.
 *
 * @retval     NRF_SUCCESS               Successfully added model to the given element.
 * @retval     NRF_ERROR_NO_MEM          @ref ACCESS_MODEL_COUNT number of models already allocated.
 * @retval     NRF_ERROR_NULL            One or more of the function parameters was NULL.
 * @retval     NRF_ERROR_FORBIDDEN       Multiple model instances per element is not allowed.
 * @retval     NRF_ERROR_NOT_FOUND       Invalid access element index.
 * @retval     NRF_ERROR_INVALID_LENGTH  Number of opcodes was zero.
 * @retval     NRF_ERROR_INVALID_PARAM   One or more of the opcodes had an invalid format.
 * @see        access_opcode_t for documentation of the valid format.
 */
uint32_t access_model_add(const access_model_add_params_t * p_model_params,
                          access_model_handle_t * p_model_handle);

/**
 * Allocates a subscription list for a model.
 *
 * @param[in]  handle                   Model handle to allocate list for.
 *
 * @retval     NRF_SUCCESS              Successfully allocated subscription list.
 * @retval     NRF_ERROR_NO_MEM         No more subscription lists available in memory pool.
 *                                      @see ACCESS_SUBSCRIPTION_LIST_COUNT.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_STATE  A subscription list is already allocated for model.
 */
uint32_t access_model_subscription_list_alloc(access_model_handle_t handle);

/**
 * Shares the subscription lists for two models.
 *
 * @note       Only one model should allocate a subscription list.
 * @note       This function is used with models that operate on bound states and should share a
 *             single subscription list.
 *
 * @param[in]  owner                    The owner of the subscription list (the model handle that
 *                                      allocated it).
 * @param[in]  other                    The model that should share the owner's subscription list.
 *
 * @retval     NRF_SUCCESS              Successfully shared the subscription list.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid for one or more of the models.
 * @retval     NRF_ERROR_INVALID_STATE  Invalid parameter combination. Only the owner should have a
 *                                      subscription list allocated.
 */
uint32_t access_model_subscription_lists_share(access_model_handle_t owner, access_model_handle_t other);

/**
 * Publishes an access layer message to the publish address of the model.
 *
 * @param[in] handle    Access handle for the model that wants to send data.
 * @param[in] p_message Access layer TX message parameter structure.
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 */
uint32_t access_model_publish(access_model_handle_t handle, const access_message_tx_t * p_message);

/**
 * Replies to an access layer message.
 *
 * @warning This function is intended to be used in pair with the opcode handle callbacks. I.e., the
 * user gets a message through the @ref access_opcode_handler_cb_t and replies with this function.
 *
 * @param[in] handle    Access handle for the model that wants to send data.
 * @param[in] p_message Incoming message that the model is replying to.
 * @param[in] p_reply   The reply data.
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 */
uint32_t access_model_reply(access_model_handle_t handle,
                            const access_message_rx_t * p_message,
                            const access_message_tx_t * p_reply);

/** @} */
#endif /* ACCESS_H__ */
