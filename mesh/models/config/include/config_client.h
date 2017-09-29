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

#ifndef CONFIG_CLIENT_H__
#define CONFIG_CLIENT_H__

#include <stdint.h>
#include "config_messages.h"
#include "config_opcodes.h"

#include "access.h"
#include "device_state_manager.h"

/**
 * @defgroup CONFIG_CLIENT Configuration client
 * @ingroup CONFIG_MODEL
 * @{
 */

/** Publication state parameter structure. */
typedef struct
{
    /** Element address of the model to set the publication state. */
    uint16_t element_address;
    /**
     * Publish address.
     * Set type to @ref NRF_MESH_ADDRESS_TYPE_VIRTUAL to set it to a virtual address.
     */
    nrf_mesh_address_t publish_address;
    /** Application key index. */
    uint16_t appkey_index;
    /**
     * Set @c true to use friendship credentials for publishing.
     * @warning Not supported.
     */
    bool frendship_credential_flag;
    /**
     * Publish TTL value.
     * Set to @ref ACCESS_TTL_USE_DEFAULT to use the default TTL configuration for the node.
     */
    uint8_t publish_ttl;
    /** Publish period. */
    uint8_t publish_period;
    /** Retransmit count. */
    uint8_t retransmit_count;
    /** Retransmit interval (in multiples of 50 ms). */
    uint8_t retransmit_interval;
    /** Model identifier. */
    access_model_id_t model_id;
} config_publication_state_t;

/** Heartbeat publication parameter structure. */
typedef struct
{
    /** Destination for heartbeat messages. */
    nrf_mesh_address_t destination;
    /**
     * Number of heartbeat messages to be sent.
     * @note The value is logarithmic: `count = 2^(count_log - 1)`
     */
    uint8_t count_log;
    /**
     * Period of the heartbeat messages in seconds.
     * @note The value is logarithmic: `count = 2^(count_log - 1)`
     */
    uint8_t period_log;
    /** TTL to be used for heartbeat messages. */
    uint8_t ttl;
    /** Bit field indicating features that trigger heartbeat messages when changed. */
    uint16_t features;
    /** Netkey index. */
    uint16_t netkey_index;
} config_heartbeat_publication_t;

/** Configuration client event types. */
typedef enum
{
    CONFIG_CLIENT_EVENT_TYPE_TIMEOUT,
    CONFIG_CLIENT_EVENT_TYPE_MSG
} config_client_event_type_t;

/** Union of possible status message responses. */
typedef union
{
    config_msg_appkey_status_t appkey_status;
    config_msg_net_beacon_status_t net_beacon_status;
    config_msg_publication_status_t publication_status;
    config_msg_subscription_status_t subscription_status;
    config_msg_netkey_status_t netkey_status;
    config_msg_proxy_status_t proxy_status;
    config_msg_friend_status_t friend_status;
    config_msg_heartbeat_publication_status_t heartbeat_publication_status;
    config_msg_heartbeat_subscription_status_t heartbeat_subscription_status;
    config_msg_app_status_t app_status;
    config_msg_identity_status_t identity_status;
    config_msg_composition_data_status_t composition_data_status;
    config_msg_relay_status_t relay_status;
    config_msg_appkey_list_t appkey_list;
    config_msg_sig_model_app_list_t sig_model_app_list;
    config_msg_vendor_model_app_list_t vendor_model_app_list;
    config_msg_sig_model_subscription_list_t sig_model_subscription_list;
    config_msg_vendor_model_subscription_list_t vendor_model_subscription_list;
} config_msg_t;

/** Configuration client event structure.  */
typedef struct
{
    /** Opcode of the status reply. */
    config_opcode_t opcode;
    /** Pointer to message structure. */
    const config_msg_t * p_msg;
} config_client_event_t;

/**
 * Configuration client event callback type.
 *
 * @param[in] event_type Event type.
 * @param[in] p_event    Pointer to event data, may be @c NULL.
 */
typedef void (*config_client_event_cb_t)(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length);


/**
 * Initializes the configuration client.
 *
 * @warning This function can only be called _once_.
 *
 * @param[in] event_cb Event callback pointer.
 *
 * @retval NRF_SUCCESS      Successfully initialized client and added to the access layer.
 * @retval NRF_ERROR_NULL   @c event_cb was @c NULL.
 * @retval NRF_ERROR_NO_MEM @ref ACCESS_MODEL_COUNT number of models already allocated.
 */
uint32_t config_client_init(config_client_event_cb_t event_cb);

/**
 * Sets the configuration server to configure.
 *
 * @note The address should be the address of the root element of the node (element 0).
 * @note The configuration client will handle the switching of device key implicitly.
 * @note The device key must be bound the client.
 *
 * @param[in] server_devkey_handle  Device key handle for the remote server.
 * @param[in] server_address_handle Handle for the address of the remote server.
 *
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_server_set(dsm_handle_t server_devkey_handle, dsm_handle_t server_address_handle);

/**
 * Binds the configuration client to a server.
 *
 * @note This function should be called for each new device that is to be configured.
 *
 * @param[in] server_devkey_handle Device key handle for the remote server.
 *
 * @retval NRF_SUCCESS             Successfully bound the server to the client.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 * @retval NRF_ERROR_INVALID_PARAM Invalid application key handle.
 */
uint32_t config_client_server_bind(dsm_handle_t server_devkey_handle);

/**
 * Sends a composition data GET request.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_composition_data_get(void);

/**
 * Sends an application key add request.
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 * @param[in] p_appkey     Pointer to @ref NRF_MESH_KEY_SIZE byte application key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_add(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey);

/**
 * Sends an application key delete request.
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_delete(uint16_t netkey_index, uint16_t appkey_index);

/**
 * Sends an application key(s) get request.
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_get(uint16_t netkey_index);

/**
 * Sends an application key update request.
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 * @param[in] p_appkey     Pointer to @ref NRF_MESH_KEY_SIZE byte application key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_update(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey);

/**
 * Sends a network key add request.
 *
 * @param[in] netkey_index Network key index.
 * @param[in] p_netkey     Pointer to @ref NRF_MESH_KEY_SIZE byte network key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_add(uint16_t netkey_index, const uint8_t * p_netkey);

/**
 * Sends a network key delete request.
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_delete(uint16_t netkey_index);

/**
 * Sends a network key(s) get request.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_get(void);

/**
 * Sends a network key update request.
 *
 * @param[in] netkey_index Network key index.
 * @param[in] p_netkey     Pointer to @ref NRF_MESH_KEY_SIZE byte network key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_update(uint16_t netkey_index, const uint8_t * p_netkey);

/**
 * Sends a publication get request
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model identifier.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_publication_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a model publication set request.
 *
 * @param[in] p_publication_state Publication state parameter struct pointer.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_publication_set(const config_publication_state_t * p_publication_state);

/**
 * Sends a subscription add request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_add(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a subscription delete request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_delete(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a subscription delete all request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_delete_all(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a subscription get request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a subscription overwrite request.
 *
 * @warning This will clear the subscription list of the model.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_overwrite(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a application bind request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] appkey_index    Application key index to bind/unbind.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_bind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id);

/**
 * Sends an application get request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a application unbind request.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] appkey_index    Application key index to bind/unbind.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_unbind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id);

/**
 * Sends a default TTL get request.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_default_ttl_get(void);

/**
 * Sends a default TTL set request.
 *
 * @param[in] ttl Default TTL value. Must be less than @ref NRF_MESH_TTL_MAX.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_default_ttl_set(uint8_t ttl);

/**
 * Sends a relay state get request.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_relay_get(void);

/**
 * Sends a relay state get request.
 *
 * @param[in] relay_state               Relay state.
 * @param[in] retransmit_count          Number of times to re-transmit relayed packets.
 * @param[in] retransmit_interval_steps Number of 10 ms steps between each re-transmission.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 * @retval NRF_ERROR_INVALID_PARAM Invalid parameter values. See @ref CONFIG_RETRANSMIT_COUNT_MAX
 *                                 and @ref CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX.
 */
uint32_t config_client_relay_set(config_relay_state_t relay_state, uint8_t retransmit_count, uint8_t retransmit_interval_steps);

/**
 * Sends a secure network beacon state get request.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_net_beacon_get(void);

/**
 * Sends a secure network beacon state set request.
 *
 * @param[in] state New secure network beacon state.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_net_beacon_set(config_net_beacon_state_t state);

/**
 * Sends a node reset request.
 *
 * @warning This will "un-provision" the node and remove it from the network.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_node_reset(void);

/** @todo Unsupported API calls */

// uint32_t config_client_friend_get(void);
// uint32_t config_client_friend_set(bool state);

// uint32_t config_client_gatt_proxy_get(void);
// uint32_t config_client_gatt_proxy_set(bool state);
// uint32_t config_client_node_identity_get(uint16_t netkey_index);
// uint32_t config_client_node_identity_set(uint16_t netkey_index, config_identity_state_t state);

// uint32_t config_client_heartbeat_publication_get(void);
// uint32_t config_client_heartbeat_publication_set(const config_heartbeat_publication_t * p_publication);
// uint32_t config_client_heartbeat_subscription_get(void);
// uint32_t config_client_heartbeat_subscription_set(uint16_t source_address_raw, uint16_t dest_address_raw, uint8_t period);

/** @} end of CONFIG_CLIENT */

#endif  /* CONFIG_CLIENT_H__ */

