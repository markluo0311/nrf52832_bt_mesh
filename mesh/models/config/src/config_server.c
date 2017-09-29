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

#include <stdint.h>
#include <string.h>

#include "access.h"
#include "access_config.h"
#include "access_status.h"

#include "config_messages.h"
#include "config_opcodes.h"
#include "config_server.h"
#include "composition_data.h"
#include "device_state_manager.h"
#include "packed_index_list.h"

#include "net_beacon.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_keygen.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_utils.h"

/** Configuration server model ID. */
#define CONFIG_SERVER_MODEL_ID  0x0000

/*
 * Checks if the length of a packet is valid. This macro is used to check the sizes
 * of packets that have a config_model_id_t member, which means the packets can vary in
 * size by sizeof(uint16_t).
 */
#define IS_PACKET_LENGTH_VALID_WITH_ID(packet_type, p_packet) \
    ((p_packet)->length == sizeof(packet_type) || (p_packet)->length == sizeof(packet_type) - sizeof(uint16_t))

/**
 * Checks if the message is a SIG or vendor model message.
 * For some messages, both variants are possible, and the difference is determined by
 * checking the length of the message; SIG models contain a 16-bit model ID, while
 * vendor models contain a 32-bit model ID, making SIG messages 2 bytes shorter than
 * vendor messages.
 */
#define IS_SIG_MODEL(p_message, message_type) \
    ((p_message)->length == sizeof(message_type) - sizeof(uint16_t))

/**
 * Calculates the size of a packet depending on whether it is a SIG model or not.
 * See the description of IS_SIG_MODEL() for an explanation of the difference.
 */
#define PACKET_LENGTH_WITH_ID(packet_type, sig_model) \
    (sizeof(packet_type) - ((sig_model) ? sizeof(uint16_t) : 0))

static access_model_handle_t m_config_server_handle = ACCESS_HANDLE_INVALID;

/********** Helper functions **********/

/*lint -ecall(569, send_reply) Conversion of size_t (returned by sizeof()) to uint16_t causes loss of precision. */

/*
 * Sends a message as a reply to an incoming message.
 */
static void send_reply(access_model_handle_t handle, const access_message_rx_t * p_message, uint16_t opcode,
        const uint8_t * p_reply, uint16_t reply_length)
{
    const access_message_tx_t reply =
    {
        .opcode =
        {
            .opcode = opcode,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .p_buffer = p_reply,
        .length = reply_length
    };

    (void) access_model_reply(handle, p_message, &reply);
}

/*
 * Sends a message with a status code as the first byte. All other bytes in the message is set to 0.
 * The total length of the message is as specified in the msg_size parameter.
 */
static void send_generic_error_reply(access_model_handle_t handle, const access_message_rx_t * p_message,
        access_status_t status, uint16_t status_opcode, uint16_t msg_size)
{
    uint8_t buffer[msg_size];
    memset(buffer, 0, msg_size);

    buffer[0] = status;
    send_reply(handle, p_message, status_opcode, buffer, msg_size);
}

/* Sends the network beacon state in response to a network beacon state set/get message: */
static void send_net_beacon_state(access_model_handle_t handle, const access_message_rx_t * p_incoming)
{
    config_msg_net_beacon_status_t beacon_status =
    {
        .beacon_state = net_beacon_state_get() ? CONFIG_NET_BEACON_STATE_ENABLED : CONFIG_NET_BEACON_STATE_DISABLED
    };
    send_reply(handle, p_incoming, CONFIG_OPCODE_BEACON_STATUS, (const uint8_t *) &beacon_status, sizeof(beacon_status));
}

/* Sends the appkey status in response to appkey manipulation functions: */
static void send_appkey_status(access_model_handle_t handle, const access_message_rx_t * p_message, config_msg_key_index_24_t key_indexes)
{
    const config_msg_appkey_status_t response =
    {
        .status = ACCESS_STATUS_SUCCESS,
        .key_indexes = key_indexes
    }; /*lint !e64 Lint incorrectly complains about a type mismach in this initializer. */
    send_reply(handle, p_message, CONFIG_OPCODE_APPKEY_STATUS, (const uint8_t *) &response, sizeof(response));
}

/* Sends the publication status in response to publication messages. */
static void send_publication_status(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
        uint16_t element_address, access_model_handle_t model_handle)
{
    /* Build the publication status packet: */
    config_msg_publication_status_t response;
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = element_address;
    response.state.credential_flag = 0; /* FIXME: When friendship is supported, this bit can be used. */
    response.state.rfu = 0;

    /* Get the model ID: */
    access_model_id_t model_id;
    NRF_MESH_ASSERT(access_model_id_get(model_handle, &model_id) == NRF_SUCCESS);

    bool sig_model = model_id.company_id == ACCESS_COMPANY_ID_NONE;
    response.state.model_id.model_id = model_id.model_id;
    response.state.model_id.company_id = model_id.company_id;

    dsm_handle_t publish_address_handle;
    NRF_MESH_ASSERT(access_model_publish_address_get(model_handle, &publish_address_handle) == NRF_SUCCESS);

    if (publish_address_handle ==  DSM_HANDLE_INVALID)
    {
        /* If no publish address is set, the rest of the packet is set to 0: */
        response.publish_address = NRF_MESH_ADDR_UNASSIGNED;
        memset(&response.state, 0, sizeof(config_publication_params_t));
    }
    else
    {
        /* Obtain the publish address: */
        nrf_mesh_address_t publish_address;
        NRF_MESH_ASSERT(dsm_address_get(publish_address_handle, &publish_address) == NRF_SUCCESS);
        response.publish_address = publish_address.value;

        /* Obtain the application key handle: */
        dsm_handle_t appkey_handle;
        NRF_MESH_ASSERT(access_model_publish_application_get(model_handle, &appkey_handle) == NRF_SUCCESS);

        /* Get the appkey index: */
        mesh_key_index_t appkey_index;
        NRF_MESH_ASSERT(dsm_appkey_handle_to_appkey_index(appkey_handle, &appkey_index) == NRF_SUCCESS);
        response.state.appkey_index = appkey_index;

        /* Obtain the publish TTL: */
        NRF_MESH_ASSERT(access_model_publish_ttl_get(model_handle, &response.state.publish_ttl) == NRF_SUCCESS);

        /* Obtain the publish period: */
        access_publish_resolution_t publish_resolution;
        uint8_t publish_steps;

        memset(&publish_resolution, 0, sizeof(access_publish_resolution_t));
        (void) access_model_publish_period_get(model_handle, &publish_resolution, &publish_steps);

        /* Set the publish period in the message: */
        response.state.publish_period  = publish_resolution << ACCESS_PUBLISH_STEP_NUM_BITS | publish_steps;

        /* Set publish retransmit count and interval: */
        response.state.retransmit_count = 0;    /* See MBTLE-1563 */
        response.state.retransmit_interval = 0; /* See MBTLE-1563 */
    }

    /* Finally send the message: */
    send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
}

static access_status_t delete_all_subscriptions(access_model_handle_t model_handle)
{
    /* Get a list of all the models: */
    dsm_handle_t subscribed_addresses[ACCESS_SUBSCRIPTION_LIST_COUNT];
    uint16_t subscription_count = ACCESS_SUBSCRIPTION_LIST_COUNT;
    uint32_t status = access_model_subscriptions_get(model_handle, subscribed_addresses, &subscription_count);
    if (status == NRF_ERROR_NOT_SUPPORTED)
    {
        return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
    }
    NRF_MESH_ASSERT(status == NRF_SUCCESS);

    for (uint16_t i = 0; i < subscription_count; ++i)
    {
        NRF_MESH_ASSERT(access_model_subscription_remove(model_handle, subscribed_addresses[i]) == NRF_SUCCESS);
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscribed_addresses[i]) == NRF_SUCCESS);
    }

    return ACCESS_STATUS_SUCCESS;
}

/* Sends the subscription status in response to subscription messages. */
static void send_subscription_status(access_model_handle_t this_handle, const access_message_rx_t * p_message,
        uint16_t element_address, uint16_t subscription_address, config_model_id_t model_id, bool sig_model)
{
    config_msg_subscription_status_t packet;

    packet.status = ACCESS_STATUS_SUCCESS;
    packet.element_address = element_address;
    packet.address = subscription_address;
    packet.model_id = model_id;

    send_reply(this_handle, p_message, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &packet,
            PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
}

static void send_netkey_status(access_model_handle_t handle, const access_message_rx_t * p_message,
        config_msg_key_index_12_t key_index)
{
    const config_msg_netkey_status_t response = { ACCESS_STATUS_SUCCESS, key_index };
    send_reply(handle, p_message, CONFIG_OPCODE_NETKEY_STATUS, (const uint8_t *) &response, sizeof(response));
}

static uint16_t get_element_index(uint16_t element_address)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if (element_address < node_address.address_start)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }

    uint16_t retval = element_address - node_address.address_start;
    if (retval >= (uint16_t) ACCESS_ELEMENT_COUNT)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }
    else
    {
        return retval;
    }
}

static inline access_status_t get_subscription_list(access_model_handle_t model_handle, uint16_t * p_sublist, uint16_t * p_subcount)
{
    uint32_t status = access_model_subscriptions_get(model_handle, p_sublist, p_subcount);
    switch (status)
    {
        case NRF_SUCCESS:
            return ACCESS_STATUS_SUCCESS;
        case NRF_ERROR_NOT_SUPPORTED:
            return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
        case NRF_ERROR_NOT_FOUND:
            return ACCESS_STATUS_INVALID_MODEL;
        default:
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
    }
}

static inline void extract_model_id(access_model_id_t * p_dest, const config_model_id_t * p_id, bool sig_model)
{
    if (sig_model)
    {
        p_dest->model_id = p_id->model_id;
        p_dest->company_id = ACCESS_COMPANY_ID_NONE;
    }
    else
    {
        p_dest->model_id = p_id->model_id;
        p_dest->company_id = p_id->company_id;
    }
}

/********** Opcode handler functions **********/

static void handle_appkey_add_update(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_add_t))
    {
        return;
    }

    const config_msg_appkey_add_t * p_pdu = (const config_msg_appkey_add_t *) p_message->p_data;

    uint16_t netkey_index, appkey_index;
    config_msg_key_index_24_get(&p_pdu->key_indexes, &netkey_index, &appkey_index);

    uint32_t status;
    dsm_handle_t appkey_handle;
    if (p_message->opcode.opcode == CONFIG_OPCODE_APPKEY_ADD)
    {
        dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
        status = dsm_appkey_add(appkey_index, network_handle, p_pdu->appkey, &appkey_handle);
    }
    else
    {
        status = dsm_appkey_update(dsm_app_key_index_to_appkey_handle(appkey_index), p_pdu->appkey);
    }

    if (status != NRF_SUCCESS)
    {
        access_status_t status_code;
        switch (status)
        {
            case NRF_ERROR_INVALID_PARAM:
                status_code = ACCESS_STATUS_INVALID_APPKEY;
                break;
            case NRF_ERROR_FORBIDDEN:
                status_code = ACCESS_STATUS_KEY_INDEX_ALREADY_STORED;
                break;
            case NRF_ERROR_NO_MEM:
                status_code = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            case NRF_ERROR_NOT_FOUND:
                if (p_message->opcode.opcode == CONFIG_OPCODE_APPKEY_ADD)
                {
                    status_code = ACCESS_STATUS_INVALID_NETKEY;
                }
                else
                {
                    status_code = ACCESS_STATUS_INVALID_APPKEY;
                }
                break;
            default:
                status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        send_generic_error_reply(handle, p_message, status_code, CONFIG_OPCODE_APPKEY_STATUS,
                sizeof(config_msg_appkey_status_t));
    }
    else
    {
        send_appkey_status(handle, p_message, p_pdu->key_indexes);
    }
}

static void handle_appkey_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_delete_t))
    {
        return;
    }

    const config_msg_appkey_delete_t * p_pdu = (const config_msg_appkey_delete_t *) p_message->p_data;

    uint16_t appkey_index;
    config_msg_key_index_24_get(&p_pdu->key_indexes, NULL, &appkey_index);

    uint32_t status = dsm_appkey_delete(appkey_index);
    switch (status)
    {
        case NRF_SUCCESS:
            send_appkey_status(handle, p_message, p_pdu->key_indexes);
            break;
        case NRF_ERROR_NOT_FOUND:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_APPKEY, CONFIG_OPCODE_APPKEY_STATUS,
                    sizeof(config_msg_appkey_status_t));
            break;
        default:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_APPKEY_STATUS,
                    sizeof(config_msg_appkey_status_t));
            break;
    }
}

static void handle_appkey_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_get_t))
    {
        return;
    }

    const config_msg_appkey_get_t * p_pdu = (const config_msg_appkey_get_t *) p_message->p_data;

    uint16_t netkey_index = p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    mesh_key_index_t appkeys[DSM_APP_MAX];
    uint32_t appkey_count = DSM_APP_MAX;
    uint32_t status = dsm_appkey_get_all(dsm_net_key_index_to_subnet_handle(netkey_index),
        appkeys, &appkey_count);

    uint8_t packet_buffer[sizeof(config_msg_appkey_list_t) + packed_index_list_size(appkey_count)];
    config_msg_appkey_list_t * p_reply = (config_msg_appkey_list_t *) packet_buffer;

    switch (status)
    {
        case NRF_SUCCESS:
            p_reply->status = ACCESS_STATUS_SUCCESS;
            p_reply->netkey_index = p_pdu->netkey_index;
            packed_index_list_create(appkeys, (uint8_t *) p_reply->packed_appkey_indexes, appkey_count);
            send_reply(handle, p_message, CONFIG_OPCODE_APPKEY_LIST, packet_buffer, sizeof(packet_buffer));
            break;
        case NRF_ERROR_NOT_FOUND:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_NETKEY, CONFIG_OPCODE_APPKEY_LIST,
                    sizeof(config_msg_appkey_list_t));
            break;
        default:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_APPKEY_LIST,
                    sizeof(config_msg_appkey_list_t));
            break;
    }
}

static void handle_config_beacon_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_net_beacon_set_t))
    {
        return;
    }

    const config_msg_net_beacon_set_t * p_pdu = (const config_msg_net_beacon_set_t *) p_message->p_data;
    if (p_pdu->beacon_state == CONFIG_NET_BEACON_STATE_ENABLED || p_pdu->beacon_state == CONFIG_NET_BEACON_STATE_DISABLED)
    {
        net_beacon_state_set(p_pdu->beacon_state == CONFIG_NET_BEACON_STATE_ENABLED ? true : false);
        send_net_beacon_state(handle, p_message);
    }
}

static void handle_config_beacon_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length == 0)
    {
        send_net_beacon_state(handle, p_message);
    }
}

static void handle_composition_data_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_composition_data_get_t))
    {
        return;
    }

    const config_msg_composition_data_get_t * p_pdu = (const config_msg_composition_data_get_t *) p_message->p_data;
    if (p_pdu->page_number != 0)
    {
        /* Only page 0 is supported in version 1.0 of the Mesh Profile specification. */
        return;
    }

    uint8_t buffer[sizeof(config_msg_composition_data_status_t) + CONFIG_COMPOSITION_DATA_SIZE];
    config_msg_composition_data_status_t * p_response = (config_msg_composition_data_status_t *) buffer;
    p_response->page_number = p_pdu->page_number;
    NRF_MESH_ASSERT(config_composition_data_get(p_response->data, CONFIG_COMPOSITION_DATA_SIZE) == NRF_SUCCESS);

    send_reply(handle, p_message, CONFIG_OPCODE_COMPOSITION_DATA_STATUS, buffer, sizeof(buffer));
}

static void handle_config_default_ttl_getset(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_default_ttl_t * p_pdu = (const config_msg_default_ttl_t *) p_message->p_data;

    uint8_t ttl;
    if (p_message->opcode.opcode == CONFIG_OPCODE_DEFAULT_TTL_SET)
    {
        if (p_message->length != sizeof(config_msg_default_ttl_t))
        {
            return; /* The Default TTL Status message does not support a status code, so simply don't respond to invalid PDUs. */
        }

        access_default_ttl_set(p_pdu->ttl);
        ttl = p_pdu->ttl;
    }
    else
    {
        if (p_message->length != 0)
        {
            return; /* Same as above, return if message is invalid. */
        }
        ttl = access_default_ttl_get();
    }

    send_reply(handle, p_message, CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t *) &ttl, sizeof(ttl));
}

static void handle_config_friend_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

    const config_msg_friend_status_t status_message = { .friend_state = CONFIG_FRIEND_STATE_UNSUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_FRIEND_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_config_friend_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_friend_set_t))
    {
        return;
    }

    const config_msg_friend_status_t status_message = { .friend_state = CONFIG_FRIEND_STATE_UNSUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_FRIEND_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_config_gatt_proxy_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

    const config_msg_proxy_status_t status_message = { .proxy_state = CONFIG_GATT_PROXY_STATE_UNSUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_GATT_PROXY_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_config_gatt_proxy_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_proxy_set_t))
    {
        return;
    }

    const config_msg_proxy_status_t status_message = { .proxy_state = CONFIG_GATT_PROXY_STATE_UNSUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_GATT_PROXY_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_config_model_publication_get(access_model_handle_t this_handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_get_t, p_message))
    {
        return;
    }

    const config_msg_publication_get_t * p_pdu = (const config_msg_publication_get_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_publication_get_t);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(get_element_index(p_pdu->element_address), model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(this_handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
    }
    else
    {
        send_publication_status(this_handle, p_message, p_pdu->element_address, model_handle);
    }
}

static void handle_config_model_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_set_t, p_message))
            ||
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_virtual_set_t, p_message))
       )
    {
        return;
    }

    bool sig_model;
    uint16_t element_address, publish_address = NRF_MESH_ADDR_UNASSIGNED;
    const uint8_t * publish_address_uuid = NULL;
    const config_publication_params_t * p_pubstate;

    /* Extract fields that are different (or placed at different offsets) based on the incoming opcode: */
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        const config_msg_publication_set_t * p_pdu = (const config_msg_publication_set_t *) p_message->p_data;
        sig_model = IS_SIG_MODEL(p_message, config_msg_publication_set_t);
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
        publish_address = p_pdu->publish_address;
    }
    else
    {
        const config_msg_publication_virtual_set_t * p_pdu = (const config_msg_publication_virtual_set_t *) p_message->p_data;
        sig_model = IS_SIG_MODEL(p_message, config_msg_publication_virtual_set_t);
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
        publish_address_uuid = p_pdu->publish_uuid;
    }

    uint16_t element_index = get_element_index(element_address);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pubstate->model_id, sig_model);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
        return;
    }

    /* Get the application key handle for the application key to publish on: */
    dsm_handle_t publish_appkey_handle = dsm_app_key_index_to_appkey_handle(p_pubstate->appkey_index);
    if (publish_appkey_handle == DSM_HANDLE_INVALID)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_APPKEY, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
        return;
    }

    /* Add the publish address to the DSM: */
    dsm_handle_t publish_address_handle;
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        status = dsm_address_publish_add(publish_address, &publish_address_handle);
    }
    else
    {
        status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
    }

    switch (status)
    {
        case NRF_ERROR_NO_MEM:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_INSUFFICIENT_RESOURCES, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
            return;
        case NRF_SUCCESS:
            break;
        default:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
            return;
    }

    NRF_MESH_ASSERT(access_model_publish_address_set(model_handle, publish_address_handle) == NRF_SUCCESS);

    access_publish_period_t publish_period;
    publish_period.step_res = p_pubstate->publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS;
    publish_period.step_num = p_pubstate->publish_period & ~(0xff << ACCESS_PUBLISH_STEP_NUM_BITS);

    if (publish_period.step_num != 0)
    {
        /* Disable publishing for the model while updating the publication parameters: */
        status = access_model_publish_period_set(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0);
        switch (status)
        {
            case NRF_SUCCESS:
                break;

            case NRF_ERROR_NOT_SUPPORTED:
                send_generic_error_reply(handle, p_message, ACCESS_STATUS_PERIODIC_NOT_SUPPORTED, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,
                                         PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model));
                return;

            default:
                /* No other error should be possible. */
                NRF_MESH_ASSERT(false);
                return;
        }

        /* Set publishing parameters for the model: */
        NRF_MESH_ASSERT(access_model_publish_period_set(model_handle, (access_publish_resolution_t) publish_period.step_res,
            publish_period.step_num) == NRF_SUCCESS);
    }

    NRF_MESH_ASSERT(access_model_publish_application_set(model_handle, publish_appkey_handle) == NRF_SUCCESS);
    NRF_MESH_ASSERT(access_model_publish_ttl_set(model_handle, p_pubstate->publish_ttl) == NRF_SUCCESS);
    access_flash_config_store();

    send_publication_status(handle, p_message, element_address, model_handle);
}

static void handle_config_model_subscription_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_add_del_owr_t);

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if (address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_ADDRESS, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Add the address to the DSM as a subscription address: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(p_pdu->address, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_INVALID_ADDR:
                access_status = ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        send_generic_error_reply(handle, p_message, access_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);

        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
        else
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
    }
    else
    {
        access_flash_config_store();
        send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
                p_pdu->model_id, sig_model);
    }
}

static void handle_config_model_subscription_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_add_del_owr_t);

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if (address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_ADDRESS, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Get the handle for the subscription address: */
    dsm_handle_t subscription_address_handle;
    nrf_mesh_address_t group_address = { NRF_MESH_ADDRESS_TYPE_GROUP, p_pdu->address };
    status = dsm_address_handle_get(&group_address, &subscription_address_handle);
    if (status == NRF_ERROR_NOT_FOUND)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_ADDRESS, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }
    else if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Remove the subscription from the model: */
    status = access_model_subscription_remove(model_handle, subscription_address_handle);
    if (status == NRF_ERROR_NOT_SUPPORTED)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }
    else if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
    access_flash_config_store();
    send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
            p_pdu->model_id, sig_model);
}

static void handle_config_model_subscription_delete_all(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_delete_all_t, p_message))
    {
        return;
    }

    const config_msg_subscription_delete_all_t * p_pdu = (const config_msg_subscription_delete_all_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_delete_all_t);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, reply_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
    }
    else
    {
        access_flash_config_store();
        send_subscription_status(handle, p_message, p_pdu->element_address, NRF_MESH_ADDR_UNASSIGNED,
                p_pdu->model_id, sig_model);
    }
}

static void handle_config_model_subscription_overwrite(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_add_del_owr_t);

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if (address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_ADDRESS, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Delete the old subscription list: */
    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, reply_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Add the new address to the DSM: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(p_pdu->address, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_INVALID_ADDR:
                access_status = ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        send_generic_error_reply(handle, p_message, access_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                   PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
        else
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                   PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
    }
    else
    {
        access_flash_config_store();
        send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
                p_pdu->model_id, sig_model);
    }
}

static void handle_config_model_subscription_virtual_address_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_virtual_add_del_owr_t);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Check if the address exists in the DSM, and if not, add it: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_virtual_add(p_pdu->virtual_uuid, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        send_generic_error_reply(handle, p_message, access_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
        else
        {
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                    PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        }
    }
    else
    {
        nrf_mesh_address_t target_address;
        NRF_MESH_ASSERT(dsm_address_get(subscription_address_handle, &target_address) == NRF_SUCCESS);
        access_flash_config_store();
        send_subscription_status(handle, p_message, p_pdu->element_address,
                target_address.value, p_pdu->model_id, sig_model);
    }
}

static void handle_config_model_subscription_virtual_address_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_virtual_add_del_owr_t);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Get the 16-bit address from the virtual UUID: */
    dsm_handle_t subscription_address_handle;
    nrf_mesh_address_t virtual_address = { NRF_MESH_ADDRESS_TYPE_VIRTUAL, NRF_MESH_ADDR_UNASSIGNED, p_pdu->virtual_uuid };
    NRF_MESH_ASSERT(nrf_mesh_keygen_virtual_address(p_pdu->virtual_uuid, &virtual_address.value) == NRF_SUCCESS);

    /* Get the address handle from the DSM: */
    status = dsm_address_handle_get(&virtual_address, &subscription_address_handle);
    if (status == NRF_ERROR_NOT_FOUND)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_ADDRESS, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }
    else if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Remove the subscription: */
    status = access_model_subscription_remove(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_CANNOT_REMOVE, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
    access_flash_config_store();
    send_subscription_status(handle, p_message, p_pdu->element_address, virtual_address.value,
            p_pdu->model_id, sig_model);
}

static void handle_config_model_subscription_virtual_address_overwrite(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_subscription_virtual_add_del_owr_t);

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Delete the old subscription list: */
    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
        return;
    }

    /* Get the 16-bit address from the virtual UUID: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_virtual_add(p_pdu->virtual_uuid, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        send_generic_error_reply(handle, p_message, access_status, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
    }
    else
    {
        /* Add the subscription to the model: */
        status = access_model_subscription_add(model_handle, subscription_address_handle);
        if (status != NRF_SUCCESS)
        {
            NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
            if (status == NRF_ERROR_NOT_SUPPORTED)
            {
                send_generic_error_reply(handle, p_message, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                        PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
            }
            else
            {
                send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,
                        PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model));
            }
        }
        else
        {
            nrf_mesh_address_t target_address;
            NRF_MESH_ASSERT(dsm_address_get(subscription_address_handle, &target_address) == NRF_SUCCESS);
            send_subscription_status(handle, p_message, p_pdu->element_address,
                    target_address.value, p_pdu->model_id, sig_model);
        }
    }
    access_flash_config_store();
}

static void send_relay_status(access_model_handle_t handle, const access_message_rx_t * p_message)
{
    config_msg_relay_status_t status_message = { 0 };

    nrf_mesh_opt_t param_value;
    NRF_MESH_ASSERT(nrf_mesh_opt_get(NRF_MESH_OPT_NET_RELAY_ENABLE, &param_value) == NRF_SUCCESS);
    if (param_value.opt.val == 0)
    {
        status_message.relay_state = CONFIG_RELAY_STATE_SUPPORTED_DISABLED;
    }
    else
    {
        status_message.relay_state = CONFIG_RELAY_STATE_SUPPORTED_ENABLED;
    }

    NRF_MESH_ASSERT(nrf_mesh_opt_get(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT, &param_value) == NRF_SUCCESS);
    status_message.relay_retransmit_count = param_value.opt.val;

#if 0 /* TODO: support for getting the retransmit interval steps must be implemented in the network layer (MBTLE-1628). */
    NRF_MESH_ASSERT(nrf_mesh_opt_get(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_STEPS, &param_value) == NRF_SUCCESS);
    status_message.relay_retransmit_interval_steps = param_value.opt.val;
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_RELAY_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_config_relay_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0) /* Ignore messages with invalid length */
    {
        return;
    }

    send_relay_status(handle, p_message);
}

static void handle_config_relay_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_relay_set_t * p_pdu = (const config_msg_relay_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_relay_set_t))
    {
        return;
    }

    nrf_mesh_opt_t param_value = { 0 };
    param_value.len = sizeof(param_value.opt.val);

    /* Set the relay state: */
    switch (p_pdu->relay_state)
    {
        case CONFIG_RELAY_STATE_SUPPORTED_ENABLED:
            param_value.opt.val = 1;
            NRF_MESH_ASSERT(nrf_mesh_opt_set(NRF_MESH_OPT_NET_RELAY_ENABLE, &param_value) == NRF_SUCCESS);
            break;
        case CONFIG_RELAY_STATE_SUPPORTED_DISABLED:
            param_value.opt.val = 0;
            NRF_MESH_ASSERT(nrf_mesh_opt_set(NRF_MESH_OPT_NET_RELAY_ENABLE, &param_value) == NRF_SUCCESS);
            break;
        case CONFIG_RELAY_STATE_UNSUPPORTED:
        default: /* Ignore invalid values. */
            break;
    }

    /* Set the retransmit count: */
    param_value.opt.val = p_pdu->relay_retransmit_count;
    NRF_MESH_ASSERT(nrf_mesh_opt_set(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT, &param_value) == NRF_SUCCESS);

#if 0 /* TODO: support for getting the retransmit interval steps must be implemented in the network layer (MBTLE-1628). */
    /* Set the retransmit interval steps: */
    param_value.opt.val = p_pdu->relay_retransmit_interval_steps;
    NRF_MESH_ASSERT(nrf_mesh_opt_set(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_STEPS, &param_value) == NRF_SUCCESS);
#endif

    send_relay_status(handle, p_message);
}

static void handle_config_sig_model_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_message->p_data;
    if (p_message->length != PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, true))
    {
        return;
    }

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_id_t model_id = { .model_id = p_pdu->model_id.model_id, .company_id = ACCESS_COMPANY_ID_NONE };
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL,
                CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, sizeof(config_msg_sig_model_subscription_list_t));
        return;
    }

    uint16_t subscription_count = DSM_ADDR_MAX;
    uint16_t subscription_list[DSM_ADDR_MAX];
    access_status_t error_code = get_subscription_list(model_handle, subscription_list, &subscription_count);
    if (error_code != ACCESS_STATUS_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, error_code, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST,
                sizeof(config_msg_sig_model_subscription_list_t));
        return;
    }

    uint8_t response_buffer[sizeof(config_msg_sig_model_subscription_list_t) + subscription_count * sizeof(uint16_t)];
    config_msg_sig_model_subscription_list_t * p_response = (config_msg_sig_model_subscription_list_t *) response_buffer;

    p_response->status = ACCESS_STATUS_SUCCESS;
    p_response->element_address = p_pdu->element_address;
    p_response->sig_model_id = p_pdu->model_id.model_id;
    memcpy((void *) p_response->subscriptions, subscription_list, subscription_count * sizeof(uint16_t));

    send_reply(handle, p_message, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, response_buffer,
            sizeof(config_msg_sig_model_subscription_list_t) + subscription_count * sizeof(uint16_t));
}

static void handle_config_vendor_model_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_message->p_data;
    if (p_message->length != PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, false))
    {
        return;
    }

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_id_t model_id = { .model_id = p_pdu->model_id.model_id, .company_id = p_pdu->model_id.company_id };
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL,
                CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, sizeof(config_msg_vendor_model_subscription_list_t));
        return;
    }

    uint16_t subscription_count = DSM_ADDR_MAX;
    uint16_t subscription_list[DSM_ADDR_MAX];
    access_status_t error_code = get_subscription_list(model_handle, subscription_list, &subscription_count);
    if (error_code != ACCESS_STATUS_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, error_code, CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST,
                sizeof(config_msg_sig_model_subscription_list_t));
        return;
    }

    const uint16_t subscription_list_size = subscription_count * sizeof(uint16_t);
    uint8_t response_buffer[sizeof(config_msg_vendor_model_subscription_list_t) + subscription_list_size];
    config_msg_vendor_model_subscription_list_t * p_response = (config_msg_vendor_model_subscription_list_t *) response_buffer;

    p_response->status = ACCESS_STATUS_SUCCESS;
    p_response->element_address = p_pdu->element_address;
    p_response->vendor_model_id = p_pdu->model_id.model_id;
    p_response->vendor_company_id = p_pdu->model_id.company_id;
    memcpy((void *) p_response->subscriptions, subscription_list, subscription_list_size);

    send_reply(handle, p_message, CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, response_buffer,
            sizeof(config_msg_vendor_model_subscription_list_t) + subscription_list_size);
}

static void handle_heartbeat_publication_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_heartbeat_publication_status_t status_message = { .status = ACCESS_STATUS_FEATURE_NOT_SUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_heartbeat_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_heartbeat_publication_status_t status_message = { .status = ACCESS_STATUS_FEATURE_NOT_SUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_heartbeat_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_heartbeat_subscription_status_t status_message = { .status = ACCESS_STATUS_FEATURE_NOT_SUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_heartbeat_subscription_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_heartbeat_subscription_status_t status_message = { .status = ACCESS_STATUS_FEATURE_NOT_SUPPORTED };
    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, (const uint8_t *) &status_message, sizeof(status_message));
}

static void handle_model_app_bind_unbind(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_app_bind_unbind_t, p_message))
    {
        return;
    }

    const config_msg_app_bind_unbind_t * p_pdu = (const config_msg_app_bind_unbind_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    bool sig_model = IS_SIG_MODEL(p_message, config_msg_app_bind_unbind_t);
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);

    config_msg_app_status_t response;
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = p_pdu->element_address;
    response.appkey_index = p_pdu->appkey_index;
    response.model_id = p_pdu->model_id;

    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(get_element_index(p_pdu->element_address), model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL,
                CONFIG_OPCODE_MODEL_APP_STATUS, PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model));
        return;
    }

    uint16_t appkey_index = p_pdu->appkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    dsm_handle_t appkey_handle = dsm_app_key_index_to_appkey_handle(appkey_index);

    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_APP_BIND)
    {
        status = access_model_application_bind(model_handle, appkey_handle);
    }
    else
    {
        status = access_model_application_unbind(model_handle, appkey_handle);
    }

    switch (status)
    {
        case NRF_SUCCESS:
            response.status = ACCESS_STATUS_SUCCESS;
            access_flash_config_store();
            break;
        case NRF_ERROR_NOT_FOUND:
            response.status = ACCESS_STATUS_INVALID_MODEL;
            break;
        case NRF_ERROR_INVALID_PARAM:
            response.status = ACCESS_STATUS_INVALID_APPKEY;
            break;
        default:
            response.status = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model));
}

static void handle_netkey_add_update(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_netkey_add_update_t * p_pdu = (const config_msg_netkey_add_update_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_netkey_add_update_t))
    {
        return;
    }

    uint16_t netkey_index = p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    uint32_t status;
    dsm_handle_t network_handle;
    if (p_message->opcode.opcode == CONFIG_OPCODE_NETKEY_ADD)
    {
        status = dsm_subnet_add(netkey_index, p_pdu->netkey, &network_handle);
    }
    else
    {
        network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
        status = dsm_subnet_update(network_handle, p_pdu->netkey);
    }

    if (status != NRF_SUCCESS)
    {
        access_status_t status_code;

        switch (status)
        {
            case NRF_ERROR_INVALID_PARAM:
                status_code = ACCESS_STATUS_INVALID_NETKEY;
                break;
            case NRF_ERROR_FORBIDDEN:
                status_code = ACCESS_STATUS_KEY_INDEX_ALREADY_STORED;
                break;
            case NRF_ERROR_NO_MEM:
                status_code = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            case NRF_ERROR_NOT_FOUND:
                status_code = ACCESS_STATUS_INVALID_NETKEY;
                break;
            default:
                status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }

        send_generic_error_reply(handle, p_message, status_code, CONFIG_OPCODE_NETKEY_STATUS, sizeof(config_msg_netkey_status_t));
        return;
    }

    send_netkey_status(handle, p_message, p_pdu->netkey_index);
}

static void handle_netkey_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_netkey_delete_t * p_pdu = (const config_msg_netkey_delete_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_netkey_delete_t))
    {
        return;
    }

    dsm_handle_t network_handle;
    network_handle = dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK);

    uint32_t status = dsm_subnet_delete(network_handle);
    switch (status)
    {
        case NRF_ERROR_NOT_FOUND:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_NETKEY, CONFIG_OPCODE_NETKEY_STATUS, sizeof(config_msg_netkey_status_t));
            break;
        case NRF_SUCCESS:
            send_netkey_status(handle, p_message, p_pdu->netkey_index);
            break;
        default:
            send_generic_error_reply(handle, p_message, ACCESS_STATUS_UNSPECIFIED_ERROR, CONFIG_OPCODE_NETKEY_STATUS, sizeof(config_msg_netkey_status_t));
            break;
    }
}

static void handle_netkey_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

    uint32_t num_netkeys = DSM_SUBNET_MAX;
    mesh_key_index_t netkey_indexes[num_netkeys];
    (void) dsm_subnet_get_all(netkey_indexes, &num_netkeys);

    /* The 12-bit netkey indexes needs to be packed into an array before being sent to the client: */
    uint8_t buffer[packed_index_list_size(num_netkeys)];
    packed_index_list_create(netkey_indexes, buffer, num_netkeys);

    send_reply(handle, p_message, CONFIG_OPCODE_NETKEY_LIST, buffer, sizeof(buffer));
}

static void handle_node_identity_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_identity_get_t))
    {
        return;
    }

    const config_msg_identity_get_t * p_pdu = (const config_msg_identity_get_t *) p_message->p_data;
    const config_msg_identity_status_t reply =
    {
        .status = ACCESS_STATUS_SUCCESS,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = CONFIG_IDENTITY_STATE_UNSUPPORTED
    };

    send_reply(handle, p_message, CONFIG_OPCODE_NODE_IDENTITY_STATUS, (const uint8_t *) &reply, sizeof(config_msg_identity_status_t));
}

static void handle_node_identity_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_identity_set_t))
    {
        return;
    }

    const config_msg_identity_set_t * p_pdu = (const config_msg_identity_set_t *) p_message->p_data;
    const config_msg_identity_status_t reply =
    {
        .status = ACCESS_STATUS_SUCCESS,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = CONFIG_IDENTITY_STATE_UNSUPPORTED
    };

    send_reply(handle, p_message, CONFIG_OPCODE_NODE_IDENTITY_STATUS, (const uint8_t *) &reply, sizeof(config_msg_identity_status_t));
}

static void handle_node_reset(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    uint8_t status = ACCESS_STATUS_FEATURE_NOT_SUPPORTED;
    send_reply(handle, p_message, CONFIG_OPCODE_NODE_RESET_STATUS, (const uint8_t *) &status, sizeof(status));

    /* TODO: Handling this message involves clearing all stored state and returning the node to an unprovisioned state, see MBTLE-1626. */
}

static void handle_model_app_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_model_app_get_t, p_message))
    {
        return;
    }

    bool sig_model = p_message->opcode.opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET;
    const config_msg_model_app_get_t * p_pdu = (const config_msg_model_app_get_t *) p_message->p_data;

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);

    /* Get the model handle: */
    access_model_handle_t model_handle;
    access_model_id_t model_id;
    extract_model_id(&model_id, &p_pdu->model_id, sig_model);
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        send_generic_error_reply(handle, p_message, ACCESS_STATUS_INVALID_MODEL,
                CONFIG_OPCODE_SIG_MODEL_APP_LIST, sizeof(config_msg_sig_model_app_list_t));
        return;
    }

    /* Get the application list: */
    dsm_handle_t appkey_handles[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    NRF_MESH_ASSERT(access_model_applications_get(model_handle, appkey_handles, &appkey_count) == NRF_SUCCESS);

    /* Retrieve the appkey indexes from the DSM: */
    mesh_key_index_t appkey_list[appkey_count];
    for (uint16_t i = 0; i < appkey_count; ++i)
    {
        NRF_MESH_ASSERT(dsm_appkey_handle_to_appkey_index(appkey_handles[i], &appkey_list[i]) == NRF_SUCCESS);
    }

    /* Create and send the response message: */
    if (p_message->opcode.opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET)
    {
        uint8_t response_buffer[sizeof(config_msg_sig_model_app_list_t) + packed_index_list_size(appkey_count)];
        config_msg_sig_model_app_list_t * p_response = (config_msg_sig_model_app_list_t *) response_buffer;
        p_response->status = ACCESS_STATUS_SUCCESS;
        p_response->element_address = p_pdu->element_address;
        p_response->sig_model_id = model_id.model_id;
        packed_index_list_create(appkey_list, p_response->key_indexes, appkey_count);

        send_reply(handle, p_message, CONFIG_OPCODE_SIG_MODEL_APP_LIST, response_buffer, sizeof(response_buffer));
    }
    else
    {
        uint8_t response_buffer[sizeof(config_msg_vendor_model_app_list_t) + packed_index_list_size(appkey_count)];
        config_msg_vendor_model_app_list_t * p_response = (config_msg_vendor_model_app_list_t *) response_buffer;
        p_response->status = ACCESS_STATUS_SUCCESS;
        p_response->element_address = p_pdu->element_address;
        p_response->vendor_model_id = model_id.model_id;
        p_response->vendor_company_id = model_id.company_id;
        packed_index_list_create(appkey_list, p_response->key_indexes, appkey_count);

        send_reply(handle, p_message, CONFIG_OPCODE_VENDOR_MODEL_APP_LIST, response_buffer, sizeof(response_buffer));
    }
}

/********** Opcode handler list *********/
static const access_opcode_handler_t opcode_handlers[] =
{
    {{ .opcode = CONFIG_OPCODE_APPKEY_ADD, .company_id = ACCESS_COMPANY_ID_NONE }, handle_appkey_add_update },
    {{ .opcode = CONFIG_OPCODE_APPKEY_UPDATE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_appkey_add_update },
    {{ .opcode = CONFIG_OPCODE_MODEL_PUBLICATION_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_publication_set },
    {{ .opcode = CONFIG_OPCODE_APPKEY_DELETE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_appkey_delete },
    {{ .opcode = CONFIG_OPCODE_APPKEY_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_appkey_get },
    {{ .opcode = CONFIG_OPCODE_COMPOSITION_DATA_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_composition_data_get },
    {{ .opcode = CONFIG_OPCODE_BEACON_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_beacon_get },
    {{ .opcode = CONFIG_OPCODE_BEACON_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_beacon_set },
    {{ .opcode = CONFIG_OPCODE_DEFAULT_TTL_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_default_ttl_getset },
    {{ .opcode = CONFIG_OPCODE_DEFAULT_TTL_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_default_ttl_getset },
    {{ .opcode = CONFIG_OPCODE_FRIEND_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_friend_get },
    {{ .opcode = CONFIG_OPCODE_FRIEND_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_friend_set },
    {{ .opcode = CONFIG_OPCODE_GATT_PROXY_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_gatt_proxy_get },
    {{ .opcode = CONFIG_OPCODE_GATT_PROXY_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_gatt_proxy_set },
    {{ .opcode = CONFIG_OPCODE_MODEL_PUBLICATION_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_publication_get },
    {{ .opcode = CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_publication_set },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_add },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_delete },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_delete_all },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_overwrite },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_virtual_address_add },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_virtual_address_delete },
    {{ .opcode = CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_model_subscription_virtual_address_overwrite },
    {{ .opcode = CONFIG_OPCODE_RELAY_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_relay_get },
    {{ .opcode = CONFIG_OPCODE_RELAY_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_relay_set },
    {{ .opcode = CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_sig_model_subscription_get },
    {{ .opcode = CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_config_vendor_model_subscription_get },
    {{ .opcode = CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_heartbeat_publication_get },
    {{ .opcode = CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_heartbeat_publication_set },
    {{ .opcode = CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_heartbeat_subscription_get },
    {{ .opcode = CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_heartbeat_subscription_set },
    {{ .opcode = CONFIG_OPCODE_MODEL_APP_BIND, .company_id = ACCESS_COMPANY_ID_NONE }, handle_model_app_bind_unbind },
    {{ .opcode = CONFIG_OPCODE_MODEL_APP_UNBIND, .company_id = ACCESS_COMPANY_ID_NONE }, handle_model_app_bind_unbind },
    {{ .opcode = CONFIG_OPCODE_NETKEY_ADD, .company_id = ACCESS_COMPANY_ID_NONE }, handle_netkey_add_update },
    {{ .opcode = CONFIG_OPCODE_NETKEY_DELETE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_netkey_delete },
    {{ .opcode = CONFIG_OPCODE_NETKEY_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_netkey_get },
    {{ .opcode = CONFIG_OPCODE_NETKEY_UPDATE, .company_id = ACCESS_COMPANY_ID_NONE }, handle_netkey_add_update },
    {{ .opcode = CONFIG_OPCODE_NODE_IDENTITY_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_node_identity_get },
    {{ .opcode = CONFIG_OPCODE_NODE_IDENTITY_SET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_node_identity_set },
    {{ .opcode = CONFIG_OPCODE_NODE_RESET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_node_reset },
    {{ .opcode = CONFIG_OPCODE_SIG_MODEL_APP_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_model_app_get },
    {{ .opcode = CONFIG_OPCODE_VENDOR_MODEL_APP_GET, .company_id = ACCESS_COMPANY_ID_NONE }, handle_model_app_get },
};

uint32_t config_server_init(void)
{
    const access_model_add_params_t init_params =
    {
        .model_id = {
            .model_id = CONFIG_SERVER_MODEL_ID,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .element_index = 0,
        .p_opcode_handlers = opcode_handlers,
        .opcode_count = sizeof(opcode_handlers) / sizeof(access_opcode_handler_t)
    };

#if !UNIT_TEST
    /* Ensure the model has not already been initialized. */
    NRF_MESH_ASSERT(m_config_server_handle == ACCESS_HANDLE_INVALID);
#endif

    return access_model_add(&init_params, &m_config_server_handle);
}

uint32_t config_server_bind(dsm_handle_t devkey_handle)
{
    return access_model_application_bind(m_config_server_handle, devkey_handle);
}
