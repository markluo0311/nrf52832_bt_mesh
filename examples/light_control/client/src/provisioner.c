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

#include "provisioner.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "log.h"
#include "nrf_mesh_assert.h"

#include "nrf_mesh_prov.h"
#include "nrf_mesh_events.h"
#include "device_state_manager.h"

#include "config_client.h"
#include "access_config.h"
#include "simple_on_off_server.h"

#include "nrf_mesh_sdk.h"

typedef enum
{
    PROV_STATE_IDLE,
    PROV_STATE_WAIT,
    PROV_STATE_PROV,
    PROV_STATE_CONFIG_FIRST,
    PROV_STATE_CONFIG_COMPOSITION_GET,
    PROV_STATE_CONFIG_APPKEY_ADD,
    PROV_STATE_CONFIG_APPKEY_BIND,
    PROV_STATE_CONFIG_PUBLICATION,
    PROV_STATE_CONFIG_SUBSCRIPTION
} prov_state_t;

/* Provisioning encryption key storage (this is not how you should store your keys). */
static uint8_t m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];

static nrf_mesh_prov_ctx_t m_prov_ctx;
static nrf_mesh_evt_handler_t m_evt_handler;

static prov_state_t m_prov_state;
static uint16_t m_next_unprov_address;


static void start_provisioning(const uint8_t * p_uuid)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
        {
            .netkey = NETKEY,
            .netkey_index = NETKEY_INDEX,
            .iv_index = 0,
            .address = m_next_unprov_address,
            .flags.iv_update = false,
            .flags.key_refresh = false
        };
    nrf_mesh_prov_oob_caps_t capabilities = {0};
    capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, m_public_key, m_private_key, &capabilities));
    ERROR_CHECK(nrf_mesh_prov_provision(&m_prov_ctx, p_uuid, &prov_data, NRF_MESH_PROV_BEARER_ADV));
    m_prov_state = PROV_STATE_PROV;
}

static void do_config_step(void)
{
    switch (m_prov_state)
    {
        case PROV_STATE_CONFIG_FIRST:
            m_prov_state = PROV_STATE_CONFIG_COMPOSITION_GET;
        case PROV_STATE_CONFIG_COMPOSITION_GET:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Getting composition data\n");
            ERROR_CHECK(config_client_composition_data_get());
            break;
        }
        case PROV_STATE_CONFIG_APPKEY_ADD:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding appkey\n");
            uint8_t appkey[NRF_MESH_KEY_SIZE] = APPKEY;
            ERROR_CHECK(config_client_appkey_add(NETKEY_INDEX, APPKEY_INDEX, appkey));
            break;
        }
        case PROV_STATE_CONFIG_APPKEY_BIND:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Binding appkey\n");
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            model_id.model_id = SIMPLE_ON_OFF_SERVER_MODEL_ID;
            uint16_t element_address = m_next_unprov_address;
            ERROR_CHECK(config_client_model_app_bind(element_address, APPKEY_INDEX, model_id));
            break;
        }
        case PROV_STATE_CONFIG_PUBLICATION:
        {
            config_publication_state_t pubstate = {0};
            pubstate.element_address = m_next_unprov_address;
            pubstate.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            pubstate.publish_address.value = PROVISIONER_ADDRESS + m_next_unprov_address - UNPROV_START_ADDRESS;
            pubstate.appkey_index = 0;
            pubstate.frendship_credential_flag = false;
            pubstate.publish_ttl = 3; /* Three servers => Client ttl: 3 -> S1 ttl: 2 -> S2 ttl: 1 -> S3*/
            pubstate.publish_period = 0;
            pubstate.retransmit_count = 1;
            pubstate.retransmit_interval = 0;
            pubstate.model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            pubstate.model_id.model_id = SIMPLE_ON_OFF_SERVER_MODEL_ID;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication state 0x%04x\n", pubstate.publish_address.value);

            ERROR_CHECK(config_client_model_publication_set(&pubstate));
            break;
        }

        case PROV_STATE_CONFIG_SUBSCRIPTION:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription\n");
            uint16_t element_address = m_next_unprov_address;
            nrf_mesh_address_t address = {NRF_MESH_ADDRESS_TYPE_INVALID, 0, NULL};
            address.type = NRF_MESH_ADDRESS_TYPE_GROUP;
            address.value = GROUP_ADDRESS;
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            model_id.model_id = SIMPLE_ON_OFF_SERVER_MODEL_ID;
            ERROR_CHECK(config_client_model_subscription_add(element_address, address, model_id));
            break;
        }

        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void mesh_evt_handler(nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_UNPROVISIONED_RECEIVED:
            if (m_prov_state == PROV_STATE_WAIT)
            {
                start_provisioning(p_evt->params.unprov_recv.device_uuid);
                m_prov_state = PROV_STATE_PROV;
            }
            break;

        case NRF_MESH_EVT_PROV_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link closed\n");
            if (m_prov_state == PROV_STATE_PROV)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning failed. Retrying...\n");
                m_prov_state = PROV_STATE_WAIT;
            }
            else if (m_prov_state == PROV_STATE_CONFIG_FIRST)
            {
                do_config_step();
            }
            break;

        case NRF_MESH_EVT_PROV_COMPLETE:
            m_prov_state = PROV_STATE_IDLE;
            provisioner_prov_complete_cb(&p_evt->params.prov_complete);
            break;

        case NRF_MESH_EVT_PROV_CAPS_RECEIVED:
        {
            uint32_t status = nrf_mesh_prov_oob_use(p_evt->params.prov_oob_caps_received.p_context,
                                                    NRF_MESH_PROV_OOB_METHOD_STATIC,
                                                    NRF_MESH_KEY_SIZE);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                      "Provisioning OOB selection rejected, error code %d. Retrying...\n", status);
                m_prov_state = PROV_STATE_WAIT;
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using static authentication\n");
            }
            break;
        }

        case NRF_MESH_EVT_PROV_STATIC_REQUEST:
        {
            uint8_t static_data[16] = STATIC_AUTH_DATA;
            ERROR_CHECK(nrf_mesh_prov_auth_data_provide(p_evt->params.prov_static_request.p_context, static_data, 16));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Static authentication data provided\n");
            break;
        }


        case NRF_MESH_EVT_PROV_LINK_ESTABLISHED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link established\n");
            break;

        case NRF_MESH_EVT_MESSAGE_RECEIVED:
        case NRF_MESH_EVT_TX_COMPLETE:
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Unexpected event: %d.\n", p_evt->type);
            break;
    }
}

void provisioner_configure(void)
{
    m_prov_state = PROV_STATE_CONFIG_FIRST;
}

void provisioner_wait_for_unprov(void)
{
    m_prov_state = PROV_STATE_WAIT;
}

void config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length)
{
    if (event_type == CONFIG_CLIENT_EVENT_TYPE_TIMEOUT)
    {
        provisioner_config_failed_cb();
        return;
    }

    NRF_MESH_ASSERT(p_event != NULL);

    if (p_event->opcode == CONFIG_OPCODE_COMPOSITION_DATA_STATUS &&
        m_prov_state == PROV_STATE_CONFIG_COMPOSITION_GET)
    {
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Composition data", ((const uint8_t *) &p_event->p_msg->composition_data_status), length);
        m_prov_state = PROV_STATE_CONFIG_APPKEY_ADD;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_APPKEY_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_APPKEY_ADD)
    {
        NRF_MESH_ASSERT(p_event->p_msg->appkey_status.status == ACCESS_STATUS_SUCCESS ||
                        p_event->p_msg->appkey_status.status == ACCESS_STATUS_KEY_INDEX_ALREADY_STORED);
        m_prov_state = PROV_STATE_CONFIG_APPKEY_BIND;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_APP_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_APPKEY_BIND)
    {
        NRF_MESH_ASSERT(p_event->p_msg->app_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_PUBLICATION;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_PUBLICATION_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_PUBLICATION)
    {
        NRF_MESH_ASSERT(p_event->p_msg->publication_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_SUBSCRIPTION;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_SUBSCRIPTION)
    {
        NRF_MESH_ASSERT(p_event->p_msg->subscription_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_IDLE;
        provisioner_config_successful_cb();
        m_next_unprov_address++;
    }
    else
    {
        /* Do nothing. */
    }
}

void provisioner_init(void)
{
    m_prov_state = PROV_STATE_IDLE;
    m_next_unprov_address = UNPROV_START_ADDRESS;
    ERROR_CHECK(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));

    m_evt_handler.evt_cb = mesh_evt_handler;
    m_evt_handler.p_next = NULL;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

