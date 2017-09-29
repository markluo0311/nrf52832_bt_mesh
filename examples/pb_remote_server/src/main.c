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

/* HAL */
#include "nrf.h"
#include "nrf_sdm.h"
#include "boards.h"
#include "nrf_mesh_sdk.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "pb_remote_server.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/**
 * Static authentication data. This data must match the data provided to the provisioner node.
 */
#define STATIC_AUTH_DATA { 0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73 }
#define REMOTE_SERVER_ELEMENT_INDEX (0)
#define PROVISIONER_ADDRESS (0x0001)

typedef enum
{
    DEVICE_STATE_UNPROVISIONED,
    DEVICE_STATE_PROVISIONED
} device_state_t;

/*****************************************************************************
 * Static data
 *****************************************************************************/

/* Provisioning encryption key storage (this is not how you should store your keys). */
static uint8_t public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t private_key[NRF_MESH_PROV_PRIVKEY_SIZE];
static nrf_mesh_prov_ctx_t m_prov_ctx;
static pb_remote_server_t m_remote_server;
static nrf_mesh_evt_handler_t m_evt_handler;
static device_state_t m_device_state;
static dsm_handle_t m_devkey_handle;
static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_provisioner_address_handle;

/*****************************************************************************
 * Static functions
 *****************************************************************************/
static void mesh_evt_handler(nrf_mesh_evt_t * p_evt);

static void on_provisioned_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned device. Settting up models...\n");
    ERROR_CHECK(pb_remote_server_init(&m_remote_server, REMOTE_SERVER_ELEMENT_INDEX));

    /* TODO: This should be handled by the configuration server model. */
    uint8_t appkey[NRF_MESH_KEY_SIZE] = {0};
    ERROR_CHECK(dsm_appkey_add(0, m_netkey_handle, appkey, &m_appkey_handle));
    ERROR_CHECK(dsm_address_publish_add(PROVISIONER_ADDRESS, &m_provisioner_address_handle));
    ERROR_CHECK(access_model_application_bind(m_remote_server.model_handle, m_appkey_handle));
    ERROR_CHECK(access_model_publish_address_set(m_remote_server.model_handle, m_provisioner_address_handle));
    ERROR_CHECK(access_model_publish_application_set(m_remote_server.model_handle, m_appkey_handle));
    ERROR_CHECK(access_model_publish_ttl_set(m_remote_server.model_handle, 6));
    ERROR_CHECK(pb_remote_server_enable(&m_remote_server));
}

static void mesh_core_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing softdevice...\n");
#if defined(S130) || defined(S132) || defined(S140)
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
#elif defined(S110)
    nrf_clock_lfclksrc_t lfc_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif
    mesh_softdevice_setup(lfc_cfg);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing mesh stack...\n");
    nrf_mesh_init_params_t mesh_init_params = {
        .lfclksrc = lfc_cfg,
        .assertion_handler = mesh_assert_handler,
    };

    ERROR_CHECK(nrf_mesh_init(&mesh_init_params));

    m_evt_handler.evt_cb = mesh_evt_handler;
    m_evt_handler.p_next = NULL;
    nrf_mesh_evt_handler_add(&m_evt_handler);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling mesh stack...\n");
    ERROR_CHECK(nrf_mesh_enable());

    dsm_init();
    access_init();
}

static void provisionee_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing provisionee context...\n");
    nrf_mesh_prov_oob_caps_t capabilities = {0};

    capabilities.num_elements = ACCESS_ELEMENT_COUNT;
    capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;

    ERROR_CHECK(nrf_mesh_prov_generate_keys(public_key, private_key));
    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, public_key, private_key, &capabilities));
}

static void mesh_evt_handler(nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_PROV_LINK_ESTABLISHED:
            /* A provisioning link has been established. This is a notification so that the application
             * can, for example, show an indication or start measuring the time of the provisioning process.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning link established!\n");
            break;

        case NRF_MESH_EVT_PROV_LINK_CLOSED:
            /* A provisioning link has been closed. This is a notification so that the application can,
             * for example, show an indication or free the provisioning context structure. The link close
             * event does not indicate if the provisioning process was successful, just that the provisioning
             * link has been closed.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning link closed!\n");
            switch (m_device_state)
            {
                case DEVICE_STATE_UNPROVISIONED:
                {
                    nrf_mesh_prov_oob_caps_t capabilities = {0};
                    capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;            /* The algorithm is the only field that needs to be set in this struct. */
                    capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;  /* This examples uses static authentication. */
                    ERROR_CHECK(nrf_mesh_prov_generate_keys(public_key, private_key));

                    /* Restart listening for incoming link requests: */
                    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, public_key, private_key, &capabilities));
                    ERROR_CHECK(nrf_mesh_prov_listen(&m_prov_ctx, NRF_MESH_PROV_BEARER_ADV, NULL, 0));
                    break;
                }
                case DEVICE_STATE_PROVISIONED:
                    on_provisioned_setup();
                    break;
                default:
                    break;
            }
            break;
        case NRF_MESH_EVT_PROV_COMPLETE:
        {
            dsm_local_unicast_address_t local_address;
            local_address.address_start = p_evt->params.prov_complete.address;
            local_address.count = ACCESS_ELEMENT_COUNT;

            ERROR_CHECK(dsm_local_unicast_addresses_set(&local_address));
            ERROR_CHECK(dsm_subnet_add(p_evt->params.prov_complete.netkey_index,
                                       p_evt->params.prov_complete.p_netkey,
                                       &m_netkey_handle));

            ERROR_CHECK(dsm_devkey_add(p_evt->params.prov_complete.address,
                                       m_netkey_handle,
                                       p_evt->params.prov_complete.p_devkey,
                                       &m_devkey_handle));


            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning complete, address %.04x!\n", p_evt->params.prov_complete.address);
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Network key:     ", p_evt->params.prov_complete.p_netkey, NRF_MESH_KEY_SIZE);
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device key:      ", p_evt->params.prov_complete.p_devkey, NRF_MESH_KEY_SIZE);

            m_device_state = DEVICE_STATE_PROVISIONED;
            break;
        }
        case NRF_MESH_EVT_PROV_STATIC_REQUEST:
        {
            uint8_t static_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
            ERROR_CHECK(nrf_mesh_prov_auth_data_provide(p_evt->params.prov_static_request.p_context, static_data, NRF_MESH_KEY_SIZE));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Static authentication data provided!\n");
            break;
        }

        case NRF_MESH_EVT_TX_COMPLETE:
            break;

        case NRF_MESH_EVT_UNPROVISIONED_RECEIVED:
            break;

        case NRF_MESH_EVT_MESSAGE_RECEIVED:
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Unexpected event: %d.\n", p_evt->type);
            break;
    }
}

int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Provisionee + Remote Provisioning Server Demo -----\n");

    mesh_core_setup();
    provisionee_setup();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting listening for incoming provisioning links...\n");
    ERROR_CHECK(nrf_mesh_prov_listen(&m_prov_ctx, NRF_MESH_PROV_BEARER_ADV, NULL, 0));

    while (true)
    {
        nrf_mesh_process();
    }
}
