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
#include "SEGGER_RTT.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "pb_remote_client.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define NET_KEY {0x50, 0x42, 0x5f, 0x52, 0x45, 0x4d, 0x4f, 0x54, 0x45, 0x5f, 0x4e, 0x45, 0x54, 0x4b, 0x45, 0x59}
#define IV_INDEX (0)
#define UNPROV_START_ADDRESS (0x1337)
#define PROVISIONER_ADDRESS (0x0001)

/**
 * Static authentication data. This data must match the data provided to the provisioner node.
 */
#define STATIC_AUTH_DATA { 0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73 }

typedef enum
{
    DEVICE_STATE_NONE,
    DEVICE_STATE_PB_ADV_MODE,
    DEVICE_STATE_PB_REMOTE_MODE
} device_state_t;

const char USAGE_STRING[] =
    "\n--------------------------------\n"
    "1) Provision first available device with PB-ADV\n"
    "2) Set current client publish handle (corresponding to a known server)\n"
    "\t 2.1) <address handle>\n"
    "3) Start remote scanning\n"
    "4) Cancel the remote scanning\n"
    "5) Start remote provisioning\n"
    "\t 5.1) Device number\n";

/*****************************************************************************
 * Static data
 *****************************************************************************/

/* Provisioning encryption key storage (this is not how you should store your keys). */
static const uint8_t m_netkey[NRF_MESH_KEY_SIZE] = NET_KEY;
static uint8_t m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];
static nrf_mesh_prov_ctx_t m_prov_ctx;
static uint16_t m_next_unprov_address = UNPROV_START_ADDRESS;
static uint16_t m_num_elements_of_last_guy = 0;
static pb_remote_client_t m_remote_client;
static nrf_mesh_evt_handler_t m_evt_handler;
static device_state_t m_device_state;
static dsm_handle_t m_devkey_handles[DSM_DEVICE_MAX];
static dsm_handle_t m_netkey_handles[DSM_SUBNET_MAX];
static dsm_handle_t m_appkey_handles[DSM_APP_MAX];
static dsm_handle_t m_device_address_handles[DSM_NONVIRTUAL_ADDR_MAX];
static uint8_t m_next_unprov_index = 0;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void mesh_evt_handler(nrf_mesh_evt_t * p_evt);
static void remote_client_event_cb(const pb_remote_event_t * p_evt);

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
}

static void provisioner_setup(void)
{
    ERROR_CHECK(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
}

static void access_setup(void)
{
    dsm_init();

    uint8_t appkey[NRF_MESH_KEY_SIZE] = {0};
    dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, 1};
    ERROR_CHECK(dsm_local_unicast_addresses_set(&local_address));
    ERROR_CHECK(dsm_subnet_add(0, m_netkey, &m_netkey_handles[0]));
    ERROR_CHECK(dsm_appkey_add(0, m_netkey_handles[0], appkey, &m_appkey_handles[0]));

    access_init();
    ERROR_CHECK(pb_remote_client_init(&m_remote_client, 0, &m_prov_ctx, remote_client_event_cb));
    ERROR_CHECK(access_model_application_bind(m_remote_client.model_handle, m_appkey_handles[0]));
    ERROR_CHECK(access_model_publish_application_set(m_remote_client.model_handle, m_appkey_handles[0]));
    ERROR_CHECK(access_model_publish_ttl_set(m_remote_client.model_handle, 6));
}

static void start_remote_provisioning(uint8_t unprov_id)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
        {
            .netkey = NET_KEY,
            .netkey_index = 0,
            .iv_index = IV_INDEX,
            .address = m_next_unprov_address,
            .flags.iv_update = false,
            .flags.key_refresh = false
        };

    nrf_mesh_prov_oob_caps_t capabilities = {0};
    capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    uint32_t status = pb_remote_client_remote_provision(&m_remote_client,
                                                        &prov_data,
                                                        m_public_key,
                                                        m_private_key,
                                                        &capabilities,
                                                        unprov_id);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not start remote provisioning\n", status);
    }
}

static void start_provisioning(const uint8_t * p_uuid)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
        {
            .netkey = NET_KEY,
            .netkey_index = 0,
            .iv_index = IV_INDEX,
            .address = m_next_unprov_address,
            .flags.iv_update = false,
            .flags.key_refresh = false
        };
    nrf_mesh_prov_oob_caps_t capabilities = {0};
    capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, m_public_key, m_private_key, &capabilities));
    ERROR_CHECK(nrf_mesh_prov_provision(&m_prov_ctx, p_uuid, &prov_data, NRF_MESH_PROV_BEARER_ADV));
}

static void mesh_evt_handler(nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_MESSAGE_RECEIVED:
            break;

        case NRF_MESH_EVT_UNPROVISIONED_RECEIVED:
            if (m_device_state == DEVICE_STATE_PB_ADV_MODE)
            {
                start_provisioning(p_evt->params.unprov_recv.device_uuid);
                m_device_state = DEVICE_STATE_NONE;
            }
            break;

        case NRF_MESH_EVT_PROV_LINK_ESTABLISHED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link established\n");
            break;

        case NRF_MESH_EVT_PROV_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link closed\n");
            break;

        case NRF_MESH_EVT_PROV_COMPLETE:
            ERROR_CHECK(dsm_address_publish_add(m_next_unprov_address,
                        &m_device_address_handles[m_next_unprov_index]));
            ERROR_CHECK(dsm_devkey_add(p_evt->params.prov_complete.address,
                                       m_netkey_handles[0],
                                       p_evt->params.prov_complete.p_devkey,
                                       &m_devkey_handles[1 + m_next_unprov_index]));
            __LOG(LOG_SRC_APP,
                  LOG_LEVEL_INFO,
                  "Provisioning complete! Added %04X as handle %u\n",
                  p_evt->params.prov_complete.address,
                  m_device_address_handles[m_next_unprov_index]);
            m_next_unprov_index++;
            m_next_unprov_address += m_num_elements_of_last_guy;
            break;

        case NRF_MESH_EVT_PROV_CAPS_RECEIVED:
        {
            uint32_t status = nrf_mesh_prov_oob_use(p_evt->params.prov_oob_caps_received.p_context,
                                                    NRF_MESH_PROV_OOB_METHOD_STATIC,
                                                    NRF_MESH_KEY_SIZE);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Provisioning OOB selection rejected, error code %d\n",
                      status);
            }
            else
            {
                m_num_elements_of_last_guy = p_evt->params.prov_oob_caps_received.oob_caps.num_elements;
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using static authentication\n");
            }

            break;
        }

        case NRF_MESH_EVT_PROV_STATIC_REQUEST:
        {
            /* Request for static authentication data. This data is used to authenticate the two nodes. */
            uint8_t static_data[16] = STATIC_AUTH_DATA;
            ERROR_CHECK(nrf_mesh_prov_auth_data_provide(p_evt->params.prov_static_request.p_context, static_data, 16));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Static authentication data provided\n");
            break;
        }

        case NRF_MESH_EVT_TX_COMPLETE:
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Unexpected event: %d.\n", p_evt->type);
            break;
    }
}

static void remote_client_event_cb(const pb_remote_event_t * p_evt)
{
    switch (p_evt->type)
    {
        case PB_REMOTE_EVENT_TX_FAILED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Communication with server failed\n");
            break;

        case PB_REMOTE_EVENT_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The remote link has closed\n");

        case PB_REMOTE_EVENT_REMOTE_UUID:
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Got remote uuid", p_evt->remote_uuid.p_uuid, NRF_MESH_UUID_SIZE);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device ID: %u\n", p_evt->remote_uuid.device_id);
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got event %u\n", p_evt->type);
            break;
    }
}

static char wait_for_valid_char(void)
{
    char cmd;
    do
    {
        cmd = SEGGER_RTT_GetKey();
    } while (cmd == '\n' || cmd == '\r' || cmd == (char) -1);
    return cmd;
}


static void user_input_get(void)
{
    uint32_t status;
    int cmd = SEGGER_RTT_GetKey();
    switch (cmd)
    {
        case '1':
        {
            m_device_state = DEVICE_STATE_PB_ADV_MODE;
            break;
        }
        case '2':
        {   
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Please enter a valid publish handle: \n");
            cmd = wait_for_valid_char();
            dsm_handle_t handle = (dsm_handle_t) cmd - '0';
            status = access_model_publish_address_set(m_remote_client.model_handle, handle);
            if (status != NRF_SUCCESS)
            {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not set publish address\n", status);
                }
            else
            {
                __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "Handle set \n");
            }
            break;
        }
        case '3':
            status = pb_remote_client_remote_scan_start(&m_remote_client);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not start remote scanning\n", status);
            }
            break;
        case '4':
            status = pb_remote_client_remote_scan_cancel(&m_remote_client);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not cancel remote scanning\n", status);
            }
            break;
        case '5':
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Please enter a valid device number: \n");
            cmd = wait_for_valid_char();
            start_remote_provisioning((uint8_t) cmd - '0');    
            break;
        }
        case '\n':
        case '\r':
        case -1:
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, USAGE_STRING);
            break;
    }
}

int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Provisioner + Remote Provisioning Client Demo -----\n");

    mesh_core_setup();
    access_setup();
    provisioner_setup();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, USAGE_STRING);

    while (true)
    {
        user_input_get();
        nrf_mesh_process();
    }
}
