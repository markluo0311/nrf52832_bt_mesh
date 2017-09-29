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
#include "nrf_delay.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "timer.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"

#include "config_client.h"
#include "simple_on_off_client.h"

#include "simple_hal.h"
#include "provisioner.h"

#include "SEGGER_RTT.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define SERVER_COUNT (3)
#define CLIENT_COUNT (SERVER_COUNT + 1)

typedef enum
{
    DEVICE_STATE_PROVISIONING,
    DEVICE_STATE_RUNNING
} device_state_t;

/*****************************************************************************
 * Static data
 *****************************************************************************/

static const uint8_t m_netkey[NRF_MESH_KEY_SIZE] = NETKEY;
static const uint8_t m_appkey[NRF_MESH_KEY_SIZE] = APPKEY;

static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_devkey_handles[SERVER_COUNT];
static dsm_handle_t m_server_handles[SERVER_COUNT];
static dsm_handle_t m_group_handle;

static simple_on_off_client_t m_clients[CLIENT_COUNT];

static device_state_t m_device_state;
static uint16_t m_unprov_index;

/* Forward declarations */
static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src);

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void mesh_core_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing softdevice\n");
#if defined(S130) || defined(S132) || defined(S140)
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
#elif defined(S110)
    nrf_clock_lfclksrc_t lfc_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif
    mesh_softdevice_setup(lfc_cfg);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing mesh stack\n");
    nrf_mesh_init_params_t mesh_init_params = {
        .lfclksrc = lfc_cfg,
        .assertion_handler = mesh_assert_handler,
    };
    ERROR_CHECK(nrf_mesh_init(&mesh_init_params));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling mesh stack\n");
    ERROR_CHECK(nrf_mesh_enable());
}

static void access_setup(void)
{
    dsm_init();
    access_init();

    /* Initialize and enable all the models before calling ***_flash_config_load. */
    ERROR_CHECK(config_client_init(config_client_event_cb));

     for (uint32_t i = 0; i < CLIENT_COUNT; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i));
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting up access layer and models\n");
    /* Set and add local addresses and keys, if recovery fails. */
    dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, ACCESS_ELEMENT_COUNT};
    if (!dsm_flash_config_load() || !access_flash_config_load())
    {
        dsm_local_unicast_addresses_set(&local_address);
        dsm_address_publish_add(GROUP_ADDRESS, &m_group_handle);
        dsm_subnet_add(0, m_netkey, &m_netkey_handle);
        dsm_appkey_add(0, m_netkey_handle, m_appkey, &m_appkey_handle);
    }
    /* Setup SERVER_COUNT clients. Publish address is set to the corresponding server on provisioning complete, only add handles if recovery failed (this is checked by looking at the publish address of the model) */
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        dsm_handle_t address_handle = DSM_HANDLE_INVALID;
        if (NRF_SUCCESS == access_model_publish_address_get(m_clients[i].model_handle, &address_handle) &&  DSM_HANDLE_INVALID != address_handle)
        {
            provisioner_config_successful_cb();
        }
        else
        {
            ERROR_CHECK(access_model_application_bind(m_clients[i].model_handle, m_appkey_handle));
            ERROR_CHECK(access_model_publish_application_set(m_clients[i].model_handle, m_appkey_handle));
        }
    }

    /* Last client is set to publish to the GROUP_ADDRESS, only add handles if recovery failed. */
    dsm_handle_t address_handle = DSM_HANDLE_INVALID;
    if (NRF_SUCCESS != access_model_publish_address_get(m_clients[SERVER_COUNT].model_handle, &address_handle) ||
        DSM_HANDLE_INVALID == address_handle)
    {
        ERROR_CHECK(access_model_application_bind(m_clients[SERVER_COUNT].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_clients[SERVER_COUNT].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_address_set(m_clients[SERVER_COUNT].model_handle, m_group_handle));
    }
}

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = (((uint32_t) p_client - ((uint32_t) &m_clients[0]))) / sizeof(m_clients[0]);
    NRF_MESH_ASSERT(index < SERVER_COUNT);
    return index;
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);
    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            hal_led_pin_set(BSP_LED_0 + server_index, true);
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            hal_led_pin_set(BSP_LED_0 + server_index, false);
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            hal_led_blink_ms(LEDS_MASK, 100, 6);
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }

    /* Set 4th LED on when all servers are on. */
    if (hal_led_pin_get(BSP_LED_0) &&
        hal_led_pin_get(BSP_LED_1) &&
        hal_led_pin_get(BSP_LED_2))
    {
        hal_led_pin_set(BSP_LED_3, true);
    }
    else
    {
        hal_led_pin_set(BSP_LED_3, false);
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    if (m_device_state != DEVICE_STATE_RUNNING)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Not in RUNNING state\n");
        return;
    }
    uint32_t status = NRF_SUCCESS;
    switch (button_number)
    {
        case 0:
        case 1:
        case 2:
            /* Invert LED. */
            status = simple_on_off_client_set(&m_clients[button_number],
                                               !hal_led_pin_get(BSP_LED_0 + button_number));
            break;
        case 3:
            /* Group message: invert all LEDs. */
            status = simple_on_off_client_set_unreliable(&m_clients[button_number],
                                                          !hal_led_pin_get(BSP_LED_0 + button_number), 3);
            break;
        default:
            break;

    }

    if (status == NRF_ERROR_INVALID_STATE ||
        status == NRF_ERROR_BUSY)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send. Device is busy.\n");
        hal_led_blink_ms(LEDS_MASK, 50, 4);
    }
    else
    {
        ERROR_CHECK(status);
    }
}

/*****************************************************************************
 * Event callbacks from the provisioner
 *****************************************************************************/

void provisioner_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_unprov_index);

    hal_led_pin_set(BSP_LED_0 + m_unprov_index, false);

    if (m_unprov_index < (SERVER_COUNT - 1))
    {
        m_unprov_index++;
        provisioner_wait_for_unprov();
        hal_led_pin_set(BSP_LED_0 + m_unprov_index, true);
    }
    else
    {
        hal_led_blink_ms(LEDS_MASK, 100, 4);
        m_device_state = DEVICE_STATE_RUNNING;
    }
}

void provisioner_config_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed\n", m_unprov_index);

    /* Delete key and address. */
    ERROR_CHECK(dsm_address_publish_remove(m_server_handles[m_unprov_index]));
    ERROR_CHECK(dsm_devkey_delete(m_devkey_handles[m_unprov_index]));
    provisioner_wait_for_unprov();
}

void provisioner_prov_complete_cb(const nrf_mesh_evt_prov_complete_t * p_prov_data)
{
    /* We should not get here if all servers are provisioned. */
    NRF_MESH_ASSERT(m_unprov_index < SERVER_COUNT);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning complete. Adding address 0x%04x.\n", p_prov_data->address);

    /* Add to local storage. */
    ERROR_CHECK(dsm_address_publish_add(p_prov_data->address, &m_server_handles[m_unprov_index]));
    ERROR_CHECK(dsm_devkey_add(p_prov_data->address, m_netkey_handle, p_prov_data->p_devkey, &m_devkey_handles[m_unprov_index]));

    /* Set publish address for the client to the corresponding server. */
    ERROR_CHECK(access_model_publish_address_set(m_clients[m_unprov_index].model_handle, m_server_handles[m_unprov_index]));

    /* Bind the device key to the configuration server and set the new node as the active server. */
    ERROR_CHECK(config_client_server_bind(m_devkey_handles[m_unprov_index]));
    ERROR_CHECK(config_client_server_set(m_devkey_handles[m_unprov_index], m_server_handles[m_unprov_index]));


    /* Move on to the configuration step. */
    provisioner_configure();
}

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Control Client Demo -----\n");

    m_device_state = DEVICE_STATE_PROVISIONING;

    hal_leds_init();
    hal_buttons_init(button_event_handler);

    /* Set the first LED */
    hal_led_pin_set(BSP_LED_0, true);

    mesh_core_setup();
    access_setup();
    provisioner_init();
    provisioner_wait_for_unprov();
    while (true)
    {
        int key = SEGGER_RTT_GetKey(); /* Returns -1 if there is no data. */
        if (key >= '0' && key <= '3')
        {
            uint32_t button_number = key - '0';
            button_event_handler(button_number);
        }
        nrf_mesh_process();
    }
}
