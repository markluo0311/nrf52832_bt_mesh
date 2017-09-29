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
#include "boards.h"
#include "nrf_mesh_sdk.h"
#include "nrf_delay.h"
#include "simple_hal.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "nrf_mesh_node_config.h"

#include "simple_on_off_server.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define LED_PIN_NUMBER (BSP_LED_0)
#define LED_PIN_MASK   (1u << LED_PIN_NUMBER)
#define STATIC_AUTH_DATA {0}

/*****************************************************************************
 * Static data
 *****************************************************************************/

static simple_on_off_server_t m_server;

/* Forward declaration */
static bool generic_get_cb(const simple_on_off_server_t * p_server);
static bool generic_set_cb(const simple_on_off_server_t * p_server, bool value);

/*****************************************************************************
 * Static utility functions
 *****************************************************************************/

static void configuration_setup(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n")
    m_server.get_cb = generic_get_cb;
    m_server.set_cb = generic_set_cb;
    ERROR_CHECK(simple_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
}

static void configuration_complete(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    hal_led_blink_ms(LED_PIN_MASK, 200, 4);
}

/*****************************************************************************
 * Light control
 *****************************************************************************/

static bool generic_get_cb(const simple_on_off_server_t * p_server)
{
    return hal_led_pin_get(LED_PIN_NUMBER);
}

static bool generic_set_cb(const simple_on_off_server_t * p_server, bool value)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET command to %u\n", value);
    hal_led_pin_set(LED_PIN_NUMBER, value);
    return value;
}

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Control Server Demo -----\n");

    hal_leds_init();

    static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
    static nrf_mesh_node_config_params_t config_params;
    config_params.prov_caps.num_elements = ACCESS_ELEMENT_COUNT;
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.p_static_data = static_auth_data;
    config_params.complete_callback = configuration_complete;
    config_params.setup_callback = configuration_setup;

#if defined(S130) || defined(S132) || defined(S140)
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
#else
    config_params.lf_clk_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif

    ERROR_CHECK(nrf_mesh_node_config(&config_params));

    while (true)
    {
        nrf_mesh_process();
    }
}
