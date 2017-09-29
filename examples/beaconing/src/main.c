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

#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <ble.h>
#include <boards.h>
#include <stdio.h>

#include "nrf_mesh.h"
#include <nrf_mesh_opt.h>

#include "utils.h"
#include "log.h"
#include "SEGGER_RTT.h"

#include "nrf_mesh_sdk.h"
#include "utils.h"

#include "nrf_mesh_dfu.h"
#include "nrf_mesh_serial.h"

/* For beaconing advertiser */
#include "bearer_adv.h"
#include "packet_mgr.h"


#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
#include "stack_depth.h"
#endif

/** Single advertiser instance. May periodically transmit one packet at a time. */
static advertiser_t m_advertiser;


static void rx_callback(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
    char msg[128];
    sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
            p_rx_data->timestamp,
            p_rx_data->rssi,
            p_rx_data->adv_type,
            p_rx_data->addr.addr[0],
            p_rx_data->addr.addr[1],
            p_rx_data->addr.addr[2],
            p_rx_data->addr.addr[3],
            p_rx_data->addr.addr[4],
            p_rx_data->addr.addr[5]);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);
    LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
}

static void init_advertiser(void)
{
    /* Configure advertiser */
    m_advertiser.adv_channel_map = 0x07; /* All three advertisement channels */
    m_advertiser.adv_int_min_ms = 100;
    m_advertiser.adv_int_max_ms = 110;
    m_advertiser.adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;

    bearer_adv_advertiser_init(&m_advertiser);
}

static void start_advertiser(void)
{
    static const uint8_t adv_data[] =
    {
        0x11, /* AD data length (including type, but not itself) */
        0x09, /* AD data type (Complete local name) */
        'N',  /* AD data payload (Name of device) */
        'o',
        'r',
        'd',
        'i',
        'c',
        ' ',
        'S',
        'e',
        'm',
        'i',
        ' ',
        'M',
        'e',
        's',
        'h'
    };

    /* Allocate packet */
    packet_t * p_packet;
    ERROR_CHECK(packet_mgr_alloc((packet_generic_t*) &p_packet, BLE_ADV_PACKET_HEADER_LENGTH + BLE_ADV_PACKET_OVERHEAD + sizeof(adv_data)));

    if (p_packet)
    {
        /* Construct packet contents */
        packet_payload_size_set(p_packet, sizeof(adv_data));
        memcpy(p_packet->payload, adv_data, sizeof(adv_data));
        /* Repeat forever */
        if (bearer_adv_tx(&m_advertiser, p_packet, BEARER_ADV_REPEAT_INFINITE) != NRF_SUCCESS)
        {
            /* TX failed, free the packet */
            packet_mgr_free(p_packet);
        }
    }

}

int main(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
    {
        nrf_gpio_pin_set(i);
    }

    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_INFO, log_callback_rtt);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Bluetooth Mesh Beacon Example -----\n");

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing softdevice...\n");
#if defined(S130) || defined(S132)
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
#elif defined(S110)
    nrf_clock_lfclksrc_t lfc_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif
    mesh_softdevice_setup(lfc_cfg);


    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing mesh stack...\n");
    nrf_mesh_init_params_t mesh_init_params = {
        .lfclksrc = lfc_cfg,
        .assertion_handler = mesh_assert_handler
    };
    ERROR_CHECK(nrf_mesh_init(&mesh_init_params));

    /* Start listening for incoming packets */
    nrf_mesh_rx_cb_set(rx_callback);
    /* Start Advertising own beacon */
    init_advertiser();
    start_advertiser();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling mesh stack...\n");
    ERROR_CHECK(nrf_mesh_enable());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");

    while (true)
    {
        nrf_mesh_process();
    }
}
