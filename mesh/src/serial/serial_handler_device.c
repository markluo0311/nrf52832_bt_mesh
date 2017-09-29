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

#include "serial_handler_device.h"

#include <stdint.h>

#include "nrf_mesh_config_serial.h"
#include "nrf_mesh_defines.h"
#include "serial.h"
#include "serial_status.h"
#include "serial_handler_common.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_dfu.h"
#include "hal.h"
#include "bearer_adv.h"
#include "radio.h"

#define BEACON_START_CMD_DATA_OVERHEAD  (sizeof(serial_cmd_device_beacon_start_t) - BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH)
#define BEACON_INTERVAL_RANDOMIZE_INTERVAL_MS   (10)

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    BEACON_STATE_UNUSED,
    BEACON_STATE_RUNNING
} beacon_state_t;

typedef struct
{
    beacon_state_t state; /**< Current state of the beacon slot. */
    advertiser_t advertiser; /**< Beacon advertiser instance. */
} beacon_slot_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static beacon_slot_t m_beacons[NRF_MESH_SERIAL_BEACON_SLOTS]; /**< Parameters for beacon operation. */

/*****************************************************************************
* Static functions
*****************************************************************************/

#if INTERNAL_EVT_ENABLE
static uint32_t event_report_cb(internal_event_t * p_event)
{
    /* Send a serial event with the contents of the event. */
    serial_packet_t * p_serial_evt;
    NRF_MESH_ASSERT(NRF_SUCCESS == serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + SERIAL_PACKET_INTERNAL_EVENT_OVERHEAD + p_event->packet_size, &p_serial_evt));
    p_serial_evt->opcode = SERIAL_OPCODE_EVT_DEVICE_INTERNAL_EVENT;
    p_serial_evt->length = SERIAL_PACKET_LENGTH_OVERHEAD + SERIAL_PACKET_INTERNAL_EVENT_OVERHEAD + p_event->packet_size;
    p_serial_evt->payload.evt.device.internal_event.event_type = p_event->type;
    p_serial_evt->payload.evt.device.internal_event.state = p_event->state.value;
    p_serial_evt->payload.evt.device.internal_event.packet_size = p_event->packet_size;
    memcpy(p_serial_evt->payload.evt.device.internal_event.packet, p_event->p_packet, p_event->packet_size);
    serial_tx(p_serial_evt);
    return NRF_SUCCESS;
}
#endif

/*****************************************************************************
* Command handlers
*****************************************************************************/
static void handle_cmd_device_echo(const serial_packet_t * p_cmd)
{
    serial_packet_t * p_serial_evt;
    NRF_MESH_ASSERT(NRF_SUCCESS == serial_packet_buffer_get(p_cmd->length, &p_serial_evt));
    p_serial_evt->opcode = SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP;
    memcpy(p_serial_evt->payload.evt.device.echo.data, p_cmd->payload.cmd.device.echo.data,
            p_serial_evt->length - SERIAL_PACKET_LENGTH_OVERHEAD);
    (void) serial_tx(p_serial_evt);
}

static void handle_cmd_device_internal_events_report(const serial_packet_t * p_cmd)
{
#if INTERNAL_EVT_ENABLE
    internal_event_init(event_report_cb);
    serial_cmd_rsp_send(p_cmd->opcode,
            SERIAL_STATUS_SUCCESS,
            NULL,
            0);
#else
    serial_cmd_rsp_send(p_cmd->opcode,
            SERIAL_STATUS_ERROR_CMD_UNKNOWN,
            NULL,
            0);
#endif
}

static void handle_cmd_device_serial_version_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_serial_version_t rsp;
    rsp.serial_ver = SERIAL_API_VERSION;
    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_device_fw_info_get(const serial_packet_t * p_cmd)
{
    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
}

static void handle_cmd_device_radio_reset(const serial_packet_t * p_cmd)
{
    hal_device_reset(BL_GPREGRET_FORCED_REBOOT);
    /* Will only trigger if something failed in the reset process. */
    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
}

static void handle_cmd_device_beacon_start(const serial_packet_t * p_cmd)
{
    uint32_t status;
    if (p_cmd->payload.cmd.device.beacon_start.beacon_slot >= NRF_MESH_SERIAL_BEACON_SLOTS)
    {
        status = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        packet_t * p_packet;
        uint32_t data_length = p_cmd->length -
            SERIAL_PACKET_LENGTH_OVERHEAD -
            BEACON_START_CMD_DATA_OVERHEAD;
        uint32_t packet_buffer_length = BLE_ADV_PACKET_HEADER_LENGTH +
            BLE_ADV_PACKET_OVERHEAD +
            data_length;
        status = packet_mgr_alloc((packet_generic_t *) &p_packet, packet_buffer_length);
        if (status == NRF_SUCCESS)
        {
            memcpy(p_packet->payload,
                    p_cmd->payload.cmd.device.beacon_start.data,
                    data_length);
            packet_payload_size_set(p_packet, data_length);
            status = bearer_adv_tx(&m_beacons[p_cmd->payload.cmd.device.beacon_start.beacon_slot].advertiser,
                p_packet,
                BEARER_ADV_REPEAT_INFINITE);

            if (status == NRF_SUCCESS)
            {
                m_beacons[p_cmd->payload.cmd.device.beacon_start.beacon_slot].state = BEACON_STATE_RUNNING;
            }
            else
            {
                packet_mgr_free(p_packet);
            }
        }
    }
    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void handle_cmd_device_beacon_stop(const serial_packet_t * p_cmd)
{
    uint32_t status;
    if (p_cmd->payload.cmd.device.beacon_stop.beacon_slot >= NRF_MESH_SERIAL_BEACON_SLOTS)
    {
        status = NRF_ERROR_INVALID_PARAM;
    }
    else if (m_beacons[p_cmd->payload.cmd.device.beacon_stop.beacon_slot].state != BEACON_STATE_RUNNING)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        status = bearer_adv_adv_stop(&m_beacons[p_cmd->payload.cmd.device.beacon_stop.beacon_slot].advertiser);
        if (status == NRF_SUCCESS)
        {
            m_beacons[p_cmd->payload.cmd.device.beacon_stop.beacon_slot].state = BEACON_STATE_UNUSED;
        }
    }
    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void handle_cmd_device_beacon_params_get(const serial_packet_t * p_cmd)
{
    if (p_cmd->payload.cmd.device.beacon_params_get.beacon_slot >= NRF_MESH_SERIAL_BEACON_SLOTS)
    {
        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_PARAMETER, NULL, 0);
    }
    else
    {
        advertiser_t * p_adv = &m_beacons[p_cmd->payload.cmd.device.beacon_params_get.beacon_slot].advertiser;

        serial_evt_cmd_rsp_data_beacon_params_t rsp;
        rsp.beacon_slot = p_cmd->payload.cmd.device.beacon_params_get.beacon_slot;
        rsp.channel_map = p_adv->adv_channel_map;
        rsp.interval_ms = p_adv->adv_int_min_ms;
        rsp.tx_power    = radio_tx_power_get();
        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
    }
}

static void handle_cmd_device_beacon_params_set(const serial_packet_t * p_cmd)
{
    uint32_t status;
    if (p_cmd->payload.cmd.device.beacon_params_set.beacon_slot >= NRF_MESH_SERIAL_BEACON_SLOTS)
    {
        status = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        advertiser_t * p_adv = &m_beacons[p_cmd->payload.cmd.device.beacon_params_set.beacon_slot].advertiser;
        p_adv->adv_int_min_ms  = p_cmd->payload.cmd.device.beacon_params_set.interval_ms;
        p_adv->adv_int_max_ms  = p_cmd->payload.cmd.device.beacon_params_set.interval_ms + BEACON_INTERVAL_RANDOMIZE_INTERVAL_MS;
        p_adv->adv_channel_map = p_cmd->payload.cmd.device.beacon_params_set.channel_map;
        // TODO: MBTLE-1171: Can't set TX-power per advertiser.

        /* Register the new interval. If this fails, we'll just wait until the
         * next advertisement before readjusting the interval, which isn't a
         * big deal: */
        (void) bearer_adv_interval_reset(p_adv);
        status = NRF_SUCCESS;
    }
    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

/* Serial command handler lookup table. */
static const serial_handler_common_opcode_to_fp_map_t m_cmd_handlers[] =
{
    {SERIAL_OPCODE_CMD_DEVICE_ECHO,                   0,                                 NRF_MESH_SERIAL_PAYLOAD_MAXLEN, handle_cmd_device_echo},
    {SERIAL_OPCODE_CMD_DEVICE_INTERNAL_EVENTS_REPORT, 0,                                                              0, handle_cmd_device_internal_events_report},
    {SERIAL_OPCODE_CMD_DEVICE_SERIAL_VERSION_GET,     0,                                                              0, handle_cmd_device_serial_version_get},
    {SERIAL_OPCODE_CMD_DEVICE_FW_INFO_GET,            0,                                                              0, handle_cmd_device_fw_info_get},
    {SERIAL_OPCODE_CMD_DEVICE_RADIO_RESET,            0,                                                              0, handle_cmd_device_radio_reset},
    {SERIAL_OPCODE_CMD_DEVICE_BEACON_START,           BEACON_START_CMD_DATA_OVERHEAD, BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH, handle_cmd_device_beacon_start},
    {SERIAL_OPCODE_CMD_DEVICE_BEACON_STOP,            sizeof(serial_cmd_device_beacon_stop_t),                        0, handle_cmd_device_beacon_stop},
    {SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_SET,      sizeof(serial_cmd_device_beacon_params_set_t),                  0, handle_cmd_device_beacon_params_set},
    {SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_GET,      sizeof(serial_cmd_device_beacon_params_get_t),                  0, handle_cmd_device_beacon_params_get},
};


/*****************************************************************************
* Interface functions
*****************************************************************************/
void serial_handler_device_init(void)
{
    for (uint32_t i = 0; i < NRF_MESH_SERIAL_BEACON_SLOTS; i++)
    {
        m_beacons[i].advertiser.adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
        m_beacons[i].advertiser.adv_int_max_ms = BEARER_ADV_INT_MAX_MS_DEFAULT;
        m_beacons[i].advertiser.adv_int_min_ms = BEARER_ADV_INT_MIN_MS_DEFAULT;
        m_beacons[i].advertiser.adv_channel_map = NRF_MESH_ADV_CHAN_DEFAULT;
        m_beacons[i].advertiser.queue_empty_cb = NULL;
        bearer_adv_advertiser_init(&m_beacons[i].advertiser);
    }
}

void serial_handler_device_rx(const serial_packet_t* p_cmd)
{
    NRF_MESH_ASSERT(p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_DEVICE_END);
    serial_handler_common_rx(p_cmd, m_cmd_handlers, sizeof(m_cmd_handlers) / sizeof(m_cmd_handlers[0]));
}
