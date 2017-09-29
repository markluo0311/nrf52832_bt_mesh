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
#include "nrf_mesh.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_utils.h"

#include <string.h>
#include <stddef.h>

#include "nrf.h"
#include <nrf_error.h>
#include "nrf_sdm.h"

#include "bearer.h"
#include "bearer_event.h"
#include "enc.h"
#include "event.h"
#include "fifo.h"
#include "msg_cache.h"
#include "network.h"
#include "packet.h"
#include "packet_mgr.h"
#include "nrf_mesh_dfu.h"
#include "dfu_types_internal.h"
#include "ticker.h"
#include "timer_scheduler.h"
#include "timeslot.h"
#include "toolchain.h"
#include "transport.h"
#include "beacon.h"
#include "dfu_types_internal.h"
#include "list.h"
#include "utils.h"
#include "log.h"
#include "mesh_flash.h"
#include "flash_manager.h"

#include "prov_bearer_adv.h"

/** Function pointer to mesh assertion handler. */
nrf_mesh_assertion_handler_t m_assertion_handler;

static bool m_is_enabled     = false;
static bool m_is_initialized = false;
static nrf_mesh_rx_cb_t m_rx_cb;

uint32_t nrf_mesh_init(const nrf_mesh_init_params_t * p_init_params)
{
    uint32_t status;

    if (m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_init_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    m_assertion_handler = p_init_params->assertion_handler;

    nrf_mesh_configure_device_uuid_reset();

#if !HOST
    uint8_t softdevice_enabled;
    status = sd_softdevice_is_enabled(&softdevice_enabled);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (softdevice_enabled != 1)
    {
        return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
    }
#endif

#ifdef __linux__
    toolchain_init_irqs();
#endif

    packet_mgr_init(p_init_params);
    msg_cache_init();
    timer_sch_init();
    bearer_event_init();

    status = bearer_init(p_init_params);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

#  if !HOST
    status = timeslot_init(p_init_params);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
#  endif

    mesh_flash_init();
#if PERSISTENT_STORAGE
    flash_manager_init();
#endif

    transport_init(p_init_params);
    network_init(p_init_params);
    beacon_init(BEACON_INTERVAL_MS_DEFAULT);

#if !HOST
    status = nrf_mesh_dfu_init();
    if ((status != NRF_SUCCESS) && (status != NRF_ERROR_NOT_SUPPORTED))
    {
        return status;
    }
#endif

    ticker_init();

    m_rx_cb = NULL;
    m_is_initialized = true;

    return NRF_SUCCESS;
}

uint32_t nrf_mesh_enable(void)
{
    if (!m_is_initialized || m_is_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
#if !HOST
        NRF_MESH_ASSERT(timeslot_resume() == NRF_SUCCESS);
#endif
        uint32_t status = bearer_enable();
        if (status != NRF_SUCCESS)
        {
            return status;
        }

        m_is_enabled = true;
        return NRF_SUCCESS;
    }
}

uint32_t nrf_mesh_disable(void)
{
    if (!m_is_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        uint32_t status = bearer_disable();
        if (status != NRF_SUCCESS)
        {
            return status;
        }

#if !HOST
        timeslot_stop();
#endif

        m_is_enabled = false;
        return NRF_SUCCESS;
    }
}

uint32_t nrf_mesh_packet_send(const nrf_mesh_tx_params_t * p_params,
                              uint32_t * const p_packet_reference)
{
    if (p_params == NULL ||
        p_params->security_material.p_net == NULL ||
        p_params->security_material.p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (nrf_mesh_address_type_get(p_params->src) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (p_params->data_len > transport_unseg_maxlen_get() || p_params->reliable)
    {
        return transport_tx_sar(p_params, p_packet_reference);
    }
    else
    {
        return transport_tx(p_params, p_packet_reference);
    }
}

uint32_t nrf_mesh_adv_send(const uint8_t * p_payload, uint8_t size)
{
#if BEARER_TYPES & (BEARER_ADV_RADIO)
    if (size > BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (p_payload == NULL)
    {
        return NRF_ERROR_NULL;
    }

    packet_t * p_packet;
    uint32_t status = packet_mgr_alloc(((packet_generic_t **) &p_packet), BLE_ADV_PACKET_MAX_LENGTH);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_API, LOG_LEVEL_ERROR, "Cannot allocate memory for advertising packet!\n");
        return status;
    }

    memcpy(p_packet->payload, p_payload, size);
    packet_payload_size_set(p_packet, size);

    /* This packet goes directly to the bearer layer: */
    status = bearer_tx(p_packet, BEARER_ADV_RADIO, 1);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_packet);
    }
    return status;
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

uint32_t nrf_mesh_process(void)
{
    uint32_t status = NRF_SUCCESS;

    /* Process all incoming packets: */
    packet_t * p_incoming_packet;
    packet_meta_t meta_data;
    while (bearer_rx(&p_incoming_packet, NULL, &meta_data) == NRF_SUCCESS)
    {
        for (ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &p_incoming_packet->payload[0];
                (uint8_t*) p_ad_data < &p_incoming_packet->payload[p_incoming_packet->header.length - BLE_ADV_PACKET_OVERHEAD];
                p_ad_data = packet_ad_type_get_next(p_ad_data))
        {
            switch (p_ad_data->type)
            {
                case AD_TYPE_MESH:
                    if (p_incoming_packet->header.type  == BLE_PACKET_TYPE_ADV_NONCONN_IND)
                    {
                        status = network_pkt_in((packet_net_t*) p_ad_data, &meta_data);

                        if (status != NRF_SUCCESS)
                        {
                            __LOG(LOG_SRC_API, LOG_LEVEL_WARN, "[er%d] Could not process mesh packet...\n", status);
                        }
                    }
                    break;

                case AD_TYPE_PB_ADV:
                    if (p_incoming_packet->header.type  == BLE_PACKET_TYPE_ADV_NONCONN_IND)
                    {
                        prov_bearer_adv_pkt_in(p_ad_data);
                    }
                    break;

                case AD_TYPE_BEACON:
                    if (p_incoming_packet->header.type  == BLE_PACKET_TYPE_ADV_NONCONN_IND)
                    {
                        status = beacon_pkt_in(p_ad_data, &meta_data);

                        if (status != NRF_SUCCESS)
                        {
                            __LOG(LOG_SRC_API, LOG_LEVEL_WARN, "[er%d] Could not process beacon packet...\n", status);
                        }
                    }
                    break;
                case AD_TYPE_DFU:
                {
                    ble_ad_data_service_data_t* p_service_data = (ble_ad_data_service_data_t*) p_ad_data->data;
                    if (p_service_data->uuid == BLE_ADV_SERVICE_DATA_UUID_DFU)
                    {
                        /* Send dfu packet pointer. */
                        (void) nrf_mesh_dfu_rx((nrf_mesh_dfu_packet_t*) p_service_data->data, p_ad_data->length - DFU_PACKET_PAYLOAD_OVERHEAD);
                    }
                    break;
                }
                default:
                    break;
            }
        }

        /* Notify the application */
        if (m_rx_cb)
        {
            nrf_mesh_adv_packet_rx_data_t rx_data;
            rx_data.addr.addr_type = meta_data.addr_type;
            memcpy(rx_data.addr.addr, meta_data.p_addr, BLE_GAP_ADDR_LEN);
            rx_data.adv_type = p_incoming_packet->header.type;
            rx_data.length = p_incoming_packet->header.length - BLE_ADV_PACKET_OVERHEAD;
            rx_data.p_payload = p_incoming_packet->payload;
            rx_data.rssi = meta_data.rssi;
            rx_data.timestamp = meta_data.timestamp;

            m_rx_cb(&rx_data);
        }

        packet_mgr_free(p_incoming_packet);
    }

    status = transport_sar_process();
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_API, LOG_LEVEL_ERROR, "Error %u when processing SAR\n", status);
    }

    return status;
}

uint32_t nrf_mesh_on_ble_evt(ble_evt_t * p_ble_evt)
{
    /**
     * @todo Populate with GATT handling when the time comes.
     */
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_on_sd_evt(uint32_t sd_evt)
{
    /**
     * @todo Add handler for ECB events.
     */
#if !HOST
    timeslot_sd_event_handler(sd_evt);
#endif
    return NRF_SUCCESS;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    event_handler_add(p_handler_params);
}

void nrf_mesh_evt_handler_remove(nrf_mesh_evt_handler_t * p_handler_params)
{
    event_handler_remove(p_handler_params);
}

void nrf_mesh_rx_cb_set(nrf_mesh_rx_cb_t rx_cb)
{
    m_rx_cb = rx_cb;
}

void nrf_mesh_rx_cb_clear(void)
{
    m_rx_cb = NULL;
}

