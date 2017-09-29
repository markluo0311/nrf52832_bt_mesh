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

#include "prov_beacon.h"

#include <string.h>

#include "nrf_mesh_configure.h"
#include "enc.h"
#include "nrf_mesh_assert.h"
#include "event.h"
#include "beacon.h"
#include "utils.h"
#include "uri.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define UNPROV_BEACON_DATA_LEN            (sizeof(prov_beacon_unprov_packet_t))                      /**< Length of the unprovisioned beacon data. */
#define UNPROV_BEACON_DATA_LEN_NO_HASH    (UNPROV_BEACON_DATA_LEN - NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE) /**< Length of the unprovisioned beacon data without a p_URI hash. */

/*****************************************************************************
* Local typedefs
*****************************************************************************/
/*lint -align_max(push) -align_max(1) */

/** Unprovisioned Node Broadcast Beacon Format. */
typedef struct __attribute((packed))
{
    uint8_t  device_uuid[NRF_MESH_UUID_SIZE];       /**< Device UUID. */
    uint16_t oob_info;                              /**< Out of band data bitfield. */
    uint8_t  uri_hash[NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE]; /**< Optional hash of p_URI advertisement. */
} prov_beacon_unprov_packet_t;

/*lint -align_max(pop) */
/*****************************************************************************
* Static globals
*****************************************************************************/
static bool m_unprov_report = true; /**< Flag deciding whether we should report unprovisioned beacons to the application. */
/*****************************************************************************
* Static functions
*****************************************************************************/
/*****************************************************************************
* Interface functions
*****************************************************************************/
packet_t* prov_beacon_unprov_build(const char* p_uri, uint16_t oob_info)
{
    prov_beacon_unprov_packet_t unprov_packet;
    uint8_t packet_len;
    if (p_uri == NULL)
    {
        packet_len = UNPROV_BEACON_DATA_LEN_NO_HASH;
    }
    else
    {
        packet_len = UNPROV_BEACON_DATA_LEN;
        uint8_t uri_data_hash[NRF_MESH_KEY_SIZE];
        enc_s1((const uint8_t*) p_uri, strlen(p_uri), uri_data_hash);
        memcpy(unprov_packet.uri_hash, uri_data_hash, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
    }

    unprov_packet.oob_info = LE2BE16(oob_info);
    memcpy(unprov_packet.device_uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE);

    return beacon_create(BEACON_TYPE_UNPROV, &unprov_packet, packet_len);
}

void prov_beacon_unprov_report(bool unprov_report)
{
    m_unprov_report = unprov_report;
}

void prov_beacon_unprov_pkt_in(const void * p_data, uint8_t length, const packet_meta_t * p_meta)
{
    NRF_MESH_ASSERT(p_data != NULL);
    NRF_MESH_ASSERT(length == UNPROV_BEACON_DATA_LEN || length == UNPROV_BEACON_DATA_LEN_NO_HASH);
    NRF_MESH_ASSERT(p_meta != NULL);
    if (m_unprov_report)
    {
        prov_beacon_unprov_packet_t* p_unprov_beacon = (prov_beacon_unprov_packet_t*) p_data;

        nrf_mesh_evt_t unprov_evt;

        memcpy(unprov_evt.params.unprov_recv.adv_addr.addr, p_meta->p_addr, BLE_GAP_ADDR_LEN);
        unprov_evt.params.unprov_recv.adv_addr.addr_type = p_meta->addr_type;
        unprov_evt.params.unprov_recv.rssi = p_meta->rssi;
        memcpy(unprov_evt.params.unprov_recv.device_uuid,
               p_unprov_beacon->device_uuid,
               NRF_MESH_UUID_SIZE);

        unprov_evt.params.unprov_recv.gatt_supported = false;
        unprov_evt.type = NRF_MESH_EVT_UNPROVISIONED_RECEIVED;
        if (length == UNPROV_BEACON_DATA_LEN)
        {
            unprov_evt.params.unprov_recv.uri_hash_present = true;
            memcpy(unprov_evt.params.unprov_recv.uri_hash, p_unprov_beacon->uri_hash, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
        }
        else
        {
            unprov_evt.params.unprov_recv.uri_hash_present = false;
            memset(unprov_evt.params.unprov_recv.uri_hash, 0, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
        }

        event_handle(&unprov_evt);
    }
}

