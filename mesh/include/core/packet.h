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
#ifndef MESH_PACKET_H__
#define MESH_PACKET_H__

#include <stdint.h>
#include <string.h>

#include <nrf_error.h>
#include <ble_gap.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_dfu_types.h"
#include "utils.h"
#include "log.h"

/**
 * @defgroup PACKET Packet Formats
 * @ingroup MESH_CORE
 * Provides definitions of the packet format and functions for manipulating
 * the provided structures.
 * @{
 */

#define BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH   (31)                /**< Core specification BLE adv packet payload max length */
#define BLE_ADV_PACKET_OVERHEAD             (BLE_GAP_ADDR_LEN)  /**< BLE-packet overhead length in addition to length of payload. */
#define BLE_ADV_PACKET_HEADER_LENGTH        (3)                 /**< BLE-packet header length in bytes (type + length + padding) */
#define BLE_ADV_PACKET_MAX_LENGTH           (BLE_ADV_PACKET_HEADER_LENGTH + BLE_ADV_PACKET_OVERHEAD + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH) /**< Longest possible BLE advertisement packet */
#define BLE_ADV_PACKET_MIN_LENGTH           (BLE_ADV_PACKET_OVERHEAD) /**< Minimum length of a BLE advertisement packet. */
#define BLE_AD_DATA_OVERHEAD                (1)                 /**< BLE AD data overhead per AD-data structure */
#define BLE_ADV_SERVICE_DATA_UUID_DFU       (0xfee4)            /**< Service data UUID for DFU packets. */

#define AD_TYPE_MESH                        (0x2a)              /**< AD type for Bluetooth mesh. */
#define AD_TYPE_BEACON                      (0x2b)              /**< AD type for Bluetooth mesh beacons. */
#define AD_TYPE_PB_ADV                      (0x29)              /**< AD type for PB-ADV messages. */
#define AD_TYPE_DFU                         (0x16)              /**< AD type for nRF OpenMesh messages. */

/** Converts the value of the szmicn bit to the size of the MIC. */
#define BIT2MICSIZE(bit) (bit * 4 + 4)

/**
 * Generic packet type.
 *
 * This type is returned by the packet manager when allocating a packet buffer.
 * It can be cast to any of the other packet format structures.
 */
typedef void packet_generic_t;

/*lint -align_max(push) -align_max(1) */

/**
 * BLE advertisement packet types.
 */
typedef enum
{
    BLE_PACKET_TYPE_ADV_IND,            /**< Indication advertisement type */
    BLE_PACKET_TYPE_ADV_DIRECT_IND,     /**< Direct advertisement type */
    BLE_PACKET_TYPE_ADV_NONCONN_IND,    /**< Nonconnectable advertisement type */
    BLE_PACKET_TYPE_SCAN_REQ,           /**< Scan request type */
    BLE_PACKET_TYPE_SCAN_RSP,           /**< Scan response type */
    BLE_PACKET_TYPE_CONN_REQ,           /**< Connection request type */
    BLE_PACKET_TYPE_ADV_DISCOVER_IND    /**< Discoverable advertisement type */
} ble_packet_type_t ;

/**
 * BLE standard adv header.
 */
typedef struct __attribute((packed))
{
    uint8_t type        : 4;            /**< BLE packet type, from ble_packet_type_t enum */
    uint8_t _rfu1       : 2;            /**< Reserved for future use */
    uint8_t addr_type   : 1;            /**< BLE GAP address type */
    uint8_t _rfu2       : 1;            /**< Reserved for future use */
    uint8_t length;                     /**< Packet length */
    uint8_t _rfu3;                      /**< Reserved for future use */
} ble_packet_hdr_t;

/**
 * BLE standard AD-data header.
 */
typedef struct __attribute((packed))
{
    uint8_t length;                     /**< Length of type + data */
    uint8_t type;                       /**< Advertisement type, assigned by Bluetooth SIG */
} ble_ad_header_t;

/**
 * BLE standard ad-data format.
 */
typedef struct __attribute((packed))
{
    uint8_t length;                     /**< Length of type + data */
    uint8_t type;                       /**< Advertisement type, assigned by Bluetooth SIG */
    uint8_t data[];                     /**< Advertisement data, is length - 1 long */
} ble_ad_data_t;

/** BLE Service data packets. */
typedef struct __attribute((packed))
{
    uint16_t uuid;      /**< UUID field. */
    uint8_t data[];     /**< Service data field. */
} ble_ad_data_service_data_t;

/** DFU packet */
typedef struct __attribute((packed))
{
    uint16_t sd;                            /**< Softdevice ID. */
    nrf_mesh_bootloader_id_t  bootloader;   /**< Bootloader ID. */
    nrf_mesh_app_id_t app;                  /**< Application ID. */
} fwid_t;

/** Types of DFU packets. */
typedef enum
{
    DFU_PACKET_TYPE_RELAY_REQ   = 0xFFF9, /**< Relay request packet. */
    DFU_PACKET_TYPE_DATA_RSP    = 0xFFFA, /**< Data response packet. */
    DFU_PACKET_TYPE_DATA_REQ    = 0xFFFB, /**< Data request packet. */
    DFU_PACKET_TYPE_DATA        = 0xFFFC, /**< Data packet. */
    DFU_PACKET_TYPE_STATE       = 0xFFFD, /**< State packet. */
    DFU_PACKET_TYPE_FWID        = 0xFFFE, /**< FWID packet. */
} dfu_packet_type_t;

/** DFU packet parameters. */
typedef struct __attribute((packed))
{
    uint16_t packet_type;                                        /**< DFU packet type, see @ref dfu_packet_type_t. */
    /** Union of all DFU packet parameters. */
    union __attribute((packed))
    {
        fwid_t fwid;                                             /**< Firmware ID packet parameters. */
        /** State packet parameters. */
        struct __attribute((packed))
        {
            uint8_t dfu_type    : 4;                             /**< DFU type of current transfer. */
            uint8_t _rfu1       : 4;                             /**< Reserved for future usage. */
            uint8_t authority   : 3;                             /**< Authority level of current transfer. */
            uint8_t flood       : 1;                             /**< Whether the current transfer should be flooded. */
            uint8_t lazy_relay  : 1;                             /**< Whether the device requires a relay request before it relays the transfer. */
            uint8_t _rfu2       : 3;                             /**< Reserved for future usage. */
            uint32_t transaction_id;                             /**< Transaction ID of current transfer. */
            nrf_mesh_fwid_t fwid;                                /**< Firmware ID of current transfer. */
        } state;
        /** Start packet parameters. */
        struct __attribute((packed))
        {
            uint16_t segment;                                    /**< Segment ID of start packet, is always 0. */
            uint32_t transaction_id;                             /**< Transaction ID of current transfer. */
            uint32_t start_address;                              /**< Start address of the transfer, or 0xFFFFFFFF if unknown. */
            uint32_t length;                                     /**< Length of transfer, in words. */
            uint16_t signature_length;                           /**< Length of signature in bytes. */
            uint8_t diff        : 1;                             /**< Whether this transfer is a diff. */
            uint8_t single_bank : 1;                             /**< Whether this transfer should be done in a single bank configuration. */
            uint8_t first       : 1;                             /**< Whether this transfer is the first in a set of transfers. */
            uint8_t last        : 1;                             /**< Whether this transfer is the last in a set of transfers. */
            uint8_t _rfu        : 4;                             /**< Reserved for future usage. */
        } start;
        /** Data packet parameters. */
        struct __attribute((packed))
        {
            uint16_t segment;                                    /**< Segment ID of the data segment. */
            uint32_t transaction_id;                             /**< Transaction ID the data segment belongs to. */
            uint8_t data[NRF_MESH_DFU_SEGMENT_LENGTH];           /**< Data in the segment. */
        } data;
        /** Data request packet parameters. */
        struct __attribute((packed))
        {
            uint16_t segment;                                    /**< Segment ID being requested. */
            uint32_t transaction_id;                             /**< Transaction ID the request is done for. */
        } req_data;
        /** Data response packet parameters. */
        struct __attribute((packed))
        {
            uint16_t segment;                                    /**< Segment ID of the data segment. */
            uint32_t transaction_id;                             /**< Transaction ID the data segment belongs to. */
            uint8_t data[NRF_MESH_DFU_SEGMENT_LENGTH];           /**< Data in the segment. */
        } rsp_data;
        /** Relay request packet parameters. */
        struct __attribute((packed))
        {
            uint32_t transaction_id;                             /**< Transaction ID of transfer to request relaying for. */
            uint8_t adv_addr[BLE_GAP_ADDR_LEN];                  /**< Advertisement address of device to request relay from. */
        } relay_req;
    } payload;
} nrf_mesh_dfu_packet_t;

/**
 * BLE standard adv packet.
 */
typedef struct __attribute((packed))
{
    ble_packet_hdr_t header;                            /**< BLE packet header */
    uint8_t addr[BLE_GAP_ADDR_LEN];                     /**< BLE GAP advertisement address */
    uint8_t payload[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; /**< BLE advertisement payload */
} packet_t;

/**
 * Network packet header.
 */
typedef struct __attribute((packed))
{
    uint8_t  nid    :  7;  /**< LSB of NetworkID. */
    uint8_t  ivi    :  1;  /**< LSB of IV_index. */
    uint32_t ttl    :  7;  /**< Time-to-live. */
    uint32_t ctl    :  1;  /**< Control message bit. */
    uint32_t seq    : 24;  /**< Message sequence number. */
    uint16_t src;          /**< Source address. */
    uint16_t dst;          /**< Destination address. */
} packet_net_hdr_t;

/**
 * Mesh network packet.
 */
typedef struct __attribute((packed))
{
    uint8_t             length;         /**< Length of network packet = ad_type+header+payload. */
    uint8_t             ad_type;        /**< Reserved for future use. */
    packet_net_hdr_t    header;         /**< Network header. */
    uint8_t             payload[];      /**< Application payload, variable size and includes MIC_app and MIC_net. */
} packet_net_t;

/**
 * Metadata related to packet (RX).
 */
typedef struct __attribute((packed))
{
    uint32_t timestamp;         /**< Timestamp at which the packet was received. */
    int8_t   rssi;              /**< RSSI value of packet. */
    uint8_t  addr_type;         /**< Advertisement address type in the packet. */
    uint8_t* p_addr;            /**< Pointer to the advertisement address field in the packet. */
} packet_meta_t;

/*lint -align_max(pop) */

/**
 * Gets pointer to the start of the AD structure of a specified type.
 *
 * This can be used to obtain a pointer to, e.g., the @ref packet_net_t structure from the payload
 * of BLE packets.
 *
 * @param[in] p_packet Pointer to BLE advertising packet.
 * @param[in] ad_type  Advertising type to filter.
 * @return             Generic pointer to ad_data field of BLE packet.
 */
uint8_t * packet_type_packet_get(packet_t * p_packet, uint8_t ad_type);

/**
 * Get next AD data structure in the packet.
 *
 * @param[in] p_ad Pointer to current ad structure to iterate from.
 *
 * @return Pointer to next ad structure after @p p_ad.
 */
static inline ble_ad_data_t* packet_ad_type_get_next(ble_ad_data_t* p_ad)
{
    return (ble_ad_data_t*) ((uint8_t*) p_ad + p_ad->length + BLE_AD_DATA_OVERHEAD);
}

/**
 * Sets the AD field type of a packet.
 *
 * This sets the AD type for the first AD structure in the payload field of a packet.
 *
 * @param p_packet Pointer to the BLE advertising packet.
 * @param type which type to set in the ad_type field.
 */
static inline void packet_ad_type_set(packet_t * p_packet, uint8_t type)
{
    ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &p_packet->payload[0];
    p_ad_data->type = type;
}

/**
 * Sets the AD field length of a packet.
 *
 * This sets the length for the first AD structure in the payload field of a packet.
 *
 * @param p_packet Pointer to the BLE advertising packet.
 * @param length   Length of the AD field (not including the size of the length field).
 */
static inline void packet_ad_length_set(packet_t * p_packet, uint8_t length)
{
    ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &p_packet->payload[0];
    p_ad_data->length = length;
}

/**
 * Gets the ad_type of the first advertising field of an advertising packet.
 * @param  p_packet Pointer to the BLE advertising packet.
 * @return type     The ad_type
 */
static inline uint8_t packet_ad_type_get(packet_t * p_packet)
{
    ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &p_packet->payload[0];
    return p_ad_data->type;
}

/**
 * Sets the payload size of a packet.
 * @param p_packet Pointer to the BLE advertising packet.
 * @param size     Size of the payload field of the packet.
 */
static inline void packet_payload_size_set(packet_t * p_packet, uint8_t size)
{
    p_packet->header.length = size + BLE_GAP_ADDR_LEN;
}

/**
 * Gets the length of the payload field of a packet.
 * @param p_packet Pointer to the BLE advertising packet.
 * @return length of the payload field of the packet.
 */
static inline uint8_t packet_payload_size_get(const packet_t * p_packet)
{
    return p_packet->header.length - BLE_GAP_ADDR_LEN;
}

/**
 * Gets the minimum buffer length required for storing a packet.
 * @param p_packet Pointer to a BLE advertising packet.
 * @return minimum size of a buffer that can store the packet.
 */
static inline uint8_t packet_buffer_size_get(const packet_t * p_packet)
{
    return packet_payload_size_get(p_packet) + sizeof(ble_packet_hdr_t) + BLE_GAP_ADDR_LEN;
}

/**
 * Gets a pointer to the mesh network payload of a given ble_adv_packet_t.
 * @param p_packet Pointer to the BLE advertising packet.
 * @return Pointer to the mesh network packet.
 */
static inline packet_net_t * packet_net_packet_get(packet_t* p_packet)
{
    return (packet_net_t*) packet_type_packet_get(p_packet, AD_TYPE_MESH);
}

/**
 * Sets the payload size of a mesh network packet.
 *
 * This sets the AD field length for the mesh packet to the correct value according to the size of
 * the mesh packet payload.
 *
 * @param p_packet Pointer to the mesh network packet.
 * @param payload_size Size of the payload field of the packet.
 */
static inline void packet_net_payload_size_set(packet_net_t * p_packet, uint8_t payload_size)
{
    p_packet->length = payload_size + sizeof(packet_net_hdr_t) + sizeof(p_packet->ad_type);
}

/**
 * Gets the payload length from a mesh network packet.
 * @param p_packet Pointer to the mesh network packet.
 * @return Size of the payload field of the packet.
 */
static inline uint8_t packet_net_payload_size_get(const packet_net_t * p_packet)
{
    return p_packet->length - sizeof(p_packet->ad_type) - sizeof(packet_net_hdr_t);
}

/**
 * Gets the minimum buffer size required for storing a network packet.
 * @note The value returned from this function does not include the space required
 *       to store the BLE packet header, see @c packet_buffer_size_get.
 * @param p_packet Pointer to a mesh network packet.
 * @return Minimum size of a buffer that can store the packet.
 */
static inline uint8_t packet_net_buffer_size_get(const packet_net_t * p_packet)
{
    return sizeof(packet_net_t) + packet_net_payload_size_get(p_packet);
}

/**
 * Gets the NET MIC from a mesh network packet.
 * @param p_packet Pointer to the mesh network packet.
 * @return NET MIC of the packet.
 */
static inline uint32_t packet_net_mic_get(const packet_net_t* p_packet)
{
    const uint8_t length = packet_net_payload_size_get(p_packet);
    if (length >= 4)
    {
        return (uint32_t) (((uint32_t) (p_packet->payload[length - 1]) << 24)
                         | ((uint32_t) (p_packet->payload[length - 2]) << 16)
                         | ((uint32_t) (p_packet->payload[length - 3]) << 8)
                         | ((uint32_t) (p_packet->payload[length - 4]) << 0));
    }
    else
    {
        return 0;
    }
}

/**
 * Set the MIC field of a network packet.
 *
 * @param p_packet Pointer to network packet.
 * @param net_mic  MIC of network packet.
 *
 * @retval NRF_ERROR_INVALID_LENGTH Invalid length of network packet (len < 4).
 * @retval NRF_SUCCESS              MIC field successfully set.
 */
static inline void packet_net_mic_set(packet_net_t* p_packet, uint32_t net_mic)
{
    const uint8_t length = packet_net_payload_size_get(p_packet);
    NRF_MESH_ASSERT(length >= 4);
    p_packet->payload[length - 1] = ((net_mic >> 24) & 0xFF);
    p_packet->payload[length - 2] = ((net_mic >> 16) & 0xFF);
    p_packet->payload[length - 3] = ((net_mic >> 8) & 0xFF);
    p_packet->payload[length - 4] = ((net_mic >> 0) & 0xFF);
}

/**
 * Get the network MIC size in bytes.
 *
 * @param p_net_packet Pointer to network packet.
 *
 * @retval micsize Size of network MIC in bytes.
 */
static inline uint8_t packet_net_micsize_get(packet_net_t * p_net_packet)
{
    return BIT2MICSIZE((p_net_packet->header.ctl));
}


/** @} */

#endif

