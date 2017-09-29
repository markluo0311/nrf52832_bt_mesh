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



#ifndef SERIAL_EVT_H
#define SERIAL_EVT_H

#include <stdint.h>
#include "serial_cmd_rsp.h"
#include "nrf_mesh_serial.h"
#include "internal_event.h"
#include "packet.h"
/**
 * @defgroup SERIAL_EVT Serial events
 * @ingroup MESH_SERIAL
 * @{
 */

#define SERIAL_OPCODE_EVT_CMD_RSP                            (0x84) /**< Params: @ref serial_evt_cmd_rsp_t */

#define SERIAL_OPCODE_EVT_DEVICE_STARTED                     (0x81) /**< Params: @ref serial_evt_device_started_t */
#define SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP                    (0x82) /**< Params: @ref serial_evt_device_echo_t */
#define SERIAL_OPCODE_EVT_DEVICE_INTERNAL_EVENT              (0x83) /**< Params: @ref serial_evt_device_internal_event_t */

#define SERIAL_OPCODE_EVT_APPLICATION                        (0x8A) /**< Params: @ref serial_evt_application_t */

#define SERIAL_OPCODE_EVT_SAR_START                          (0x8B) /**< Params: None. */
#define SERIAL_OPCODE_EVT_SAR_CONTINUE                       (0x8C) /**< Params: None. */

#define SERIAL_OPCODE_EVT_DFU_REQ_RELAY                      (0xA0) /**< Params: @ref serial_evt_dfu_req_relay_t */
#define SERIAL_OPCODE_EVT_DFU_REQ_SOURCE                     (0xA1) /**< Params: @ref serial_evt_dfu_req_source_t */
#define SERIAL_OPCODE_EVT_DFU_START                          (0xA2) /**< Params: @ref serial_evt_dfu_start_t */
#define SERIAL_OPCODE_EVT_DFU_END                            (0xA3) /**< Params: @ref serial_evt_dfu_end_t */
#define SERIAL_OPCODE_EVT_DFU_BANK_AVAILABLE                 (0xA4) /**< Params: @ref serial_evt_dfu_bank_t */
#define SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED              (0xA5) /**< Params: @ref serial_evt_dfu_firmware_outdated_t */
#define SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH      (0xA6) /**< Params: @ref serial_evt_dfu_firmware_outdated_t */

#define SERIAL_OPCODE_EVT_OPENMESH_NEW                       (0xB3) /**< Params: None. */
#define SERIAL_OPCODE_EVT_OPENMESH_UPDATE                    (0xB4) /**< Params: None. */
#define SERIAL_OPCODE_EVT_OPENMESH_CONFLICTING               (0xB5) /**< Params: None. */
#define SERIAL_OPCODE_EVT_OPENMESH_TX                        (0xB6) /**< Params: None. */

#define SERIAL_OPCODE_EVT_PROV_UNPROVISIONED_RECEIVED        (0xC0) /**< Params: @ref serial_evt_prov_unprov_t */
#define SERIAL_OPCODE_EVT_PROV_LINK_ESTABLISHED              (0xC1) /**< Params: @ref serial_evt_prov_link_established_t */
#define SERIAL_OPCODE_EVT_PROV_LINK_CLOSED                   (0xC2) /**< Params: @ref serial_evt_prov_link_closed_t */
#define SERIAL_OPCODE_EVT_PROV_CAPS_RECEIVED                 (0xC3) /**< Params: @ref serial_evt_prov_caps_received_t */
#define SERIAL_OPCODE_EVT_PROV_COMPLETE                      (0xC5) /**< Params: @ref serial_evt_prov_complete_t */
#define SERIAL_OPCODE_EVT_PROV_AUTH_REQUEST                  (0xC6) /**< Params: @ref serial_evt_prov_auth_request_t */
#define SERIAL_OPCODE_EVT_PROV_ECDH_REQUEST                  (0xC7) /**< Params: @ref serial_evt_prov_ecdh_request_t */
#define SERIAL_OPCODE_EVT_PROV_OUTPUT_REQUEST                (0xC8) /**< Params: @ref serial_evt_prov_output_request_t */
#define SERIAL_OPCODE_EVT_PROV_FAILED                        (0xC9) /**< Params: @ref serial_evt_prov_failed_t */

#define SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_UNICAST      (0xD0) /**< Params: @ref serial_evt_mesh_message_received_t */
#define SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_SUBSCRIPTION (0xD1) /**< Params: @ref serial_evt_mesh_message_received_t */
#define SERIAL_OPCODE_EVT_MESH_TX_COMPLETE                   (0xD2) /**< Params: None. */
#define SERIAL_OPCODE_EVT_MESH_IV_UPDATE_NOTIFICATION        (0xD3) /**< Params: @ref serial_evt_mesh_iv_update_t */
#define SERIAL_OPCODE_EVT_MESH_KEY_REFRESH_START             (0xD5) /**< Params: None. */
#define SERIAL_OPCODE_EVT_MESH_KEY_REFRESH_END               (0xD6) /**< Params: None. */
#define SERIAL_OPCODE_EVT_MESH_SAR_FAILED                    (0xD7) /**< Params: None. */

#define SERIAL_OPCODE_EVT_MODEL_SPECIFIC                     (0xF0) /**< Params: @ref serial_evt_model_specific_t */

/** Message received event overhead. */
#define SERIAL_EVT_MESH_MESSAGE_RECEIVED_LEN_OVERHEAD    (19 + NRF_MESH_SERIAL_PACKET_OVERHEAD)
/** Message received event data max length */
#define SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN    (NRF_MESH_SERIAL_PAYLOAD_MAXLEN - (SERIAL_EVT_MESH_MESSAGE_RECEIVED_LEN_OVERHEAD - NRF_MESH_SERIAL_PACKET_OVERHEAD))
/** Internal event packet overhead: event_type, state, packet_size */
#define SERIAL_PACKET_INTERNAL_EVENT_OVERHEAD  (3)

/** Invalid key index. */
#define SERIAL_EVT_KEY_INDEX_INVALID              (0xFF)

/** Device operating mode. */
typedef enum
{
    SERIAL_DEVICE_OPERATING_MODE_TEST = 0,           /**< Testing operating mode. */
    SERIAL_DEVICE_OPERATING_MODE_BOOTLOADER = 1,     /**< Bootloader operating mode. */
    SERIAL_DEVICE_OPERATING_MODE_APPLICATION = 2,    /**< Regular operating mode. */

    /** @internal Largest number in the enum. */
    SERIAL_DEVICE_OPERATING_MODE__LAST = 2,
} serial_device_operating_mode_t;

/*lint -align_max(push) -align_max(1) */

/** Device started event packet. */
typedef struct __attribute((packed))
{
    uint8_t operating_mode; /**< Operating mode of the device. @see serial_device_operating_mode_t. */
    uint8_t hw_error; /**< Hardware error code, or 0 if no error occured. */
    uint8_t data_credit_available; /**< The number of bytes available in each of the tx and rx buffers. */
} serial_evt_device_started_t;

/** Echo data. */
typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN]; /**< Data received in the echo command. */
} serial_evt_device_echo_t;

/** Internal event data */
typedef struct __attribute((packed))
{
    uint8_t event_type;  /**< Reported event. See @ref internal_event_type_t for values. */
    uint8_t state;       /**< State information about the event type reported. */
    uint8_t packet_size; /**< Size (in bytes) of the packet. */
    uint8_t packet[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; /**< Event data. */
} serial_evt_device_internal_event_t;

/** Device event parameters. */
typedef union __attribute((packed))
{
    serial_evt_device_started_t started;  /**< Device started parameters. */
    serial_evt_device_echo_t echo;/**< Echo parameters. */
    serial_evt_device_internal_event_t internal_event; /**< Internal event data. */
} serial_evt_device_t;

/** Unprovisioned beacon received. */
typedef struct __attribute((packed))
{
    uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< UUID in the unprovisioned beacon. */
    int8_t  rssi; /**< RSSI of the received unprovisioned beacon. */
    uint8_t gatt_supported; /**< Whether the unprovisioned device supports GATT provisioning. */
    uint8_t adv_addr_type; /**< The advertisement address type of the sender of the unprovisioned beacon. */
    uint8_t adv_addr[BLE_GAP_ADDR_LEN]; /**< The advertisement address of the sender of the unprovisioned beacon. */
} serial_evt_prov_unprov_t;

/** Provisioning link established event. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the established link. */
} serial_evt_prov_link_established_t;

/** Provisioning link closed event. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the closed link. */
    uint8_t close_reason; /**< Reason for closing the link. */
} serial_evt_prov_link_closed_t;

/** Provisioning capabilities received event. */
typedef struct __attribute((packed))
{
    uint8_t context_id;          /**< Context ID of the link the capabilities were received on. */

    uint8_t num_elements;        /**< The number of elements on the unprovisoined device. */
    uint8_t public_key_type;     /**< The public key type used for the provisioning session. */
    uint8_t static_oob_types;    /**< The available static OOB authentication methods. */

    uint8_t output_oob_size;     /**< Maximum size of the output OOB supported. */
    uint16_t output_oob_actions; /**< Available OOB output actions. */

    uint8_t input_oob_size;      /**< Maximum size of the input OOB supported. */
    uint16_t input_oob_actions;  /**< Available OOB input actions. */
} serial_evt_prov_caps_received_t;

/** Provisioning complete event packet. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the completed provisioning link. */
    uint32_t iv_index; /**< IV index for the network. */
    uint16_t net_key_index; /**< Network key index. */
    uint16_t address; /**< Unicast address for the device. */
    uint8_t iv_update_flag; /**< IV update in progress flag. */
    uint8_t key_refresh_flag; /**< Key refresh in progress flag. */
    uint8_t device_key[NRF_MESH_KEY_SIZE]; /**< The device key of the provisioned device. */
    uint8_t net_key[NRF_MESH_KEY_SIZE]; /**< The network key of the provisioned device. */
} serial_evt_prov_complete_t;

/** Provisioning authentication data request event packet. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the link the authorization request appeared on. */
    uint8_t method; /**< Method of authentication requested. */
    uint8_t action; /**< Authentication action. */
    uint8_t size; /**< Authentication size. */
} serial_evt_prov_auth_request_t;

/** ECDH request packet. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the link the ECDH request appeared on. */
    uint8_t peer_public[NRF_MESH_ECDH_PUBLIC_KEY_SIZE]; /**< ECDH public key. */
    uint8_t node_private[NRF_MESH_ECDH_PRIVATE_KEY_SIZE]; /**< ECDH private key. */
} serial_evt_prov_ecdh_request_t;

/** Output request packet. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the link the output request appeared on. */
    uint8_t output_action; /**< Output action requested. */
    uint8_t data[16]; /**< Data for the output request. */
} serial_evt_prov_output_request_t;

/** Provisioning failed packet. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< Context ID of the link the error happened on. */
    uint8_t error_code; /**< Provisioning error code. */
} serial_evt_prov_failed_t;
/** Provisioning event parameters. */
typedef union __attribute((packed))
{
    serial_evt_prov_unprov_t           unprov;           /**< Unprovisioned event parameters. */
    serial_evt_prov_link_established_t link_established; /**< Link established event parameters. */
    serial_evt_prov_link_closed_t      link_closed;      /**< Link closed event parameters. */
    serial_evt_prov_caps_received_t    caps_received;    /**< Capabilities received parameters. */
    serial_evt_prov_complete_t         complete;         /**< Provisioning complete event parameters. */
    serial_evt_prov_auth_request_t     auth_request;     /**< Authorization request event parameters. */
    serial_evt_prov_ecdh_request_t     ecdh_request;     /**< ECDH request event parameters. */
    serial_evt_prov_output_request_t   output_request;   /**< Output request event parameters. */
    serial_evt_prov_failed_t           failed;           /**< Provisioning failed event parameters. */
} serial_evt_prov_t;

/** Header for the model specific events */
typedef struct __attribute((packed))
{
    access_model_id_t model_id; /**< ID of the model generating the event. */
    uint8_t  evt_type;   /**< Type of the event generated. */
} serial_evt_model_specific_header_t;

/** Event generated by one of the initialized models */
typedef struct __attribute((packed))
{
    /** Contains the model id the event generates from and the model specific event type. */
    serial_evt_model_specific_header_t model_evt_info;
    /** Additional data provided by the event */
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN - sizeof(serial_evt_model_specific_header_t)];
} serial_evt_model_specific_t;

/** Application data event parameters */
typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN]; /**< Application data. */
} serial_evt_application_t;

/** Mesh message received event parameters. */
typedef struct __attribute((packed))
{
    /** Source address of the received packet. */
    uint16_t src;
    /** Destination unicast address or subscription handle. */
    uint16_t dst;
    /** Handle of the application the message was received on. */
    uint16_t  appkey_handle;
    /** Handle of the subnetwork the message was received on. */
    uint16_t  subnet_handle;
    /** Packet time to live value when first received. */
    uint8_t  ttl;
    /** Advertisement address type of the last hop sender. */
    uint8_t  adv_addr_type;
    /** Advertisement address of the last hop sender. */
    uint8_t  adv_addr[BLE_GAP_ADDR_LEN];
    /** RSSI value of the message when received. */
    int8_t   rssi;
    /** Length of the received message, may be larger than the data reported if @ref
     * SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN is not big enough. */
    uint16_t actual_length;
    /** Data payload of the packet. */
    uint8_t  data[SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN];
} serial_evt_mesh_message_received_t;

/** Mesh IV update event parameters. */
typedef struct __attribute((packed))
{
    uint32_t iv_index;      /**< IV index updated to. */
} serial_evt_mesh_iv_update_t;

/** Union of all serial event parameters */
typedef union __attribute((packed))
{
    serial_evt_mesh_message_received_t  message_received;  /**< Message received parameters. */
    serial_evt_mesh_iv_update_t         iv_update;/**< IV update parameters. */
} serial_evt_mesh_t;

/********* DFU parameters *********/

/** DFU relay request event parameters. */
typedef struct __attribute((packed))
{
    uint8_t         dfu_type;  /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
    nrf_mesh_fwid_t fwid;      /**< Firmware ID of the requested transfer. */
    uint8_t         authority; /**< Authority level of the transfer. */
} serial_evt_dfu_req_relay_t;

/** DFU source request event parameters. */
typedef struct __attribute((packed))
{
    uint8_t         dfu_type; /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
} serial_evt_dfu_req_source_t;

/** DFU start event parameters. */
typedef struct __attribute((packed))
{
    uint8_t             role;     /**< The device's role in the transfer. See @ref nrf_mesh_dfu_role_t. */
    uint8_t             dfu_type; /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
    nrf_mesh_fwid_t     fwid;     /**< Firmware ID of the transfer. */
} serial_evt_dfu_start_t;

/** DFU end event parameters. */
typedef struct __attribute((packed))
{
    uint8_t             role;     /**< The device's role in the transfer. See @ref nrf_mesh_dfu_role_t. */
    uint8_t             dfu_type; /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
    nrf_mesh_fwid_t     fwid;     /**< Firmware ID of the transfer. */
    uint8_t             end_reason; /**< Reason for ending the transfer. See @ref nrf_mesh_dfu_end_t. */
} serial_evt_dfu_end_t;

/** DFU bank available event parameters. */
typedef struct __attribute((packed))
{
    uint8_t         dfu_type;   /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
    nrf_mesh_fwid_t fwid;       /**< Firmware ID of the transfer. */
    uint32_t        start_addr; /**< Start address of the bank. */
    uint32_t        length;     /**< Length of the banked firmware. */
    uint8_t         is_signed;  /**< Whether the bank is signed or not. */
} serial_evt_dfu_bank_t;

/** DFU firmware outdated event parameters. */
typedef struct __attribute((packed))
{
    uint8_t         dfu_type;       /**< DFU type of the transfer. See @ref nrf_mesh_dfu_type_t. */
    nrf_mesh_fwid_t available_fwid; /**< Firmware ID of the newest firmware available. */
    nrf_mesh_fwid_t current_fwid;   /**< Firmware ID of the current version of the outdated firmware. */
} serial_evt_dfu_firmware_outdated_t;

/** DFU event parameters. */
typedef union __attribute((packed))
{
    serial_evt_dfu_req_relay_t         req_relay;         /**< DFU relay request parameters. */
    serial_evt_dfu_req_source_t        req_source;        /**< DFU source request parameters. */
    serial_evt_dfu_start_t             start;             /**< DFU start parameters. */
    serial_evt_dfu_end_t               end;               /**< DFU end parameters. */
    serial_evt_dfu_bank_t              bank;              /**< DFU bank available parameters. */
    serial_evt_dfu_firmware_outdated_t firmware_outdated; /**< Firmware outdated parameters. */
} serial_evt_dfu_t;

/** Union of all serial event parameters */
typedef union __attribute((packed))
{
    serial_evt_cmd_rsp_t     cmd_rsp;      /**< Command response parameters. */
    serial_evt_device_t      device;       /**< Device parameters. */
    serial_evt_prov_t        prov;         /**< Provisioning parameters. */
    serial_evt_application_t application;  /**< Application parameters */
    serial_evt_mesh_t        mesh;         /**< Mesh parameters */
    serial_evt_dfu_t         dfu;          /**< DFU parameters. */
    serial_evt_model_specific_t model;     /**< Event generated by one of the models. */
} serial_evt_t;

/*lint -align_max(pop) */
/** @} */

#endif

