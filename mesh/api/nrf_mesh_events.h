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

#ifndef MESH_EVT_H__
#define MESH_EVT_H__

#include <stdbool.h>
#include "nrf_mesh.h"
#include "nrf_mesh_dfu_types.h"

/**
 * @defgroup NRF_MESH_EVENTS Mesh events API
 * @ingroup MESH_API_GROUP_CORE
 * @{
 */

/**
 * Mesh event types.
 */
typedef enum
{
    /** A message has been received. */
    NRF_MESH_EVT_MESSAGE_RECEIVED,
    /** Transmission completed. */
    NRF_MESH_EVT_TX_COMPLETE,
    /** An IV update was performed. */
    NRF_MESH_EVT_IV_UPDATE_NOTIFICATION,
    /** Received an unprovisioned node beacon. */
    NRF_MESH_EVT_UNPROVISIONED_RECEIVED,
    /** Provisioning link established. */
    NRF_MESH_EVT_PROV_LINK_ESTABLISHED,
    /** Provisioning link lost. */
    NRF_MESH_EVT_PROV_LINK_CLOSED,
    /** Provisioning output request. */
    NRF_MESH_EVT_PROV_OUTPUT_REQUEST,
    /** Provisioning input request. */
    NRF_MESH_EVT_PROV_INPUT_REQUEST,
    /** Provisioning static data request. */
    NRF_MESH_EVT_PROV_STATIC_REQUEST,
    /** OOB public key requested. */
    NRF_MESH_EVT_PROV_OOB_PUBKEY_REQUEST,
    /** Provisionee capabilities received. */
    NRF_MESH_EVT_PROV_CAPS_RECEIVED,
    /** Provisioning completed. */
    NRF_MESH_EVT_PROV_COMPLETE,
    /** ECDH calculation requested. */
    NRF_MESH_EVT_PROV_ECDH_REQUEST,
    /** Provisioning failed message received. */
    NRF_MESH_EVT_PROV_FAILED,
    /** Key refresh procedure initiated. */
    NRF_MESH_EVT_KEY_REFRESH_START,
    /** Key refresh procedure completed. */
    NRF_MESH_EVT_KEY_REFRESH_END,
    /** DFU request for this node to be the relay of a transfer. */
    NRF_MESH_EVT_DFU_REQ_RELAY,
    /** DFU request for this node to be the source of a transfer. */
    NRF_MESH_EVT_DFU_REQ_SOURCE,
    /** DFU transfer starting. */
    NRF_MESH_EVT_DFU_START,
    /** DFU transfer ended. */
    NRF_MESH_EVT_DFU_END,
    /** DFU bank available. */
    NRF_MESH_EVT_DFU_BANK_AVAILABLE,
    /** The device firmware is outdated, according to a trusted source. */
    NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED,
    /** The device firmware is outdated, according to an un-authenticated source. */
    NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH,
    /** RX failed. */
    NRF_MESH_EVT_RX_FAILED,
    /** SAR session failed. */
    NRF_MESH_EVT_SAR_FAILED,
    /** Flash has malfunctioned. */
    NRF_MESH_EVT_FLASH_FAILED
} nrf_mesh_evt_type_t;

/**
 * Message received event structure.
 */
typedef struct
{
    /** Buffer containing the message data. */
    const uint8_t * p_buffer;
    /** Message length. */
    uint16_t length;
    /** Source address of the message. */
    nrf_mesh_address_t src;
    /** Destination address of the message. */
    nrf_mesh_address_t dst;
    /** Security material used in the decryption of the payload. */
    nrf_mesh_secmat_t secmat;
    /** Time-to-live for the message, this is a 7-bit value. */
    uint8_t ttl;
    /** Advertisement address. */
    ble_gap_addr_t adv_addr;
    /** RSSI value of received packet */
    int8_t rssi;
} nrf_mesh_evt_message_t;

/**
 * IV update event structure.
 */
typedef struct
{
    /** IV update state. */
    net_state_iv_update_t state;
    /** The new IV index. */
    uint32_t iv_index;
} nrf_mesh_evt_iv_update_notification_t;

/**
 * Unprovisioned node beacon received event structure.
 */
typedef struct
{
    /** Device UUID of the unprovisioned node. */
    uint8_t device_uuid[NRF_MESH_UUID_SIZE];
    /** Advertisement address of the unprovisioned node. */
    ble_gap_addr_t adv_addr;
    /** Provisioning over GATT supported by the unprovisioned node.  */
    bool gatt_supported;
    /** RSSI of the received beacon. */
    int8_t rssi;
    /** Whether the beacon has a URI hash or not. */
    bool uri_hash_present;
    /** Hash of the URI in the beacon, or 0 if no URI is present. */
    uint8_t uri_hash[NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE];
} nrf_mesh_evt_unprov_recv_t;

/**
 * Provisioning link established event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_prov_link_established_evt_t;

/**
 * Provisioning link closed event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Reason for closing the link. */
    nrf_mesh_prov_link_close_reason_t close_reason;
} nrf_mesh_prov_link_closed_evt_t;

/**
 * Provisioning input requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Input action requested. */
    nrf_mesh_prov_input_action_t action;
    /** Size of the input data requested. */
    uint8_t size;
} nrf_mesh_evt_prov_input_request_t;

/**
 * Provisioning output requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Requested action. */
    nrf_mesh_prov_output_action_t action;
    /** Size of the output data provided. */
    uint8_t size;
    /** Pointer to the data to output. */
    const uint8_t * p_data;
} nrf_mesh_evt_prov_output_request_t;

/**
 * Static provisioning data requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_evt_prov_static_request_t;

/**
 * OOB authentication capabilities received from the provisionee.
 *
 * When this event is received, the application must provide the provisioning
 * stack with the OOB authentication mechanism to use via the
 * @ref nrf_mesh_prov_oob_use() function.
 *
 * @note Only _provisioner_ devices will receive this event. To set the OOB
 * capabilities for a _provisionee_ device, specify the capabilities when
 * initializing the provisioning stack.
 *
 * @see nrf_mesh_prov_oob_use()
 */
typedef struct
{
    /** Provisioning context where the capabilities were received. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Capabilities reported by the provisionee node. */
    nrf_mesh_prov_oob_caps_t oob_caps;
} nrf_mesh_evt_prov_caps_received_t;

/**
 * Provisioning complete event. Signals the completion of a provisioning session.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Pointer to the received network key. This must be copied into static memory before use. */
    const uint8_t * p_netkey;
    /** Device key of the provisioned device. This must be copied into static memory before use. */
    const uint8_t * p_devkey;
    /** IV index for the network. */
    uint32_t iv_index;
    /** Network key index. */
    uint16_t netkey_index;
    /** Unicast address for the device. */
    uint16_t address;
    /** Flags. */
    struct {
        /** IV update in progress flag. */
        uint8_t iv_update   : 1;
        /** Key refresh in progress flag. */
        uint8_t key_refresh : 1;
    } flags;
} nrf_mesh_evt_prov_complete_t;

/**
 * Provisioning failed event. Signals that a Provisioning Failed message has been
 * sent or received because of a provisioning protocol error.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Failure code indicating which error occured. */
    nrf_mesh_prov_failure_code_t failure_code;
} nrf_mesh_evt_prov_failed_t;

/**
 * Out-of-band public key requested event. Upon receiving this event, a provisioner
 * should read the public key from a node and provide it to the provisioning module
 * through the nrf_mesh_prov_pubkey_provide() function.
 */
typedef struct
{
    /** Provisioning context. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_evt_prov_oob_pubkey_request_t;

/**
 * Request for the application to perform the ECDH calculation. This event will only
 * be emitted if ECDH offloading is enabled by the appropriate option with
 * @ref nrf_mesh_opt_set().
 */
typedef struct
{
    /** Provisioning context. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Pointer to the public key of the peer node. */
    const uint8_t * p_peer_public;
    /** Private key of this node. */
    const uint8_t * p_node_private;
} nrf_mesh_evt_prov_ecdh_request_t;



/**
 * Key refresh initiated event structure.
 */
typedef struct
{
    /** Network ID. */
    uint8_t network_id[8];
} nrf_mesh_evt_key_refresh_t;

/**
 * Transmission complete event structure.
 */
typedef struct
{
    /** Packet identifier. */
    uint32_t packet_id;
} nrf_mesh_evt_tx_complete_t;

/** DFU event parameters. */
typedef union
{
    /** Firmware outdated event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        nrf_mesh_fwid_t current;                /**< FWID union containing the current firmware ID of the given type. */
    } fw_outdated;
    /** DFU Relay request event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        uint8_t authority;                      /**< Authority level of the transfer. */
    } req_relay;
    /** DFU Source request event parameters. */
    struct
    {
        nrf_mesh_dfu_type_t dfu_type;           /**< DFU type and firmware ID of the transfer. */
    } req_source;
    /** DFU Start event parameters. */
    struct
    {
        nrf_mesh_dfu_role_t     role;           /**< The device's role in the transfer. */
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
    } start;
    /** DFU end event parameters. */
    struct
    {
        nrf_mesh_dfu_role_t     role;           /**< The device's role in the transfer. */
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        nrf_mesh_dfu_end_t      end_reason;     /**< Reason for the end event. */
    } end;
    /** Bank available event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the bank. */
        uint32_t* p_start_addr;                 /**< Start address of the bank. */
        uint32_t length;                        /**< Length of the firmware in the bank. */
        bool is_signed;                         /**< Flag indicating whether the bank is signed with an encryption key. */
    } bank;
} nrf_mesh_evt_dfu_t;

/**
 * SAR complete event structure.
 */
typedef struct
{
    /** Packet ID of the SAR session. */
    uint32_t packet_id;
} nrf_mesh_evt_sar_t;


/**
 * RX failure reason codes.
 */
typedef enum
{
    /** The replay protection cache is full. */
    NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL
}nrf_mesh_rx_failed_reason_t;

/**
 * RX failed event structure.
 */
typedef struct
{
    /** Unicast address of the sender */
    uint16_t src;
    /**IV index bit of the rx packet */
    uint8_t ivi : 1;
    /** Reason for the rx failure */
    nrf_mesh_rx_failed_reason_t reason;
}nrf_mesh_evt_rx_failed_t;

/**
 * SAR session cancelled reason codes.
 */
typedef enum
{
    /** The transport SAR session timed out. */
    NRF_MESH_SAR_CANCEL_REASON_TIMEOUT,
    /** The transport SAR session TX retry limit was exceeded. */
    NRF_MESH_SAR_CANCEL_REASON_RETRY,
    /** There were not enough resources to process the transport SAR session. */
    NRF_MESH_SAR_CANCEL_REASON_NO_MEM,
    /** The peer has cancelled the SAR session */
    NRF_MESH_SAR_CANCEL_BY_PEER
} nrf_mesh_sar_session_cancel_reason_t;

/**
 * SAR failed event structure.
 */
typedef struct
{
    /** Packet ID of the SAR session. */
    uint32_t packet_id;
    /** Reason for closing the session. */
    nrf_mesh_sar_session_cancel_reason_t reason;
} nrf_mesh_evt_sar_failed_t;

/**
 * Type of users of the flash manager.
 */
typedef enum
{
    NRF_MESH_FLASH_USER_CORE,
    NRF_MESH_FLASH_USER_DEVICE_STATE_MANAGER,
    NRF_MESH_FLASH_USER_ACCESS
} nrf_mesh_flash_user_module_t;

typedef struct
{
    /** The module the event is reported from. */
    nrf_mesh_flash_user_module_t user;
    /** The flash entry that has failed. */
    const void * p_flash_entry;
    /** The address of the flash page the attempted operation failed. */
    void * p_flash_page;
    /** The start of area owned by the flash manager of the module reporting the event. */
    const void * p_area;
    /** The number of pages given to the flash manager of the module reporting the event. */
    uint32_t page_count;
} nrf_mesh_evt_flash_failed_t;

/**
 * Mesh event structure.
 */
typedef struct
{
    /** Type of event. */
    nrf_mesh_evt_type_t type;

    /** Event parameters. */
    union {
        /** Incoming message. */
        nrf_mesh_evt_message_t                  message;
        /** Transmission complete. */
        nrf_mesh_evt_tx_complete_t              tx_complete;
        /** IV update notification event. */
        nrf_mesh_evt_iv_update_notification_t   iv_update;
        /** Unprovisioned beacon received event. */
        nrf_mesh_evt_unprov_recv_t              unprov_recv;

        /** Provisioning link established event. */
        nrf_mesh_prov_link_established_evt_t    prov_link_established;
        /** Provisioning link lost event. */
        nrf_mesh_prov_link_closed_evt_t         prov_link_closed;
        /** Provisioning input requested. */
        nrf_mesh_evt_prov_input_request_t       prov_input_request;
        /** Provisioning output requested. */
        nrf_mesh_evt_prov_output_request_t      prov_output_request;
        /** Static provisioning data requested. */
        nrf_mesh_evt_prov_static_request_t      prov_static_request;
        /** OOB public key requested. */
        nrf_mesh_evt_prov_oob_pubkey_request_t  prov_oob_pubkey_request;
        /** Provisioning capabilities received. */
        nrf_mesh_evt_prov_caps_received_t       prov_oob_caps_received;
        /** Provisioning complete. */
        nrf_mesh_evt_prov_complete_t            prov_complete;
        /** ECDH request. */
        nrf_mesh_evt_prov_ecdh_request_t        prov_ecdh_request;
        /** Provisioning failed. */
        nrf_mesh_evt_prov_failed_t              prov_failed;

        /** Key refresh initiated event. */
        nrf_mesh_evt_key_refresh_t              key_refresh;
        /** DFU event. */
        nrf_mesh_evt_dfu_t                      dfu;
        /** RX failed. */
        nrf_mesh_evt_rx_failed_t                rx_failed;
        /** SAR event */
        nrf_mesh_evt_sar_t                      sar;
        /** SAR failed event. */
        nrf_mesh_evt_sar_failed_t               sar_failed;
        /** Flash failed event */
        nrf_mesh_evt_flash_failed_t             flash_failed;
    } params;
} nrf_mesh_evt_t;

/**
 * Mesh event handler callback type.
 *
 * Registering this callback forwards all events generated during
 * @ref nrf_mesh_process() to the application.
 *
 * @param[in] p_evt Mesh event pointer.
 */
typedef void (*nrf_mesh_evt_handler_cb_t)(nrf_mesh_evt_t * p_evt);

/**
 * Mesh event handler context structure.
 *
 * @note This structure must be statically allocated.
 */
typedef struct
{
    /** Callback function pointer. */
    nrf_mesh_evt_handler_cb_t evt_cb;
    /** Pointer to next handler in linked list. Set and used internally. */
    void * p_next;
} nrf_mesh_evt_handler_t;

/**
 * Registers an event handler to get events from the core stack.
 *
 * @todo Allow masking out certain events.
 *
 * @param[in] p_handler_params Event handler parameters.
 */
void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params);

/**
 * Removes an event handler.
 *
 * @param[in] p_handler_params Event handler parameters.
 */
void nrf_mesh_evt_handler_remove(nrf_mesh_evt_handler_t * p_handler_params);

/** @} */

#endif /* MESH_EVT_H__ */

