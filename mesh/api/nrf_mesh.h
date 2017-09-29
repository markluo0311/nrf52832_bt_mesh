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

#ifndef NRF_MESH_H__
#define NRF_MESH_H__

#include <stdint.h>
#include <stdbool.h>

#include <ble.h>
#include <ble_gap.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"

#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_mesh_prov.h"

/**
 * @defgroup NRF_MESH Core Mesh API
 * @ingroup MESH_API_GROUP_CORE
 * @{
 */

/**
 * @defgroup MESH_CORE_COMMON_TYPES Core type definitions
 * @{
 */

/**
 * Mesh assertion handler type.
 *
 * When an unexpected, fatal error occurs within the Mesh stack, it will call the Mesh
 * assertion handler callback. The Mesh stack will be in an undefined state when
 * this happens and the only way to recover will be to perform a reset, e.g. by
 * using CMSIS NVIC_SystemReset().
 *
 * @param[in] pc            The program counter of the failed assertion.
  */
typedef void (*nrf_mesh_assertion_handler_t)(uint32_t pc);

/**
 * Callback function type for checking if a given packet should be relayed on to other nodes.
 *
 * The mesh can be initialized with this callback if the application wants to have control over
 * which packets should be relayed to other nodes. This is done by passing the function pointer
 * to @ref nrf_mesh_init via the @ref nrf_mesh_init_params_t struct.
 *
 * The default behaviour, which is always applied in addition to the callback function, is
 * that the mesh will only relay new packets with a TTL larger than 1.
 *
 * @see nrf_mesh_init()
 *
 * @param[in] src The packet source address
 * @param[in] dst The packet destination address
 * @param[in] ttl The time-to-live value for the packet (6-bits)
 *
 * @returns @c true if the packet received should be relayed on to the other mesh nodes, @c false otherwise.
 */
typedef bool (*nrf_mesh_relay_check_cb_t)(uint16_t src, uint16_t dst, uint8_t ttl);

/** Arguments structure for the RX callback function. */
typedef struct
{
    ble_gap_addr_t addr;      /**< Advertisement address in the packet. */
    int8_t         rssi;      /**< RSSI of the received packet. */
    uint8_t        adv_type;  /**< BLE GAP advertising type. */
    uint8_t        length;    /**< Length of the advertisement packet payload. */
    uint8_t *      p_payload; /**< Pointer to the raw advertisement packet payload. */
    uint32_t       timestamp; /**< Timestamp of the packet reception in microseconds. */
} nrf_mesh_adv_packet_rx_data_t;

/**
 * Advertisement received callback function type.
 *
 * This callback can be used to receive raw advertisement packets.
 * This can be useful to listen for specific packets that are not handled by the mesh,
 * such as beacon packets or regular BLE advertisements.
 *
 * @param[in] p_rx_data Received advertisement packet data and metadata.
 */
typedef void (*nrf_mesh_rx_cb_t)(const nrf_mesh_adv_packet_rx_data_t * p_rx_data);

/**
 * Bluetooth Mesh beacon timer structure.
 *
 * @warning  All parameters are for internal use.
 * @internal If the interval is zero, a default broadcasting interval is used,
 *           otherwise the provided value is used.
 */
typedef struct nrf_mesh_beacon_timer
{
    /** Timestamp at which the beacon is sent. */
    uint32_t timestamp;
    /** Callback function pointer. */
    void (*callback)(uint32_t, void*);
    /** Interval between each beacon packet. */
    uint32_t interval;
    /** Beacon context pointer. */
    void * p_context;
    /** Pointer to next beacon in linked list. */
    struct nrf_mesh_beacon_timer * p_next;
} nrf_mesh_beacon_timer_t;

/**
 * Application security material structure.
 *
 * This structure is required for the encryption of the application data.
 *
 * @note This is intended to be managed by the device_state_manager.c, and the setters and getters
 * in that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Indicates whether the device key or the application is used. */
    bool is_device_key;
    /** Application ID. Calculated and used internally. */
    uint8_t aid;
    /** Application or device key storage. */
    uint8_t key[NRF_MESH_KEY_SIZE];
} nrf_mesh_application_secmat_t;

/**
 * Network security material structure.
 *
 * @note This is intended to be managed by the device_state_manager.c, and the setters and getters
 * in that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Network identifier. */
    uint8_t nid;
    /** Encryption key. */
    uint8_t encryption_key[NRF_MESH_KEY_SIZE];
    /** Privacy key. */
    uint8_t privacy_key[NRF_MESH_KEY_SIZE];
} nrf_mesh_network_secmat_t;

/**
 * Security material for the Bluetooth Mesh network beacons.
 * This structure is used when sending a mesh network beacon advertisement.
 *
 * @note This is managed by the device_state_manager.c, and the setters and getters in
 * that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Beacon key */
    uint8_t key[NRF_MESH_KEY_SIZE];
    /** Network ID */
    uint8_t net_id[NRF_MESH_NETID_SIZE];
} nrf_mesh_beacon_secmat_t;

/**
 * Run-time transmission information for individual beacons. Used for internal
 * logic.
 */
typedef struct
{
    /** Number of beacons received since this beacon was last transmitted. */
    uint16_t rx_count;
    /** Current beacon interval in seconds. */
    uint16_t tx_interval_seconds;
    /** Last transmission time for this beacon. */
    uint32_t tx_timestamp;
} nrf_mesh_beacon_tx_info_t;

/**
 * Information structure for the Bluetooth Mesh network beacons.
 * This structure keeps track of all information related to a single mesh
 * network beacon.
 */
typedef struct
{
    /** Flag indicating whether the given structure is allowed to initiate an
     * IV update. */
    bool iv_update_permitted;
    /** Pointer to a transmission info structure. */
    nrf_mesh_beacon_tx_info_t * p_tx_info;
    /** Beacon security material. */
    nrf_mesh_beacon_secmat_t secmat;
} nrf_mesh_beacon_info_t;

/**
 * Bluetooth Mesh security material structure.
 *
 * This structure is used when sending a mesh packet and it includes pointers to the network
 * and application security material structures.
 *
 * @note This can be populated by the @ref dsm_tx_secmat_get function.
 */
typedef struct
{
    /** Required for network layer encryption @see nrf_mesh_network_secmat_t.*/
    const nrf_mesh_network_secmat_t * p_net;
    /** Required for transport layer encryption @see nrf_mesh_application_secmat_t. */
    const nrf_mesh_application_secmat_t * p_app;
} nrf_mesh_secmat_t;

/** State of IV update procedure */
typedef enum
{
    /** In normal operation. */
    NET_STATE_IV_UPDATE_NORMAL,
    /** IV update procedure in progress. */
    NET_STATE_IV_UPDATE_IN_PROGRESS,
} net_state_iv_update_t;

/**
 * Bluetooth Mesh address types.
 *
 * Bluetooth Mesh defines 3 address types:
 *   - Unicast   <pre> 0xxx xxxx xxxx xxxx </pre>
 *   - Virtual   <pre> 10xx xxxx xxxx xxxx </pre>
 *   - Group     <pre> 11xx xxxx xxxx xxxx </pre>
 */
typedef enum
{
    /** Invalid address. */
    NRF_MESH_ADDRESS_TYPE_INVALID,
    /** Unicast address. */
    NRF_MESH_ADDRESS_TYPE_UNICAST,
    /** Virtual address. */
    NRF_MESH_ADDRESS_TYPE_VIRTUAL,
    /** Group address. */
    NRF_MESH_ADDRESS_TYPE_GROUP,
} nrf_mesh_address_type_t;

/**
 * Bluetooth Mesh address.
 *
 * @note When used to add RX addresses to the stack, the struct needs to be statically allocated.
 */
typedef struct
{
    /** Address type. */
    nrf_mesh_address_type_t type;
    /** Address value. */
    uint16_t value;
    /** 128-bit virtual label UUID, will be NULL if type is not @c NRF_MESH_ADDRESS_VIRTUAL . */
    const uint8_t * p_virtual_uuid;
} nrf_mesh_address_t;

/**
 * Mesh packet transmission parameters.
 *
 * @note If the length of the message is greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX, the
 * message will be sent as a segmented message and reassembled on the peer side.
 */
typedef struct
{
    /** Packet destination address. */
    nrf_mesh_address_t dst;
    /** Address of the element the packet originates from (must be a unicast address). */
    uint16_t src;
    /** Time to live value for the packet, this is a 7 bit value. */
    uint8_t ttl;
    /** See Section 3.7.5.2 in Bluetooth Mesh Core spec d09r23. */
    bool reliable;
    /** Points to the payload to be sent. */
    const uint8_t * p_data;
    /** Length of the payload being sent. */
    uint16_t data_len;
    /** Required for encryption @see nrf_mesh_secmat_t. */
    nrf_mesh_secmat_t security_material;
} nrf_mesh_tx_params_t;

/**
 * Initialization parameters structure.
 */
typedef struct
{
#if (defined(S140) || defined(S130) || defined(S132) || defined(HOST))
    nrf_clock_lf_cfg_t lfclksrc; /**< See nrf_sdm.h or SoftDevice documentation. */
#elif defined(S110)
    nrf_clock_lfclksrc_t lfclksrc; /**< See nrf_sdm.h or SoftDevice documentation. */
#else
    #error "Unknown target softdevice version"
#endif
    nrf_mesh_assertion_handler_t assertion_handler; /**< Assert callback function. */
    nrf_mesh_relay_check_cb_t relay_cb; /**< Application call back for relay decisions, can be NULL. */
} nrf_mesh_init_params_t;

/** @} end of MESH_CORE_COMMON_TYPES */

/**
 * Initializes the Bluetooth Mesh stack.
 *
 * @note The Nordic Semiconductor SoftDevice must be initialized by the
 *       application before this function is called.
 * @note Calling this function only initializes the Mesh stack. To start transmitting
 *       and receiving messages, @ref nrf_mesh_enable() must also be called. In addition,
 *       network and application keys must be added for the device to participate in a
 *       mesh network.
 * @note The Mesh is initialized with default parameters for the radio. To change
 *       these, use the options API, @ref nrf_mesh_opt_set().
 *
 * @warning Enabling _any_ proprietary extensions will break Bluetooth Mesh
 *          compatibility.
 *
 * @see nrf_mesh_enable(), nrf_mesh_opt_set()
 *
 * @param[in] p_init_params  Pointer to initialization parameter structure.
 *
 * @retval NRF_SUCCESS                      The mesh system was successfully initialized.
 * @retval NRF_ERROR_SOFTDEVICE_NOT_ENABLED The SoftDevice has not been enabled.
 * @retval NRF_ERROR_INVALID_STATE          The mesh stack has already been initialized.
 * @retval NRF_ERROR_NULL                   The @c p_init_params parameter was @c NULL.
 */
uint32_t nrf_mesh_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Enables the Mesh.
 *
 * @note Calling this function alone will not generate any events unless:
 *         - Network and application keys have been added.
 *         - At least one RX address has been added.
 *
 * @see nrf_mesh_rx_addr_add()
 *
 * @retval NRF_SUCCESS             The Mesh was started successfully.
 * @retval NRF_ERROR_INVALID_STATE The mesh was not initialized,
 *                                 see @ref nrf_mesh_init().
 */
uint32_t nrf_mesh_enable(void);

/**
 * Disables the Mesh.
 *
 * Calling this function will stop the Mesh, i.e, it will stop ordering
 * time slots from the SoftDevice and will not generate events.
 *
 * @retval NRF_SUCCESS The Mesh was stopped successfully.
 */
uint32_t nrf_mesh_disable(void);

/**
 * Queues a mesh packet for transmission.
 *
 * @note    Calling this function will give an @ref NRF_MESH_EVT_TX_COMPLETE event
 *          when the packet has been sent on air. The parameter given with the
 *          event is the same reference as returned in p_packet_reference.
 * @note    If the length of the message is greater than @ref
 *          NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX, the message will be sent as a
 *          segmented message and reassembled on the peer side.
 *
 * @param[in]  p_params            Pointer to a structure containing the
 *                                 parameters for the message to send.
 * @param[out] p_packet_reference  Pointer to store a reference to the packet
 *                                 queued for transmission.
 *                                 This parameter may be set to @c NULL for
 *                                 ignoring the reference.
 *
 * @retval NRF_SUCCESS             The message was successfully queued for transmission.
 * @retval NRF_ERROR_BUSY          The message was not sent. This can occur if the stack is
 *                                 in the process of changing IV indices. The application should
 *                                 try to send the packet again at a later point.
 * @retval NRF_ERROR_NO_MEM        A packet buffer could not be allocated for the packet. The application
 *                                 should try to send the packet again at a later point.
 * @retval NRF_ERROR_INVALID_PARAM A parameter was invalid. This can happen if
 *                                 the TTL field is <= 1 or if the destination
 *                                 address is invalid.
 * @retval NRF_ERROR_NULL          @c p_params is a @c NULL pointer or a required field of the struct
 *                                 (other than @c p_data) is @c NULL.
 */
uint32_t nrf_mesh_packet_send(const nrf_mesh_tx_params_t * p_params,
                              uint32_t * const p_packet_reference);

/**
 * Transmits an advertising packet with a custom payload.
 *
 * @note This is a one-shot function. It cannot be used to send periodic advertisements.
 * @note An @ref ::NRF_MESH_EVT_TX_COMPLETE event is generated when the payload has been sent.
 *
 * @param[in] p_payload         Pointer to the payload to include in the packet.
 * @param[in] length            Length of the payload data.
 *
 * @retval NRF_ERROR_INVALID_LENGTH The specified length was too large for an advertising packet.
 * @retval NRF_ERROR_NULL           The @c p_payload parameter is @c NULL.
 * @retval NRF_SUCCESS              The packet was successfully sent.
 */
uint32_t nrf_mesh_adv_send(const uint8_t * p_payload, uint8_t length);

/**
 * Runs the mesh packet processing process.
 *
 * Calling this function allows the mesh to run. The mesh stack will process buffered
 * incoming packets and send outgoing messages.
 *
 * @note During @c nrf_mesh_process(), all events generated by the Mesh will
 * be directly forwarded to the application if it has registered an event
 * callback using @ref nrf_mesh_evt_handler_add().
 *
 * @warning The Mesh will discard any data as soon as it has passed it on to the
 * application.
 *
 * @retval NRF_SUCCESS The packet processing loop completed successfully.
 */
uint32_t nrf_mesh_process(void);

/**
 * Pass SoftDevice BLE events to the Mesh.
 *
 * @warning This API call is not implemented.
 *
 * Add this function in the BLE event dispatcher function used with the
 * SoftDevice handler module (see softdevice_ble_evt_handler_set()
 * softdevice_handler.h in the SDK).
 *
 * @param[in] p_ble_evt Pointer to SoftDevice BLE event.
 *
 * @retval NRF_SUCCESS
 */
uint32_t nrf_mesh_on_ble_evt(ble_evt_t * p_ble_evt);

/**
 * Pass SoftDevice SoC events to the Mesh.
 *
 * Add this function in the SoC event dispatcher function used with the
 * SoftDevice handler module (see softdevice_sys_evt_handler_set()
 * softdevice_handler.h in the SDK).
 *
 * @warning It is vital for the Mesh to retrieve SoC events for it to function.
 *
 * @param[in] sd_evt SoftDevice SoC event.
 *
 * @retval NRF_SUCCESS Event successfully received.
 */
uint32_t nrf_mesh_on_sd_evt(uint32_t sd_evt);

/**
 * Set a callback which will be called for every packet being received.
 *
 * @param[in] rx_cb Receive callback function.
 */
void nrf_mesh_rx_cb_set(nrf_mesh_rx_cb_t rx_cb);

/**
 * Unregister the RX callback, if any.
 */
void nrf_mesh_rx_cb_clear(void);

/**
 * Allocate a packet for transmission from the nrf mesh stack.
 *
 * @todo Implement alloc function.
 *
 * @param[out] pp_packet         The pointer to the payload part of the mesh packet, should be used
 *                               with @ref nrf_mesh_tx_params_t.
 * @param[in]  size              The size of the payload in bytes.
 *
 * @retval     NRF_SUCCESS       The packet has been allocated and is ready for transmission.
 * @retval     NRF_ERROR_NO_MEM  Not enough memory available for the requested size of tx packet
 *                               allocation, transmit the existing packets and try again.
 */
// uint32_t nrf_mesh_packet_alloc(uint8_t ** pp_packet, uint16_t size);

/**
 * Release an allocated packet
 *
 *  * @todo Implement release function.
 */
// uint32_t nrf_mesh_packet_release(uint8_t * p_packet);

/** @} end of MESH_API_GROUP_CORE */
/** @} */
#endif  /* NRF_MESH_H__ */
