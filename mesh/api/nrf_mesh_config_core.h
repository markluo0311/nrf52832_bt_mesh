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

#ifndef NRF_MESH_CONFIG_CORE_H__
#define NRF_MESH_CONFIG_CORE_H__

#include "nrf_mesh_defines.h"

/**
 * @defgroup NRF_MESH_CONFIG_CORE Mesh core configuration parameters
 * @ingroup MESH_API_GROUP_CORE
 * @{
 */

/**
 * @defgroup MESH_CONFIG_GENERAL General configuration parameters
 * @{
 */

/**
 * Enable persistent storage.
 */
#ifndef PERSISTENT_STORAGE
#define PERSISTENT_STORAGE 1
#endif

/**
 * Define to "1" if the uECC libray is linked to the mesh stack.
 */
#ifndef NRF_MESH_UECC_ENABLE
#define NRF_MESH_UECC_ENABLE 1
#endif

/**
 * Default advertisement channel map (37, 38 and 39).
 */
#ifndef NRF_MESH_ADV_CHAN_DEFAULT
#define NRF_MESH_ADV_CHAN_DEFAULT NRF_MESH_ADV_CHAN_ALL
#endif

/** @} end of MESH_CONFIG_GENERAL */



/**
 * @defgroup MESH_CONFIG_BEACON Beacon configuration parameters
 * @{
 */

/**
 * Default interval for secure network broadcast beacons, in seconds.
 *
 * @note BLE Mesh specification: ~10s.
 */
#ifndef NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS
#define NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS 10
#endif

/** Default beacon interval. */
#ifndef BEACON_INTERVAL_MS_DEFAULT
#define BEACON_INTERVAL_MS_DEFAULT (1 * 1000)
#endif

/** @} end of MESH_CONFING_BEACON */



/**
 * @defgroup MESH_CONFIG_ENC Encryption configuration parameters
 * @{
 */

/**
 * Use the hardware AES-ECB block.
 *
 * @warning The S110 SoftDevice protects this hardware peripheral, but does not
 *          use it when we are in a timeslot. If there is not enough time to
 *          finish the AES-ECB operation before our timeslot ends, the module
 *          will use sd_ecb_block_encrypt().
 */
#ifndef AES_USE_HARDWARE
#define AES_USE_HARDWARE 1
#endif

/** Select AES-ECB software implementation (0, 1). */
#ifndef AES_SOFT_VERSION
#define AES_SOFT_VERSION 1
#endif

/** @} end of MESH_CONFIG_ENC */

/**
 * @defgroup MESH_CONFIG_BEARER Mesh bearer configuration
 * @{
 */

/** Specify which bearers are available to the mesh by default. */
#ifndef BEARER_TYPES
#define BEARER_TYPES BEARER_ADV_RADIO
#endif

/** Default number of repeated transmissions of one packet. */
#ifndef BEARER_ADV_REPEAT_DEFAULT
#define BEARER_ADV_REPEAT_DEFAULT 1
#endif

/** Size of the TX queue. */
#ifndef BEARER_TX_QUEUE_LENGTH
#define BEARER_TX_QUEUE_LENGTH 16
#endif

/** Size of the RX queue. */
#ifndef BEARER_RX_QUEUE_LENGTH
#define BEARER_RX_QUEUE_LENGTH 8
#endif

/** Enable assert on wrong RX length. */
#ifndef BEARER_ASSERT_ON_WRONG_RX_LENGTH
#define BEARER_ASSERT_ON_WRONG_RX_LENGTH 0
#endif

/** Default min advertisement interval. */
#ifndef BEARER_ADV_INT_MIN_MS_DEFAULT
#define BEARER_ADV_INT_MIN_MS_DEFAULT 20
#endif

/** Default max advertisement interval. */
#ifndef BEARER_ADV_INT_MAX_MS_DEFAULT
#define BEARER_ADV_INT_MAX_MS_DEFAULT 30
#endif

/** @} end of MESH_CONFIG_BEARER */

/**
 * @defgroup MESH_CONFIG_BEARER_ADV Advertisement bearer configuration
 * @{
 */

/** BLE-spec defined advertisement access address */
#ifndef BEARER_ADV_ACCESS_ADDR
#define BEARER_ADV_ACCESS_ADDR 0x8E89BED6
#endif

/** Lower boundary on advertisement interval. */
#ifndef BEARER_ADV_INT_MIN_MS
#define BEARER_ADV_INT_MIN_MS 20
#endif

/** Upper boundary on advertisement interval. */
#ifndef BEARER_ADV_INT_MAX_MS
#define BEARER_ADV_INT_MAX_MS 10240
#endif

/** Lower boundary on scan interval. */
#ifndef BEARER_ADV_SCAN_INT_MIN
#define BEARER_ADV_SCAN_INT_MIN 3
#endif

/** Upper boundary on scan interval. */
#ifndef BEARER_ADV_SCAN_INT_MAX
#define BEARER_ADV_SCAN_INT_MAX 10240
#endif

/** @} end of MESH_CONFIG_BEARER_ADV */

/**
 * @defgroup MESH_CONFIG_BEARER_EVENT Bearer event handler configuration
 * @{
 */

/** Length of the asynchronous processing queue. */
#ifndef BEARER_EVENT_FIFO_SIZE
#define BEARER_EVENT_FIFO_SIZE 16
#endif

/** Number of flags available for allocation. */
#ifndef BEARER_EVENT_FLAG_COUNT
#define BEARER_EVENT_FLAG_COUNT 4
#endif

/** @} end of MESH_CONFIG_BEARER_EVENT */

/**
 * @defgroup MESH_CONFIG_CCM AES-CCM module configuration
 * @{
 */

/** Enable debug printing from CCM module  */
#ifndef CCM_DEBUG_MODE_ENABLED
#define CCM_DEBUG_MODE_ENABLED 0
#endif

/** @} end of MESH_CONFIG_CCM */

/**
 * @defgroup MESH_CONFIG_EVENT Mesh event configuration
 * @{
 */

/** Maximum number of events to queue up for the application. */
#ifndef EVENT_QUEUE_SIZE
#define EVENT_QUEUE_SIZE 32
#endif

/** @} */

/**
 * @defgroup MESH_CONFIG_FIFO FIFO configuration
 * @{
 */

/**
 * Define to 1 to enable FIFO statistics tracking.
 * This adds additional performance counters to FIFO instances.
 */
#ifndef FIFO_STATS
#define FIFO_STATS 0
#endif

/** @} */

/**
 * @defgroup MESH_CONFIG_INTERNAL_EVENTS Internal event logging configuration
 * @{
 */

/** Define "1" to enable internal event logging.
 *
 * @warning This will be forced to 0 if the build type (@c CMAKE_BUILD_TYPE)
 * is not set to @c Debug or @c RelWithDebInfo when configuring CMake.
 */
#ifndef INTERNAL_EVT_ENABLE
#define INTERNAL_EVT_ENABLE 1
#endif

/** Internal event buffer size. */
#ifndef INTERNAL_EVENT_BUFFER_SIZE
#define INTERNAL_EVENT_BUFFER_SIZE 32
#endif

/** @} end of MESH_CONFIG_INTERNAL */

/**
 * @defgroup MESH_CONFIG_LOG Log module configuration
 * @{
 */

/** Enable logging module. */
#ifndef NRF_MESH_LOG_ENABLE
#define NRF_MESH_LOG_ENABLE 1
#endif

/** Default log level. */
#ifndef LOG_LEVEL_DEFAULT
#define LOG_LEVEL_DEFAULT 2
#endif

/** Default log mask. */
#ifndef LOG_MSK_DEFAULT
#define LOG_MSK_DEFAULT LOG_GROUP_STACK
#endif

/** Enable logging with RTT callback. */
#ifndef LOG_ENABLE_RTT
#define LOG_ENABLE_RTT 1
#endif

/** The default callback function to use. */
#ifndef LOG_CALLBACK_DEFAULT
#if defined(NRF51) || defined(NRF52)
    #define LOG_CALLBACK_DEFAULT log_callback_rtt
#else
    #define LOG_CALLBACK_DEFAULT log_callback_stdout
#endif
#endif

/** @} end of MESH_CONFIG_LOG */

/**
 * @defgroup MESH_CONFIG_MSG_CACHE Message cache configuration
 * @{
 */

/** Number of entries in cache.  */
#ifndef MSG_CACHE_ENTRY_COUNT
#define MSG_CACHE_ENTRY_COUNT 32
#endif

/** @} end of MESH_CONFIG_MSG_CACHE */

/**
 * @defgroup MESH_CONFIG_NETWORK Network configuration
 * @{
 */

/**
 * Set to 1 to enable the message cache.
 * The message cache prevents relaying of messages that have already been relayed
 * or received by the node.
 *
 * @warning The message cache is required to have a Bluetooth Mesh compatible mesh.
 */
#ifndef NETWORK_CACHE_ENABLE
#define NETWORK_CACHE_ENABLE 1
#endif

/** Which bearer to use when transmitting packets. */
#ifndef NETWORK_BEARER
#define NETWORK_BEARER BEARER_ADV_RADIO
#endif

/**
 * The sequence number value that triggers the start of an IV update procedure.
 * This value should be set so that there are enough sequence numbers left for running the IV update procedure.
 */
#ifndef NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD (NETWORK_SEQNUM_MAX / 2)
#endif

/**
 * The sequence number value that triggers the end of an IV update procedure.
 * This value should be set so that there are enough sequence numbers left for finishing any ongoing Transport SAR sessions.
 */
#ifndef NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD (NETWORK_SEQNUM_MAX - 1000)
#endif

/* Sanity check for NETWORK_SEQNUM_THRESHOLD */
#if NETWORK_SEQNUM_THRESHOLD > NETWORK_SEQNUM_MAX
#error "The network sequence number IV update threshold cannot be larger than the maximum sequence number!"
#endif

/**
 * The minimum time between IV updates, in minutes.
 * @warning The value of this constant must equate to 96 hours for the stack to be compatible with
 * the Bluetooth Mesh specification.
 */
#ifndef NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES
#define NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES (96 * 60)
#endif

/**
 * Limit for how much larger IV indices in incoming beacons can be than the current node before recovery is impossible.
 * @warning The value of this constant must be 42 for the stack to be compatible with the Bluetooth Mesh specification.
 */
#ifndef NETWORK_IV_RECOVERY_LIMIT
#define NETWORK_IV_RECOVERY_LIMIT 42
#endif

/**
 * Enable or disable the IV index recovery limit.
 * @warning This must be enabled for the stack to be compatible with the Bluetooth Mesh specification.
 */
#ifndef NETWORK_IV_RECOVERY_LIMIT_ENABLE
#define NETWORK_IV_RECOVERY_LIMIT_ENABLE 1
#endif

/**
 * Number of sequence numbers between every write to flash. In case of power failure, the device
 * will resume transmissions with the first sequence number in the next block. A larger block size
 * means that the device can do fewer resets between every IV Update, while a smaller will result
 * in a reduced lifetime for the flash hardware.
 */
#ifndef NETWORK_SEQNUM_FLASH_BLOCK_SIZE
#define NETWORK_SEQNUM_FLASH_BLOCK_SIZE 8192
#endif

/**
 * Number of sequence numbers left before allocating the next block. Allocating a new block can
 * take at least 200ms, and the device would be blocked from sending new messages if it runs out.
 */
#ifndef NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD
#define NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD 64
#endif

/**
 * Number of flash pages reserved for the network flash area.
 */
#ifndef NET_FLASH_PAGE_COUNT
#define NET_FLASH_PAGE_COUNT 1
#endif

/** @} end of MESH_CONFIG_NETWORK */

/**
 * @defgroup MESH_CONFIG_PACMAN Packet manager configuration
 * @{
 */

/**
 * Set to 1 to enable debug mode for the packet manager.
 *
 * @warning In debug mode, the packet manager will print out a lot of
 * information and this _will_ cause the system to assert due to timeslot
 * violation.
 */
#ifndef PACKET_MGR_DEBUG_MODE
#define PACKET_MGR_DEBUG_MODE 0
#endif


/**
 * Size of the packet manager memory pool in bytes.
 * @warning The value of the memory pool cannot currently exceed the value of
 * PACKET_MGR_PACKET_MAXLEN, due to the current design of the memory manager.
 */
#ifndef PACKET_MGR_MEMORY_POOL_SIZE
#define PACKET_MGR_MEMORY_POOL_SIZE 4096
#endif

/**
 * Packet manager blame mode.
 *
 * The blame mode adds an additional field to the packet header which records
 * the last location a packet manager function was called from. This can be
 * useful in some debugging scenarioes, such as when a bug is suspected of being
 * caused by a reference counting error. It is not intended for regular use, so
 * do not enable unless you are debugging the packet manager usage.
 */
#ifndef PACKET_MGR_BLAME_MODE
#define PACKET_MGR_BLAME_MODE 0
#endif

/** @} end of MESH_CONFIG_PACMAN */

/**
 * @defgroup MESH_CONFIG_PACKET_BUFFER Packet buffer configuration
 * @{
 */

/**
 * Set to 1 to enable debug mode for the packet buffer.
 *
 * @warning In debug mode, the packet buffer will print out a lot
 * of information and this _will_ cause the system to assert due to
 * timeslot violation.
 */
#ifndef PACKET_BUFFER_DEBUG_MODE
#define PACKET_BUFFER_DEBUG_MODE 0
#endif

/** @} end of MESH_CONFIG_PACKET_BUFFER */

/**
 * @defgroup MESH_CONFIG_RADIO Radio configuration
 * @{
 */

/** Enable debugging in radio module. */
#ifndef RADIO_DEBUG_MODE
#define RADIO_DEBUG_MODE 0
#endif

/** @} end of MESH_CONFIG_RADIO */

/**
 * @defgroup MESH_CONFIG_REPLAY_CACHE Replay cache configuration.
 * @{
 */

/**
 * Number of entries in the replay protection cache.
 *
 * @note Note that the number of entries in the replay cache directly limits the
 * number of peer nodes one can receive messages from for the current IV index.
 */
#ifndef REPLAY_CACHE_ENTRIES
#define REPLAY_CACHE_ENTRIES 32
#endif

/** @} end of MESH_CONFIG_REPLAY_CACHE */

/**
 * @defgroup MESH_CONFIG_FLASH_MANAGER Flash manager configuration defines
 * @{
 */

/** Maximum number of pages that can be owned by a single flash manager */
#ifndef FLASH_MANAGER_PAGE_COUNT_MAX
#define FLASH_MANAGER_PAGE_COUNT_MAX 255
#endif

/** Size of the flash manager data pool, storing pending writes. */
#ifndef FLASH_MANAGER_POOL_SIZE
#define FLASH_MANAGER_POOL_SIZE 256
#endif

/** Maximum size of a single flash entry in bytes. */
#ifndef FLASH_MANAGER_ENTRY_MAX_SIZE
#define FLASH_MANAGER_ENTRY_MAX_SIZE 128
#endif

/** @} end of MESH_CONFIG_FLASH_MANAGER */

/** @} end of NRF_MESH_CONFIG_CORE */

#endif
