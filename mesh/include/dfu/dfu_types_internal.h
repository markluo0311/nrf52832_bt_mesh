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
#ifndef DFU_TYPES_INTERNAL_H__
#define DFU_TYPES_INTERNAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh_dfu_types.h"
#include "timer_scheduler.h"
#include "packet.h"
#include "packet_mgr.h"
#include "cache.h"
#include "bl_if.h"

typedef struct bl_evt bl_evt_t; /**< Forward declaration of @c bl_evt_t from bl_if.h. */
typedef struct bl_cmd bl_cmd_t; /**< Forward declaration of @c bl_cmd_t from bl_if.h. */

/**
 * @defgroup DFU_TYPES_INTERNAL Internal DFU defines and functions
 * @ingroup MESH_API_GROUP_DFU
 * @{
 */

/**
 * @defgroup GPREGRET_CODES Retention register values for the bootloader
 * @{
 */

/** Normal retention value, no defined reason for rebooting. */
#define BL_GPREGRET_NORMAL          (0)
/** Forced reboot retention value, The application decided to reboot on purpose. */
#define BL_GPREGRET_FORCED_REBOOT   (1)

/** @} */

/** Advertisement payload overhead */
#define DFU_PACKET_PAYLOAD_OVERHEAD (1 /* AD type */ + 2 /* service UUID */)
/** Total packet overhead. */
#define DFU_PACKET_OVERHEAD         (BLE_ADV_PACKET_OVERHEAD + 1 /* AD length */ + DFU_PACKET_PAYLOAD_OVERHEAD) /**< DFU packet total overhead. */

/** Length of the APP firmware ID */
#define DFU_FWID_LEN_APP            (10)
/** Length of the BL firmware ID */
#define DFU_FWID_LEN_BL             (2)
/** Length of the SD firmware ID */
#define DFU_FWID_LEN_SD             (2)

/** First OpenMesh handle considered a DFU packet. */
#define DFU_HANDLE_RANGE_START      (0xFFF9)

/**
 * @defgroup DFU_PACKET_LENGTH Retention register values for the bootloader
 * @{
 */

/** FWID packet packet length */
#define DFU_PACKET_LEN_FWID         (2 + 2 + 2 + 4 + 2 + 4)
/** READY packet with Softdevice packet length */
#define DFU_PACKET_LEN_READY_SD     (2 + 1 + 1 + 4 + DFU_FWID_LEN_SD)
/** READY packet with bootloader packet length */
#define DFU_PACKET_LEN_READY_BL     (2 + 1 + 1 + 4 + DFU_FWID_LEN_BL)
/** READY packet with application packet length */
#define DFU_PACKET_LEN_READY_APP    (2 + 1 + 1 + 4 + DFU_FWID_LEN_APP)
/** START packet packet length */
#define DFU_PACKET_LEN_START        (2 + 2 + 4 + 4 + 4 + 2 + 1)
/** DATA packet packet length */
#define DFU_PACKET_LEN_DATA         (2 + 2 + 4 + SEGMENT_LENGTH)
/** DATA REQUEST packet packet length */
#define DFU_PACKET_LEN_DATA_REQ     (2 + 2 + 4)
/** DATA RESPONSE packet packet length */
#define DFU_PACKET_LEN_DATA_RSP     (2 + 2 + 4 + SEGMENT_LENGTH)
/** RELAY REQUEST packet packet length */
#define DFU_PACKET_LEN_RELAY_REQ    (2 + 2 + 4 + BLE_GAP_ADDR_LEN)

/** @} */

/** Size of the data request cache */
#define DFU_DATA_REQ_CACHE_SIZE             (8)
/** Size of the data response cache */
#define DFU_DATA_RSP_CACHE_SIZE             (8)

/** Gets the address of the DFU device page. */
#if defined(NRF52) || defined(NRF51)
    /** Get the device page location for the current device configuration. */
    #define DFU_DEVICE_PAGE_GET()   (bootloader_info_t*)(NRF_FICR->CODEPAGESIZE * (NRF_FICR->CODESIZE - 1))
#else
    /** Get a dummy device page location. */
    #define DFU_DEVICE_PAGE_GET()   (bootloader_info_t*) (0xB00710AD)
#endif

/**
* Send a raw command to the dfu module.
*
* @warning The application should not be using this function directly, unless
* absolutely needed. It is intended for internal use.
*
* @param[in] p_cmd A pointer to a command structure.
*
* @retval NRF_SUCCESS The command was successfully sent and executed.
* @retval NRF_ERROR_NOT_SUPPORTED The dfu functionality is not available.
* @retval NRF_ERROR_* The given command did not succeed. The meaning of each
* error code depends on the command.
*/
uint32_t nrf_mesh_dfu_cmd_send(bl_cmd_t* p_cmd);

/** @} */

#endif /* DFU_TYPES_INTERNAL_H__ */
