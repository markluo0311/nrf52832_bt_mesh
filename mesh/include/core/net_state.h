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

#ifndef NET_STATE_H__
#define NET_STATE_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"

/**
 * @defgroup NET_STATE Network State Module
 * @ingroup MESH_CORE
 * Manages the device's network state.
 * Internal network layer file.
 * @{
 */

/** Mask for the IVI field of the network packet. */
#define NETWORK_IVI_MASK     (0x00000001)

/**
 * Initializes the net state module.
 */
void net_state_init(void);

/**
 * Recover network state from flash storage.
 *
 * @warning The user should not call this function while the flash manager is unstable.
 */
void net_state_recover_from_flash(void);

/**
 * Get a pointer to the net state flash area.
 *
 * @returns A pointer to the net state flash area.
 */
const void * net_state_flash_area_get(void);

/**
 * Reset the sequence number and IV index, and wipe the flashed state.
 */
void net_state_reset(void);

/**
 * Allocates a new network sequence number.
 *
 * @param[out]     p_seqnum    Pointer to an integer where the allocated sequence number is written.
 *
 * @retval NRF_SUCCESS         A sequence number was successfully allocated.
 * @retval NRF_ERROR_FORBIDDEN The sequence number has wrapped and the IV index has not yet
 *                             been updated. It would be a security issue to re-use old sequence
 *                             numbers without increasing the IV index, so a new sequence number
 *                             could not be allocated.
 */
uint32_t net_state_seqnum_alloc(uint32_t * p_seqnum);

/**
 * Sets the IV Update test mode.
 * @note Mesh core spec Section 3.10.5.1 details how IV Update Test Mode works.
 *
 * @param[in] test_mode_on Turns on the test mode (true/false).
 */
void net_state_iv_update_test_mode_set(bool test_mode_on);

/**
 * Initiates an IV update procedure.
 *
 * The time between IV updates must be at least 96 hours. An IV update procedure takes at least
 * 96 hours to run.
 *
 * @retval NRF_SUCCESS             The IV update procedure was started successfully.
 * @retval NRF_ERROR_INVALID_STATE The IV update procedure is already in progress, or cannot be started yet.
 */
uint32_t net_state_iv_update_start(void);

/**
 * Locks or unlocks the IV index. While locked, the network won't initiate
 * an IV update or otherwise make changes to the network state.
 *
 * @param[in] lock Whether to lock or unlock the IV index.
 */
void net_state_iv_index_lock(bool lock);

/**
 * Callback for when an authenticated beacon is received by the net beacon
 * module. The network state module will adjust its state according to Mesh
 * specification rules.
 *
 * @param[in] iv_index    IV index of the incoming beacon.
 * @param[in] iv_update   Value of the IV update flag of the incoming beacon.
 * @param[in] key_refresh Value of the key refresh flag of the incoming beacon.
 */
void net_state_beacon_received(uint32_t iv_index, bool iv_update, bool key_refresh);


/**
 * Gets the IV index that should be beaconed with the Secure Network Beacon.
 *
 * @returns The current IV index.
 */
uint32_t net_state_beacon_iv_index_get(void);

/**
 * Gets the IV index to transmit outgoing packets on.
 *
 * @returns The IV index currently used for transmitting packets.
 */
uint32_t net_state_tx_iv_index_get(void);

/**
 * Gets the correct IV index based on the least significant bit of the specified @c
 * ivi. The device will always be able to receive on two subsequent IV indexes
 * (depending on the current IV index and the IV update state).
 *
 * @param[in] ivi The least significant bit of the IV index to get from the set of
 *                operational IV indexes. Only the LSB of the parameter is used.
 *
 * @returns The operational IV index which matches the specified ivi bit.
 */
uint32_t net_state_rx_iv_index_get(uint8_t ivi);

/**
 * Gets the current IV update state.
 * @returns The current IV update state.
 */
net_state_iv_update_t net_state_iv_update_get(void);

/** @} */

#endif /* NET_STATE_H__ */

