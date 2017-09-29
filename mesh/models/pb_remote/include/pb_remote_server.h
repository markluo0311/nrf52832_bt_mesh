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

#ifndef PB_REMOTE_SERVER_H__
#define PB_REMOTE_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "access_reliable.h"
#include "provisioning.h"

/**
 * @defgroup PB_REMOTE_SERVER Remote Provisioning Server
 * @ingroup  PB_REMOTE
 * This module implements the remote provisioning server (PB-remote server) model.
 *
 * @mscfile pb_remote_server_init.msc Initializing the Remote Provisioning Server
 * @dotfile pb_remote_server.dot Remote Provisioning Server state diagram
 *
 * @{
 */

/** Remote provisioning server model identifier. */
#define PB_REMOTE_SERVER_MODEL_ID (0x0004)

/**
 * Remote Provisioning Server states.
 */
typedef enum
{
    PB_REMOTE_SERVER_STATE_NONE,                /**< Uninitialized state. */
    PB_REMOTE_SERVER_STATE_DISABLED,            /**< Disabled state. */
    PB_REMOTE_SERVER_STATE_IDLE,                /**< Initialized and idle state. */
    PB_REMOTE_SERVER_STATE_SCANNING,            /**< Scanning for unprovisioned devices. */
    PB_REMOTE_SERVER_STATE_SCANNING_FILTER,     /**< Scanning with a UUID filter. */
    PB_REMOTE_SERVER_STATE_LINK_OPENING,        /**< Waiting for local link to open. */
    PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED,    /**< Local link established. */
    PB_REMOTE_SERVER_STATE_WAIT_FOR_LOCAL_ACK,  /**< Waiting for local acknowledgment. */
    PB_REMOTE_SERVER_STATE_WAIT_FOR_REMOTE_ACK, /**< Waiting for remote acknowledgment. */
    PB_REMOTE_SERVER_STATE_WAIT_LINK_CLOSING    /**< Waiting for a closing link. */
} pb_remote_server_state_t;

/**
 * Remote provisioning server context structure.
 * @note This struct is intended to be stored statically by the user. All fields are internal.
 * @todo This internal structure should be stored internally in the module.
 */
typedef struct
{
    /** Access layer model handle. */
    access_model_handle_t model_handle;
    /** Remote provisioning server current state. */
    pb_remote_server_state_t state;
    /** Remote provisioning server previous state. */
    pb_remote_server_state_t prev_state;
    /** Provisioning bearer context structure. */
    prov_bearer_t prov_bearer;
    /** Access layer reliable message parameter structure. @todo Keep this state in a better way. */
    access_reliable_t reliable;
    /** If set true, the server automatically returns to scanning mode after provisioning. */
    bool return_to_scan_enabled;
    /** Current transaction ID. */
    uint8_t ctid;
} pb_remote_server_t;

/**
 * Initializes the remote provisioning server model.
 *
 * @note This function is expected to be called once for a remote server.
 *
 * @param[in,out] p_server        Remote server context structure pointer.
 * @param[in]     element_index   Element index to bind the remote server model.
 *
 * @retval NRF_SUCCESS             Successfully initialized the model.
 * @retval NRF_ERROR_NULL          NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM        No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN     Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND     Invalid element index.
 * @retval NRF_ERROR_INVALID_STATE Model was already initialized.
 */
uint32_t pb_remote_server_init(pb_remote_server_t * p_server, uint16_t element_index);

/**
 * Enables the remote provisioning server.
 *
 * @param[in,out] p_server Remote server context pointer.
 *
 * @retval NRF_SUCCESS             Successfull enabled remote server.
 * @retval NRF_ERROR_NULL          NULL pointer in function arguments.
 * @retval NRF_ERROR_INVALID_STATE A remote server may only enabled in the
 *                                 @ref PB_REMOTE_SERVER_STATE_DISABLED state.
 */
uint32_t pb_remote_server_enable(pb_remote_server_t * p_server);

/**
 * Disables the remote provisioning server.
 *
 * @param[in,out] p_server Remote server context pointer.
 *
 * @retval NRF_SUCCESS             Successfully disabled remote server.
 * @retval NRF_ERROR_NULL          NULL pointer in function arguments.
 * @retval NRF_ERROR_INVALID_STATE A remote server may only disabled in the
 *                                 @ref PB_REMOTE_SERVER_STATE_IDLE,
 *                                 @ref PB_REMOTE_SERVER_STATE_SCANNING or
 *                                 @ref PB_REMOTE_SERVER_STATE_SCANNING_FILTER states.
 */
uint32_t pb_remote_server_disable(pb_remote_server_t * p_server);

/**
 * Sets the return to scan state.
 *
 * @param[in,out] p_server Remote server context pointer.
 * @param[in]     state    Enable or disable the return to scanning mode.
 *
 * @retval NRF_SUCCESS    Successfully set return to scan state.
 * @retval NRF_ERROR_NULL NULL pointer in function arguments.
 */
uint32_t pb_remote_server_return_to_scan_set(pb_remote_server_t * p_server, bool state);

/** @} end of PB_REMOTE_SERVER */

#endif /* PB_REMOTE_SERVER_H_ */

