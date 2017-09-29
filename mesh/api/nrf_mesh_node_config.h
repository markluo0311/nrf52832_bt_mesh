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

#ifndef NRF_MESH_NODE_CONFIG_H__
#define NRF_MESH_NODE_CONFIG_H__

#include <stdint.h>
#include "nrf.h"
#include <nrf_sdm.h>

#include "nrf_mesh.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh_prov.h"

/**
 * @defgroup NRF_MESH_NODE_CONFIG Node configuration
 * This module provides functionality for configuration of mesh nodes.
 * @{
 */

/* This allows us to use the same typenames for all versions of the SoftDevice: */
#if defined(S110)
#define nrf_fault_handler_t softdevice_assertion_handler_t
#define nrf_clock_lf_cfg_t nrf_clock_lfclksrc_t
#endif

/**
 * Configuration complete callback.
 *
 * This function is called to indicate that the configuration module has successfully configured
 * the node.
 *
 * @param[in]     p_data Contextual data pointer.
 */
typedef void (*nrf_mesh_node_config_complete_cb_t)(void * p_data);

/**
 * Setup callback.
 *
 * This function is called to allow an application to initialize any models it needs before
 * configuration starts.
 *
 * @param[in] p_data Contextual data pointer.
 */
typedef void (*nrf_mesh_node_config_setup_cb_t)(void * p_data);

/**
 * Node configuration parameters.
 *
 * Some fields are optional; the description of the specific fields notes if the
 * value of a field can be omitted. In this case, the value of the field should be
 * set to 0 (@c NULL for pointer values).
 */
typedef struct
{
    /**
     * Pointer to a function used to signal the completion of the node configuration
     * procedure. Can be set to @c NULL if not used.
     */
    nrf_mesh_node_config_complete_cb_t complete_callback;

    /**
     * Pointer to a function used to allow initialization of application-specific models.
     * Any models used by the application should be initialized in this callback function,
     * which ensure that these models are available immediately after the node has been
     * provisioned. Can be set to @c NULL if not used.
     */
    nrf_mesh_node_config_setup_cb_t setup_callback;

    /**
     * Data pointer to pass to the callback functions.
     */
    void * p_data;

    /**
     * Low frequency clock configuration.
     * Used for SoftDevice initialization. See nrf_sdm.h or the SoftDevice documentation
     * for more information about this parameter.
     */
    nrf_clock_lf_cfg_t   lf_clk_cfg;

    /**
     * Handler for SoftDevice assertions.
     * This is an optional field for specifying a handler for critical errors from the
     * SoftDevice. If set to @c NULL, a default handler will be used instead.
     */
    nrf_fault_handler_t sd_assertion_handler;

    /**
     * Handler for mesh assertions.
     * This is an optional field for specifying a handler that is called on critical error
     * conditions in the mesh stack, that usually indicates a bug in either the application
     * or internally in the mesh stack. If set to @c NULL, a default handler will be used
     * instead.
     */
    nrf_mesh_assertion_handler_t mesh_assertion_handler;

    /**
     * Device capabilities for OOB authentication.
     * It is recommended to support at least the static authentication method. If no
     * authentication methods are supported, the device will fall back to using non-secure
     * provisioning, making the device vulnerable to certain attacks during provisioning.
     */
    nrf_mesh_prov_oob_caps_t prov_caps;

    /**
     * NULL-terminated device URI string.
     * This is an optional field that can be used to add additional data to the unprovisioned
     * node broadcast beacon.
     */
    const char * p_device_uri;

    /**
     * Device OOB information sources.
     * This adds information to the unprovisioned node broadcast beacon about where the user
     * of a device can find information about the available OOB authentication methods
     * supported by the device.
     */
    uint16_t oob_info_sources;

    /**
     * Data used for OOB authentication when the static authentication method is used.
     * This should be a pointer to a 16 byte long data array. This field is optional,
     * not used unless static authentication is one of the supported authentication
     * methods.
     */
    const uint8_t * p_static_data;
} nrf_mesh_node_config_params_t;

/**
 * Configures the mesh node.
 *
 * This function wraps all the functionality required to go from the boot-up state
 * to a fully functional mesh node.
 *
 * @retval NRF_ERROR_NULL          The @c p_params parameter was @c NULL.
 * @retval NRF_ERROR_INVALID_STATE The node has already been configured.
 * @retval NRF_ERROR_INVALID_PARAM One or more of the parameters in the @c p_params structure
 *                                 were invalid.
 * @retval NRF_SUCCESS             Node initialization was successfully started. However, the
 *                                 node initialization should not be considered finished until
 *                                 the configuration complete callback has been called.
 */
uint32_t nrf_mesh_node_config(const nrf_mesh_node_config_params_t * p_params);

/**
 * Clears the saved configuration and network state of the mesh node.
 * @warning This function must not be called from an interrupt.
 */
void nrf_mesh_node_config_clear(void);
/**
 * @}
 */

#endif
