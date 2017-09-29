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

#ifndef NRF_MESH_CONFIG_APP_H__
#define NRF_MESH_CONFIG_APP_H__

/**
 * @defgroup NRF_MESH_CONFIG_APP nRF Mesh app config
 *
 * Application side configuration file. Should be copied into every
 * application, and customized to fit its requirements.
 * @{
 */

/**
 * @defgroup DEVICE_CONFIG Device configuration
 *
 * @{
 */

/** Device company identifier. */
#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)

/** Device product identifier*/
#define DEVICE_PRODUCT_ID (0x0000)

/** Device vendor identifier */
#define DEVICE_VENDOR_ID (0x0000)

/** Supported features of the device. @see config_feature_bit_t */
#define DEVICE_FEATURES (CONFIG_FEATURE_RELAY_BIT)

/** @} end of DEVICE_CONFIG */

/**
 * @defgroup ACCESS_CONFIG Access layer configuration
 * @{
 */

/**
 * The default TTL value for the node.
 */
#define ACCESS_DEFAULT_TTL (3)

/**
 * The number of models in the application.
 *
 * @note This value has to be greater than one to fit the configuration model plus the number of
 * models needed by the application.
 */
#define ACCESS_MODEL_COUNT (5)

/**
 * The number of elements in the application.
 *
 * @warning If the application is to support multiple _instances_ of the _same_ model, they cannot
 * belong in the same element and a separate element is needed for the new instance.
 */
#define ACCESS_ELEMENT_COUNT (4)

/**
 * The number of allocated subscription lists for the application.
 *
 * @note The application should set this number to @ref ACCESS_MODEL_COUNT minus the number of
 * models operating on shared states.
 */
#define ACCESS_SUBSCRIPTION_LIST_COUNT (ACCESS_MODEL_COUNT)

/**
 * The number of pages of flash storage reserved for the access layer for persistent data storage.
 */
#define ACCESS_FLASH_PAGE_COUNT (1)

/**
 * @defgroup ACCESS_RELIABLE_CONFIG Access reliable transfer configuration
 * @{
 */

/** Number of allowed parallel transfers (size of internal context pool). */
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)

/** @} end of ACCESS_RELIABLE_CONFIG */


/** @} end of ACCESS_CONFIG */


/**
 * @defgroup DSM_CONFIG Device State Manager configuration
 * Sizes for the internal storage of the Device State Manager.
 * @{
 */
/** Maximum number of subnetworks. */
#define DSM_SUBNET_MAX                                  (1)
/** Maximum number of applications */
#define DSM_APP_MAX                                     (1)
/** Maximum number of device keys */
#define DSM_DEVICE_MAX                                  (4)
/** Maximum number of concurrent key refresh keys. */
#define DSM_KEY_REFRESH_KEYS_MAX                        (1)
/** Maximum number of virtual addresses. */
#define DSM_VIRTUAL_ADDR_MAX                            (1)
/** Maximum number of non-virtual addresses. One for each of the servers. */
#define DSM_NONVIRTUAL_ADDR_MAX                         (DSM_DEVICE_MAX)
/** Number of flash pages reserved for the DSM storage */
#define DSM_FLASH_PAGE_COUNT                            (1)
/** @} end of DSM_CONFIG */


/** @} */

#endif /* NRF_MESH_CONFIG_APP_H__ */
