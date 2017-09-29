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

#ifndef NRF_MESH_PROV_H__
#define NRF_MESH_PROV_H__

#include <stdbool.h>
#include <stdint.h>
#include <ble.h>

#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_config_core.h"

/**
 * @defgroup NRF_MESH_PROV Provisioning API
 * @ingroup MESH_API_GROUP_PROV
 * Functionality for supporting provisioning of a node.
 * @{
 */

/** Provisioning context structure size in bytes */
#define NRF_MESH_PROVISIONING_CTX_SIZE 704 /* Keeping this at host-compile size (max 704 as of 18.01.2017) */
/** Size of the elliptic curve public key. */
#define NRF_MESH_PROV_PUBKEY_SIZE      64
/** Size of the elliptic curve private key. */
#define NRF_MESH_PROV_PRIVKEY_SIZE     32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_ECDHSECRET_SIZE  32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_DATANONCE_SIZE  13

/** Max OOB size permitted by the specification. */
#define NRF_MESH_PROV_OOB_SIZE_MAX    8

/**
 * @defgroup NRF_MESH_PROV_CAPABILITIES Provisioning capabilities bit fields
 *
 * Bitfield definitions for the provisioning capabilities fields.
 * See Section 5.4.1.2 Provisioning Capabilities in the mech core spec.
 * @{
 */
/** Capabilities bit indicating that the FIPS P256EC algorithm is supported. */
#define NRF_MESH_PROV_ALGORITHM_FIPS_P256EC         (1 << 0)
/** Capabilities bit indicating that the public key is available OOB. */
#define NRF_MESH_PROV_OOB_PUBKEY_TYPE_OOB           (1 << 0)
/** Capabilities bit indicating that static OOB authentication is supported. */
#define NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED     (1 << 0)
/** Capabilities bit indicating that the public key is available in-band. If no public key type is set, this is the default */
#define NRF_MESH_PROV_OOB_PUBKEY_TYPE_INBAND        (0)
/** Capabilities bit indicating that the device supports blinking as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_BLINK       (1 << 0)
/** Capabilities bit indicating that the device supports beeping as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_BEEP        (1 << 1)
/** Capabilities bit indicating that the device supports vibrating as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_VIBRATE     (1 << 2)
/** Capabiliities bit indicating that the device supports displaying numberic data as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC     (1 << 3)
/** Capabilities bit indicating that the device supports displaying alphanumeric data as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_ALPHANUMERIC (1 << 4)
/** Capabilities bit indicating that the device supports pushing something as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_PUSH         (1 << 0)
/** Capabilities bit indicating that the device supports twisting something as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_TWIST        (1 << 1)
/** Capabilities bit indicating that the device supports entering a number as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER (1 << 2)
/** Capabilities bit indicating that the device supports entering a string as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_STRING (1 << 3)
/** @} */

/**
 * @defgroup NRF_MESH_PROV_OOB_INFO_SOURCES Provisioning OOB information sources.
 *
 * OOB information sources for the OOB info bitfield in unprovisioned beacons.
 * Denotes a set of sources from where the user can get information on
 * available OOB data for the product, to aid the provisioning process.
 * @{
 */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_OTHER                    (1 << 0)   /**< Other location. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ELECTRONIC_OR_URI        (1 << 1)   /**< Electronic / URI. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_2D_MACHINE_READABLE_CODE (1 << 2)   /**< 2D machine-readable code. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_BAR_CODE                 (1 << 3)   /**< Bar code. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_NFC                      (1 << 4)   /**< Near Field Communication (NFC). */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_NUMBER                   (1 << 5)   /**< Number. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_STRING                   (1 << 6)   /**< String. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_BOX                   (1 << 11)  /**< On box. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_BOX               (1 << 12)  /**< Inside box. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_PIECE_OF_PAPER        (1 << 13)  /**< On piece of paper. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL            (1 << 14)  /**< Inside manual. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_DEVICE                (1 << 15)  /**< On device. */
/** @} */

/** Provisioning role */
typedef enum
{
    NRF_MESH_PROV_ROLE_PROVISIONER, /**< The device will act as a provisioner, distributing provisioning data. */
    NRF_MESH_PROV_ROLE_PROVISIONEE  /**< The device will act as a provisionee, receiving provisioning data. */
} nrf_mesh_prov_role_t;

/*lint -align_max(push) -align_max(1) */

/**
 * Provisioning authentication capabilities.
 * This structure is filled in with the preferred values for how authentication is to be performed.
 * Only one method can be chosen for each of input/output authentication.
 */
typedef struct __attribute((packed))
{
    uint8_t  num_elements;       /**< Number of elements in the device. */
    uint16_t algorithms;         /**< Supported authentication algorithms. */
    uint8_t  pubkey_type;        /**< Supported public key types. */
    uint8_t  oob_static_types;   /**< Supported static OOB types. */
    uint8_t  oob_output_size;    /**< Output OOB data size. */
    uint16_t oob_output_actions; /**< Supported output OOB actions. */
    uint8_t  oob_input_size;     /**< Input OOB data size. */
    uint16_t oob_input_actions;  /**< Supported input OOB actions. */
} nrf_mesh_prov_oob_caps_t;

/**
 * Out-of-band authentication methods for provisioning.
 */
typedef enum
{
    NRF_MESH_PROV_OOB_METHOD_NONE   = 0x00, /**< No authentication method. */
    NRF_MESH_PROV_OOB_METHOD_STATIC = 0x01, /**< Static OOB authentication method. */
    NRF_MESH_PROV_OOB_METHOD_OUTPUT = 0x02, /**< Output OOB authentication method. */
    NRF_MESH_PROV_OOB_METHOD_INPUT  = 0x03, /**< Input OOB authentication method. */
} nrf_mesh_prov_oob_method_t;

/**
 * Enumeration for the OOB input actions.
 */
typedef enum
{
    NRF_MESH_PROV_INPUT_ACTION_NONE         = 0x00, /**< No input action specified. */
    NRF_MESH_PROV_INPUT_ACTION_PUSH         = 0x01, /**< The user should do a push action as input action. */
    NRF_MESH_PROV_INPUT_ACTION_TWIST        = 0x02, /**< The user should do a twist action as input action. */
    NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER = 0x03, /**< The user should enter a number into the device as input action. */
    NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING = 0x04, /**< The user should enter a string into the device as input action. */
} nrf_mesh_prov_input_action_t;

/**
 * Enumeration for the OOB output actions.
 */
typedef enum
{
    NRF_MESH_PROV_OUTPUT_ACTION_NONE            = 0x00, /**< No output action specified. */
    NRF_MESH_PROV_OUTPUT_ACTION_BLINK           = 0x01, /**< The device should use blinking as output action. */
    NRF_MESH_PROV_OUTPUT_ACTION_BEEP            = 0x02, /**< The device should use beeping as output action. */
    NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE         = 0x03, /**< The device should vibrate as output action. */
    NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC = 0x04, /**< The device should display a number as output action. */
    NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC    = 0x05, /**< The device should display an alpha-numberic value as output action. */
} nrf_mesh_prov_output_action_t;

/**
 * Reason for why a provisioning link was closed.
 */
typedef enum
{
    NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS = 0,    /**< The link was closed because provisioning was completed successfully. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT = 1,    /**< The link timed out and was closed. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR   = 2,   /**< The link was closed beacuse of an error. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_LAST = NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR /**< The highest known link close reason, values higher are unrecognized */
} nrf_mesh_prov_link_close_reason_t;

/**
 * Provisioning failure codes.
 */
typedef enum
{
    NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU         = 0x01, /**< An invalid provisioning PDU was received. */
    NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT      = 0x02, /**< An incoming provisioning packet had an invalid format. */
    NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU      = 0x03, /**< The incoming packet was a different packet type than what was expected. */
    NRF_MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED = 0x04, /**< The OOB authentication between provisioner and provisionee failed. */
    NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES    = 0x05, /**< The device does not have enough resources (memory, CPU time) to complete the provisioning. */
    NRF_MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED   = 0x06, /**< The provisioning data could not be decrypted. */
    NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_ERROR    = 0x07, /**< An unexpected error occured. */
    NRF_MESH_PROV_FAILURE_CODE_CANNOT_ASSIGN_ADDR  = 0x08, /**< Consecutive unicast addresses could not be assigned. */
} nrf_mesh_prov_failure_code_t;

/**
 * Provisioning bearer types.
 */
typedef enum
{
    NRF_MESH_PROV_BEARER_ADV,         /**< Advertising-based provisioning bearer, PB-ADV. */
    NRF_MESH_PROV_BEARER_GATT,        /**< GATT-based provisioning bearer, PB-GATT. */
    NRF_MESH_PROV_BEARER_MESH_CLIENT /**< PB-Mesh client bearer. Uses PB-Mesh for communicating with a target device, and reports incoming traffic to the provisioner state machine. */
} nrf_mesh_prov_bearer_type_t;

/**
 * Provisioning data to transmit to a device.
 */
typedef struct __attribute((packed))
{
    /** Network key for the device. */
    uint8_t  netkey[NRF_MESH_KEY_SIZE];
    /** Network key index. */
    uint16_t netkey_index;
    /** IV_index value for the device, in little endian format. */
    uint32_t iv_index;
    /** Unicast address for the device. */
    uint16_t address;
    /** Flags. */
    struct __attribute((packed)) {
        /** IV update in progress flag. */
        uint8_t iv_update   : 1;
        /** Key refresh in progress flag. */
        uint8_t key_refresh : 1;
    } flags;
} nrf_mesh_prov_provisioning_data_t;

/*lint -align_max(pop) */

/**
 * Provisioning state machine states.
 * These are used internally in the provisioning stack to run the provisioning protocol.
 */
typedef enum
{
    NRF_MESH_PROV_STATE_IDLE,                   /**< Idle state, no action is taken. */
    NRF_MESH_PROV_STATE_WAIT_ACK,               /**< Waiting for an ACK to a previously sent packet. */
    NRF_MESH_PROV_STATE_WAIT_LINK,              /**< Waiting for a link to be established. */
    NRF_MESH_PROV_STATE_INVITE,                 /**< Waiting for a provisioning invitation message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS,              /**< Waiting for a provisioning capabilities message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM,      /**< Waiting for the application to confirm the capabilities to use. */
    NRF_MESH_PROV_STATE_WAIT_START,             /**< Waiting for a provisioning start message. */
    NRF_MESH_PROV_STATE_WAIT_START_ACK,         /**< Waiting for the message acknowledgement for the start message. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY_ACK,       /**< Waiting for the public key to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY,           /**< Waiting for the peer node to send its public key. */
    NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY,       /**< Waiting for the OOB public key to be retrieved. */
    NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH,     /**< Waiting for the offloaded ECDH calculation to complete. */
    NRF_MESH_PROV_STATE_WAIT_OOB_INPUT,         /**< Waiting for OOB input. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC,        /**< Waiting for static OOB data. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD, /**< Waiting for static OOB data, confirmation already received. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION_ACK,  /**< Waiting for the confirmation to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION,      /**< Waiting for a provisioning confirmation message. */
    NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE,    /**< Waiting for an input complete message. */
    NRF_MESH_PROV_STATE_WAIT_RANDOM,            /**< Waiting for a provisioning random message. */
    NRF_MESH_PROV_STATE_WAIT_DATA,              /**< Waiting for the provisioning data message. */
    NRF_MESH_PROV_STATE_WAIT_COMPLETE,          /**< Waiting for the provisioning complete message. */
    NRF_MESH_PROV_STATE_COMPLETE,               /**< Provisioning complete state. */
    NRF_MESH_PROV_STATE_FAILED,                 /**< Provisioning failed state. */
} nrf_mesh_prov_state_t;

/**
 * Struct used for allocating the amount of memory needed by the provisioning module on the
 * application space.
 */
typedef struct __attribute((aligned(4)))
{
    /** Contents are only for the provisioning module and should not be altered by the application. */
    uint8_t contents[NRF_MESH_PROVISIONING_CTX_SIZE];
}nrf_mesh_prov_ctx_t;

/**
 * Initializes the provisioning context structure.
 *
 * @warning If calling this function the first time, it is required that the @c p_ctx is zero initialized.
 *          Any further calls require that @c p_ctx is left untouched.
 *
 * @param[in, out]  p_ctx       Pointer to the provisioning context structure to initialize.
 * @param[in]  p_public_key     Pointer to the node's public key. The public key is 64 bytes long.
 * @param[in]  p_private_key    Pointer to the node's private key. The private key is 32 bytes long.
 * @param[in]  p_caps           Pointer to a structure containing the node's out-of-band authentication capabilities.
 *
 * @retval NRF_SUCCESS The library was successfully initialized.
 */
uint32_t nrf_mesh_prov_init(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_public_key, const uint8_t * p_private_key,
        const nrf_mesh_prov_oob_caps_t * p_caps);

/**
 * Listens for an incoming provisioning link.
 *
 * @param[in, out] p_ctx       Pointer to a statically allocated provisioning context structure.
 * @param[in] bearer_type      The bearer type to listen for a provisioning link on.
 * @param[in] URI              Optional device URI string used as identifier in some other context. May be NULL.
 * @param[in] oob_info_sources Known OOB information sources, see @ref NRF_MESH_PROV_OOB_INFO_SOURCES.
 *
 * @retval NRF_SUCCESS The provisioning bearer was successfully put into listening mode.
 */
uint32_t nrf_mesh_prov_listen(nrf_mesh_prov_ctx_t * p_ctx,
        nrf_mesh_prov_bearer_type_t bearer_type,
        const char * URI,
        uint16_t oob_info_sources);

/**
 * Generates a valid keypair for use with the provisioning cryptography.
 *
 * @param[out] p_public  Pointer to where the generated public key is stored.
 * @param[out] p_private Pointer to where the generated private key is stored.
 *
 * @retval NRF_SUCCESS The keypair was successfully generated.
 */
uint32_t nrf_mesh_prov_generate_keys(uint8_t * p_public, uint8_t * p_private);

/**
 * Provisions a device.
 *
 * @param[in,out] p_ctx         Pointer to a statically allocated provisioning context structure.
 * @param[in]     p_target_uuid Device UUID of the device that is to be provisioned.
 * @param[in]     p_data        Pointer to a structure containing the provisioning data for the device.
 * @param[in]     bearer        Which bearer to establish the provisioning link on.
 *
 * @retval NRF_SUCCESS The provisioning process was started.
 * @retval NRF_ERROR_NULL One or more parameters were NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer type is not supported in the current configuration.
 * @retval NRF_ERROR_INVALID_DATA The provisioning data failed some boundary conditions.
 * @retval NRF_ERROR_INVALID_STATE The given context is in use.
 */
uint32_t nrf_mesh_prov_provision(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_target_uuid,
        const nrf_mesh_prov_provisioning_data_t * p_data, nrf_mesh_prov_bearer_type_t bearer);

/**
 * Selects which out-of-band authentication method to use.
 *
 * This function is used in response to the reception of a @ref NRF_MESH_EVT_PROV_CAPS_RECEIVED event.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] method    Specifies the authentication method to use. The action that must be taken for the
 *                      specified method is dependent on the provisionee device, and can be read from the
 *                      @c NRF_MESH_EVT_PROV_CAPS_RECEIVED event.
 * @param[in] size      Size of the out-of-band authentication data. Must be between 1 and 8 inclusive or
 *                      0 when @c NRF_MESH_PROV_OOB_METHOD_NONE is used.
 *
 * @retval NRF_SUCCESS              The out-of-band method was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_LENGTH The size of the authentication data was invalid.
 */
uint32_t nrf_mesh_prov_oob_use(nrf_mesh_prov_ctx_t * p_ctx, nrf_mesh_prov_oob_method_t method, uint8_t size);

/**
 * Provides out-of-band authentication data input to the provisioning stack.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_data    Pointer to an array of authentication data. The size of this array should match the size of the data
 *                      requested in the request event for @ref NRF_MESH_EVT_PROV_INPUT_REQUEST, or be 16 bytes for a
 *                      @ref NRF_MESH_EVT_PROV_STATIC_REQUEST event. The maximum size of the data is 16 bytes.
 * @param[in] size      Size of the array provided in @c p_data.
 *
 * @retval NRF_SUCCESS              The authentication data was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE  Authentication data was provided even though it was not requested by
 *                                  the current provisioning context.
 * @retval NRF_ERROR_INVALID_LENGTH The size of the authentication data was invalid.
 */
uint32_t nrf_mesh_prov_auth_data_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size);

/**
 * Provides the shared secret to the provisioning stack after running a requested ECDH calculation.
 * This function is used only if ECDH offloading is enabled in the options API.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_shared  Pointer to the shared secret calculated by the external ECDH code.
 *
 * @retval NRF_SUCCESS             The shared secret was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE A shared secret was not requested by the current provisioning context.
 * @retval NRF_ERROR_NULL          The pointer provided to the shared secret was NULL.
 */
uint32_t nrf_mesh_prov_shared_secret_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);

/**
 * Provides a public key to the provisioner if the provisionee has exposed it out-of-band.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_key     Pointer to the start of an array containing the public key for the provisionee.
 *
 * @retval NRF_SUCCESS             The public key was valid and accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE The public key was provided even tough it was not requested by the
 *                                 specified provisioning context.
 * @retval NRF_ERROR_INVALID_PARAM The public key was invalid.
 * @retval NRF_ERROR_NULL          The @c p_key argument was NULL.
 */
uint32_t nrf_mesh_prov_pubkey_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key);

/** @} */

#endif

