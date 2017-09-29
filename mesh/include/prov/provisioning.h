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

#ifndef PROVISIONING_H__
#define PROVISIONING_H__

#include "prov_bearer.h"
#include "prov_bearer_adv.h"
#include "prov_pdu.h"

#include "nrf_mesh_prov.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"

/**
 * @defgroup MESH_PROV Provisioning components
 * @ingroup MESH_API_GROUP_PROV
 * @{
 */

/** Mesh core spec 5.4.4: "The provisioning protocol shall have a minimum timeout of 60 seconds
  * that is reset each time a provisioning protocol PDU is sent or received."
  */
#define PROV_PROVISIONING_LINK_TIMEOUT_MIN_US 60000000

/**
 * @defgroup PROV_FIELD_LENGTHS Provisioning field lengths
 * Length of various provisioning fields.
 * @{
 */

/** Length of Random value. */
#define PROV_RANDOM_LEN         (16)
/** Length of Confirmation value. */
#define PROV_CONFIRMATION_LEN   (NRF_MESH_KEY_SIZE)
/** Length of Auth value. */
#define PROV_AUTH_LEN           (NRF_MESH_KEY_SIZE)
/** Length of Salt value. */
#define PROV_SALT_LEN           (NRF_MESH_KEY_SIZE)
/** Length of Nonce. */
#define PROV_NONCE_LEN          (13)
/** Length of confirmation inputs */
#define PROV_CONFIRMATION_INPUT_LEN     ((sizeof(prov_pdu_invite_t) - 1) + (sizeof(prov_pdu_caps_t) - 1) + (sizeof(prov_pdu_prov_start_t) - 1))
/** @} */

/**
 * Provisioning bearer context.
 */
typedef struct prov_bearer_t
{
    union
    {
        prov_bearer_adv_t pb_adv;                 /**< Context for the PB-ADV bearer. */
    } bearer;                                     /**< Bearer specific context. */
    const prov_bearer_callbacks_t *  p_callbacks; /**< Provisioning module user callbacks. */
    const prov_bearer_interface_t *  p_interface; /**< Provisioning bearer interface. */
    uint32_t            timeout;                  /**< Link timeout value (in us). */
} prov_bearer_t;

/**
 * Common provisioning context.
 */
typedef struct prov_common_ctx_t
{
    prov_bearer_t bearer; /**< Provisioning bearer information. */

    const uint8_t * p_public_key;  /**< Public key of this node. */
    const uint8_t * p_private_key; /**< Private key of this node. */

    uint8_t peer_public_key[NRF_MESH_PROV_PUBKEY_SIZE];   /**< Public key of the peer node. */
    uint8_t shared_secret[NRF_MESH_PROV_ECDHSECRET_SIZE]; /**< ECDH shared secret: P-256(private key, peer public key). */

    uint8_t device_key[NRF_MESH_KEY_SIZE];  /**< Node device key. */
    uint8_t session_key[NRF_MESH_KEY_SIZE]; /**< Provisioning session key. */
    uint8_t data_nonce[PROV_NONCE_LEN];  /**< Provisioning data nonce. Only 13 bytes are used. */

    uint8_t node_random[PROV_RANDOM_LEN]; /**< Random number for the current node. */
    uint8_t peer_random[PROV_RANDOM_LEN]; /**< Random number for the peer node. */
    uint8_t auth_value[PROV_AUTH_LEN];    /**< Authentication value. */

    uint8_t confirmation_salt[PROV_SALT_LEN];   /**< Confirmation salt value. */
    uint8_t peer_confirmation[PROV_CONFIRMATION_LEN];   /**< Confirmation value for the peer node. */
    uint8_t confirmation_inputs[PROV_CONFIRMATION_INPUT_LEN]; /**< Confirmation inputs, used to calculate the confirmation key. */

    uint8_t oob_size;   /**< Size of the chosen OOB authentication data. */
    uint8_t oob_action; /**< Chosen OOB action. */
    bool pubkey_oob;    /**< Uses out-of-band public key. */

    nrf_mesh_prov_role_t role;                 /**< Provisioning role, provisioner or provisionee. */
    nrf_mesh_prov_failure_code_t failure_code; /**< Error code sent with the previous provisioning failed packet. */
    nrf_mesh_prov_state_t state;               /**< Provisioning state machine state. */
    nrf_mesh_prov_oob_method_t oob_method;     /**< Chosen OOB authentication method. */
    nrf_mesh_prov_oob_caps_t capabilities;     /**< Node OOB and authentication capabilities. */
    nrf_mesh_prov_provisioning_data_t data;    /**< Provisioning data to send to the provisionee or received from the provisioner. */

    struct prov_common_ctx_t * p_next;  /**< Pointer to the next provisioner context structure, used for parallel provisioning. */
} prov_common_ctx_t;

/****************** Member helper functions ******************/
/**
 * @defgroup PROVISIONING_MEMBER_HELPER Provisioning structure member helper functions
 * @{
 */

/**
 * Get the prov bearer object holding the given prov_bearer_adv_t.
 *
 * @param[in] p_bearer_adv Pointer to a PB-ADV bearer instance contained in a
 * prov_bearer structure.
 *
 * @returns A pointer to the PB-ADV structure's prov_bearer_t parent.
 */
static inline prov_bearer_t * prov_bearer_adv_parent_get(prov_bearer_adv_t * p_bearer_adv)
{
    return (prov_bearer_t *) ((uint8_t*) p_bearer_adv - offsetof(prov_bearer_t, bearer));
}

/**
 * Get the prov context object holding the given prov_bearer_t.
 *
 * @param[in] p_bearer Pointer to a provisioning bearer instance contained in a
 * provisioning context structure.
 *
 * @returns A pointer to the provisioning bearer structure's context parent.
 */
static inline prov_common_ctx_t * prov_bearer_ctx_get(prov_bearer_t * p_bearer)
{
    return (prov_common_ctx_t *) ((uint8_t*) p_bearer - offsetof(prov_common_ctx_t, bearer));
}


/**
 * Verify provisioning data.
 *
 * @param[in] p_data Data to verify
 *
 * @returns Whether the provisioning data satisfies all boundary conditions.
 */
static inline bool prov_data_is_valid(const nrf_mesh_prov_provisioning_data_t * p_data)
{
    return (p_data->netkey_index <= NRF_MESH_GLOBAL_KEY_INDEX_MAX &&
            nrf_mesh_address_type_get(p_data->address) == NRF_MESH_ADDRESS_TYPE_UNICAST);
}


/**
 * Checks if the length of a packet is valid.
 *
 * This is done by looking at the first byte (the PDU type) to determine the type of the
 * packet, and then comparing the specified length with the expected length of the packet.
 *
 * @param[in] p_buffer Pointer to the packet buffer.
 * @param[in] length Length of the packet buffer.
 *
 * @returns Whether the length of the packet is valid.
 */
bool prov_packet_length_valid(const uint8_t * p_buffer, uint16_t length);

/** @} end of PROVISIONING_BEARER_HELPER*/

/****************** Provisioning bearer control/message transmission functions ******************/
/**
 * @defgroup PROVISIONING_TX Provisioning message transmission functions
 * @{
 */

/**
 * Initialize a provisioning bearer instance.
 *
 * @param[in] p_bearer Bearer instance to initialize.
 * @param[in] p_interface Collection of functions used to call the specified provisioning bearer.
 * @param[in] p_callbacks Collection of callbacks to use for reporting upstream.
 * @param[in] link_timeout_us Time the protocol can go without interaction
 * before timing out (in microseconds). Must be equal to or larger than @ref
 * PROV_PROVISIONING_LINK_TIMEOUT_MIN_US.
 *
 * @retval NRF_SUCCESS The provisioning bearer instance was successfully initialized.
 * @retval NRF_ERROR_NULL One or more of the pointer parameters were NULL.
 * @retval NRF_ERROR_INVALID_PARAM The protocol timeout was too short.
 */
uint32_t prov_init(prov_bearer_t * p_bearer,
        const prov_bearer_interface_t * p_interface,
        const prov_bearer_callbacks_t * p_callbacks,
        uint32_t link_timeout_us);

/**
 * Attempts to establish a link by sending a Link Open message.
 * @note Provisioner only.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_target_uuid UUID of the peer device.
 *
 * @retval NRF_SUCCESS Successfully sent a link establishment request.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer was not in idle state.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 */
uint32_t prov_link_open(prov_bearer_t * p_bearer, const uint8_t * p_target_uuid);

/**
 * Closes a provisioning link.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] close_reason Close reason to send to the peer device.
 */
void prov_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);

/****************** Provisioning PDU transmit functions ******************/

/**
 * Sends the public key message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_public_key The public key of the user.
 *
 * @retval NRF_SUCCESS Successfully sent the public key.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_public_key(prov_bearer_t * p_bearer, const uint8_t * p_public_key);

/**
 * Sends the confirmation message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_confirmation_value The confirmation value.
 *
 * @retval NRF_SUCCESS Successfully sent the confirmation message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_confirmation(prov_bearer_t * p_bearer, const uint8_t * p_confirmation_value);

/**
 * Sends the provisioning random message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_random The confirmation key (@see prov_common_ctx_t).
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning random message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_random(prov_bearer_t * p_bearer, const uint8_t * p_random);

/**
 * @defgroup PROVISIONING_TX_PROVISIONER TX functions meant only for provisioner role
 * @{
 */

 /**
 * Sends the provisioning invite message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] attention_duration The attention timer value in seconds.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, @see prov_common_ctx_t.
 *
 * @retval NRF_SUCCESS Successfully sent a link establishment request.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_invite(prov_bearer_t * p_bearer, uint8_t attention_duration, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning start message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_start The assembled provisioning start pdu.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, @see prov_common_ctx_t.
 *
 * @retval NRF_SUCCESS Successfully sent a provisioning start message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_start(prov_bearer_t * p_bearer, const prov_pdu_prov_start_t * p_start, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning data message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_data The confirmation key (@see prov_common_ctx_t).
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning data message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_data(prov_bearer_t * p_bearer, const prov_pdu_data_t * p_data);

/** @} end of PROVISIONING_TX_PROVISIONER */

/**
 * @defgroup PROVISIONING_TX_PROVISIONEE TX functions meant only for provisionee role
 * @{
 */

/**
 * Sets up the advertiser for a provisioning instance to start listening for provisioning links
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] URI Optional device URI string used as identifier in some other context.
 * @param[in] oob_info_sources Known OOB information sources, @see NRF_MESH_PROV_OOB_INFO_SOURCES.
 *
 * @retval NRF_SUCCESS The bearer successfully started listening for link establishment requests.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer was not in idle state.
 * @retval NRF_ERROR_INTERNAL Something went wrong when initializing the substructures.
 */
uint32_t prov_listen_start(prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources);

/**
 * Stops listening for provisioning links on a provisioning instance.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 *
 * @retval NRF_SUCCESS The bearer successfully stopped listening for link establishment requests.
 */
uint32_t prov_listen_stop(prov_bearer_t * p_bearer);

/**
 * Sends the provisioning capabilities message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_caps The assembled provisioning capabilities pdu.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, @see prov_common_ctx_t.
 *
 * @retval NRF_SUCCESS Successfully sent the provisionee capabilities.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_capabilities(prov_bearer_t * p_bearer, const prov_pdu_caps_t * p_caps, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning input complete message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 *
 * @retval NRF_SUCCESS Successfully sent the input complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_input_complete(prov_bearer_t * p_bearer);

/**
 * Sends the provisioning complete message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_complete(prov_bearer_t * p_bearer);

/**
 * Sends the provisioning failed message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] failure_code The reason for the provisioning failure.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_failed(prov_bearer_t * p_bearer, nrf_mesh_prov_failure_code_t failure_code);

/** @} end of PROVISIONING_TX_PROVISIONEE*/

/**
 * Sends a raw provisioning PDU.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in]      p_data   PDU data pointer.
 * @param[in]      length   Length of PDU.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

/** @} end of PROVISIONING_TX*/


/****************** Call-back functions to be used by the provisioning bearers ******************/

/**
 * @defgroup PROVISIONING_CB Provisioning callback functions
 * These functions are used by the provisioning bearers to indicate when certain events occur.
 * @{
 */

/**
 * Notifies the provisioning.c module that a link has been opened.
 *
 * @param[in, out] p_bearer The bearer instance making the call (@see prov_bearer_adv_parent_get).
 */
void prov_cb_link_opened(prov_bearer_t * p_bearer);

/**
 * Notifies the provisioning.c module that a link has been closed.
 *
 * @param[in, out] p_bearer The bearer instance making the call (@see prov_bearer_adv_parent_get).
 * @param[in] close_reason The reason for the link close.
 */
void prov_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);

/**
 * Passes up an incoming packet to the provisioning.c module.
 *
 * @param[in, out] p_bearer The bearer instance making the call (@see prov_bearer_adv_parent_get).
 * @param[in] p_data The data received from the peer.
 * @param[in] length Length of the data received (in bytes).
 */
void prov_cb_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

/**
 * Notifies the provisioning.c module that an acknowledgement packet has been received.
 *
 * @param[in, out] p_bearer The bearer instance making the call (@see prov_bearer_adv_parent_get).
 */
void prov_cb_ack_in(prov_bearer_t * p_bearer);

/** @} end of PROVISIONING_CB*/

/** @} */

#endif /* PROVISIONING_H__ */
