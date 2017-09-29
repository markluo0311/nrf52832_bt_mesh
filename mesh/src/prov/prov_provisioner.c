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

#include "prov_provisioner.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_prov.h"

#include "log.h"

#include "enc.h"
#include "event.h"
#include "packet_mgr.h"

#include "provisioning.h"
#include "prov_bearer.h"
#include "prov_pdu.h"
#include "prov_utils.h"

/****************** Call-back function declarations ******************/
static void prov_provisioner_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length);
static void prov_provisioner_cb_ack_received(prov_bearer_t * p_bearer);
static void prov_provisioner_cb_link_established(prov_bearer_t * p_bearer);
static void prov_provisioner_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);

static const prov_bearer_callbacks_t m_prov_callbacks =
             {
                .rx = prov_provisioner_pkt_in,
                .ack = prov_provisioner_cb_ack_received,
                .opened = prov_provisioner_cb_link_established,
                .closed = prov_provisioner_cb_link_closed
             };

/****************** Local functions ******************/
static void send_data(prov_common_ctx_t * p_ctx, const uint8_t * p_session_key, const uint8_t * p_session_nonce)
{
    prov_pdu_data_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_DATA;

    prov_pdu_data_block_t data_block;
    memcpy(data_block.netkey, p_ctx->data.netkey, NRF_MESH_KEY_SIZE);
    data_block.iv_index = LE2BE32(p_ctx->data.iv_index);
    data_block.address = LE2BE16(p_ctx->data.address);
    data_block.netkey_index = LE2BE16(p_ctx->data.netkey_index);
    data_block.flags.key_refresh = p_ctx->data.flags.key_refresh;
    data_block.flags.iv_update = p_ctx->data.flags.iv_update;
    data_block.flags._rfu = 0;

    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_session_key;
    ccm_data.p_nonce = p_session_nonce;
    ccm_data.p_m = (uint8_t *) &data_block;
    ccm_data.m_len = sizeof(prov_pdu_data_block_t);
    ccm_data.p_out = (uint8_t *) &pdu.data;
    ccm_data.p_mic = pdu.mic;
    ccm_data.p_a = NULL;
    ccm_data.a_len = 0;
    ccm_data.mic_len = PROV_PDU_DATA_MIC_LENGTH;

    enc_aes_ccm_encrypt(&ccm_data);
#if PROV_DEBUG_MODE
    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: sending provisioning data\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
    if (NRF_SUCCESS == prov_tx_data(&p_ctx->bearer,  &pdu))
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_COMPLETE;
    }
    else
    {
        prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
    }
}


static uint32_t send_confirmation(prov_common_ctx_t * p_ctx)
{
    uint8_t confirmation_value[PROV_CONFIRMATION_LEN];
    prov_utils_authentication_values_derive(p_ctx, p_ctx->confirmation_salt, confirmation_value, p_ctx->node_random);
    uint32_t status = prov_tx_confirmation(&p_ctx->bearer, confirmation_value);
    if (status == NRF_SUCCESS)
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
    }
    return status;
}

static uint32_t request_authentication(prov_common_ctx_t * p_ctx)
{
    uint32_t retval = NRF_SUCCESS;
    nrf_mesh_evt_t event;
    switch (p_ctx->oob_method) /* Note that the OOB method is the _provisionee_'s OOB method */
    {
        case NRF_MESH_PROV_OOB_METHOD_INPUT:
            prov_utils_generate_oob_data(p_ctx, p_ctx->auth_value);

            /* Request display of data for entering into the provisionee: */
            event.type = NRF_MESH_EVT_PROV_OUTPUT_REQUEST;
            event.params.prov_output_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            event.params.prov_output_request.size = p_ctx->oob_size;
            event.params.prov_output_request.action = (nrf_mesh_prov_output_action_t) p_ctx->oob_action;
            event.params.prov_output_request.p_data = p_ctx->auth_value;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE;
            event_handle(&event);
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: requested OOB output from application, size = %d\n", p_ctx->bearer.bearer.pb_adv.link_id, p_ctx->oob_size);
#endif
            break;
        case NRF_MESH_PROV_OOB_METHOD_OUTPUT:
            /* Request input of data read from the provisionee: */
            event.type = NRF_MESH_EVT_PROV_INPUT_REQUEST;
            event.params.prov_input_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            event.params.prov_input_request.size = p_ctx->oob_size;
            event.params.prov_input_request.action = (nrf_mesh_prov_input_action_t) p_ctx->oob_action;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_INPUT;
            event_handle(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: requested OOB input from application, size = %d\n", p_ctx->bearer.bearer.pb_adv.link_id, p_ctx->oob_size);
#endif
            break;
        case NRF_MESH_PROV_OOB_METHOD_STATIC:
            /* Request static provisioning data from the application. */
            event.type = NRF_MESH_EVT_PROV_STATIC_REQUEST;
            event.params.prov_static_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_STATIC;
            event_handle(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: requested static OOB data from application\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
            break;
        case NRF_MESH_PROV_OOB_METHOD_NONE:
            memset(p_ctx->auth_value, 0, sizeof(p_ctx->auth_value));
            retval = send_confirmation(p_ctx);
            break;
        default:
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisioner L%d: unrecognized OOB mode!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
            retval = NRF_ERROR_INTERNAL;
            break;
    }

    return retval;
}

static uint32_t start_authentication(prov_common_ctx_t * p_ctx)
{
    uint32_t status = NRF_SUCCESS;
    if (prov_utils_use_ecdh_offloading())
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH;
        nrf_mesh_evt_t app_event;
        app_event.type = NRF_MESH_EVT_PROV_ECDH_REQUEST;
        app_event.params.prov_ecdh_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
        app_event.params.prov_ecdh_request.p_node_private = p_ctx->p_private_key;
        app_event.params.prov_ecdh_request.p_peer_public = p_ctx->peer_public_key;
        event_handle(&app_event);
    }
    else
    {
        status = prov_utils_calculate_shared_secret(p_ctx, p_ctx->shared_secret);
        if (NRF_SUCCESS != status)
        {
            return status;
        }
        status = request_authentication(p_ctx);
    }
    return status;
}
/****************** Call-back functions ******************/
static void prov_provisioner_cb_link_established(prov_bearer_t * p_bearer)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_LINK)
    {
        nrf_mesh_evt_t app_event;
        app_event.type = NRF_MESH_EVT_PROV_LINK_ESTABLISHED;
        app_event.params.prov_link_established.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
        event_handle(&app_event);

        /* Send the provisioning invite (with attention timer off). */
        uint32_t status = prov_tx_invite(&p_ctx->bearer, 0, p_ctx->confirmation_inputs);
        if (status == NRF_SUCCESS)
        {
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CAPS;
        }
        else
        {
            prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
        }
    }
}

static void prov_provisioner_cb_ack_received(prov_bearer_t * p_bearer)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);

    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_START_ACK)
    {
        if (p_ctx->pubkey_oob)
        {
            nrf_mesh_evt_t event;
            event.type = NRF_MESH_EVT_PROV_OOB_PUBKEY_REQUEST;
            event.params.prov_oob_pubkey_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY;
            event_handle(&event);
        }
        else
        {
            if (NRF_SUCCESS == prov_tx_public_key(&p_ctx->bearer, p_ctx->p_public_key))
            {
                p_ctx->state = NRF_MESH_PROV_STATE_WAIT_PUB_KEY;
            }
            else
            {
                prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
            }
        }
    }
}

static void prov_provisioner_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    if (p_ctx->state != NRF_MESH_PROV_STATE_IDLE)
    {
        p_ctx->state = NRF_MESH_PROV_STATE_IDLE;
        nrf_mesh_evt_t app_event;
        app_event.type = NRF_MESH_EVT_PROV_LINK_CLOSED;
        app_event.params.prov_link_closed.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
        app_event.params.prov_link_closed.close_reason = reason;
        event_handle(&app_event);
    }
}

static void prov_provisioner_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);

    if (!prov_packet_length_valid(p_buffer, length))
    {
        prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
        return;
    }

    switch (p_buffer[0])
    {
        case PROV_PDU_TYPE_CAPABILITIES:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_CAPS)
            {
                const prov_pdu_caps_t * p_pdu = (const prov_pdu_caps_t *) p_buffer;

                /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
                memcpy(p_ctx->confirmation_inputs + PROV_CONFIRM_INPUTS_CAPS_OFFSET, ((uint8_t *) p_pdu) + 1, sizeof(prov_pdu_caps_t) - 1);

                /* Wait for app to tell us which OOB method to use */
                p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM;

                nrf_mesh_evt_t event;
                event.type = NRF_MESH_EVT_PROV_CAPS_RECEIVED;
                event.params.prov_oob_caps_received.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
                event.params.prov_oob_caps_received.oob_caps.num_elements = p_pdu->num_components;
                event.params.prov_oob_caps_received.oob_caps.algorithms = BE2LE16(p_pdu->algorithms);
                event.params.prov_oob_caps_received.oob_caps.pubkey_type = p_pdu->pubkey_type;
                event.params.prov_oob_caps_received.oob_caps.oob_static_types = p_pdu->oob_static_types;
                event.params.prov_oob_caps_received.oob_caps.oob_output_size = p_pdu->oob_output_size;
                event.params.prov_oob_caps_received.oob_caps.oob_output_actions = BE2LE16(p_pdu->oob_output_actions);
                event.params.prov_oob_caps_received.oob_caps.oob_input_size = p_pdu->oob_input_size;
                event.params.prov_oob_caps_received.oob_caps.oob_input_actions = BE2LE16(p_pdu->oob_input_actions);
                event_handle(&event);
            }
            break;
        case PROV_PDU_TYPE_PUBLIC_KEY:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_PUB_KEY)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: provisionee's public key received!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif

                const prov_pdu_pubkey_t * p_pdu = (const prov_pdu_pubkey_t *) p_buffer;
                memcpy(p_ctx->peer_public_key, p_pdu->public_key, NRF_MESH_PROV_PUBKEY_SIZE);

                if (start_authentication(p_ctx) != NRF_SUCCESS)
                {
                    prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                }
            }
            break;
        case PROV_PDU_TYPE_INPUT_COMPLETE:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: input complete message received!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
                uint32_t err_code = send_confirmation(p_ctx);
                if (err_code != NRF_SUCCESS)
                {
                    prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                }
            }
            break;
        case PROV_PDU_TYPE_CONFIRMATION:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_CONFIRMATION)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: provisioning confirmation received!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
                const prov_pdu_confirm_t * p_pdu = (const prov_pdu_confirm_t *) p_buffer;
                memcpy(p_ctx->peer_confirmation, p_pdu->confirmation, sizeof(p_ctx->peer_confirmation));

                if (NRF_SUCCESS == prov_tx_random(&p_ctx->bearer, p_ctx->node_random))
                {
                    p_ctx->state = NRF_MESH_PROV_STATE_WAIT_RANDOM;
                }
                else
                {
                    prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                }
            }

            break;
        case PROV_PDU_TYPE_RANDOM:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_RANDOM)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: provisionee's random number received!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
                /* TODO: Peer random is only used here, no need to store it. */
                const prov_pdu_random_t * p_pdu = (const prov_pdu_random_t *) p_buffer;
                memcpy(p_ctx->peer_random, p_pdu->random, sizeof(p_pdu->random));

                if (!prov_utils_confirmation_check(p_ctx))
                {
#if PROV_DEBUG_MODE
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisioner L%d: could not authenticate provisionee!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif
                    prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                }
                else
                {
                    uint8_t session_key[NRF_MESH_KEY_SIZE];
                    uint8_t session_nonce[PROV_NONCE_LEN];
                    prov_utils_derive_keys(p_ctx, session_key, session_nonce, p_ctx->device_key);
                    send_data(p_ctx, session_key, session_nonce);
                }
            }
            break;
        case PROV_PDU_TYPE_COMPLETE:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_COMPLETE)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioner L%d: received provisioning complete message!\n", p_ctx->bearer.bearer.pb_adv.link_id);
#endif

                nrf_mesh_evt_t app_event;
                app_event.type = NRF_MESH_EVT_PROV_COMPLETE;
                app_event.params.prov_complete.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
                app_event.params.prov_complete.p_devkey = p_ctx->device_key;
                app_event.params.prov_complete.p_netkey = p_ctx->data.netkey;
                app_event.params.prov_complete.iv_index = p_ctx->data.iv_index;
                app_event.params.prov_complete.netkey_index = p_ctx->data.netkey_index;
                app_event.params.prov_complete.address = p_ctx->data.address;
                app_event.params.prov_complete.flags.iv_update = p_ctx->data.flags.iv_update;
                app_event.params.prov_complete.flags.key_refresh = p_ctx->data.flags.key_refresh;
                event_handle(&app_event);

                prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS);
            }
            break;
        case PROV_PDU_TYPE_FAILED:
        {
            const prov_pdu_failed_t * p_pdu = (const prov_pdu_failed_t *) p_buffer;

            nrf_mesh_evt_t app_event;
            app_event.type = NRF_MESH_EVT_PROV_FAILED;
            app_event.params.prov_failed.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            app_event.params.prov_failed.failure_code = (nrf_mesh_prov_failure_code_t) p_pdu->failure_code;
            event_handle(&app_event);

            prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
            break;
        }
        default:
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Provisioner L%d: unrecognized/unsupported message received, type = %.02x\n", p_ctx->bearer.bearer.pb_adv.link_id, p_buffer[0]);
#endif
            prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
            break;
    }

}

/****************** Interface functions ******************/
uint32_t prov_provisioner_provision(prov_common_ctx_t * p_ctx, const prov_bearer_interface_t * p_bearer,
            const uint8_t * p_uuid, const nrf_mesh_prov_provisioning_data_t * p_data)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t status = prov_init(&p_ctx->bearer, p_bearer, &m_prov_callbacks, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_ctx->role = NRF_MESH_PROV_ROLE_PROVISIONER;

    /* copy data (to be used later)*/
    memcpy(&p_ctx->data, p_data, sizeof(nrf_mesh_prov_provisioning_data_t));
    status = prov_link_open(&p_ctx->bearer, p_uuid);
    if (status == NRF_SUCCESS)
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_LINK;
    }
    return status;
}

uint32_t prov_provisioner_oob_use(prov_common_ctx_t * p_ctx, nrf_mesh_prov_oob_method_t method, uint8_t size)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (size > PROV_AUTH_LEN
            || (size == 0             && method != NRF_MESH_PROV_OOB_METHOD_NONE)
            || (size != 0             && method == NRF_MESH_PROV_OOB_METHOD_NONE)
            || (size != PROV_AUTH_LEN && method == NRF_MESH_PROV_OOB_METHOD_STATIC))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_ctx->oob_method = method;
    p_ctx->oob_size = size;

    prov_pdu_prov_start_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_START;

    pdu.algorithm = PROV_PDU_START_ALGO_FIPS_P256; /* FIPS P256 EC is the only supported algorithm. */
    pdu.public_key = p_ctx->capabilities.pubkey_type ? 0x01 : 0x00;
    pdu.auth_method = (uint8_t) p_ctx->oob_method;
    pdu.auth_action = p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_STATIC ? 0 : p_ctx->oob_action;
    pdu.auth_size = p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_STATIC ? 0 : p_ctx->oob_size;

    uint32_t retval = prov_tx_start(&p_ctx->bearer, &pdu, p_ctx->confirmation_inputs);
    if (retval == NRF_SUCCESS)
    {
        /* If the start message is not successfully sent, do not update the state.
         * This will allow the application to try again (if desired) until the
         * message is successfully sent or the link times out.
         */
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_START_ACK;
    }

    return retval;
}

uint32_t prov_provisioner_auth_data(prov_common_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_INPUT && p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_STATIC)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (size > 16 || size != p_ctx->oob_size)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    memcpy(p_ctx->auth_value, p_data, size);
    memset(&p_ctx->auth_value[size], 0, sizeof(p_ctx->auth_value) - size); /* Add zero-padding to the authentication data. */

    return send_confirmation(p_ctx);
}

uint32_t prov_provisioner_shared_secret(prov_common_ctx_t * p_ctx, const uint8_t * p_shared)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_shared == NULL)
    {
        return NRF_ERROR_NULL;
    }

    memcpy(p_ctx->shared_secret, p_shared, sizeof(p_ctx->shared_secret));

    return request_authentication(p_ctx);
}

uint32_t prov_provisioner_oob_pubkey(prov_common_ctx_t * p_ctx, const uint8_t * p_key)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_key == NULL)
    {
        return NRF_ERROR_NULL;
    }

    memcpy(p_ctx->peer_public_key, p_key, NRF_MESH_PROV_PUBKEY_SIZE);

    uint32_t retval = prov_tx_public_key(&p_ctx->bearer, p_ctx->p_public_key);
    if (retval == NRF_SUCCESS)
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_PUB_KEY_ACK;
    }

    return retval;
}

