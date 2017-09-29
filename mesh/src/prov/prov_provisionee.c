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

#include "prov_provisionee.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_error.h"

#include "enc.h"
#include "event.h"
#include "packet_mgr.h"
#include "utils.h"
#include "log.h"

#include "provisioning.h"
#include "prov_bearer.h"
#include "prov_pdu.h"
#include "prov_utils.h"

/****************** Call-back function declarations ******************/
static void prov_provisionee_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length);
static void prov_provisionee_cb_ack_received(prov_bearer_t * p_bearer);
static void prov_provisionee_cb_link_established(prov_bearer_t * p_bearer);
static void prov_provisionee_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);
static const prov_bearer_callbacks_t m_prov_callbacks =
             {
                .rx = prov_provisionee_pkt_in,
                .ack = prov_provisionee_cb_ack_received,
                .opened = prov_provisionee_cb_link_established,
                .closed = prov_provisionee_cb_link_closed
             };

/****************** Local functions ******************/
static void send_failed(prov_common_ctx_t * p_ctx, nrf_mesh_prov_failure_code_t failure_code)
{
    nrf_mesh_evt_t app_event;
    p_ctx->failure_code = failure_code;
    app_event.type = NRF_MESH_EVT_PROV_FAILED;
    app_event.params.prov_failed.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
    app_event.params.prov_failed.failure_code = failure_code;
    if (NRF_SUCCESS == prov_tx_failed(&p_ctx->bearer, p_ctx->failure_code))
    {
        event_handle(&app_event);
        p_ctx->state = NRF_MESH_PROV_STATE_FAILED;
    }
    else
    {
        prov_link_close(&p_ctx->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
        event_handle(&app_event);
        p_ctx->state = NRF_MESH_PROV_STATE_FAILED;
    }
}

static void send_capabilities(prov_common_ctx_t * p_ctx)
{
    prov_pdu_caps_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_CAPABILITIES;

    pdu.num_components = p_ctx->capabilities.num_elements;
    pdu.algorithms = LE2BE16(p_ctx->capabilities.algorithms);
    pdu.pubkey_type = p_ctx->capabilities.pubkey_type;
    pdu.oob_static_types = p_ctx->capabilities.oob_static_types;
    pdu.oob_output_size = p_ctx->capabilities.oob_output_size;
    pdu.oob_output_actions = LE2BE16(p_ctx->capabilities.oob_output_actions);
    pdu.oob_input_size = p_ctx->capabilities.oob_input_size;
    pdu.oob_input_actions = LE2BE16(p_ctx->capabilities.oob_input_actions);

#if PROV_DEBUG_MODE
    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: sending capabilities\n");
#endif
    if (NRF_SUCCESS != prov_tx_capabilities(&p_ctx->bearer, &pdu, p_ctx->confirmation_inputs))
    {
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
    }
}

static void handle_prov_start(prov_common_ctx_t * p_ctx, const uint8_t * p_buffer)
{
    const prov_pdu_prov_start_t * p_pdu = (const prov_pdu_prov_start_t *) p_buffer;

    /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
    memcpy(p_ctx->confirmation_inputs + PROV_CONFIRM_INPUTS_START_OFFSET, p_buffer + 1, sizeof(prov_pdu_prov_start_t) - 1);

    if (p_pdu->public_key == 0)
    {
        p_ctx->pubkey_oob = false;
    }
    else
    {
        p_ctx->pubkey_oob = true;
    }

    p_ctx->oob_method = (nrf_mesh_prov_oob_method_t) p_pdu->auth_method;
    p_ctx->oob_size = p_pdu->auth_method != NRF_MESH_PROV_OOB_METHOD_STATIC ? p_pdu->auth_size : 16;
    p_ctx->oob_action = p_pdu->auth_action;
}

/* Handles an incoming provisioning data message. */
static uint32_t handle_data(prov_common_ctx_t * p_ctx, const uint8_t * p_buffer)
{
    const prov_pdu_data_t * p_pdu = (const prov_pdu_data_t *) p_buffer;

    prov_pdu_data_t unencrypted_pdu;

    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_ctx->session_key;
    ccm_data.p_nonce = p_ctx->data_nonce;
    ccm_data.p_m = (uint8_t *) &p_pdu->data;
    ccm_data.m_len = sizeof(prov_pdu_data_block_t);
    ccm_data.p_out = (uint8_t *) &unencrypted_pdu.data;
    ccm_data.p_mic = (uint8_t *) p_pdu->mic;
    ccm_data.p_a = NULL;
    ccm_data.a_len = 0;
    ccm_data.mic_len = PROV_PDU_DATA_MIC_LENGTH;

    bool mic_passed = false;
    enc_aes_ccm_decrypt(&ccm_data, &mic_passed);
    if (!mic_passed)
    {
#if PROV_DEBUG_MODE
        __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisionee: provisioning data could not be authenticated!\n");
#endif
        return NRF_ERROR_INVALID_DATA;
    }

    memcpy(p_ctx->data.netkey, unencrypted_pdu.data.netkey, NRF_MESH_KEY_SIZE);
    p_ctx->data.iv_index = BE2LE32(unencrypted_pdu.data.iv_index);
    p_ctx->data.address = BE2LE16(unencrypted_pdu.data.address);
    p_ctx->data.netkey_index = BE2LE16(unencrypted_pdu.data.netkey_index);
    p_ctx->data.flags.iv_update = unencrypted_pdu.data.flags.iv_update;
    p_ctx->data.flags.key_refresh = unencrypted_pdu.data.flags.key_refresh;
    return NRF_SUCCESS;
}

static uint32_t request_authentication(prov_common_ctx_t * p_ctx)
{
    uint32_t retval = NRF_SUCCESS;

    /* Request OOB data/action for authentication. */
    switch (p_ctx->oob_method)
    {
        case NRF_MESH_PROV_OOB_METHOD_INPUT:
        {
            nrf_mesh_evt_t event;
            event.type = NRF_MESH_EVT_PROV_INPUT_REQUEST;
            event.params.prov_input_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            event.params.prov_input_request.size = p_ctx->oob_size;
            event.params.prov_input_request.action = (nrf_mesh_prov_input_action_t) p_ctx->oob_action;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_INPUT;
            event_handle(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested OOB input from application, size = %d\n", p_ctx->oob_size);
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_OUTPUT:
        {
            prov_utils_generate_oob_data(p_ctx, p_ctx->auth_value);

            nrf_mesh_evt_t event;
            event.type = NRF_MESH_EVT_PROV_OUTPUT_REQUEST;
            event.params.prov_output_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            event.params.prov_output_request.size = p_ctx->oob_size;
            event.params.prov_output_request.action = (nrf_mesh_prov_output_action_t) p_ctx->oob_action;
            event.params.prov_output_request.p_data = p_ctx->auth_value;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            event_handle(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested OOB output from application, size = %d\n", p_ctx->oob_size);
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_STATIC:
        {
            /* Request static provisioning data from the application. */
            nrf_mesh_evt_t event;
            event.type = NRF_MESH_EVT_PROV_STATIC_REQUEST;
            event.params.prov_static_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_STATIC;
            event_handle(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested static OOB data from application\n");
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_NONE:
            memset(p_ctx->auth_value, 0, sizeof(p_ctx->auth_value));
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            break;
        default:
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisioning: unrecognized OOB mode!\n");
#endif
            retval = NRF_ERROR_INTERNAL;
            break;
    }

    return retval;
}

static void start_authentication(prov_common_ctx_t * p_ctx)
{
    if (prov_utils_use_ecdh_offloading())
    {
        nrf_mesh_evt_t app_event;
        app_event.type = NRF_MESH_EVT_PROV_ECDH_REQUEST;
        app_event.params.prov_ecdh_request.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
        app_event.params.prov_ecdh_request.p_node_private = p_ctx->p_private_key;
        app_event.params.prov_ecdh_request.p_peer_public = p_ctx->peer_public_key;
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH;
        event_handle(&app_event);
    }
    else if (NRF_SUCCESS == prov_utils_calculate_shared_secret(p_ctx, p_ctx->shared_secret))
    {
        if (NRF_SUCCESS != request_authentication(p_ctx))
        {
            send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
        }
    }
    else
    {
        /* The ECDH fails if the public key is not valid: */
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
    }
}

/****************** Callback functions ******************/
static void prov_provisionee_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);

    if (!prov_packet_length_valid(p_buffer, length))
    {
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU);
        return;
    }

    switch (p_buffer[0])
    {
        case PROV_PDU_TYPE_INVITE:
            if (p_ctx->state == NRF_MESH_PROV_STATE_INVITE)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: invite received!\n");
#endif
                /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
                memcpy(p_ctx->confirmation_inputs + PROV_CONFIRM_INPUTS_INVITE_OFFSET, p_buffer + 1, sizeof(prov_pdu_invite_t) - 1);

                send_capabilities(p_ctx);
            }
            break;
        case PROV_PDU_TYPE_START:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_START)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: provisioning start message received!\n");
#endif
                handle_prov_start(p_ctx, p_buffer);
                p_ctx->state = NRF_MESH_PROV_STATE_WAIT_PUB_KEY;
                // if (p_ctx->pubkey_oob)
                // {
                //      /*TODO: The node should be exposing its public key OOB at this point. Add event to ensure this?*/
                // }
            }
            break;
        case PROV_PDU_TYPE_PUBLIC_KEY:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_PUB_KEY)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: public key message received!\n");
#endif
                const prov_pdu_pubkey_t * p_pdu = (const prov_pdu_pubkey_t *) p_buffer;
                memcpy(p_ctx->peer_public_key, p_pdu->public_key, NRF_MESH_PROV_PUBKEY_SIZE);

                if (!p_ctx->pubkey_oob)
                {
                    if (NRF_SUCCESS != prov_tx_public_key(&p_ctx->bearer, p_ctx->p_public_key))
                    {
                        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                        break;
                    }
                }
                else
                {
                    start_authentication(p_ctx);
                }
            }
            break;
        case PROV_PDU_TYPE_CONFIRMATION:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_CONFIRMATION)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioning: provisioning confirmation received!\n");
#endif
                const prov_pdu_confirm_t * p_pdu = (const prov_pdu_confirm_t *) p_buffer;
                memcpy(p_ctx->peer_confirmation, p_pdu->confirmation, sizeof(p_ctx->peer_confirmation));

                uint8_t confirmation_value[PROV_CONFIRMATION_LEN];
                prov_utils_authentication_values_derive(p_ctx, p_ctx->confirmation_salt, confirmation_value, p_ctx->node_random);
                uint32_t err_code = prov_tx_confirmation(&p_ctx->bearer, confirmation_value);
                if (NRF_SUCCESS != err_code)
                {
                    /* NRF_ERROR_NO_MEM is the only expected error code */
                    NRF_MESH_ASSERT(NRF_ERROR_NO_MEM == err_code);
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                    break;
                }
            }
            else if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_STATIC)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioning: provisioning confirmation received!\n");
#endif
                const prov_pdu_confirm_t * p_pdu = (const prov_pdu_confirm_t *) p_buffer;
                memcpy(p_ctx->peer_confirmation, p_pdu->confirmation, sizeof(p_ctx->peer_confirmation));
                p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD;
            }

            break;
        case PROV_PDU_TYPE_RANDOM:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_RANDOM)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: provisioner's random number received!\n");
#endif
                const prov_pdu_random_t * p_pdu = (const prov_pdu_random_t *) p_buffer;
                memcpy(p_ctx->peer_random, p_pdu->random, sizeof(p_pdu->random));

                if (!prov_utils_confirmation_check(p_ctx))
                {
#if PROV_DEBUG_MODE
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Provisionee: could not authenticate provisioner!\n");
#endif
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED);
                }
                else if (NRF_SUCCESS == prov_tx_random(&p_ctx->bearer, p_ctx->node_random))
                {
                    prov_utils_derive_keys(p_ctx, p_ctx->session_key, p_ctx->data_nonce, p_ctx->device_key);
                }
                else
                {
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                }
            }
            break;
        case PROV_PDU_TYPE_DATA:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_DATA)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: received provisioning data!\n");
#endif
                if (NRF_SUCCESS != handle_data(p_ctx, p_buffer))
                {
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED);
                }
                else if (!prov_data_is_valid(&p_ctx->data))
                {
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
                }
                else if (NRF_SUCCESS != prov_tx_complete(&p_ctx->bearer))
                {
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                }
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

            p_ctx->state = NRF_MESH_PROV_STATE_FAILED;

            break;
        }
        default:
            if (p_ctx->state != NRF_MESH_PROV_STATE_FAILED)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Provisioning: unknown provisioning message received, type = %.02x\n", p_buffer[0]);
#endif
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU);
            }
            break;
    }

}

static void complete_provisioning(prov_common_ctx_t * p_ctx)
{
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
}

static void prov_provisionee_cb_ack_received(prov_bearer_t * p_bearer)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    switch (p_ctx->state)
    {
        case NRF_MESH_PROV_STATE_INVITE:
            /* This is a response to send_capabilities */
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_START;
            break;
        case NRF_MESH_PROV_STATE_WAIT_START:
            break;
        case NRF_MESH_PROV_STATE_WAIT_PUB_KEY:
            start_authentication(p_ctx);
            break;
        case NRF_MESH_PROV_STATE_WAIT_OOB_INPUT:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            break;
        case NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD:
        case NRF_MESH_PROV_STATE_WAIT_CONFIRMATION:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_RANDOM;
            break;
        case NRF_MESH_PROV_STATE_WAIT_RANDOM:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_DATA;
            break;
        case NRF_MESH_PROV_STATE_WAIT_DATA:
            complete_provisioning(p_ctx);
            p_ctx->state = NRF_MESH_PROV_STATE_COMPLETE;
            break;
       case NRF_MESH_PROV_STATE_FAILED:
            break;
        default:
            __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisionee: unexpected ack while in state %u\n", p_ctx->state);
    }
}

static void prov_provisionee_cb_link_established(prov_bearer_t * p_bearer)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    p_ctx->state = NRF_MESH_PROV_STATE_INVITE;
    nrf_mesh_evt_t app_event;
    app_event.type = NRF_MESH_EVT_PROV_LINK_ESTABLISHED;
    app_event.params.prov_link_established.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
    event_handle(&app_event);
}
static void prov_provisionee_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    prov_common_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_DATA && reason == NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS)
    {
        complete_provisioning(p_ctx);
    }
    p_ctx->state = NRF_MESH_PROV_STATE_IDLE;
    nrf_mesh_evt_t app_event;
    app_event.type = NRF_MESH_EVT_PROV_LINK_CLOSED;
    app_event.params.prov_link_closed.p_context = (nrf_mesh_prov_ctx_t *) p_ctx;
    app_event.params.prov_link_closed.close_reason = reason;
    event_handle(&app_event);
}

/****************** Interface functions ******************/
uint32_t prov_provisionee_init(prov_common_ctx_t * p_ctx, const prov_bearer_interface_t * p_bearer, const char * URI, uint16_t oob_info_sources)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_LINK;
        p_ctx->role = NRF_MESH_PROV_ROLE_PROVISIONEE;
        uint32_t status = prov_init(&p_ctx->bearer, p_bearer, &m_prov_callbacks, PROV_PROVISIONING_LINK_TIMEOUT_MIN_US);
        if (NRF_SUCCESS == status)
        {
            return prov_listen_start(&p_ctx->bearer, URI, oob_info_sources);
        }
        else
        {
            return status;
        }
    }
}

uint32_t prov_provisionee_auth_data(prov_common_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size)
{
    //TODO: This interaction could be done in init. We don't provide any new information in the event triggering this.
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_INPUT
            && p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_STATIC
            && p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (size > PROV_AUTH_LEN || size != p_ctx->oob_size) /* TODO: I'm pretty sure we that's not how oob size works. */
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t retval = NRF_SUCCESS;

    /* Add zero-padding to the authentication data: */
    memset(&p_ctx->auth_value[size], 0, sizeof(p_ctx->auth_value) - size);
    memcpy(p_ctx->auth_value, p_data, size);

    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_INPUT)
    {
        retval = prov_tx_input_complete(&p_ctx->bearer);
        if (NRF_SUCCESS != retval)
        {
            return retval;
        }
    }
    else if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD)
    {
        uint8_t confirmation_value[PROV_CONFIRMATION_LEN];
        prov_utils_authentication_values_derive(p_ctx, p_ctx->confirmation_salt, confirmation_value, p_ctx->node_random);
        retval = prov_tx_confirmation(&p_ctx->bearer, confirmation_value);
    }
    else
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
    }

    return retval;
}

uint32_t prov_provisionee_shared_secret(prov_common_ctx_t * p_ctx, const uint8_t * p_shared)
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

