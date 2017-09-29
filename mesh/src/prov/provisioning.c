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

#include "provisioning.h"

#include "nrf_error.h"

#include "log.h"
#include "nrf_mesh.h"
#include "enc.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_prov.h"
#include "prov_utils.h"

NRF_MESH_STATIC_ASSERT(NRF_MESH_PROVISIONING_CTX_SIZE >= sizeof(prov_common_ctx_t));

/******************** Utility functions ********************/

static uint32_t send_data(prov_bearer_t * p_bearer, const uint8_t * p_payload, uint16_t size)
{
    NRF_MESH_ASSERT(p_payload != NULL);
    NRF_MESH_ASSERT(p_bearer  != NULL);

    return p_bearer->p_interface->tx(p_bearer, p_payload, size);
}

/******************** Interface functions ********************/

bool prov_packet_length_valid(const uint8_t * p_buffer, uint16_t length)
{
    static const uint8_t pdu_sizes[PROV_PDU_TYPE_COUNT] =
                        {sizeof(prov_pdu_invite_t), sizeof(prov_pdu_caps_t), sizeof(prov_pdu_prov_start_t),
                         sizeof(prov_pdu_pubkey_t), sizeof(prov_pdu_input_complete_t), sizeof(prov_pdu_confirm_t),
                         sizeof(prov_pdu_random_t), sizeof(prov_pdu_data_t), sizeof(prov_pdu_complete_t),
                         sizeof(prov_pdu_failed_t)};
    bool valid;
    if (p_buffer == NULL || p_buffer[0] >= PROV_PDU_TYPE_INVALID)
    {
        valid = false;
    }
    else if (length == pdu_sizes[p_buffer[0]])
    {
        valid = true;
    }
    else
    {
        valid = false;
    }
    return valid;
}


uint32_t prov_init(prov_bearer_t * p_bearer,
        const prov_bearer_interface_t * p_interface,
        const prov_bearer_callbacks_t * p_callbacks,
        uint32_t link_timeout_us)
{
    if (p_bearer == NULL || p_interface == NULL || p_callbacks == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (link_timeout_us < PROV_PROVISIONING_LINK_TIMEOUT_MIN_US)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    p_bearer->timeout = link_timeout_us;
    p_bearer->p_callbacks = p_callbacks;
    p_bearer->p_interface = p_interface;

    return NRF_SUCCESS;
}

uint32_t prov_listen_start(prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources)
{
    return p_bearer->p_interface->listen_start(p_bearer, URI, oob_info_sources, p_bearer->timeout);
}

uint32_t prov_listen_stop(prov_bearer_t * p_bearer)
{
    return p_bearer->p_interface->listen_stop(p_bearer);
}

uint32_t prov_link_open(prov_bearer_t * p_bearer, const uint8_t * p_target_uuid)
{
    return p_bearer->p_interface->link_open(p_bearer, p_target_uuid, p_bearer->timeout);
}

void prov_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    p_bearer->p_interface->link_close(p_bearer, close_reason);
}

uint32_t prov_tx_invite(prov_bearer_t * p_bearer, uint8_t attention_duration, uint8_t * p_confirmation_inputs)
{
    prov_pdu_invite_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_INVITE;
    pdu.attention_duration = attention_duration;

    /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
    memcpy(p_confirmation_inputs + PROV_CONFIRM_INPUTS_INVITE_OFFSET, ((const uint8_t *) &pdu) + 1, sizeof(prov_pdu_invite_t) - 1);
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_invite_t));
}

uint32_t prov_tx_capabilities(prov_bearer_t * p_bearer, const prov_pdu_caps_t * p_caps, uint8_t * p_confirmation_inputs)
{
    /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
    memcpy(p_confirmation_inputs + PROV_CONFIRM_INPUTS_CAPS_OFFSET, ((const uint8_t *) p_caps) + 1, sizeof(prov_pdu_caps_t) - 1);
    return send_data(p_bearer, (const uint8_t *) p_caps, sizeof(prov_pdu_caps_t));
}

uint32_t prov_tx_start(prov_bearer_t * p_bearer, const prov_pdu_prov_start_t * p_start, uint8_t * p_confirmation_inputs)
{
    /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
    memcpy(p_confirmation_inputs + PROV_CONFIRM_INPUTS_START_OFFSET, ((const uint8_t *) p_start) + 1, sizeof(prov_pdu_prov_start_t) - 1);
    return send_data(p_bearer, (const uint8_t *) p_start, sizeof(prov_pdu_prov_start_t));
}

uint32_t prov_tx_public_key(prov_bearer_t * p_bearer, const uint8_t * p_public_key)
{
    prov_pdu_pubkey_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_PUBLIC_KEY;
    memcpy(pdu.public_key, p_public_key, sizeof(pdu.public_key));
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_pubkey_t));
}

uint32_t prov_tx_input_complete(prov_bearer_t * p_bearer)
{
    prov_pdu_input_complete_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_INPUT_COMPLETE;
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_input_complete_t));
}

uint32_t prov_tx_confirmation(prov_bearer_t * p_bearer, const uint8_t * p_confirmation_value)
{
    prov_pdu_confirm_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_CONFIRMATION;
    memcpy(pdu.confirmation, p_confirmation_value, sizeof(pdu.confirmation));
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_confirm_t));
}

uint32_t prov_tx_random(prov_bearer_t * p_bearer, const uint8_t * p_random)
{
    prov_pdu_random_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_RANDOM;
    memcpy(pdu.random, p_random, sizeof(pdu.random));
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_random_t));
}

uint32_t prov_tx_complete(prov_bearer_t * p_bearer)
{
    prov_pdu_complete_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_COMPLETE;
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_complete_t));
}

uint32_t prov_tx_failed(prov_bearer_t * p_bearer, const nrf_mesh_prov_failure_code_t failure_code)
{
    prov_pdu_failed_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_FAILED;
    pdu.failure_code = (uint8_t) failure_code;
    return send_data(p_bearer, (const uint8_t *) &pdu, sizeof(prov_pdu_failed_t));
}

uint32_t prov_tx_data(prov_bearer_t * p_bearer, const prov_pdu_data_t * p_data)
{
    return send_data(p_bearer, (const uint8_t *) p_data, sizeof(prov_pdu_data_t));
}

uint32_t prov_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    return send_data(p_bearer, p_data, length);
}

/****************** Call-back functions ******************/
/* These will only be called by the transport module used by the provisioning bearer. */
void prov_cb_link_opened(prov_bearer_t * p_bearer)
{
    p_bearer->p_callbacks->opened(p_bearer);
}

void prov_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    p_bearer->p_callbacks->closed(p_bearer, close_reason);
}

void prov_cb_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "RX OPCODE [ppdu%x]\n", p_data[0]);
    p_bearer->p_callbacks->rx(p_bearer, p_data, length);
}

void prov_cb_ack_in(prov_bearer_t * p_bearer)
{
    p_bearer->p_callbacks->ack(p_bearer);
}
