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

#include <stdbool.h>
#include <string.h>

#include "nrf_mesh.h"
#include "nrf_mesh_prov.h"

#include "prov_bearer.h"
#include "provisioning.h"
#include "prov_utils.h"
#include "prov_provisionee.h"
#include "prov_provisioner.h"
#include "utils.h"
#include "log.h"


uint32_t nrf_mesh_prov_init(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_public_key, const uint8_t * p_private_key,
        const nrf_mesh_prov_oob_caps_t * p_caps)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    if (p_common_ctx->state != NRF_MESH_PROV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        p_common_ctx->p_public_key = p_public_key;
        p_common_ctx->p_private_key = p_private_key;
        memcpy(&p_common_ctx->capabilities, p_caps, sizeof(nrf_mesh_prov_oob_caps_t));
        return NRF_SUCCESS;
    }
}

uint32_t nrf_mesh_prov_generate_keys(uint8_t * p_public, uint8_t * p_private)
{
    return prov_utils_keys_generate(p_public, p_private);
}

uint32_t nrf_mesh_prov_listen(nrf_mesh_prov_ctx_t * p_ctx,
        nrf_mesh_prov_bearer_type_t bearer_type,
        const char * URI,
        uint16_t oob_info_sources)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    const prov_bearer_interface_t * p_bearer;

    switch (bearer_type)
    {
        case NRF_MESH_PROV_BEARER_ADV:
            p_bearer = prov_bearer_adv_interface_get();
            break;
        default:
            p_bearer = NULL;
            break;
    }

    if (p_bearer == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else
    {
        return prov_provisionee_init(p_common_ctx, p_bearer, URI, oob_info_sources);
    }
}

uint32_t nrf_mesh_prov_provision(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_target_uuid,
        const nrf_mesh_prov_provisioning_data_t * p_data, nrf_mesh_prov_bearer_type_t bearer_type)
{
    if (p_ctx == NULL || p_target_uuid == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    const prov_bearer_interface_t * p_bearer;

    switch (bearer_type)
    {
        case NRF_MESH_PROV_BEARER_ADV:
            p_bearer = prov_bearer_adv_interface_get();
            break;
        default:
            p_bearer = NULL;
            break;
    }

    if (p_bearer == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else if (!prov_data_is_valid(p_data))
    {
        return NRF_ERROR_INVALID_DATA;
    }
    else
    {
        return prov_provisioner_provision(p_common_ctx, p_bearer, p_target_uuid, p_data);
    }
}

uint32_t nrf_mesh_prov_oob_use(nrf_mesh_prov_ctx_t * p_ctx, nrf_mesh_prov_oob_method_t method, uint8_t size)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    if (p_common_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_oob_use(p_common_ctx, method, size);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}

uint32_t nrf_mesh_prov_auth_data_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    if (p_common_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_auth_data(p_common_ctx, p_data, size);
    }
    else
    {
        return prov_provisionee_auth_data(p_common_ctx, p_data, size);
    }
}

uint32_t nrf_mesh_prov_shared_secret_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    if (p_common_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_shared_secret(p_common_ctx, p_shared);
    }
    else
    {
        return prov_provisionee_shared_secret(p_common_ctx, p_shared);
    }
}

uint32_t nrf_mesh_prov_pubkey_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key)
{
    prov_common_ctx_t * p_common_ctx = (prov_common_ctx_t *) p_ctx;
    if (p_common_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONEE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        return prov_provisioner_oob_pubkey(p_common_ctx, p_key);
    }
}

