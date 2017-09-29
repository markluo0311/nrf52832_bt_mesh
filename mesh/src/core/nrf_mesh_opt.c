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
#include "nrf_mesh_opt.h"

#include "bearer_adv.h"
#include "network.h"
#include "prov_utils.h"
#include "transport.h"

#include "log.h"
#include "utils.h"

uint32_t nrf_mesh_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt)
{
    if (NRF_MESH_OPT_RADIO_START <= id && id < NRF_MESH_OPT_PROV_START)
    {
        return bearer_adv_opt_set(id, p_opt);
    }
    else if (NRF_MESH_OPT_PROV_START <= id && id < NRF_MESH_OPT_TRS_START)
    {
        return prov_utils_opt_set(id, p_opt);
    }
    else if (NRF_MESH_OPT_TRS_START <= id && id < NRF_MESH_OPT_NET_START)
    {
        return transport_opt_set(id, p_opt);
    }
    else if (NRF_MESH_OPT_NET_START <= id)
    {
        return network_opt_set(id, p_opt);
    }
    else
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
}

uint32_t nrf_mesh_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt)
{
    if (NRF_MESH_OPT_RADIO_START <= id && id < NRF_MESH_OPT_PROV_START)
    {
        return bearer_adv_opt_get(id, p_opt);
    }
    else if (NRF_MESH_OPT_PROV_START <= id && id < NRF_MESH_OPT_TRS_START)
    {
        return prov_utils_opt_get(id, p_opt);
    }
    else if (NRF_MESH_OPT_TRS_START <= id && id < NRF_MESH_OPT_NET_START)
    {
        return transport_opt_get(id, p_opt);
    }
    else if (NRF_MESH_OPT_NET_START <= id)
    {
        return network_opt_get(id, p_opt);
    }
    else
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
}
