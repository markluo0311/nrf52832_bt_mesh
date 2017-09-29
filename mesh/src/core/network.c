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
#include "network.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nrf_error.h>

#include "bearer.h"
#include "enc.h"
#include "msg_cache.h"
#include "packet.h"
#include "packet_mgr.h"
#include "transport.h"
#include "nrf_mesh_assert.h"
#include "net_beacon.h"
#include "net_state.h"
#include "utils.h"
#include "log.h"
#include "internal_event.h"
#include "bearer_event.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"

#define PRIVACY_RANDOM_SIZE 7
#define PECB_SIZE           6

/******************
 * Local Typedefs *
 ******************/

/*lint -align_max(push) -align_max(1) */

/**
 * PECB data according to the specification.
 */
typedef struct __attribute((packed))
{
    uint8_t          zero_padding[5];                     /**< Zero padding. */
    uint32_t         iv_index_be;                         /**< Current IV index (big endian). */
    uint8_t          privacy_random[PRIVACY_RANDOM_SIZE]; /**< The 7 LSBs of the privacy random value. */
} pecb_data_t;

/*lint -align_max(pop) */

/********************
 * Static variables *
 ********************/

static bool m_relay_enable;
static uint8_t m_relay_retransmit_count;
static nrf_mesh_relay_check_cb_t m_relay_check_cb;

/********************
 * Static functions *
 ********************/

/** Redefinition of header_transfuscate() for clarity. */
#define header_obfuscate(pkt_in, net, iv_index, pkt_out)    \
    header_transfuscate(pkt_in, net, iv_index, pkt_out)
/** Redefinition of header_transfuscate() for clarity. */
#define header_deobfuscate(pkt_in, net, iv_index, pkt_out)  \
    header_transfuscate(pkt_in, net, iv_index, pkt_out)

/**
 * (De-)obfuscates a network header.
 *
 * The whole network header, except NID+IVI and DST fields, is obfuscated.
 *
 * @param[in]      p_net_packet_in  Network packet pointer.
 * @param[in]      p_network        Network context structure pointer.
 * @param[in]      iv_index_be      IV index of the packet in big endian.
 * @param[out]     p_net_packet_out Network packet pointer. May be the
 *                                  same as @c p_net_packet_in.
 */
static void header_transfuscate(const packet_net_t * p_net_packet_in,
                                const nrf_mesh_network_secmat_t * p_net_secmat,
                                uint32_t iv_index_be,
                                packet_net_t * p_net_packet_out)
{
    uint8_t pecb[NRF_MESH_KEY_SIZE];

    /* Calculate the PECB: */
    pecb_data_t pecb_data;
    memset(&pecb_data.zero_padding[0], 0, sizeof(pecb_data.zero_padding));
    pecb_data.iv_index_be = iv_index_be;
    memcpy(pecb_data.privacy_random, (const void *) &p_net_packet_in->header.dst, PRIVACY_RANDOM_SIZE); /*lint !e420 Apparent access beyond p_net_packet_in->header.dst */
    enc_aes_encrypt(p_net_secmat->privacy_key, (const uint8_t *) &pecb_data, pecb);

    utils_xor(((uint8_t *) &p_net_packet_out->header) + sizeof(uint8_t) /* Skip the first byte, not obfuscated */,
              ((uint8_t *) &p_net_packet_in->header) + sizeof(uint8_t),
              pecb, offsetof(packet_net_hdr_t, dst) - sizeof(uint8_t));
}

/**
 * Encrypts a network packet.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 * | ... ---------|------------- AES-CCM encrypted -------------|----------|
 * | ... | SRC:16 | DST:16 | Trans. payload [0-12] | Trans. MIC | Net. MIC |
 * -------------------------------------------------------------------------
 * @endverbatim
 *
 * @param[in, out] p_net_packet Network packet pointer.
 * @param[in]      p_network    Network context structure pointer.
 * @param[in]      iv_index_le  Little endian representation of the IV index to use for encryption.
 *
 */
static void network_pkt_encrypt(packet_net_t * p_net_packet,
                                const nrf_mesh_network_secmat_t * const p_net_secmat,
                                uint32_t iv_index_le)
{
    uint8_t nonce[CCM_NONCE_LENGTH];
    const uint32_t iv_index_be = LE2BE32(iv_index_le);

    enc_nonce_generate(&p_net_packet->header, iv_index_be, ENC_NONCE_NET, 0, nonce);

    ccm_soft_data_t ccm_params;
    ccm_params.mic_len = packet_net_micsize_get(p_net_packet);
    ccm_params.p_key   = p_net_secmat->encryption_key;
    ccm_params.p_nonce = nonce;
    ccm_params.p_m     = (uint8_t *) &p_net_packet->header.dst;
    ccm_params.m_len   = (packet_net_payload_size_get(p_net_packet)
                          + sizeof(p_net_packet->header.dst)
                          - ccm_params.mic_len);
    ccm_params.a_len   = 0;
    ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;
    ccm_params.p_out   = (uint8_t *) &p_net_packet->header.dst;

    enc_aes_ccm_encrypt(&ccm_params);

    header_obfuscate(p_net_packet, p_net_secmat, iv_index_be, p_net_packet);
}

/**
 * Decrypts a network packet.
 *
 * @param[in]  p_net_packet Pointer to encrypted network packet.
 * @param[out] p_net_decrypted_packet Pointer to buffer in which the encrypted
 *                                    packet is decrypted into.
 * @param[out] pp_net_secmat          Network context structure double pointer, that will return the network used to decrypt the packet.
 * @param[out] p_iv_index             IV index the packet was decrypted with in little endian.
 *
 * @returns Whether the decryption was successful and authenticated.
 */
static bool network_pkt_decrypt(packet_net_t * p_net_packet,
                                packet_net_t * p_net_decrypted_packet,
                                const nrf_mesh_network_secmat_t ** pp_net_secmat,
                                uint32_t * p_iv_index)
{
    NRF_MESH_ASSERT(p_net_packet != NULL &&
                    p_net_decrypted_packet != NULL &&
                    pp_net_secmat != NULL &&
                    p_iv_index != NULL);

    uint8_t nonce[CCM_NONCE_LENGTH];

    /* Configure CCM. */
    ccm_soft_data_t ccm_params;
    ccm_params.a_len   = 0;
    ccm_params.p_nonce = nonce;
    ccm_params.p_m     = (uint8_t *) &p_net_packet->header.dst;
    ccm_params.p_out   = (uint8_t *) &p_net_decrypted_packet->header.dst;

    uint32_t iv_index_le = net_state_rx_iv_index_get(p_net_packet->header.ivi);
    uint32_t iv_index_be = LE2BE32(iv_index_le);
    bool authenticated = false;

    *pp_net_secmat = NULL;
    nrf_mesh_net_secmat_next_get(p_net_packet->header.nid, pp_net_secmat);

    for (;
         *pp_net_secmat != NULL;
         nrf_mesh_net_secmat_next_get(p_net_packet->header.nid, pp_net_secmat))
    {
        header_deobfuscate(p_net_packet, *pp_net_secmat, iv_index_be, p_net_decrypted_packet);

        ccm_params.mic_len = packet_net_micsize_get(p_net_decrypted_packet);
        ccm_params.m_len   = (packet_net_payload_size_get(p_net_decrypted_packet)
                              + sizeof(p_net_decrypted_packet->header.dst)
                              - ccm_params.mic_len);
        ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;

        /* Create a nonce for use when authenticating the packet from the de-obfuscated header: */
        enc_nonce_generate(&p_net_decrypted_packet->header, iv_index_be, ENC_NONCE_NET, 0, nonce);

        ccm_params.p_key = (*pp_net_secmat)->encryption_key;
        enc_aes_ccm_decrypt(&ccm_params, &authenticated);

        if (authenticated)
        {
            *p_iv_index = iv_index_le;
            __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_INFO, "Unencrypted data: ", ccm_params.p_out, ccm_params.m_len);
            break;
        }
    }
    return authenticated;
}

uint32_t network_pkt_relay(packet_net_t * p_net_packet,
                           const nrf_mesh_network_secmat_t * const p_net_secmat)
{
    uint32_t status = NRF_SUCCESS;
    if (p_net_packet->header.ttl <= 1 || !m_relay_enable)
    {
        /* Ignore */
        return status;
    }
    else if (m_relay_check_cb != NULL)
    {
        bool application_relay_ok =
            m_relay_check_cb(LE2BE16(p_net_packet->header.src),
                    LE2BE16(p_net_packet->header.dst),
                    p_net_packet->header.ttl);
        if (!application_relay_ok)
        {
            return status;
        }
    }

    packet_t * p_relay_packet;
    packet_net_t * p_net_relay_packet;
    status = packet_mgr_alloc((packet_generic_t **) &p_relay_packet,
                              (packet_net_buffer_size_get(p_net_packet)
                               + sizeof(ble_packet_hdr_t)
                               + BLE_GAP_ADDR_LEN));
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_NETWORK, LOG_LEVEL_WARN, "Unable to allocate memory for relay packet.\n");
        return NRF_ERROR_NO_MEM;
    }

    /* Copy payload into the relay packet. */
    memcpy(p_relay_packet->payload,
           p_net_packet,
           packet_net_buffer_size_get(p_net_packet));

    packet_payload_size_set(p_relay_packet, packet_net_buffer_size_get(p_net_packet));
    packet_ad_type_set(p_relay_packet, AD_TYPE_MESH);

    p_net_relay_packet = packet_net_packet_get(p_relay_packet);

    NRF_MESH_ASSERT(p_net_relay_packet != NULL);

    p_net_relay_packet->header.ttl--;

    network_pkt_encrypt(p_net_relay_packet, p_net_secmat, net_state_rx_iv_index_get(p_net_packet->header.ivi));

    status = bearer_tx(p_relay_packet, NETWORK_BEARER, 1);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_relay_packet);
    }
    else
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_RELAYED, 0, p_net_packet->length+1, p_net_packet);
    }

    return status;
}
/******************************
 * Public interface functions *
 ******************************/

void network_init(const nrf_mesh_init_params_t * p_init_params)
{
    if (p_init_params == NULL)
    {
        m_relay_check_cb = NULL;
    }
    else
    {
        m_relay_check_cb = p_init_params->relay_cb;
    }

    m_relay_enable = true;
    m_relay_retransmit_count = 0;

    net_state_init();
    net_state_recover_from_flash();
    net_beacon_init();
}

uint32_t network_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt)
{
    if (p_opt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_STEPS:
            return NRF_ERROR_NOT_SUPPORTED;
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
            m_relay_enable = p_opt->opt.val;
            break;
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
            if (p_opt->opt.val > NETWORK_RELAY_RETRANSMITS_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            else
            {
                m_relay_retransmit_count = p_opt->opt.val;
            }
            break;
        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t network_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt)
{
    if (p_opt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_STEPS:
            return NRF_ERROR_NOT_SUPPORTED;
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
            p_opt->opt.val = m_relay_enable;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
            p_opt->opt.val = m_relay_retransmit_count;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t network_pkt_out(packet_t * p_packet, const nrf_mesh_network_secmat_t * const p_net_secmat, bool local_packet)
{
    packet_net_t * p_net_packet = packet_net_packet_get(p_packet);

    NRF_MESH_ASSERT(p_net_packet != NULL);

    uint16_t src = BE2LE16(p_net_packet->header.src);
    NRF_MESH_ASSERT(nrf_mesh_address_type_get(src) == NRF_MESH_ADDRESS_TYPE_UNICAST);

    /* Construct the packet header: */
    uint32_t iv_index = net_state_tx_iv_index_get();
    p_net_packet->header.nid = p_net_secmat->nid;
    p_net_packet->header.ivi = iv_index & NETWORK_IVI_MASK;
    /* other fields added by higher layers */

    network_pkt_encrypt(p_net_packet, p_net_secmat, iv_index);

    uint32_t status = bearer_tx(p_packet, NETWORK_BEARER, local_packet ? BEARER_ADV_REPEAT_DEFAULT : 1);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_NETWORK, LOG_LEVEL_WARN, "Unable to transmit packet %p [er%u]\n", ((void*) p_packet), status);
    }
    else
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_PACKET_QUEUED_TX, 0, p_net_packet->length, p_net_packet);
    }

    return status;
}

uint32_t network_pkt_in(packet_net_t* p_net_packet, const packet_meta_t * p_packet_meta)
{
    uint32_t status = NRF_SUCCESS;

    /* Create a target buffer to decrypt into, don't have to allocate a new packet. */
    uint8_t decrypt_buf[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
    packet_net_t* p_net_decrypted_packet = (packet_net_t*) &decrypt_buf[0];

    /* Packet fields not touched by the encryption. */
    p_net_decrypted_packet->length     = p_net_packet->length;
    p_net_decrypted_packet->ad_type    = p_net_packet->ad_type;
    p_net_decrypted_packet->header.nid = p_net_packet->header.nid;
    p_net_decrypted_packet->header.ivi = p_net_packet->header.ivi;

    uint32_t decrypted_iv_index = 0;
    const nrf_mesh_network_secmat_t * p_net_secmat = NULL;
    bool authenticated = network_pkt_decrypt(p_net_packet, p_net_decrypted_packet, &p_net_secmat, &decrypted_iv_index);
    if (authenticated)
    {
        NRF_MESH_ASSERT(p_net_secmat != NULL);
#if NETWORK_CACHE_ENABLE
        const bool in_cache = msg_cache_entry_exists(p_net_decrypted_packet);
#else
        const bool in_cache = false;
#endif
        if (in_cache)
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_NETWORK_CACHE, p_net_packet->length+1, p_net_decrypted_packet);
        }
        else
        {
            uint16_t src_addr_le = BE2LE16(p_net_decrypted_packet->header.src);

            if (nrf_mesh_address_type_get(src_addr_le) == NRF_MESH_ADDRESS_TYPE_UNICAST)
            {
                /* If the source address is one of our unicast rx addresses, we
                 * sent it ourselves, and shouldn't process it: */
                nrf_mesh_address_t dummy;
                if (!nrf_mesh_rx_address_get(src_addr_le, &dummy))
                {
                    status = transport_pkt_in(p_net_decrypted_packet, p_packet_meta, p_net_secmat, decrypted_iv_index);
#if NETWORK_CACHE_ENABLE
                    msg_cache_entry_add(p_net_decrypted_packet->header.src, BE2LE24(p_net_decrypted_packet->header.seq));
#endif
                }
            }
        }
    }
    return status;
}
