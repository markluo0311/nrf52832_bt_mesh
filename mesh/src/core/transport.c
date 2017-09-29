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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include <nrf_error.h>

#include "utils.h"
#include "log.h"
#include "enc.h"
#include "event.h"
#include "nrf_mesh.h"
#include "network.h"
#include "net_state.h"
#include "packet_mgr.h"
#include "beacon.h"
#include "list.h"
#include "replay_cache.h"
#include "internal_event.h"
#include "timer_scheduler.h"
#include "bearer_event.h"
#include "toolchain.h"
#include "transport.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"

#include "packet_mesh.h"

/*********************
 * Local definitions *
 *********************/

/** Mask for SeqZero value. */
#define TRANSPORT_SAR_SEQZERO_MASK (0x1FFF)

#define TRANSPORT_SAR_SEGMENT_MAX_SIZE (PACKET_MESH_SEG_PDU_MAX_SIZE - BIT2MICSIZE(0))
#define TRANSPORT_SAR_PACKET_MAX_SIZE (32 * TRANSPORT_SAR_SEGMENT_MAX_SIZE)

/********************
 * Static variables *
 ********************/

static transport_config_t m_trs_config;

static bool m_trs_sar_ack_timeout;
static bool m_trs_sar_ack_timer_active;
static bool m_trs_sar_tx_timeout;

static timer_event_t m_trs_sar_rx_ack_timer;
static timer_event_t m_trs_sar_tx_timer;
static timer_event_t m_trs_sar_session_timer;

static trs_sar_ctx_t m_trs_sar_tx_sessions[TRANSPORT_SAR_TX_SESSIONS_MAX];
static trs_sar_ctx_t m_trs_sar_rx_sessions[TRANSPORT_SAR_RX_SESSIONS_MAX];

static transport_sar_alloc_t    m_sar_alloc;   /**< Allocation function for SAR packets. */
static transport_sar_release_t  m_sar_release; /**< Release function for SAR packets. */

/* Temporary buffers */
static uint8_t m_transport_pkt_buf[PACKET_MESH_UNSEG_PDU_MAX_SIZE];

/********************
 * Static functions *
 ********************/

/**
 * Get the authentication sequence number.
 *
 * Gets the sequence number that encrypted the SAR packet.
 *
 * @todo Do this more clever
 *
 * @param[in] p_mesh_packet Mesh packet pointer.
 *
 * @return Sequence number calculated from seqzero (little endian).
 */
static inline
uint32_t packet_mesh_trs_seq_auth_get(const packet_mesh_t * p_mesh_packet)
{
    uint32_t seq = packet_mesh_net_seq_get(p_mesh_packet);
    uint16_t seqzero = packet_mesh_trs_seqzero_get(p_mesh_packet);
    uint32_t seq_auth = 0;

    if ((seq & 0x1fff) < seqzero)
    {
        seq_auth = ((seq - ((seq & 0x1fff) - seqzero) - 0x2000));
    }
    else
    {
        seq_auth = ((seq - ((seq & 0x1fff) - seqzero)));
    }
    return seq_auth;
}

static inline
bool is_unseg_access_packet(const packet_mesh_t * p_mesh_packet)
{

    return (!packet_mesh_net_ctl_get(p_mesh_packet) && !packet_mesh_trs_seg_get(p_mesh_packet));
}

static inline
bool is_seg_access_packet(const packet_mesh_t * p_mesh_packet)
{
    return (!packet_mesh_net_ctl_get(p_mesh_packet) && packet_mesh_trs_seg_get(p_mesh_packet));
}

static inline
bool is_seg_ack_packet(const packet_mesh_t * p_mesh_packet)
{
    return (packet_mesh_net_ctl_get(p_mesh_packet) && packet_mesh_trs_opcode_get(p_mesh_packet) == 0x00);

}

static void trs_sar_ack_timeout_cb(timestamp_t timeout, void * p_context)
{
    m_trs_sar_ack_timeout = true;
    m_trs_sar_ack_timer_active = false;
}

static void trs_sar_tx_timeout_cb(timestamp_t timeout, void * p_context)
{
    m_trs_sar_tx_timeout = true;
}

static void m_send_replay_cache_full_event(uint16_t src, uint8_t ivi, nrf_mesh_rx_failed_reason_t reason)
{
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_RX_FAILED,
            .params.rx_failed.src = src,
            .params.rx_failed.ivi = ivi,
            .params.rx_failed.reason = reason
        };
    event_handle(&evt);
}

static void m_send_sar_cancel_event(uint32_t packet_id, nrf_mesh_sar_session_cancel_reason_t reason)
{
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_SAR_FAILED,
            .params.sar_failed.packet_id = packet_id,
            .params.sar_failed.reason = reason
        };
    event_handle(&evt);
}

static void trs_sar_drop_session(trs_sar_ctx_t * p_sar_ctx, nrf_mesh_sar_session_cancel_reason_t reason)
{
    NRF_MESH_ASSERT(p_sar_ctx != NULL);

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_SAR_CANCELLED, reason, 0, NULL);
    m_send_sar_cancel_event((uint32_t) p_sar_ctx->payload, reason);

    m_sar_release(p_sar_ctx->payload);
    p_sar_ctx->payload = NULL;
    p_sar_ctx->header.session_active = 0;

    net_state_iv_index_lock(false);

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Dropped SAR session %u\n", p_sar_ctx->header.session_type);
}

static void trs_sar_session_next_timeout_get(timestamp_t timeout, trs_sar_ctx_t ** pp_sar_ctx)
{
    for (int i = 0; i < TRANSPORT_SAR_TX_SESSIONS_MAX; ++i)
    {
        if (!m_trs_sar_tx_sessions[i].header.session_active)
        {
            continue;
        }

        if (TIMER_OLDER_THAN(timeout,  m_trs_sar_tx_sessions[i].header.timeout))
        {
            timeout = m_trs_sar_tx_sessions[i].header.timeout;
            *pp_sar_ctx = &m_trs_sar_tx_sessions[i];
        }
    }

    for (int i = 0; i < TRANSPORT_SAR_RX_SESSIONS_MAX; ++i)
    {
        if (!m_trs_sar_rx_sessions[i].header.session_active)
        {
            continue;
        }

        if (TIMER_OLDER_THAN(timeout,  m_trs_sar_rx_sessions[i].header.timeout))
        {
            timeout = m_trs_sar_rx_sessions[i].header.timeout;
            *pp_sar_ctx = &m_trs_sar_rx_sessions[i];
        }
    }
}

static void trs_sar_session_cb(timestamp_t timeout, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = (trs_sar_ctx_t *) p_context;

    if (p_sar_ctx->header.session_active)
    {
        /* Could have been dropped */
        trs_sar_drop_session(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_TIMEOUT); // Make ASYNC
    }
    else
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Session already cancelled\n");
    }

    /* Done with the context. */

    trs_sar_ctx_t * p_next_ctx = p_sar_ctx;
    trs_sar_session_next_timeout_get(timeout, &p_next_ctx);

    if (p_next_ctx != p_sar_ctx)
    {
        /* More timeouts awaiting! */

        /* In timer_event context here. */
        /* bearer_event_critical_section_begin(); */
        m_trs_sar_session_timer.p_context = p_next_ctx;

        timer_sch_reschedule(&m_trs_sar_session_timer, p_next_ctx->header.timeout);
        /* bearer_event_critical_section_end(); */
    }
}

static void trs_sar_session_timer_order(trs_sar_ctx_t * p_sar_ctx)
{

    trs_sar_ctx_t * p_next_ctx = p_sar_ctx;
    trs_sar_session_next_timeout_get(p_sar_ctx->header.timeout, &p_next_ctx);

    if (p_next_ctx == p_sar_ctx)
    {
        /* We're up next! */
        bearer_event_critical_section_begin();
        m_trs_sar_session_timer.p_context = p_sar_ctx;
        bearer_event_critical_section_end();

        timer_sch_reschedule(&m_trs_sar_session_timer, p_sar_ctx->header.timeout);
    }
}

static uint32_t trs_sar_pkt_ack(const trs_sar_ctx_t * p_sar_ctx)
{
    uint32_t status;
    packet_t * p_packet;
    packet_mesh_t * p_mesh_packet;

    status = packet_mgr_alloc((packet_generic_t**) &p_packet,
                              (PACKET_MESH_SEG_ACK_SIZE
                               + BLE_ADV_PACKET_HEADER_LENGTH
                               + BLE_ADV_PACKET_OVERHEAD));
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_mesh_packet = (packet_mesh_t *) p_packet->payload;

    uint32_t seq = 0;
    status = net_state_seqnum_alloc(&seq);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_packet);
        return status;
    }

    memset(p_mesh_packet, 0, PACKET_MESH_SEG_ACK_SIZE);
    packet_payload_size_set(p_packet, PACKET_MESH_SEG_ACK_SIZE);
    p_mesh_packet->ad_type = AD_TYPE_MESH;
    p_mesh_packet->length  = PACKET_MESH_SEG_ACK_SIZE - 1;

    packet_mesh_net_ttl_set(p_mesh_packet, m_trs_config.segack_ttl);
    packet_mesh_net_ctl_set(p_mesh_packet, 1);
    packet_mesh_net_seq_set(p_mesh_packet, seq);
    packet_mesh_net_src_set(p_mesh_packet, p_sar_ctx->header.dst);
    packet_mesh_net_dst_set(p_mesh_packet, p_sar_ctx->header.src);
    packet_mesh_trs_seqzero_set(p_mesh_packet, p_sar_ctx->header.seq_auth & TRANSPORT_SAR_SEQZERO_MASK);
    packet_mesh_trs_block_ack_set(p_mesh_packet, p_sar_ctx->header.block_ack);

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_ACK_QUEUED, 0, p_mesh_packet->length, p_mesh_packet);
    status = network_pkt_out(p_packet, p_sar_ctx->p_net_secmat, true);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_packet);
    }

    return status;
}

static uint32_t trs_sar_seg_pkt_in(packet_mesh_t * p_mesh_packet, const nrf_mesh_network_secmat_t * p_net_secmat)
{
    /* Check if the packet is part of an ongoing transaction. */
    uint32_t status = NRF_SUCCESS;

    trs_sar_ctx_t * p_sar_ctx = NULL;

    for (int i = 0; i < TRANSPORT_SAR_RX_SESSIONS_MAX; ++i)
    {

        if (!m_trs_sar_rx_sessions[i].header.session_active)
        {
            /* Use this context for now, but keep searching for an existing session allocated to this message. */
            p_sar_ctx = &m_trs_sar_rx_sessions[i];
            if (p_sar_ctx->header.src == packet_mesh_net_src_get(p_mesh_packet) &&
                m_trs_sar_rx_sessions[i].header.seq_auth == packet_mesh_trs_seq_auth_get(p_mesh_packet))
            {
                __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Got packet from old SAR session\n");
                if (p_sar_ctx->header.seg_left == 0)
                {
                    p_sar_ctx->header.seq = packet_mesh_net_seq_get(p_mesh_packet);
                    /* Acknowledge that all packets were already received */
                    if (nrf_mesh_address_type_get(packet_mesh_net_dst_get(p_mesh_packet)) == NRF_MESH_ADDRESS_TYPE_UNICAST)
                    {
                        (void)trs_sar_pkt_ack(p_sar_ctx);
                    }
                    status = replay_cache_add(packet_mesh_net_src_get(p_mesh_packet),
                                              packet_mesh_net_seq_get(p_mesh_packet),
                                              packet_mesh_net_ivi_get(p_mesh_packet));
                    /* We should already have space reserved in the replay_cache! */
                    NRF_MESH_ASSERT(NRF_SUCCESS == status);
                }
                return NRF_SUCCESS;
            }
        }
        else if (m_trs_sar_rx_sessions[i].header.src      == packet_mesh_net_src_get(p_mesh_packet) &&
                 m_trs_sar_rx_sessions[i].header.seq_auth == packet_mesh_trs_seq_auth_get(p_mesh_packet))
        {
            p_sar_ctx = &m_trs_sar_rx_sessions[i];
            break;
        }
    }

    if (p_sar_ctx == NULL)
    {
        m_send_sar_cancel_event(0, NRF_MESH_SAR_CANCEL_REASON_NO_MEM);
        return NRF_ERROR_NO_MEM;
    }

    const uint8_t seg_n = packet_mesh_trs_segn_get(p_mesh_packet);

    if (!p_sar_ctx->header.session_active)
    {
        /** @todo How do we determine what to accept? Just available memory? */

        const uint32_t total_length = (seg_n + 1) * TRANSPORT_SAR_SEGMENT_MAX_SIZE;

        if (total_length > TRANSPORT_SAR_PACKET_MAX_SIZE)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Invalid length: %u\n", total_length);
            return NRF_ERROR_INVALID_DATA;
        }

        p_sar_ctx->header.block_ack    = 0;
        p_sar_ctx->header.seg_left     = seg_n + 1; /**< seg_left-- a couple of lines below */
        p_sar_ctx->header.session_type = TRS_SAR_SESSION_RX;
        p_sar_ctx->header.timeout      = timer_now() + m_trs_config.rx_timeout;
        p_sar_ctx->header.seq_auth     = packet_mesh_trs_seq_auth_get(p_mesh_packet);
        p_sar_ctx->header.ttl          = packet_mesh_net_ttl_get(p_mesh_packet);
        p_sar_ctx->header.src          = packet_mesh_net_src_get(p_mesh_packet);
        p_sar_ctx->header.dst          = packet_mesh_net_dst_get(p_mesh_packet);

        if (!replay_cache_has_room(packet_mesh_net_src_get(p_mesh_packet), packet_mesh_net_ivi_get(p_mesh_packet)))
        {
            (void)m_send_replay_cache_full_event(packet_mesh_net_src_get(p_mesh_packet), packet_mesh_net_ivi_get(p_mesh_packet), NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
            status = NRF_ERROR_NO_MEM;
        }
        else
        {
            NRF_MESH_ASSERT(p_sar_ctx->payload == NULL);
            p_sar_ctx->payload = m_sar_alloc(TRANSPORT_SAR_SEGMENT_MAX_SIZE * (seg_n + 1));
            if (p_sar_ctx->payload == NULL)
            {
                status = NRF_ERROR_NO_MEM;
            }
        }

        if (status != NRF_SUCCESS)
        {
            if (nrf_mesh_address_type_get(p_sar_ctx->header.dst) == NRF_MESH_ADDRESS_TYPE_UNICAST)
            {
                (void)trs_sar_pkt_ack(p_sar_ctx);
            }
            m_send_sar_cancel_event(0, NRF_MESH_SAR_CANCEL_REASON_NO_MEM);
            return status;
        }

        memset(p_sar_ctx->payload, 0, TRANSPORT_SAR_SEGMENT_MAX_SIZE * (seg_n + 1));
        p_sar_ctx->header.session_active = 1;

        /* Can't change IV index for the duration of a SAR session. */
        net_state_iv_index_lock(true);

        /* Worst case. Updated on the last segment received. */
        p_sar_ctx->header.length    = total_length;
        p_sar_ctx->header.ivi       = packet_mesh_net_ivi_get(p_mesh_packet);
        p_sar_ctx->header.akf       = packet_mesh_trs_akf_get(p_mesh_packet);
        p_sar_ctx->header.aid       = packet_mesh_trs_aid_get(p_mesh_packet);
        p_sar_ctx->p_net_secmat     = p_net_secmat;
        p_sar_ctx->header.szmic     = packet_mesh_trs_szmic_get(p_mesh_packet);
        p_sar_ctx->header.seq       = packet_mesh_net_seq_get(p_mesh_packet);

        trs_sar_session_timer_order(p_sar_ctx);
    }

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Got segment %u\n", packet_mesh_trs_sego_get(p_mesh_packet));
    if (p_sar_ctx->header.block_ack & (1u << packet_mesh_trs_sego_get(p_mesh_packet)))
    {
        /* Segment already received. */
        return NRF_SUCCESS;
    }

    p_sar_ctx->header.block_ack |= (1u << packet_mesh_trs_sego_get(p_mesh_packet));
    p_sar_ctx->header.seg_left--;

    uint8_t len;
    if (packet_mesh_trs_sego_get(p_mesh_packet) == seg_n)
    {
        len = packet_mesh_seg_pdu_length_get(p_mesh_packet) - BIT2MICSIZE(0);
        /* Last segment might not be the full length of a normal segment, update total length */
        p_sar_ctx->header.length = len + (seg_n * TRANSPORT_SAR_SEGMENT_MAX_SIZE);
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Last segment length: %u, total length: %u\n", len, p_sar_ctx->header.length);
    }
    else
    {
        len = TRANSPORT_SAR_SEGMENT_MAX_SIZE;
    }

    memcpy(&p_sar_ctx->payload[packet_mesh_trs_sego_get(p_mesh_packet) * TRANSPORT_SAR_SEGMENT_MAX_SIZE],
           packet_mesh_seg_payload_get(p_mesh_packet),
           len);
    __LOG_XB(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Segment payload ", packet_mesh_seg_payload_get(p_mesh_packet), len);

    /* Only ACK if sent to unicast address. */
    if (nrf_mesh_address_type_get(p_sar_ctx->header.dst) == NRF_MESH_ADDRESS_TYPE_UNICAST && !m_trs_sar_ack_timer_active)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Re-scheduling ack timer. 0x%04x\n", p_sar_ctx->header.dst);
        timer_sch_reschedule(&m_trs_sar_rx_ack_timer, m_trs_config.rx_ack_timeout + timer_now());
        m_trs_sar_ack_timer_active = true;
    }

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_DECRYPT_TRS_SEG, 0,  p_mesh_packet->length+1,p_mesh_packet);

    return NRF_SUCCESS;
}

static uint32_t trs_sar_segack_pkt_in(packet_mesh_t * p_mesh_packet)
{

    trs_sar_ctx_t * p_sar_ctx = NULL;

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    for (int i = 0; i < TRANSPORT_SAR_RX_SESSIONS_MAX; ++i)
    {

        if (!m_trs_sar_tx_sessions[i].header.session_active)
        {
            continue;
        }

        uint16_t seqzero = packet_mesh_trs_seqzero_get(p_mesh_packet);

        if ((m_trs_sar_tx_sessions[i].header.src == packet_mesh_net_dst_get(p_mesh_packet)) &&
            ((m_trs_sar_tx_sessions[i].header.seq_auth & TRANSPORT_SAR_SEQZERO_MASK) == seqzero))
        {
            p_sar_ctx = &m_trs_sar_tx_sessions[i];
            break;
        }
        else
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO,
                  "%0x != %0x, %0x != %0x\n",
                  m_trs_sar_tx_sessions[i].header.src,
                  packet_mesh_net_dst_get(p_mesh_packet),
                  m_trs_sar_tx_sessions[i].header.seq_auth,
                  seqzero);
        }
    }
    _ENABLE_IRQS(was_masked);

    if (p_sar_ctx == NULL)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Could not find active session for ACK packet\n");
        return NRF_SUCCESS;
    }

   __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_TRS_ACK_RECEIVED, 0, p_mesh_packet->length, p_mesh_packet);


    uint32_t block_ack = packet_mesh_trs_block_ack_get(p_mesh_packet);
    p_sar_ctx->header.block_ack &= ~(block_ack);
    if (block_ack == 0)
    {
        trs_sar_drop_session(p_sar_ctx, NRF_MESH_SAR_CANCEL_BY_PEER);
    }
    else
    {
        /** @todo check return  */
        timer_sch_reschedule(&m_trs_sar_tx_timer, timer_now() + m_trs_config.tx_retry_timeout);
    }
    return NRF_SUCCESS;
}

static bool test_transport_decrypt(const nrf_mesh_application_secmat_t ** pp_app_secmat, ccm_soft_data_t * p_ccm_data)
{
    bool mic_passed = false;
    if (*pp_app_secmat != NULL)
    {
        p_ccm_data->p_key = (*pp_app_secmat)->key;
        enc_aes_ccm_decrypt(p_ccm_data, &mic_passed);
        if (mic_passed)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Message decrypted\n");
        }
    }
    return mic_passed;
}

uint32_t trs_sar_pkt_decrypt(const trs_sar_ctx_t * p_sar_ctx)
{
    uint32_t status;
    uint8_t decrypt_buf[p_sar_ctx->header.length - BIT2MICSIZE(p_sar_ctx->header.szmic)];
    uint32_t iv_index = net_state_rx_iv_index_get(p_sar_ctx->header.ivi);

    /** @todo IV index non-trivial on IV update. Check index bit. */
    enc_nonce_app_t nonce =
        {
            .type     = p_sar_ctx->header.akf ? ENC_NONCE_APP : ENC_NONCE_DEV,
            .aszmic   = p_sar_ctx->header.szmic,
            .padding  = 0,
            .seq      = LE2BE24(p_sar_ctx->header.seq_auth),
            .src      = LE2BE16(p_sar_ctx->header.src),
            .dst      = LE2BE16(p_sar_ctx->header.dst),
            .iv_index = LE2BE32(iv_index)
        };

    __LOG_XB(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "SAR APP nonce: ", ((uint8_t *) &nonce), sizeof(nonce));

    ccm_soft_data_t ccm_data;
    ccm_data.p_nonce = (uint8_t *) &nonce;
    ccm_data.p_m     = p_sar_ctx->payload;
    ccm_data.mic_len = BIT2MICSIZE(p_sar_ctx->header.szmic);
    ccm_data.m_len   = p_sar_ctx->header.length - ccm_data.mic_len;
    ccm_data.p_mic   = (uint8_t *) ccm_data.p_m + ccm_data.m_len;
    ccm_data.p_out   = &decrypt_buf[0];

    nrf_mesh_address_t dst_address = { NRF_MESH_ADDRESS_TYPE_INVALID };
    if (!nrf_mesh_rx_address_get(p_sar_ctx->header.dst, &dst_address))
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (dst_address.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        //FIXME: Iterate through the virtual addresses that match the 16bit version
        ccm_data.p_a   = dst_address.p_virtual_uuid;
        ccm_data.a_len = NRF_MESH_UUID_SIZE;
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Using Virtual Address (*)\n");
    }
    else
    {
        ccm_data.a_len = 0;
    }

    const nrf_mesh_application_secmat_t * p_app_secmat = NULL;

    bool mic_passed = false;

    if (p_sar_ctx->header.akf)
    {
        for (nrf_mesh_app_secmat_next_get(p_sar_ctx->p_net_secmat, p_sar_ctx->header.aid, &p_app_secmat);
             p_app_secmat != NULL;
             nrf_mesh_app_secmat_next_get(p_sar_ctx->p_net_secmat, p_sar_ctx->header.aid, &p_app_secmat))
        {
            mic_passed = test_transport_decrypt(&p_app_secmat, &ccm_data);
            if (mic_passed)
            {
                break;
            }
        }
    }
    else
    {
        p_app_secmat = NULL;
        if (dst_address.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            nrf_mesh_devkey_secmat_get(p_sar_ctx->header.dst, &p_app_secmat);
            mic_passed = test_transport_decrypt(&p_app_secmat, &ccm_data);
        }

        if (!mic_passed)
        {
            /* try the src address */
            p_app_secmat = NULL;
            nrf_mesh_devkey_secmat_get(p_sar_ctx->header.src, &p_app_secmat);
            mic_passed = test_transport_decrypt(&p_app_secmat, &ccm_data);
        }
    }

    if (!mic_passed)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Unable to decrypt message\n");
        status = NRF_ERROR_NOT_FOUND;
    }
    else
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Message decrypted\n");
        nrf_mesh_address_t src_address =
        {
            .type = NRF_MESH_ADDRESS_TYPE_UNICAST,
            .value = p_sar_ctx->header.src,
            .p_virtual_uuid = NULL
        };
        nrf_mesh_evt_t event;
        event.type                        = NRF_MESH_EVT_MESSAGE_RECEIVED;
        event.params.message.p_buffer     = &decrypt_buf[0];
        event.params.message.length       = p_sar_ctx->header.length - BIT2MICSIZE(p_sar_ctx->header.szmic);
        event.params.message.src          = src_address;
        event.params.message.dst          = dst_address;
        event.params.message.secmat.p_net = p_sar_ctx->p_net_secmat;
        event.params.message.secmat.p_app = p_app_secmat;
        event.params.message.ttl          = p_sar_ctx->header.ttl; /**@todo This TTL value
                                                                 * doesn't really make
                                                                 * sense..  */
        event_handle(&event);
        status = NRF_SUCCESS;

        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "SAR assembling complete\n");
    }

    return status;
}

/** Process ongoing SAR RX sessions. */
static uint32_t trs_sar_rx_process(void)
{
    uint32_t status = NRF_SUCCESS;

    trs_sar_ctx_t * p_sar_ctx = NULL;

    for (int i = 0; i < TRANSPORT_SAR_RX_SESSIONS_MAX; ++i)
    {
        p_sar_ctx = &m_trs_sar_rx_sessions[i];
        if (!p_sar_ctx->header.session_active)
        {
            continue;
        }

        if ((m_trs_sar_ack_timeout || p_sar_ctx->header.seg_left == 0))
        {
            /* The replay cache may be full by now*/
            if (!replay_cache_has_room(p_sar_ctx->header.src, p_sar_ctx->header.ivi))
            {
                m_send_replay_cache_full_event(p_sar_ctx->header.src, p_sar_ctx->header.ivi, NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
                p_sar_ctx->header.block_ack = 0;
                /* We're already in failure path don't care about return of these here, but perhaps they can be pushed as events?*/
                if (nrf_mesh_address_type_get(p_sar_ctx->header.dst) == NRF_MESH_ADDRESS_TYPE_UNICAST)
                {
                    (void)trs_sar_pkt_ack(p_sar_ctx);
                }
                trs_sar_drop_session(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_NO_MEM);
                return NRF_ERROR_NO_MEM;
            }

            if (nrf_mesh_address_type_get(p_sar_ctx->header.dst) == NRF_MESH_ADDRESS_TYPE_UNICAST)
            {
                status = trs_sar_pkt_ack(p_sar_ctx);
            }
            if (status != NRF_SUCCESS)
            {
                return status;
            }
            m_trs_sar_ack_timeout      = false;
            m_trs_sar_ack_timer_active = false;
            timer_sch_abort(&m_trs_sar_rx_ack_timer);
        }

        if (p_sar_ctx->header.seg_left == 0)
        {
            status = trs_sar_pkt_decrypt(p_sar_ctx);
            switch (status)
            {
                case NRF_SUCCESS:
                    status = replay_cache_add(p_sar_ctx->header.src,
                                              p_sar_ctx->header.seq,
                                              p_sar_ctx->header.ivi);
                    /* We had already checked that the replay_cache still has space! */
                    NRF_MESH_ASSERT(NRF_SUCCESS == status);

                    /* fallthrough */
                case NRF_ERROR_NOT_FOUND:
                {
                    /* @todo should only need to set active => 0 */

                    m_sar_release(p_sar_ctx->payload);
                    p_sar_ctx->payload = NULL;

                    /* Disable session */
                    p_sar_ctx->header.session_active = 0;

                    net_state_iv_index_lock(false);

                    status = NRF_SUCCESS;
                    break;
                }
                default:
                    m_sar_release(p_sar_ctx->payload);
                    p_sar_ctx->payload = NULL;
                    return status;
            }
        }
    }
    return status;
}

static uint32_t sar_segment_send(trs_sar_ctx_t * p_sar_ctx, uint8_t seg_index, uint8_t seg_n, uint8_t trs_payload_size)
{
    packet_t * p_packet;
    packet_mesh_t * p_mesh_packet;
    uint32_t status = packet_mgr_alloc((packet_generic_t**) &p_packet, BLE_ADV_PACKET_MAX_LENGTH);
    if (status != NRF_SUCCESS)
    {
        p_sar_ctx->header.seg_left = seg_index;
        return status;
    }

    uint32_t seqnum;
    status = net_state_seqnum_alloc(&seqnum);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_packet);
        p_sar_ctx->header.seg_left = seg_index;
        return status;
    }

    uint32_t packet_payload_size = sizeof(packet_mesh_t) + PACKET_MESH_SEG_PDU_OFFSET + trs_payload_size + BIT2MICSIZE(0);
    memset(p_packet->payload, 0, packet_payload_size);
    packet_payload_size_set(p_packet, packet_payload_size);

    p_mesh_packet = (packet_mesh_t *) &p_packet->payload[0];

    p_mesh_packet->ad_type = AD_TYPE_MESH;
    p_mesh_packet->length  = packet_payload_size - 1;

    packet_mesh_net_ivi_set(p_mesh_packet, p_sar_ctx->header.ivi);
    packet_mesh_net_ctl_set(p_mesh_packet, 0);
    packet_mesh_net_ttl_set(p_mesh_packet, p_sar_ctx->header.ttl);
    packet_mesh_net_seq_set(p_mesh_packet, seqnum);
    packet_mesh_net_src_set(p_mesh_packet, p_sar_ctx->header.src);
    packet_mesh_net_dst_set(p_mesh_packet, p_sar_ctx->header.dst);
    packet_mesh_trs_seg_set(p_mesh_packet, 1);
    packet_mesh_trs_aid_set(p_mesh_packet, p_sar_ctx->header.aid);
    packet_mesh_trs_akf_set(p_mesh_packet, p_sar_ctx->header.akf);
    packet_mesh_trs_szmic_set(p_mesh_packet, p_sar_ctx->header.szmic);
    packet_mesh_trs_seqzero_set(p_mesh_packet, p_sar_ctx->header.seq_auth & TRANSPORT_SAR_SEQZERO_MASK);
    packet_mesh_trs_sego_set(p_mesh_packet, seg_index);
    packet_mesh_trs_segn_set(p_mesh_packet, seg_n);

    memcpy(packet_mesh_seg_payload_get(p_mesh_packet),
           &p_sar_ctx->payload[seg_index * TRANSPORT_SAR_SEGMENT_MAX_SIZE],
           trs_payload_size);

    status = network_pkt_out(p_packet, p_sar_ctx->p_net_secmat, true);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_packet);
        p_sar_ctx->header.seg_left = seg_index;
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Error %u: Unable to transmit segment. Segments left: %u\n", status, p_sar_ctx->header.seg_left);
    }
    return status;
}

static uint32_t trs_sar_pkt_out(trs_sar_ctx_t * p_sar_ctx)
{
    uint32_t status = NRF_SUCCESS;
    uint8_t seg_n = ((p_sar_ctx->header.length +
                      TRANSPORT_SAR_SEGMENT_MAX_SIZE - 1) / TRANSPORT_SAR_SEGMENT_MAX_SIZE) - 1;

    if (p_sar_ctx->header.retries == 0)
    {
        trs_sar_drop_session(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_RETRY);
        return NRF_SUCCESS;
    }

    if (p_sar_ctx->header.seg_left == 0)
    {
        /* Retries is decreased for each time all segments has been sent. */
        p_sar_ctx->header.retries--;
    }

    /* Starts at seg_left if, e.g., there was no memory available last round. */
    for (uint8_t i = p_sar_ctx->header.seg_left; i < seg_n; ++i)
    {
        if ((p_sar_ctx->header.block_ack & (1u << i)) == 0)
        {
            /* Segment acknowledged */
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Segment %u already acked!\n", i);
            continue;
        }

        status = sar_segment_send(p_sar_ctx, i, seg_n, TRANSPORT_SAR_SEGMENT_MAX_SIZE);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
    }

    /* Last segment */
    uint8_t seg_n_length = p_sar_ctx->header.length - seg_n*TRANSPORT_SAR_SEGMENT_MAX_SIZE;
    status = sar_segment_send(p_sar_ctx, seg_n, seg_n, seg_n_length);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_sar_ctx->header.seg_left = 0;
    return NRF_SUCCESS;
}

static void sar_ctx_free(trs_sar_ctx_t * p_sar_ctx)
{
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_TX_COMPLETE,
            .params.tx_complete.packet_id = (uint32_t) p_sar_ctx->payload
        };
    event_handle(&evt);

    if (p_sar_ctx->payload != NULL)
    {
        m_sar_release(p_sar_ctx->payload);
        p_sar_ctx->payload = NULL;
    }

    memset(p_sar_ctx, 0, sizeof(trs_sar_ctx_t));
}

/** Process ongoing SAR TX sessions. */
static uint32_t trs_sar_tx_process(void)
{
    uint32_t status = NRF_SUCCESS;

    if (!m_trs_sar_tx_timeout)
    {
        return NRF_SUCCESS;
    }

    m_trs_sar_tx_timeout = false;

    trs_sar_ctx_t * p_sar_ctx = NULL;
    bool update_timer = false;
    /* Loop through all active sessions and (re-)transmit remaining packets. */
    for (int i = 0; i < TRANSPORT_SAR_TX_SESSIONS_MAX; ++i)
    {
        p_sar_ctx = &m_trs_sar_tx_sessions[i];
        if (!p_sar_ctx->header.session_active)
        {
            continue;
        }

        if (p_sar_ctx->header.block_ack == 0)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "All segments acked!\n");
            sar_ctx_free(p_sar_ctx);
            continue;
        }
        else
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Blocks remaining 0x%0x\n", p_sar_ctx->header.block_ack);
        }

        status = trs_sar_pkt_out(p_sar_ctx);
        if (status == NRF_SUCCESS)
        {
            /* Check if session was dropped*/
            if (!p_sar_ctx->header.session_active)
            {
                continue;
            }
            /* All messages sent (queued), update timer. */
            else
            {
                update_timer = true;
            }
        }
        else if (status == NRF_ERROR_NO_MEM || status == NRF_ERROR_BUSY)
        {
            update_timer = true;
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "No memory for sending SAR message or bearer was busy\n");
        }
        else
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Unable to send SAR message %u\n", status);

            if (p_sar_ctx->payload != NULL)
            {
                m_sar_release(p_sar_ctx->payload);
                p_sar_ctx->payload = NULL;
            }
            return status;
        }

    }

    if (update_timer)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Rescheduling TX timer\n");
        timer_sch_reschedule(&m_trs_sar_tx_timer, timer_now() + m_trs_config.tx_retry_timeout);
    }

    return NRF_SUCCESS;
}

static uint32_t transport_pkt_decrypt(packet_mesh_t * p_mesh_packet,
                                      const nrf_mesh_network_secmat_t * p_net_secmat,
                                      const nrf_mesh_application_secmat_t ** pp_app_secmat,
                                      nrf_mesh_address_t * p_address,
                                      uint32_t iv_index)
{
    uint8_t aid = packet_mesh_trs_aid_get(p_mesh_packet);
    uint8_t akf = packet_mesh_trs_akf_get(p_mesh_packet);

    /* Calculate nonce */
    uint8_t app_nonce_buf[CCM_NONCE_LENGTH];
    enc_nonce_generate((packet_net_hdr_t*) &p_mesh_packet->pdu[0],
                       LE2BE32(iv_index),
                       ((akf == 1) ? ENC_NONCE_APP : ENC_NONCE_DEV), 0,
                       app_nonce_buf);
    ccm_soft_data_t ccm_data;
    ccm_data.p_nonce = app_nonce_buf;
    ccm_data.p_m     = packet_mesh_unseg_payload_get(p_mesh_packet);
    ccm_data.m_len   = (packet_mesh_unseg_pdu_length_get(p_mesh_packet)
                        - BIT2MICSIZE(0)   /* Transport MIC */
                        - BIT2MICSIZE(0)); /* Network MIC */
    ccm_data.p_mic   = (uint8_t *) ccm_data.p_m + ccm_data.m_len;
    ccm_data.p_out   = m_transport_pkt_buf;
    ccm_data.mic_len = BIT2MICSIZE(0);

    if (p_address->type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        ccm_data.p_a   = p_address->p_virtual_uuid;
        ccm_data.a_len = NRF_MESH_UUID_SIZE;
    }
    else
    {
        ccm_data.a_len = 0;
    }

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Using app key\n");
    if (akf)
    {
        /* Application key */
        *pp_app_secmat = NULL;
        for (nrf_mesh_app_secmat_next_get(p_net_secmat, aid, pp_app_secmat);
             *pp_app_secmat != NULL;
             nrf_mesh_app_secmat_next_get(p_net_secmat, aid, pp_app_secmat))
        {
            if (test_transport_decrypt(pp_app_secmat, &ccm_data))
            {
                return NRF_SUCCESS;
            }
        }
    }
    else
    {
        *pp_app_secmat = NULL;
        if (p_address->type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            nrf_mesh_devkey_secmat_get(p_address->value, pp_app_secmat);
            if (test_transport_decrypt(pp_app_secmat, &ccm_data))
            {
                return NRF_SUCCESS;
            }
        }

        /* try the src address */
        *pp_app_secmat = NULL;
        nrf_mesh_devkey_secmat_get(packet_mesh_net_src_get(p_mesh_packet), pp_app_secmat);
        if (test_transport_decrypt(pp_app_secmat, &ccm_data))
        {
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NOT_FOUND;
}

static void transport_pkt_encrypt(packet_mesh_t * p_mesh_packet,
                                  const nrf_mesh_address_t * p_dst,
                                  const nrf_mesh_application_secmat_t * p_app_secmat)
{
    NRF_MESH_ASSERT(p_mesh_packet != NULL);

    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_app_secmat->key;

    uint8_t app_nonce_buf[CCM_NONCE_LENGTH];
    uint32_t iv_index = net_state_tx_iv_index_get();
    enc_nonce_generate((packet_net_hdr_t *) &p_mesh_packet->pdu[0],
                       LE2BE32(iv_index),
                       (packet_mesh_trs_akf_get(p_mesh_packet) == 1 ? ENC_NONCE_APP : ENC_NONCE_DEV),
                       0, app_nonce_buf);

    /* Encryption length is the payload size minus the size of the MIC_app and
     * MIC_net. The address of MIC_app is right after the app_payload:
     *
     * [payload |   MIC_app  |   MIC_net  ]
     * |--------|---- 32 ----|---- 32 ----|
     * |--------- payload_length ---------|
     */
    ccm_data.p_nonce = app_nonce_buf;
    ccm_data.p_m     = packet_mesh_unseg_payload_get(p_mesh_packet);
    ccm_data.m_len   = (packet_mesh_unseg_pdu_length_get(p_mesh_packet)
                        - BIT2MICSIZE(0)   /* Transport MIC */
                        - BIT2MICSIZE(0)); /* Network MIC */
    ccm_data.p_mic   = (uint8_t *) ccm_data.p_m + ccm_data.m_len;
    ccm_data.p_out   = (uint8_t *) ccm_data.p_m;                            /* Inline crypto. */
    ccm_data.mic_len = BIT2MICSIZE(0);

    if (p_dst->type == NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
        p_dst->p_virtual_uuid != NULL)
    {
        ccm_data.p_a   = p_dst->p_virtual_uuid;
        ccm_data.a_len = NRF_MESH_KEY_SIZE;
    }
    else
    {
        ccm_data.a_len = 0;
    }

    enc_aes_ccm_encrypt(&ccm_data);
}

/**************
 * Public API *
 **************/
void transport_init(const nrf_mesh_init_params_t * p_init_params)
{
    m_trs_sar_ack_timeout = false;
    m_trs_sar_tx_timeout  = false;
    m_trs_sar_ack_timer_active = false;
    transport_sar_mem_funcs_reset();

    memset(&m_trs_sar_tx_sessions[0], 0, sizeof(m_trs_sar_tx_sessions));
    memset(&m_trs_sar_rx_sessions[0], 0, sizeof(m_trs_sar_rx_sessions));

    memset(&m_trs_sar_tx_timer, 0, sizeof(timer_event_t));
    m_trs_sar_tx_timer.cb = trs_sar_tx_timeout_cb;

    memset(&m_trs_sar_rx_ack_timer, 0, sizeof(timer_event_t));
    m_trs_sar_rx_ack_timer.cb = trs_sar_ack_timeout_cb;

    memset(&m_trs_sar_session_timer, 0, sizeof(timer_event_t));
    m_trs_sar_session_timer.cb = trs_sar_session_cb;

    replay_cache_init();

    m_trs_config.rx_timeout       = TRANSPORT_SAR_RX_TIMEOUT_DEFAULT;
    m_trs_config.rx_ack_timeout   = TRANSPORT_SAR_RX_ACK_TIMEOUT_DEFAULT;
    m_trs_config.tx_retry_timeout = TRANSPORT_SAR_TX_RETRY_TIMEOUT_DEFAULT;
    m_trs_config.tx_timeout       = TRANSPORT_SAR_TX_TIMEOUT_DEFAULT;
    m_trs_config.tx_retries       = TRANSPORT_SAR_TX_RETRIES_DEFAULT;
    m_trs_config.szmic            = 0;
    m_trs_config.segack_ttl       = TRANSPORT_SAR_SEGACK_TTL_DEFAULT;
}

uint32_t transport_sar_mem_funcs_set(transport_sar_alloc_t alloc_func, transport_sar_release_t release_func)
{
    if ((alloc_func == NULL) != (release_func == NULL)) /*lint !e731 Boolean arguments to equal/not equal operator */
    {
        /* Both functions must be NULL or non-NULL, but not a mix of both. */
        return NRF_ERROR_NULL;
    }
    else if (alloc_func == NULL && release_func == NULL)
    {
        transport_sar_mem_funcs_reset();
        return NRF_SUCCESS;
    }
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    m_sar_alloc = alloc_func;
    m_sar_release = release_func;
    _ENABLE_IRQS(was_masked);
    return NRF_SUCCESS;
}

void transport_sar_mem_funcs_reset(void)
{
    NRF_MESH_ERROR_CHECK(transport_sar_mem_funcs_set(malloc, free));
}

uint32_t transport_pkt_in(packet_net_t * p_net_packet,
                          const packet_meta_t * p_packet_meta,
                          const nrf_mesh_network_secmat_t * p_net_secmat,
                          uint32_t iv_index)
{
    uint32_t status = NRF_SUCCESS;
    uint16_t rx_addr_le;
    uint16_t src_addr;

    packet_mesh_t * p_mesh_packet = (packet_mesh_t *) p_net_packet;

    if (p_net_packet == NULL || p_packet_meta == NULL || p_net_secmat == NULL || p_packet_meta->p_addr == NULL)
    {
        return NRF_ERROR_NULL;
    }

    rx_addr_le = packet_mesh_net_dst_get(p_mesh_packet);
    src_addr = packet_mesh_net_src_get(p_mesh_packet);

    if (nrf_mesh_address_type_get(rx_addr_le) == NRF_MESH_ADDRESS_TYPE_INVALID ||
        nrf_mesh_address_type_get(src_addr) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_INVALID_ADDRESS, p_net_packet->length+1, p_net_packet);
        return NRF_ERROR_INVALID_ADDR;
    }

    nrf_mesh_address_t rx_address = { NRF_MESH_ADDRESS_TYPE_INVALID };
    if (!nrf_mesh_rx_address_get(rx_addr_le, &rx_address))
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_UNKNOWN_ADDRESS, p_net_packet->length+1, p_net_packet);
        status = network_pkt_relay(p_net_packet, p_net_secmat);
        return status;
    }

    if (replay_cache_has_elem(src_addr,
                              packet_mesh_net_seq_get(p_mesh_packet),
                              packet_mesh_net_ivi_get(p_mesh_packet)))
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Packet in replay protection cache.\n");
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_REPLAY_CACHE, p_net_packet->length+1, p_net_packet);
        status = NRF_SUCCESS;
        return status;
    }

    if (rx_address.type == NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        status = network_pkt_relay(p_net_packet, p_net_secmat);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Error %u: Unable to relay network packet. Continuing...\n",
                  status);
        }
        else
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_RELAYED, 0, p_net_packet->length+1, p_net_packet);
        }
    }

    const nrf_mesh_application_secmat_t * p_app_secmat = NULL;
    if (is_unseg_access_packet(p_mesh_packet))
    {
        status = transport_pkt_decrypt(p_mesh_packet, p_net_secmat, &p_app_secmat, &rx_address, iv_index);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_DBG2, "Could not decrypt transport layer data.\n");

            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                                  (packet_mesh_trs_akf_get(p_mesh_packet) == 1 ? PACKET_DROPPED_INVALID_APPKEY : PACKET_DROPPED_INVALID_DEVKEY), p_net_packet->length+1, p_net_packet);
            return status;
        }
        else
        {/* IOP TESTING */
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_DECRYPT_TRS, 0,  p_mesh_packet->length+1, p_mesh_packet);
        }
    }
    else if (is_seg_access_packet(p_mesh_packet))
    {
        return trs_sar_seg_pkt_in(p_mesh_packet, p_net_secmat);
    }
    else if (is_seg_ack_packet(p_mesh_packet))
    {
        return trs_sar_segack_pkt_in(p_mesh_packet);
    }
    else
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Unknown transport packet type.\n");
        return NRF_ERROR_NOT_SUPPORTED;
    }

    status = replay_cache_add(src_addr,
                              packet_mesh_net_seq_get(p_mesh_packet),
                              packet_mesh_net_ivi_get(p_mesh_packet));
    if (status != NRF_SUCCESS)
    {
        m_send_replay_cache_full_event(src_addr,
                                       packet_mesh_net_ivi_get(p_mesh_packet),
                                       NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
        return status;
    }

    uint8_t app_payload_size = (packet_mesh_unseg_pdu_length_get(p_mesh_packet)
                                - BIT2MICSIZE(0)    /* Network */
                                - BIT2MICSIZE(0));  /* Transport */

    memcpy(packet_mesh_unseg_payload_get(p_mesh_packet),
           m_transport_pkt_buf,
           app_payload_size);

    nrf_mesh_address_t src_address =
        {
            .type = NRF_MESH_ADDRESS_TYPE_UNICAST,
            .value = src_addr,
            .p_virtual_uuid = NULL
        };
    nrf_mesh_evt_t rx_event;
    rx_event.type                         = NRF_MESH_EVT_MESSAGE_RECEIVED;
    rx_event.params.message.p_buffer      = packet_mesh_unseg_payload_get(p_mesh_packet);
    rx_event.params.message.length        = app_payload_size;
    rx_event.params.message.src           = src_address;
    rx_event.params.message.dst           = rx_address;
    rx_event.params.message.ttl           = packet_mesh_net_ttl_get(p_mesh_packet);
    rx_event.params.message.rssi          = p_packet_meta->rssi;
    rx_event.params.message.secmat.p_net  = p_net_secmat;
    rx_event.params.message.secmat.p_app  = p_app_secmat;

    rx_event.params.message.adv_addr.addr_type = nrf_mesh_gap_address_type_get(p_packet_meta->p_addr, p_packet_meta->addr_type);
    memcpy(rx_event.params.message.adv_addr.addr, p_packet_meta->p_addr, BLE_GAP_ADDR_LEN);

    event_handle(&rx_event);
    return NRF_SUCCESS;
}

uint32_t transport_sar_process(void)
{
    uint32_t status;

    status = trs_sar_rx_process();
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = trs_sar_tx_process();

    return status;
}

uint32_t transport_tx(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference)
{
    if (p_params == NULL ||
        p_params->p_data == NULL ||
        p_params->security_material.p_app == NULL ||
        p_params->security_material.p_net == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Fetch the seqnum early, as the consequence and probabilty of failure is
     * small */
    uint32_t seq;
    uint32_t status = net_state_seqnum_alloc(&seq);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Could not allocate sequence number for packet!\n");
        return status;
    }

    packet_t * p_buffer;
    packet_mesh_t * p_mesh_packet;
    /* Allocate a buffer to use for sending the packet down the stack. */
    status = packet_mgr_alloc(((packet_generic_t **) &p_buffer), BLE_ADV_PACKET_MAX_LENGTH);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (p_packet_reference != NULL)
    {
        *p_packet_reference = (uint32_t) p_buffer;
    }

    uint32_t total_payload_size = (p_params->data_len
                                   + PACKET_MESH_UNSEG_OVERHEAD_SIZE
                                   + BIT2MICSIZE(m_trs_config.szmic) /* Transport MIC */
                                   + BIT2MICSIZE(0));                /* Network MIC */
    NRF_MESH_ASSERT(total_payload_size <= BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);

    /* Clear header */
    memset(p_buffer->payload, 0, PACKET_MESH_UNSEG_OVERHEAD_SIZE);

    packet_ad_length_set(p_buffer, total_payload_size - 1);
    packet_payload_size_set(p_buffer, total_payload_size);
    packet_ad_type_set(p_buffer, AD_TYPE_MESH);

    /* Copy packet data and add headers from the packet parameter struct: */
    p_mesh_packet = (packet_mesh_t *) p_buffer->payload;
    memcpy(packet_mesh_unseg_payload_get(p_mesh_packet), p_params->p_data, p_params->data_len);

    /* CTL is zero */
    /* MD  is zero */
    packet_mesh_net_ttl_set(p_mesh_packet, p_params->ttl);
    packet_mesh_net_src_set(p_mesh_packet, p_params->src);
    packet_mesh_net_dst_set(p_mesh_packet, p_params->dst.value);
    packet_mesh_trs_akf_set(p_mesh_packet, !p_params->security_material.p_app->is_device_key);
    packet_mesh_trs_aid_set(p_mesh_packet, p_params->security_material.p_app->aid);
    packet_mesh_net_seq_set(p_mesh_packet, seq);

    transport_pkt_encrypt(p_mesh_packet,
            &p_params->dst,
            p_params->security_material.p_app);

    status = network_pkt_out(p_buffer, p_params->security_material.p_net, true);
    if (status != NRF_SUCCESS)
    {
        packet_mgr_free(p_buffer);
    }
    return status;
}

uint32_t transport_tx_sar(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference)
{
    uint32_t status = NRF_SUCCESS;

    if (p_packet_reference != NULL)
    {
        *p_packet_reference = 0;
    }

    NRF_MESH_ASSERT((p_params->data_len < transport_seg_maxlen_get() &&
                     p_params->data_len >= transport_unseg_maxlen_get())
                    ||
                    (p_params->data_len < transport_unseg_maxlen_get() &&
                     p_params->reliable));

    trs_sar_ctx_t * p_sar_ctx = NULL;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked); //TODO: Can this be changed to bearer_event_critical_section, or removed all-together?
    for (int i = 0; i < TRANSPORT_SAR_TX_SESSIONS_MAX; ++i)
    {
        if (!m_trs_sar_tx_sessions[i].header.session_active)
        {
            p_sar_ctx = &m_trs_sar_tx_sessions[i];
            p_sar_ctx->header.session_active = 1;
            p_sar_ctx->header.session_type   = TRS_SAR_SESSION_TX;
            break;
        }
    }
    _ENABLE_IRQS(was_masked);

    if (p_sar_ctx == NULL)
    {
        status = NRF_ERROR_NO_MEM;
        return status;
    }

    NRF_MESH_ASSERT(p_sar_ctx->payload == NULL);
    p_sar_ctx->payload = m_sar_alloc(p_params->data_len + BIT2MICSIZE(m_trs_config.szmic));

    if (p_sar_ctx->payload == NULL)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Unable to allocate memory %u\n", status);
        p_sar_ctx->header.session_active = 0;
        return status;
    }

    if (p_packet_reference != NULL)
    {
        *p_packet_reference = (uint32_t) p_sar_ctx->payload;
    }

    enc_nonce_app_t * p_nonce;

    /* Setup header */
    p_sar_ctx->p_net_secmat = p_params->security_material.p_net;

    uint32_t num_segments = ((p_params->data_len
                              + BIT2MICSIZE(p_sar_ctx->header.szmic)
                              + TRANSPORT_SAR_SEGMENT_MAX_SIZE - 1)
                             / TRANSPORT_SAR_SEGMENT_MAX_SIZE);

    /* Keep configuration consistent across session. */
    net_state_iv_index_lock(true);
    p_sar_ctx->header.szmic = m_trs_config.szmic;
    p_sar_ctx->header.ttl = p_params->ttl;
    p_sar_ctx->header.block_ack = (0xFFFFFFFF >> (32 - num_segments));

    uint32_t iv_index = net_state_tx_iv_index_get();
    uint32_t seqnum;
    status = net_state_seqnum_alloc(&seqnum);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Could not allocate sequence number for transaction!\n");
        goto _trs_sar_exit;
    }

    p_sar_ctx->header.seq_auth = seqnum;
    p_sar_ctx->header.src = p_params->src;
    p_sar_ctx->header.dst = p_params->dst.value;

    p_sar_ctx->header.length = p_params->data_len + BIT2MICSIZE(p_sar_ctx->header.szmic);
    p_sar_ctx->header.ivi = iv_index & NETWORK_IVI_MASK;
    p_sar_ctx->header.seg_left = 0;
    p_sar_ctx->header.akf = !p_params->security_material.p_app->is_device_key;
    p_sar_ctx->header.aid = p_params->security_material.p_app->aid;

    /* Encryption */
    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_params->security_material.p_app->key;

    uint8_t app_nonce_buf[CCM_NONCE_LENGTH];
    p_nonce = (enc_nonce_app_t *) app_nonce_buf;

    p_nonce->type  = ((p_params->security_material.p_app->is_device_key) ?
                      ENC_NONCE_DEV : ENC_NONCE_APP);
    p_nonce->aszmic   = p_sar_ctx->header.szmic;
    p_nonce->padding  = 0;
    p_nonce->seq      = LE2BE24(p_sar_ctx->header.seq_auth);
    p_nonce->src      = LE2BE16(p_sar_ctx->header.src);
    p_nonce->dst      = LE2BE16(p_sar_ctx->header.dst);
    p_nonce->iv_index = LE2BE32(iv_index);


    if (p_params->dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        ccm_data.p_a   = p_params->dst.p_virtual_uuid;
        ccm_data.a_len = NRF_MESH_KEY_SIZE;
    }
    else
    {
        ccm_data.p_a   = NULL;
        ccm_data.a_len = 0;
    }

    ccm_data.p_m     = p_params->p_data;
    ccm_data.p_out   = &p_sar_ctx->payload[0];
    ccm_data.m_len   = p_params->data_len;
    ccm_data.p_mic   = &p_sar_ctx->payload[p_params->data_len];
    ccm_data.mic_len = BIT2MICSIZE(p_sar_ctx->header.szmic);
    ccm_data.p_nonce = app_nonce_buf;
    __LOG_XB(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "SAR App nonce ", ccm_data.p_nonce, sizeof(enc_nonce_app_t));

    enc_aes_ccm_encrypt(&ccm_data);

    /* Force resend. */
    m_trs_sar_tx_timeout = true;

    p_sar_ctx->header.retries = m_trs_config.tx_retries;
    p_sar_ctx->header.timeout = timer_now() + m_trs_config.tx_timeout;
    trs_sar_session_timer_order(p_sar_ctx);

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Successfully created SAR packet\n");
    __LOG_XB(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "SAR packet: ", p_sar_ctx->payload, p_sar_ctx->header.length);
    return status;

_trs_sar_exit:
    m_sar_release(p_sar_ctx->payload);
    p_sar_ctx->payload = NULL;
    p_sar_ctx->header.session_active = 0;
    net_state_iv_index_lock(false);
    return status;
}

uint32_t transport_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt)
{
    if (!p_opt)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT:
            if (p_opt->opt.val < TRANSPORT_SAR_RX_TIMEOUT_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_RX_TIMEOUT_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.rx_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT:
            if (p_opt->opt.val < TRANSPORT_SAR_RX_ACK_TIMEOUT_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_RX_ACK_TIMEOUT_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.rx_ack_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT:
            if (p_opt->opt.val < TRANSPORT_SAR_TX_RETRY_TIMEOUT_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_TX_RETRY_TIMEOUT_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.tx_retry_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_TIMEOUT:
            if (p_opt->opt.val < TRANSPORT_SAR_TX_TIMEOUT_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_TX_TIMEOUT_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.tx_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRIES:
            if (p_opt->opt.val > TRANSPORT_SAR_TX_RETRIES_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.tx_retries = (uint8_t) p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_SEGACK_TTL:
            if (p_opt->opt.val > NRF_MESH_TTL_MAX || p_opt->opt.val < 1)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.segack_ttl = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SZMIC:
            if (p_opt->opt.val > 1)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.szmic = p_opt->opt.val;
            break;

        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t transport_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt)
{
    if (!p_opt)
    {
        return NRF_ERROR_NULL;
    }

    memset(p_opt, 0, sizeof(nrf_mesh_opt_t));

    switch (id)
    {
        case NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT:
            p_opt->opt.val = m_trs_config.rx_timeout;
            break;

        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT:
            p_opt->opt.val = m_trs_config.rx_ack_timeout;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT:
            p_opt->opt.val = m_trs_config.tx_retry_timeout;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_TIMEOUT:
            p_opt->opt.val = m_trs_config.tx_timeout;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRIES:
            p_opt->opt.val = m_trs_config.tx_retries;
            break;

        case NRF_MESH_OPT_TRS_SAR_SEGACK_TTL:
            p_opt->opt.val = m_trs_config.segack_ttl;
            break;

        case NRF_MESH_OPT_TRS_SZMIC:
            p_opt->opt.val = m_trs_config.szmic;
            break;

        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t transport_seg_maxlen_get(void)
{
    return TRANSPORT_SAR_PACKET_MAX_SIZE - BIT2MICSIZE(m_trs_config.szmic);
}

uint32_t transport_unseg_maxlen_get(void)
{
    return PACKET_MESH_UNSEG_PDU_MAX_SIZE - BIT2MICSIZE(0) - BIT2MICSIZE(0);
}
