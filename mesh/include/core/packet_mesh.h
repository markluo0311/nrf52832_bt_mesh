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

#ifndef PACKET_MESH_H__
#define PACKET_MESH_H__

/*lint -align_max(push) -align_max(1) */

/**
 * Generic mesh/advertising packet.
 */
typedef struct __attribute((packed))
{
    uint8_t length;  /**< The length of the ad_type + pdu.*/
    uint8_t ad_type; /**< Advertising type. */
    uint8_t pdu[];   /**< Mesh PDU. */
} packet_mesh_t;
/*lint -align_max(pop) */

#define PACKET_MESH_IVI_OFFSET   (0)       /**< Offset to the ivi field.*/
#define PACKET_MESH_IVI_MASK     (0x80)    /**< Mask for ivi field. */
#define PACKET_MESH_IVI_MASK_INV (0x7F)    /**< Inverse mask for ivi field. */

#define PACKET_MESH_NID_OFFSET   (0)       /**< Offset to the nid field.*/
#define PACKET_MESH_NID_MASK     (0x7F)    /**< Mask for nid field. */
#define PACKET_MESH_NID_MASK_INV (0x80)    /**< Inverse mask for nid field. */

#define PACKET_MESH_CTL_OFFSET   (1)       /**< Offset to the ctl field.*/
#define PACKET_MESH_CTL_MASK     (0x80)    /**< Mask for ctl field. */
#define PACKET_MESH_CTL_MASK_INV (0x7F)    /**< Inverse mask for ctl field. */

#define PACKET_MESH_TTL_OFFSET   (1)       /**< Offset to the ttl field.*/
#define PACKET_MESH_TTL_MASK     (0x7F)    /**< Mask for ttl field. */
#define PACKET_MESH_TTL_MASK_INV (0x80)    /**< Inverse mask for ttl field. */

#define PACKET_MESH_SEQ0_OFFSET   (2)       /**< Offset to the seq field (0).*/
#define PACKET_MESH_SEQ1_OFFSET   (3)       /**< Offset to the seq field (1).*/
#define PACKET_MESH_SEQ2_OFFSET   (4)       /**< Offset to the seq field (2).*/

#define PACKET_MESH_SRC0_OFFSET   (5)       /**< Offset to the src field (0).*/
#define PACKET_MESH_SRC1_OFFSET   (6)       /**< Offset to the src field (1).*/

#define PACKET_MESH_DST0_OFFSET   (7)       /**< Offset to the dst field (0).*/
#define PACKET_MESH_DST1_OFFSET   (8)       /**< Offset to the dst field (1).*/

#define PACKET_MESH_SEG_OFFSET   (9)       /**< Offset to the seg field.*/
#define PACKET_MESH_SEG_MASK     (0x80)    /**< Mask for seg field. */
#define PACKET_MESH_SEG_MASK_INV (0x7F)    /**< Inverse mask for seg field. */

#define PACKET_MESH_AKF_OFFSET   (9)       /**< Offset to the akf field.*/
#define PACKET_MESH_AKF_MASK     (0x40)    /**< Mask for akf field. */
#define PACKET_MESH_AKF_MASK_INV (0xBF)    /**< Inverse mask for akf field. */

#define PACKET_MESH_AID_OFFSET   (9)       /**< Offset to the aid field.*/
#define PACKET_MESH_AID_MASK     (0x3F)    /**< Mask for aid field. */
#define PACKET_MESH_AID_MASK_INV (0xC0)    /**< Inverse mask for aid field. */

#define PACKET_MESH_SZMIC_OFFSET   (10)      /**< Offset to the szmic field.*/
#define PACKET_MESH_SZMIC_MASK     (0x80)    /**< Mask for szmic field. */
#define PACKET_MESH_SZMIC_MASK_INV (0x7F)    /**< Inverse mask for szmic field. */

#define PACKET_MESH_SEQZERO0_OFFSET   (10)      /**< Offset to the seqzero field (0).*/
#define PACKET_MESH_SEQZERO1_OFFSET   (11)      /**< Offset to the seqzero field (1).*/
#define PACKET_MESH_SEQZERO0_MASK     (0x7F)    /**< Mask for seqzero field (0). */
#define PACKET_MESH_SEQZERO1_MASK     (0xFC)    /**< Mask for seqzero field (1). */
#define PACKET_MESH_SEQZERO0_MASK_INV (0x80)    /**< Inverse mask for seqzero field (0). */
#define PACKET_MESH_SEQZERO1_MASK_INV (0x03)    /**< Inverse mask for seqzero field (1). */

#define PACKET_MESH_SEGO0_OFFSET   (11)      /**< Offset to the sego field (0).*/
#define PACKET_MESH_SEGO1_OFFSET   (12)      /**< Offset to the sego field (1).*/
#define PACKET_MESH_SEGO0_MASK     (0x03)    /**< Mask for sego field (0). */
#define PACKET_MESH_SEGO1_MASK     (0xE0)    /**< Mask for sego field (1). */
#define PACKET_MESH_SEGO0_MASK_INV (0xFC)    /**< Inverse mask for sego field (0). */
#define PACKET_MESH_SEGO1_MASK_INV (0x1F)    /**< Inverse mask for sego field (1). */

#define PACKET_MESH_SEGN_OFFSET   (12)      /**< Offset to the segn field.*/
#define PACKET_MESH_SEGN_MASK     (0x1F)    /**< Mask for segn field. */
#define PACKET_MESH_SEGN_MASK_INV (0xE0)    /**< Inverse mask for segn field. */

#define PACKET_MESH_OPCODE_OFFSET   (9)       /**< Offset to the opcode field.*/
#define PACKET_MESH_OPCODE_MASK     (0x7F)    /**< Mask for opcode field. */
#define PACKET_MESH_OPCODE_MASK_INV (0x80)    /**< Inverse mask for opcode field. */

#define PACKET_MESH_OBO_OFFSET   (10)      /**< Offset to the obo field.*/
#define PACKET_MESH_OBO_MASK     (0x80)    /**< Mask for obo field. */
#define PACKET_MESH_OBO_MASK_INV (0x7F)    /**< Inverse mask for obo field. */

#define PACKET_MESH_BLOCK_ACK0_OFFSET   (12)      /**< Offset to the block_ack field (0).*/
#define PACKET_MESH_BLOCK_ACK1_OFFSET   (13)      /**< Offset to the block_ack field (1).*/
#define PACKET_MESH_BLOCK_ACK2_OFFSET   (14)      /**< Offset to the block_ack field (2).*/
#define PACKET_MESH_BLOCK_ACK3_OFFSET   (15)      /**< Offset to the block_ack field (3).*/

#define PACKET_MESH_UNSEG_PDU_OFFSET    (10)    /**< Offset to unseg payload. */
#define PACKET_MESH_UNSEG_PDU_MAX_SIZE  (19)    /**< Max PDU size for unseg packets. */
#define PACKET_MESH_UNSEG_OVERHEAD_SIZE (12)    /**< Header overhead size for unseg packets. */
#define PACKET_MESH_SEG_PDU_OFFSET    (13)    /**< Offset to seg payload. */
#define PACKET_MESH_SEG_PDU_MAX_SIZE  (16)    /**< Max PDU size for seg packets. */
#define PACKET_MESH_SEG_OVERHEAD_SIZE (15)    /**< Header overhead size for seg packets. */
#define PACKET_MESH_SEG_ACK_SIZE (26)    /**< Size of seg_ack packet. */ 

/**
 * Gets the unseg payload pointer.
 * @param[in,out] p_pkt Packet pointer.
 * @returns Pointer to the start of the upper transport PDU.
 */
static inline uint8_t * packet_mesh_unseg_payload_get(packet_mesh_t * p_pkt)
{
    return &p_pkt->pdu[PACKET_MESH_UNSEG_PDU_OFFSET];
}

/**
 * Gets the length of the upper transport PDU.
 * @param[in] p_pkt Packet pointer.
 * @returns Length of the upper transport PDU.
 */
static inline uint8_t packet_mesh_unseg_pdu_length_get(const packet_mesh_t * p_pkt)
{
    return p_pkt->length - sizeof(p_pkt->ad_type) - PACKET_MESH_UNSEG_PDU_OFFSET;
}

/**
 * Gets the seg payload pointer.
 * @param[in,out] p_pkt Packet pointer.
 * @returns Pointer to the start of the upper transport PDU.
 */
static inline uint8_t * packet_mesh_seg_payload_get(packet_mesh_t * p_pkt)
{
    return &p_pkt->pdu[PACKET_MESH_SEG_PDU_OFFSET];
}

/**
 * Gets the length of the upper transport PDU.
 * @param[in] p_pkt Packet pointer.
 * @returns Length of the upper transport PDU.
 */
static inline uint8_t packet_mesh_seg_pdu_length_get(const packet_mesh_t * p_pkt)
{
    return p_pkt->length - sizeof(p_pkt->ad_type) - PACKET_MESH_SEG_PDU_OFFSET;
}
/**
 * Gets the IV index least significant bit.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the IV index least significant bit.
 */
static inline uint8_t packet_mesh_net_ivi_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_IVI_OFFSET] & PACKET_MESH_IVI_MASK) > 0);
}

/**
 * Sets the IV index least significant bit.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the IV index least significant bit.
 */
static inline void packet_mesh_net_ivi_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_IVI_OFFSET] &= PACKET_MESH_IVI_MASK_INV;
    p_pkt->pdu[PACKET_MESH_IVI_OFFSET] |= (val << 7) & PACKET_MESH_IVI_MASK;
}

/**
 * Gets the least significant bits of the Network Identifier.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the least significant bits of the Network Identifier.
 */
static inline uint8_t packet_mesh_net_nid_get(const packet_mesh_t * p_pkt)
{
    return (p_pkt->pdu[PACKET_MESH_NID_OFFSET] & PACKET_MESH_NID_MASK);
}

/**
 * Sets the least significant bits of the Network Identifier.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the least significant bits of the Network Identifier.
 */
static inline void packet_mesh_net_nid_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_NID_OFFSET] &= PACKET_MESH_NID_MASK_INV;
    p_pkt->pdu[PACKET_MESH_NID_OFFSET] |= (val & PACKET_MESH_NID_MASK);
}

/**
 * Gets the control message bit.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the control message bit.
 */
static inline uint8_t packet_mesh_net_ctl_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_CTL_OFFSET] & PACKET_MESH_CTL_MASK) > 0);
}

/**
 * Sets the control message bit.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the control message bit.
 */
static inline void packet_mesh_net_ctl_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_CTL_OFFSET] &= PACKET_MESH_CTL_MASK_INV;
    p_pkt->pdu[PACKET_MESH_CTL_OFFSET] |= (val << 7) & PACKET_MESH_CTL_MASK;
}

/**
 * Gets the time-to-live value.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the time-to-live value.
 */
static inline uint8_t packet_mesh_net_ttl_get(const packet_mesh_t * p_pkt)
{
    return (p_pkt->pdu[PACKET_MESH_TTL_OFFSET] & PACKET_MESH_TTL_MASK);
}

/**
 * Sets the time-to-live value.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the time-to-live value.
 */
static inline void packet_mesh_net_ttl_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_TTL_OFFSET] &= PACKET_MESH_TTL_MASK_INV;
    p_pkt->pdu[PACKET_MESH_TTL_OFFSET] |= (val & PACKET_MESH_TTL_MASK);
}

/**
 * Gets the message sequence number.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the message sequence number.
 */
static inline uint32_t packet_mesh_net_seq_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_SEQ0_OFFSET] << 16) | 
            (p_pkt->pdu[PACKET_MESH_SEQ1_OFFSET] << 8) |
            p_pkt->pdu[PACKET_MESH_SEQ2_OFFSET]);
}

/**
 * Sets the message sequence number.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the message sequence number.
 */
static inline void packet_mesh_net_seq_set(packet_mesh_t * p_pkt, uint32_t val)
{
    p_pkt->pdu[PACKET_MESH_SEQ0_OFFSET] = (val >> 16) & 0xFF;
    p_pkt->pdu[PACKET_MESH_SEQ1_OFFSET] = (val >> 8) & 0xFF;
    p_pkt->pdu[PACKET_MESH_SEQ2_OFFSET] = val & 0xFF;
}

/**
 * Gets the source address.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the source address.
 */
static inline uint16_t packet_mesh_net_src_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_SRC0_OFFSET] << 8) | 
            p_pkt->pdu[PACKET_MESH_SRC1_OFFSET]);
}

/**
 * Sets the source address.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the source address.
 */
static inline void packet_mesh_net_src_set(packet_mesh_t * p_pkt, uint16_t val)
{
    p_pkt->pdu[PACKET_MESH_SRC0_OFFSET] = (val >> 8) & 0xFF;
    p_pkt->pdu[PACKET_MESH_SRC1_OFFSET] = val & 0xFF;
}

/**
 * Gets the destination address.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the destination address.
 */
static inline uint16_t packet_mesh_net_dst_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_DST0_OFFSET] << 8) | 
            p_pkt->pdu[PACKET_MESH_DST1_OFFSET]);
}

/**
 * Sets the destination address.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the destination address.
 */
static inline void packet_mesh_net_dst_set(packet_mesh_t * p_pkt, uint16_t val)
{
    p_pkt->pdu[PACKET_MESH_DST0_OFFSET] = (val >> 8) & 0xFF;
    p_pkt->pdu[PACKET_MESH_DST1_OFFSET] = val & 0xFF;
}

/**
 * Gets the segmentation bit.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the segmentation bit.
 */
static inline uint8_t packet_mesh_trs_seg_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_SEG_OFFSET] & PACKET_MESH_SEG_MASK) > 0);
}

/**
 * Sets the segmentation bit.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the segmentation bit.
 */
static inline void packet_mesh_trs_seg_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_SEG_OFFSET] &= PACKET_MESH_SEG_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEG_OFFSET] |= (val << 7) & PACKET_MESH_SEG_MASK;
}

/**
 * Gets the application key flag.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the application key flag.
 */
static inline uint8_t packet_mesh_trs_akf_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_AKF_OFFSET] & PACKET_MESH_AKF_MASK) > 0);
}

/**
 * Sets the application key flag.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the application key flag.
 */
static inline void packet_mesh_trs_akf_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_AKF_OFFSET] &= PACKET_MESH_AKF_MASK_INV;
    p_pkt->pdu[PACKET_MESH_AKF_OFFSET] |= (val << 6) & PACKET_MESH_AKF_MASK;
}

/**
 * Gets the application key identifier.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the application key identifier.
 */
static inline uint8_t packet_mesh_trs_aid_get(const packet_mesh_t * p_pkt)
{
    return (p_pkt->pdu[PACKET_MESH_AID_OFFSET] & PACKET_MESH_AID_MASK);
}

/**
 * Sets the application key identifier.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the application key identifier.
 */
static inline void packet_mesh_trs_aid_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_AID_OFFSET] &= PACKET_MESH_AID_MASK_INV;
    p_pkt->pdu[PACKET_MESH_AID_OFFSET] |= (val & PACKET_MESH_AID_MASK);
}

/**
 * Gets the size of MIC flag.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the size of MIC flag.
 */
static inline uint8_t packet_mesh_trs_szmic_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_SZMIC_OFFSET] & PACKET_MESH_SZMIC_MASK) > 0);
}

/**
 * Sets the size of MIC flag.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the size of MIC flag.
 */
static inline void packet_mesh_trs_szmic_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_SZMIC_OFFSET] &= PACKET_MESH_SZMIC_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SZMIC_OFFSET] |= (val << 7) & PACKET_MESH_SZMIC_MASK;
}

/**
 * Gets the least significant bits of SeqAuth.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the least significant bits of SeqAuth.
 */
static inline uint16_t packet_mesh_trs_seqzero_get(const packet_mesh_t * p_pkt)
{
    return (((p_pkt->pdu[PACKET_MESH_SEQZERO0_OFFSET] & PACKET_MESH_SEQZERO0_MASK) << 6) | 
            ((p_pkt->pdu[PACKET_MESH_SEQZERO1_OFFSET] & PACKET_MESH_SEQZERO1_MASK) >> 2));
}

/**
 * Sets the least significant bits of SeqAuth.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the least significant bits of SeqAuth.
 */
static inline void packet_mesh_trs_seqzero_set(packet_mesh_t * p_pkt, uint16_t val)
{
    p_pkt->pdu[PACKET_MESH_SEQZERO0_OFFSET] &= PACKET_MESH_SEQZERO0_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEQZERO0_OFFSET] |= (val >> 6) & PACKET_MESH_SEQZERO0_MASK;
    p_pkt->pdu[PACKET_MESH_SEQZERO1_OFFSET] &= PACKET_MESH_SEQZERO1_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEQZERO1_OFFSET] |= (val << 2) & PACKET_MESH_SEQZERO1_MASK;
}

/**
 * Gets the segment offset number.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the segment offset number.
 */
static inline uint8_t packet_mesh_trs_sego_get(const packet_mesh_t * p_pkt)
{
    return (((p_pkt->pdu[PACKET_MESH_SEGO0_OFFSET] & PACKET_MESH_SEGO0_MASK) << 3) |
             (p_pkt->pdu[PACKET_MESH_SEGO1_OFFSET] & PACKET_MESH_SEGO1_MASK) >> 5);
}

/**
 * Sets the segment offset number.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the segment offset number.
 */
static inline void packet_mesh_trs_sego_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_SEGO0_OFFSET] &= PACKET_MESH_SEGO0_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEGO0_OFFSET] |= (val >> 3) & PACKET_MESH_SEGO0_MASK;
    p_pkt->pdu[PACKET_MESH_SEGO1_OFFSET] &= PACKET_MESH_SEGO1_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEGO1_OFFSET] |= (val << 5) & PACKET_MESH_SEGO1_MASK;
}

/**
 * Gets the last segment number.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the last segment number.
 */
static inline uint8_t packet_mesh_trs_segn_get(const packet_mesh_t * p_pkt)
{
    return (p_pkt->pdu[PACKET_MESH_SEGN_OFFSET] & PACKET_MESH_SEGN_MASK);
}

/**
 * Sets the last segment number.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the last segment number.
 */
static inline void packet_mesh_trs_segn_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_SEGN_OFFSET] &= PACKET_MESH_SEGN_MASK_INV;
    p_pkt->pdu[PACKET_MESH_SEGN_OFFSET] |= (val & PACKET_MESH_SEGN_MASK);
}

/**
 * Gets the transport control opcode.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the transport control opcode.
 */
static inline uint8_t packet_mesh_trs_opcode_get(const packet_mesh_t * p_pkt)
{
    return (p_pkt->pdu[PACKET_MESH_OPCODE_OFFSET] & PACKET_MESH_OPCODE_MASK);
}

/**
 * Sets the transport control opcode.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the transport control opcode.
 */
static inline void packet_mesh_trs_opcode_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_OPCODE_OFFSET] &= PACKET_MESH_OPCODE_MASK_INV;
    p_pkt->pdu[PACKET_MESH_OPCODE_OFFSET] |= (val & PACKET_MESH_OPCODE_MASK);
}

/**
 * Gets the 'on behalf of' flag.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the 'on behalf of' flag.
 */
static inline uint8_t packet_mesh_trs_obo_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_OBO_OFFSET] & PACKET_MESH_OBO_MASK) > 0);
}

/**
 * Sets the 'on behalf of' flag.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the 'on behalf of' flag.
 */
static inline void packet_mesh_trs_obo_set(packet_mesh_t * p_pkt, uint8_t val)
{
    p_pkt->pdu[PACKET_MESH_OBO_OFFSET] &= PACKET_MESH_OBO_MASK_INV;
    p_pkt->pdu[PACKET_MESH_OBO_OFFSET] |= (val << 7) & PACKET_MESH_OBO_MASK;
}

/**
 * Gets the block acknowledgement field.
 * @param[in] p_pkt Packet pointer.
 * @returns Value of the block acknowledgement field.
 */
static inline uint32_t packet_mesh_trs_block_ack_get(const packet_mesh_t * p_pkt)
{
    return ((p_pkt->pdu[PACKET_MESH_BLOCK_ACK0_OFFSET] << 24) | 
            (p_pkt->pdu[PACKET_MESH_BLOCK_ACK1_OFFSET] << 16) |
            (p_pkt->pdu[PACKET_MESH_BLOCK_ACK2_OFFSET] << 8) |
            p_pkt->pdu[PACKET_MESH_BLOCK_ACK3_OFFSET]);
}

/**
 * Sets the block acknowledgement field.
 * @param[in,out] p_pkt Packet pointer.
 * @param[in]     val   Value of the block acknowledgement field.
 */
static inline void packet_mesh_trs_block_ack_set(packet_mesh_t * p_pkt, uint32_t val)
{
    p_pkt->pdu[PACKET_MESH_BLOCK_ACK0_OFFSET] = (val >> 24) & 0xFF;
    p_pkt->pdu[PACKET_MESH_BLOCK_ACK1_OFFSET] = (val >> 16) & 0xFF;
    p_pkt->pdu[PACKET_MESH_BLOCK_ACK2_OFFSET] = (val >> 8) & 0xFF;
    p_pkt->pdu[PACKET_MESH_BLOCK_ACK3_OFFSET] = val & 0xFF;
}

#endif
