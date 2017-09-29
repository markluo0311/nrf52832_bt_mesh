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
#ifndef AES_CMAC_H__
#define AES_CMAC_H__

#include <stdint.h>

/**
 * @defgroup AES_CMAC AES-CMAC software implementation.
 * @ingroup MESH_CORE
 * @{
 */

/**
 * Generates AES-CMAC Subkeys K1 and K2.
 *
 * @param p_key         Pointer to a 128-bit encryption key.
 * @param p_subkey1_out Pointer to a 128-bit buffer to store K1.
 * @param p_subkey2_out Pointer to a 128-bit buffer to store K2.
 */
void aes_cmac_subkey_generate(const uint8_t * const p_key, uint8_t * p_subkey1_out, uint8_t * p_subkey2_out);

/**
 * Performs an AES-CMAC operation.
 * @param p_key         Pointer to a 128-bit encryption key.
 * @param p_msg         Pointer to the data that should be hashed.
 * @param msg_len       Length of the input data.
 * @param p_out         Pointer to where the 128-bit result should be stored.
 */
void aes_cmac(const uint8_t * const p_key, const uint8_t * const p_msg, uint16_t msg_len, uint8_t * const p_out);

/** @} */
#endif
