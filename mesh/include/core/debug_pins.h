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
#ifndef DEBUG_PINS_H__
#define DEBUG_PINS_H__

#ifdef DEBUG_PINS_ENABLED

#define DEBUG_PIN0 16
#define DEBUG_PIN1 17
#define DEBUG_PIN2 18
#define DEBUG_PIN3 19
#define DEBUG_PIN4 20
#define DEBUG_PIN5 21
#define DEBUG_PIN6 22
#define DEBUG_PIN7 23

#define DEBUG_PIN_ON(p)               NRF_GPIO->OUTSET = (1UL << (p))
#define DEBUG_PIN_OFF(p)              NRF_GPIO->OUTCLR = (1UL << (p))
#define DEBUG_PIN_TOGGLE(p)           do{DEBUG_PIN_ON(p);__NOP();DEBUG_PIN_OFF(p);}while(0)
#define DEBUG_PIN_TOGGLE_NTIMES(p, n) do{uint8_t CTR = (n);do{DEBUG_PIN_TOGGLE(p);}while(CTR--);}while(0)

#ifdef DEBUG_PINS_PACKET_MGR_ENABLE

#define DEBUG_PACKET_MGR_ALLOC_BEGIN()          DEBUG_PIN_ON(DEBUG_PIN0)
#define DEBUG_PACKET_MGR_ALLOC_END()            DEBUG_PIN_OFF(DEBUG_PIN0)
#define DEBUG_PACKET_MGR_BLOCK_SEARCH_TOGGLE()  DEBUG_PIN_TOGGLE(DEBUG_PIN2)
#define DEBUG_PACKET_MGR_ALLOC_FAIL()           DEBUG_PIN_TOGGLE(DEBUG_PIN1)

#endif // DEBUG_PINS_PACKET_MGR_ENABLE

#endif //DEBUG_PINS_ENABLED

#ifndef DEBUG_PACKET_MGR_ALLOC_BEGIN
#define DEBUG_PACKET_MGR_ALLOC_BEGIN()
#endif
#ifndef DEBUG_PACKET_MGR_ALLOC_END
#define DEBUG_PACKET_MGR_ALLOC_END()
#endif
#ifndef DEBUG_PACKET_MGR_BLOCK_SEARCH_TOGGLE
#define DEBUG_PACKET_MGR_BLOCK_SEARCH_TOGGLE()
#endif
#ifndef DEBUG_PACKET_MGR_ALLOC_FAIL
#define DEBUG_PACKET_MGR_ALLOC_FAIL()
#endif
#endif //DEBUG_PINS_H__
