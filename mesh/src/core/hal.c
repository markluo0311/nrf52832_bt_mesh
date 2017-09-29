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

#include "hal.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_mesh_assert.h"
#if defined(S130) || defined(S132)
#include "nrf_nvic.h"
#elif defined(S110)
#include "nrf_soc.h"
#endif

/*****************************************************************************
* Local defines
*****************************************************************************/
/** Mask of all potential reset reasons in the NRF_POWER->RESETREAS hardware
 * register. */
#define RESET_REASON_MASK   (0xFFFFFFFF)

/*****************************************************************************
* Interface functions
*****************************************************************************/
void hal_device_reset(uint8_t gpregret_value)
{
#if defined(SOFTDEVICE_PRESENT)
    (void) sd_power_reset_reason_clr(RESET_REASON_MASK); /* avoid wrongful state-readout on reboot */
#if defined(S130) || defined(S110)
    (void) sd_power_gpregret_set(gpregret_value);
#elif defined(S132)
    (void) sd_power_gpregret_set(gpregret_value, RESET_REASON_MASK);
#endif
    (void) sd_nvic_SystemReset();
#else
    NRF_POWER->RESETREAS = RESET_REASON_MASK; /* avoid wrongful state-readout on reboot */
    NRF_POWER->GPREGRET = gpregret_value;
    NVIC_SystemReset();
#endif
    NRF_MESH_ASSERT(false);
}

