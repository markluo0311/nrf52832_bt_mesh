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
#ifndef MESH_TIMESLOT_H__
#define MESH_TIMESLOT_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "timer.h"
#include "nrf_sdm.h"
#include "nrf_mesh.h"

/**
 * @defgroup TIMESLOT Timeslot handler
 * @ingroup MESH_CORE
 *   Module responsible for providing a safe interface to Softdevice
 *   Timeslot API. Handles all system events, makes sure all timeslots are
 *   ended in time, provides some simple functions for manipulating the way
 *   timeslots behave.
 * @{
 */

/**
 * Event handler for softdevice events.
 *
 * @param[in] evt Softdevice event to process.
 */
void timeslot_sd_event_handler(uint32_t evt);

/**
 * Initialize timeslot handler.
 *
 * @param[in] p_init_params Global init parameter structure.
 *
 * @retval NRF_SUCCESS The timeslot module was successfully initialized.
 * @retval NRF_ERROR_INVALID_STATE The timeslot module has already been
 *         initialized.
 */
uint32_t timeslot_init(const nrf_mesh_init_params_t * p_init_params);

/** Forcibly stop the timeslot execution */
void timeslot_stop(void);

/** Restart the current timeslot. */
void timeslot_restart(void);

/**
 * Resume a stopped timeslot.
 *
 * @retval NRF_SUCCESS The timeslot was successfully resumed.
 * @retval NRF_ERROR_INVALID_STATE There is already a timesot in progress.
 */
uint32_t timeslot_resume(void);

/**
 * Get the timestamp sampled at the beginning of the timeslot.
 *
 * @return The start of the current timeslot.
 */
timestamp_t timeslot_start_time_get(void);

/**
 * Get the timestamp for the projected end of the current timeslot.
 *
 * @return The projected end of the current timeslot.
 */
timestamp_t timeslot_end_time_get(void);

/**
 * Get the remaining time of the current timeslot.
 *
 * @return The remaining time of the current timeslot.
 */
timestamp_t timeslot_remaining_time_get(void);

/**
 * Get whether the framework is currently in a timeslot.
 *
 * @return Whether the framework is currently in a timeslot.
 */
bool timeslot_is_in_ts(void);

/** @} */

#endif /* TIMESLOT_H__ */
