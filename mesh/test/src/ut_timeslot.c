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
#include <string.h>
#include <stdbool.h>

#include <unity.h>

#include <nrf_soc.h>
#include <nrf_error.h>

#include "mesh_flash.h"
#include "nrf_mesh.h"
#include "radio.h"
#include "timeslot.h"

NRF_RTC_Type*                       NRF_RTC0;
static NRF_RTC_Type                 m_rtc;
static uint32_t                     m_radio_event_handler_calls;
static uint32_t                     m_timer_event_handler_calls;
static uint32_t                     m_sd_radio_session_open_calls;
static uint32_t                     m_sd_radio_request_calls;
static nrf_radio_signal_callback_t  m_signal_cb;
static nrf_radio_request_t*         mp_radio_req;
static uint32_t                     m_ts_len;
static timer_callback_t             m_timer_cb;
static timestamp_t                  m_timeout;

uint32_t setRTC(uint64_t time_us);

nrf_mesh_assertion_handler_t m_assertion_handler;

void setUp(void)
{
    NRF_RTC0 = &m_rtc;
    m_radio_event_handler_calls     = 0;
    m_timer_event_handler_calls     = 0;
    m_sd_radio_session_open_calls   = 0;
    m_sd_radio_request_calls        = 0;
    mp_radio_req                    = 0;
    m_ts_len                        = 0;
    (void) setRTC(0);
}

void tearDown(void)
{

}

/******************************************/

/* TODO: Use CMock functions for mocking. */
/*lint -e522 Function lacks side effects */

uint32_t setRTC(uint64_t time_us)
{
    uint64_t time = ((time_us << 15) / 1000000);
#if defined(__linux__)
    printf("time: %llu\n", time);
#elif defined(_WIN32)
    printf("time: %I64u\n", time);
#endif
    memcpy((void*) &m_rtc.COUNTER, &time, 4);
    return (((uint64_t)time) * 1000000) >> 15;
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    TEST_FAIL();
}

void radio_event_handler(void)
{
    m_radio_event_handler_calls++;
}

void timer_event_handler(void)
{
    m_timer_event_handler_calls++;
    m_timer_cb(m_timeout);
}

timestamp_t timer_now(void)
{
    return 0;
}

uint32_t timer_abort(uint8_t index)
{
    return NRF_SUCCESS;
}

uint32_t timer_order_cb(uint8_t index, timestamp_t timestamp, timer_callback_t cb, timer_attr_t attrs)
{
    m_timer_cb = cb;
    m_timeout = timestamp;
    TEST_ASSERT_EQUAL((TIMER_ATTR_TIMESLOT_LOCAL | TIMER_ATTR_SYNCHRONOUS), attrs);
    return NRF_SUCCESS;
}

uint32_t sd_radio_session_open(nrf_radio_signal_callback_t p_cb)
{
    m_sd_radio_session_open_calls++;
    m_signal_cb = p_cb;
    return NRF_SUCCESS;
}

uint32_t sd_radio_session_close(void)
{
    return NRF_SUCCESS;
}

uint32_t sd_radio_request(nrf_radio_request_t* p_req)
{
    m_sd_radio_request_calls++;
    mp_radio_req = p_req;
    return NRF_SUCCESS;
}

uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn)
{
    return NRF_SUCCESS;
}

void timer_on_ts_begin(timestamp_t ts_end_time)
{
}

void timer_on_ts_end(timestamp_t ts_end_time)
{
}

void radio_on_ts_end(void)
{
}

void radio_on_ts_begin(void)
{
}

void mesh_flash_op_execute(timestamp_t available_time)
{
}

/******************************************/
void test_timeslot_start(void)
{
    nrf_mesh_init_params_t init_params = {.lfclksrc = {NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM}};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timeslot_init(&init_params));
    TEST_ASSERT_EQUAL(0, m_sd_radio_request_calls);
    TEST_ASSERT_EQUAL(0, m_sd_radio_session_open_calls);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, timeslot_resume());

    TEST_ASSERT_EQUAL(1, m_sd_radio_request_calls);
    TEST_ASSERT_EQUAL(1, m_sd_radio_session_open_calls);

    TEST_ASSERT_EQUAL(NRF_RADIO_REQ_TYPE_EARLIEST, mp_radio_req->request_type);
    TEST_ASSERT_EQUAL(NRF_RADIO_PRIORITY_NORMAL, mp_radio_req->params.earliest.priority);
    TEST_ASSERT_TRUE(mp_radio_req->params.earliest.length_us >= 100);
    TEST_ASSERT_TRUE(mp_radio_req->params.earliest.length_us <= 100000);
    m_ts_len = mp_radio_req->params.earliest.length_us;
    nrf_radio_signal_callback_return_param_t* p_retparam = m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_NOT_NULL(p_retparam);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_retparam->callback_action);
    while (m_ts_len + p_retparam->params.extend.length_us < 16700000 /* = TIMER0-rollover */
            && p_retparam->callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        m_ts_len += p_retparam->params.extend.length_us;
        p_retparam = m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    }

    if (p_retparam->callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        (void) m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    }

    TEST_ASSERT_TRUE(m_timeout < m_ts_len );
    TEST_ASSERT_TRUE(m_timeout > m_ts_len - 10000);
}

void test_timeslot_signal_propagation(void)
{
    (void) m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO);
    TEST_ASSERT_EQUAL(1, m_radio_event_handler_calls);
    (void) m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(1, m_timer_event_handler_calls);
}

void test_timeslot_end_timer(void)
{
    nrf_radio_signal_callback_return_param_t* p_retparam = m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(1, m_timer_event_handler_calls);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_retparam->callback_action);

}

void test_timeslot_rtc_carryover(void)
{
    uint32_t actual_starttime = setRTC(10000000);
    nrf_radio_signal_callback_return_param_t* p_retparam = m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_NOT_NULL(p_retparam);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_retparam->callback_action);
    while (m_ts_len + p_retparam->params.extend.length_us < 16700000 /* = TIMER0-rollover */
            && p_retparam->callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        m_ts_len += p_retparam->params.extend.length_us;
        p_retparam = m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    }

    if (p_retparam->callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        (void) m_signal_cb(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    }

    TEST_ASSERT_EQUAL(actual_starttime, timeslot_start_time_get());
}

