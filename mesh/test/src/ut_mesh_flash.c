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

#include <unity.h>
#include <cmock.h>

#include "mesh_flash.h"
#include "nrf_flash_mock.h"
#include "bearer_event.h"
#include "timer_mock.h"
#include "bl_if.h"

typedef struct
{
    mesh_flash_user_t user;
    uint32_t expected_cb_count;
    uint32_t cb_all_count;
    flash_operation_t expected_op;
    uint16_t expected_callback_token;
} flash_user_end_expect_t;
static flash_user_end_expect_t m_end_expect_users[2];
static bearer_event_flag_callback_t m_event_cb;
static bool m_delayed_flag_event;

extern void mesh_flash_reset(void);

static void mesh_assert(uint32_t pc)
{
    printf("ASSERT AT PC %u", pc);
    TEST_FAIL();
}
nrf_mesh_assertion_handler_t m_assertion_handler = mesh_assert;

void setUp(void)
{
    m_event_cb = NULL;
    memset(m_end_expect_users, 0, sizeof(m_end_expect_users));
    m_end_expect_users[0].user = MESH_FLASH_USER_TEST;
    m_end_expect_users[1].user = MESH_FLASH_USER_DFU;
    m_delayed_flag_event = false;
    nrf_flash_mock_Init();
    timer_mock_Init();
}

void tearDown(void)
{
    mesh_flash_reset();
    nrf_flash_mock_Verify();
    nrf_flash_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

static void mesh_flash_op_cb(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    /* fetch the relevant expect-values for the incoming user */
    flash_user_end_expect_t * p_expect = NULL;
    for (uint32_t i = 0; i < 2; i++)
    {
        if (m_end_expect_users[i].user == user)
        {
            p_expect = &m_end_expect_users[i];
            break;
        }
    }
    TEST_ASSERT_NOT_NULL(p_expect); /* Unknown user */


    if (p_expect->expected_cb_count == 0) /** Always expect a final "all operations" call after the expected count. */
    {
        TEST_ASSERT_EQUAL(FLASH_OP_TYPE_ALL, p_op->type);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.p_start_addr);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.p_data);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.length);
        p_expect->cb_all_count++;
    }
    else
    {
        TEST_ASSERT_EQUAL(p_expect->expected_callback_token, token);
        p_expect->expected_callback_token++;
        TEST_ASSERT(p_expect->expected_cb_count > 0);
        p_expect->expected_cb_count--;
        if (p_expect->expected_op.type == FLASH_OP_TYPE_WRITE)
        {
            TEST_ASSERT_EQUAL_MEMORY(&p_expect->expected_op, p_op, offsetof(flash_operation_t, params) + sizeof(p_op->params.write));
        }
        else
        {
            TEST_ASSERT_EQUAL_MEMORY(&p_expect->expected_op, p_op, offsetof(flash_operation_t, params) + sizeof(p_op->params.erase));
        }
    }
}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t cb)
{
    TEST_ASSERT_NOT_NULL(cb);
    m_event_cb = cb;
    return 0x1234;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    TEST_ASSERT_NOT_NULL(m_event_cb);
    if (!m_delayed_flag_event)
    {
        m_event_cb();
    }
}

/******** Tests ********/
void test_op_push(void)
{
    flash_operation_t flash_op;
    uint8_t data[4];
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD1ADD0;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    uint16_t token = 0xFFFF;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, NULL, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.type = FLASH_OP_TYPE_NONE;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.type = FLASH_OP_TYPE_ALL;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD1ADD1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD0ADD0;
    flash_op.params.write.length = 1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.write.length = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.write.length = 4;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0, token);
    token = 0xFFFF;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USERS, &flash_op, &token)); /* invalid user */
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1ADD1;
    flash_op.params.erase.length = 4;
    flash_op.type = FLASH_OP_TYPE_ERASE;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1ADD0; /* Only page aligned erases allowed (word aligned shouldn't) */
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1A000;
    flash_op.params.erase.length = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);
    flash_op.params.erase.length = 4; // must be page aligned too
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    /* Reset module to flush queue. */
    mesh_flash_reset();
    /* test full queue */
    uint16_t expected_token = 0;
    flash_op.params.erase.length = 1024;
    while (mesh_flash_op_available_slots(MESH_FLASH_USER_TEST))
    {
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        TEST_ASSERT_EQUAL(expected_token, token);
        expected_token++;
    }
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_NO_MEM, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
}

void test_execute_write(void)
{
    flash_operation_t flash_op;
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);

    const uint32_t lengths[] = {4, 8, 1028};
    uint16_t token = 0xFFFF;
    uint16_t expected_token = 0;

    for (uint32_t i = 0; i < 3; ++i)
    {
        flash_op.params.write.length = lengths[i];
        flash_op.type = FLASH_OP_TYPE_WRITE;
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        TEST_ASSERT_EQUAL(expected_token, token);
        expected_token++;
        mesh_flash_op_execute(0); /* Execute no operations */
        mesh_flash_op_execute(5); /* Execute no operations */

        if (lengths[i] > 4)
        {
            timer_now_ExpectAndReturn(0);
            nrf_flash_write_ExpectAndReturn(&dest[0], (uint32_t*) data, flash_op.params.write.length - 4, NRF_SUCCESS);
            timer_now_ExpectAndReturn(50 * (flash_op.params.write.length - 4) / 4);
        }
        mesh_flash_op_execute(50 * (lengths[i]/4) + 499); /* Execute all except last word */
        timer_mock_Verify();

        nrf_flash_write_ExpectAndReturn(&dest[lengths[i]/4 - 1], (uint32_t*) &data[lengths[i] - 4], 4, NRF_SUCCESS);
        memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
        m_end_expect_users[0].expected_cb_count = 1;
        m_end_expect_users[0].cb_all_count = 0;
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(50 * (lengths[i]/4));
        mesh_flash_op_execute(50 * (lengths[i]/4) + 501); /* Execute the operation */
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
        m_end_expect_users[0].cb_all_count = 0;
        timer_mock_Verify();
        nrf_flash_mock_Verify();
    }

    /* Flash chunk by chunk */
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 1024;
    TEST_ASSERT_FALSE(mesh_flash_in_progress());
    flash_op.type = FLASH_OP_TYPE_WRITE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(expected_token, token);
    expected_token++;
    const uint32_t chunk_size = 64;
    m_end_expect_users[0].cb_all_count = 0;
    m_end_expect_users[0].expected_cb_count = 1;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    TEST_ASSERT_TRUE(mesh_flash_in_progress());
    for (uint32_t i = 0; i < 1024; i += chunk_size)
    {
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].expected_cb_count);
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count);
        TEST_ASSERT_TRUE(mesh_flash_in_progress());
        nrf_flash_write_ExpectAndReturn(&dest[i/4], (uint32_t*) &data[i], chunk_size, NRF_SUCCESS);
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(50 * (chunk_size/4));
        mesh_flash_op_execute(50 * (chunk_size/4) + 501); /* Execute only some of the operation */
        timer_mock_Verify();
    }
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    timer_mock_Verify();

    /* run again, but spend much less time than expected per chunk, and finish faster. */
    flash_op.type = FLASH_OP_TYPE_WRITE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(expected_token, token);
    m_end_expect_users[0].cb_all_count = 0;
    m_end_expect_users[0].expected_cb_count = 1;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    TEST_ASSERT_TRUE(mesh_flash_in_progress());
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_TRUE(mesh_flash_in_progress());
#define BYTES_WRITTEN_WITHIN(time) (((time - 500) * 4) / 50)
    uint32_t remaining_time = 5000;
    uint32_t bytes = 0;
    timer_now_ExpectAndReturn(0);
    for (uint32_t i = 0; i < 1024; i += bytes)
    {
        bytes = BYTES_WRITTEN_WITHIN(remaining_time);
        if (i + bytes > 1024) bytes = 1024 - i;
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].expected_cb_count);
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count);
        TEST_ASSERT_TRUE(mesh_flash_in_progress());
        nrf_flash_write_ExpectAndReturn(&dest[i/4], (uint32_t*) &data[i], bytes, NRF_SUCCESS);
        timer_now_ExpectAndReturn(0); /* no time elapsed while writing */
    }
#undef BYTES_WRITTEN_WITHIN
    mesh_flash_op_execute(5000);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
}

void test_execute_erase(void)
{
    flash_operation_t flash_op;
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[4] = {0xab, 0xcd, 0xef, 0x01};
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);

    const uint32_t lengths[] = {1024, 2048};

    flash_op.params.erase.p_start_addr = dest;
    uint16_t token = 0xFFFF;

    for (uint32_t i = 0; i < 2; ++i)
    {
        flash_op.params.erase.length = lengths[i];
        flash_op.type = FLASH_OP_TYPE_ERASE;
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        mesh_flash_op_execute(0); /* Execute no operations */
        mesh_flash_op_execute(5); /* Execute no operations */
        timer_now_ExpectAndReturn(0);
        //timer_now_ExpectAndReturn(20000 * (lengths[i]/PAGE_SIZE));
        if (lengths[i] > PAGE_SIZE)
        {
            /* Erase all pages but the last */
            nrf_flash_erase_ExpectAndReturn(&dest[0], lengths[i] - PAGE_SIZE, NRF_SUCCESS);
            timer_now_ExpectAndReturn(20000 * (lengths[i]/PAGE_SIZE));
        }
        mesh_flash_op_execute(20000 * (lengths[i]/PAGE_SIZE) + 500 - 1); /* Execute no operations */
        timer_mock_Verify();
        /* erase last flash page */
        nrf_flash_erase_ExpectAndReturn(&dest[(lengths[i] - PAGE_SIZE) / 4], PAGE_SIZE, NRF_SUCCESS);
        memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
        m_end_expect_users[0].expected_cb_count = 1;
        m_end_expect_users[0].cb_all_count = 0;
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(20000 * (lengths[i]/PAGE_SIZE));
        mesh_flash_op_execute(20000 * (lengths[i]/PAGE_SIZE) + 501); /* Execute the operation */
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
        m_end_expect_users[0].cb_all_count = 0;
        timer_mock_Verify();
    }

    /* Erase chunk by chunk */
    flash_op.params.erase.p_start_addr = dest;
    flash_op.params.erase.length = 4 * PAGE_SIZE;
    TEST_ASSERT_FALSE(mesh_flash_in_progress());
    flash_op.type = FLASH_OP_TYPE_ERASE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    m_end_expect_users[0].cb_all_count = 0;
    m_end_expect_users[0].expected_cb_count = 1;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    TEST_ASSERT_TRUE(mesh_flash_in_progress());
    for (uint32_t i = 0; i < 4; i++)
    {
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].expected_cb_count);
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count);
        TEST_ASSERT_TRUE(mesh_flash_in_progress());
        nrf_flash_erase_ExpectAndReturn(&dest[i*PAGE_SIZE/4], PAGE_SIZE, NRF_SUCCESS);
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(20000);
        mesh_flash_op_execute(20000 + 501); /* Execute only some of the operation */
        timer_mock_Verify();
    }
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    timer_mock_Verify();
}

void test_available_slots(void)
{
    flash_operation_t flash_op;
    uint32_t dest;
    uint8_t data[4];
    uint16_t token = 0xFFFF;
    flash_op.params.write.p_start_addr = &dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    TEST_ASSERT_EQUAL(0, mesh_flash_op_available_slots(MESH_FLASH_USERS)); /* out of bounds, so there are no slots. */
    uint32_t available_slots = mesh_flash_op_available_slots(MESH_FLASH_USER_TEST);
    TEST_ASSERT_NOT_EQUAL(0, available_slots);
    flash_op.type = FLASH_OP_TYPE_WRITE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(available_slots - 1, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST));
    TEST_ASSERT_EQUAL(available_slots - 1, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* shouldn't alter anything. */
    flash_op.type = FLASH_OP_TYPE_WRITE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(available_slots - 2, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* shouldn't alter anything. */
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    m_end_expect_users[0].expected_cb_count = 2;
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    timer_now_ExpectAndReturn(50);
    mesh_flash_op_execute(10000); /* Execute the two operations */
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(available_slots, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* Back to full */
}

void test_multiple_users(void)
{
    uint16_t token = 0xFFFF;
    flash_operation_t flash_op;
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    mesh_flash_user_callback_set(MESH_FLASH_USER_DFU, mesh_flash_op_cb);

    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    memcpy(&m_end_expect_users[1].expected_op, &flash_op, sizeof(m_end_expect_users[1].expected_op));

    /* push the same event to two users */
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_DFU, &flash_op, &token));

    /* execute first event (the one with the lowest user index) */
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    m_end_expect_users[1].expected_cb_count = 1;
    m_end_expect_users[1].cb_all_count = 0;
    mesh_flash_op_execute(551);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[1].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[1].expected_cb_count);
    /* execute the second */
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    mesh_flash_op_execute(551);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);

    /* Schedule one long and one short event. */
    flash_operation_t long_flash_op;
    long_flash_op.type = FLASH_OP_TYPE_ERASE;
    long_flash_op.params.erase.p_start_addr = dest;
    long_flash_op.params.erase.length = 1024;

    flash_op.params.write.length = 12;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_DFU, &long_flash_op, &token));
    memcpy(&m_end_expect_users[1].expected_op, &long_flash_op, sizeof(m_end_expect_users[1].expected_op));

    /* execute with enough time to do a bit of the short one, but not enough to
     * do the long one. Should wait with the long one for now. */
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    mesh_flash_op_execute(551);
    nrf_flash_mock_Verify();
    /* Now execute with enough time to do the long one, but spend less time
     * than expected, and fit the rest of the short one afterwards. */
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(100); /* less time than expected */
    timer_now_ExpectAndReturn(100);
    nrf_flash_erase_ExpectAndReturn(dest, 1024, NRF_SUCCESS);
    nrf_flash_write_ExpectAndReturn(&dest[1], (uint32_t *) &data[4], 8, NRF_SUCCESS);
    m_end_expect_users[0].expected_op.params.write.length = 12;
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    m_end_expect_users[1].expected_cb_count = 1;
    m_end_expect_users[1].cb_all_count = 0;
    mesh_flash_op_execute(20501);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[1].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[1].expected_cb_count);
}

/** The module keeps track of its queue with indexes. These will
 * eventually roll over. Verify that this doesn't break the queue.
 */
void test_queue_index_rollover(void)
{
    uint16_t token = 0xFFFF;
    flash_operation_t flash_op;
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    /* rollover happens at 256: */
    for (uint32_t i = 0; i < 270; i++)
    {
        /* Keep pushing and processing */
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(50);
        nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
        m_end_expect_users[0].expected_cb_count = 1;
        m_end_expect_users[0].cb_all_count = 0;
        mesh_flash_op_execute(551);
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    }
}

/** Delay execution of the async flag event, to let the module collect several
 * operations to report at the same time. Ensure that it pushes all the events
 * at a single execution step.
 */
void test_multifire(void)
{
    uint16_t token = 0xFFFF;
    m_delayed_flag_event = true;
    flash_operation_t flash_op;
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    for (uint32_t i = 0; i < 4; i++)
    {
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        timer_now_ExpectAndReturn(0);
        timer_now_ExpectAndReturn(50);
        nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
        mesh_flash_op_execute(551);
    }

    /* keep one operation in the pipeline, but fire the callback. The reports
     * on individual events should come, but not the final ALL events callback.
     * */
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));

    m_end_expect_users[0].expected_cb_count = 4;
    m_end_expect_users[0].cb_all_count = 0;
    m_event_cb();
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count); /* still one event in the pipeline. */
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    mesh_flash_op_execute(551);
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    m_event_cb();
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count); /* All events fired. */
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
}

void test_suspend(void)
{
    uint16_t token = 0xFFFF;
    flash_operation_t flash_op;
    uint32_t dest;
    uint8_t data[4];
    flash_op.params.write.p_start_addr = &dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    uint32_t available_slots = mesh_flash_op_available_slots(MESH_FLASH_USER_TEST);
    TEST_ASSERT_NOT_EQUAL(0, available_slots);
    flash_op.type = FLASH_OP_TYPE_WRITE;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    mesh_flash_set_suspended(true);
    mesh_flash_op_execute(10000); /* Execute the operation, don't expect anything to happen */
    nrf_flash_mock_Verify();
    mesh_flash_set_suspended(false);
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(50);
    mesh_flash_op_execute(10000); /* Execute the operation */
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
}
