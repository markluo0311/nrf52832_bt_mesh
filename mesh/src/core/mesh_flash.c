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
#include <string.h>

#include "mesh_flash.h"
#include "nrf.h"

#include "bearer_event.h"
#include "fifo.h"
#include "nrf_flash.h"
#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "timer.h"
#include "msqueue.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/** Number of flash operations that can be queued at once. */
#define FLASH_OP_QUEUE_LEN					(8)

/** Maximum overhead of processing the flash queue. */
#define FLASH_PROCESS_TIME_OVERHEAD		    (500)

#ifdef UNIT_TEST
    /** Longest time spent on a single flash operation. Longer operations will be broken up. */
    #define FLASH_OP_MAX_TIME_US                (50000)
    /** Timer to erase a single flash page. */
    #define FLASH_TIME_TO_ERASE_PAGE_US         (20000)
    /** Timer to write a single flash word. */
    #define FLASH_TIME_TO_WRITE_ONE_WORD_US     (50)

#elif defined(NRF51)
    /** Longest time spent on a single flash operation. Longer operations will be broken up. */
    #define FLASH_OP_MAX_TIME_US                (50000)
    /** Timer to erase a single flash page. */
    #define FLASH_TIME_TO_ERASE_PAGE_US         (22050)
    /** Timer to write a single flash word. */
    #define FLASH_TIME_TO_WRITE_ONE_WORD_US     (48)

#elif defined(NRF52)
    /** Longest time spent on a single flash operation. Longer operations will be broken up. */
    #define FLASH_OP_MAX_TIME_US                (100000)
    /** Timer to erase a single flash page. */
    #define FLASH_TIME_TO_ERASE_PAGE_US         (89700)
    /** Timer to write a single flash word. */
    #define FLASH_TIME_TO_WRITE_ONE_WORD_US     (338)

#else
    #error "Unsupported platform"
#endif

/* The queue length will overflow if it's longer than 256 */
NRF_MESH_STATIC_ASSERT(FLASH_OP_QUEUE_LEN < 256);
/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    FLASH_OP_STAGE_FREE,
    FLASH_OP_STAGE_QUEUED,
    FLASH_OP_STAGE_PROCESSED,
    FLASH_OP_STAGES
} flash_op_stage_t;

typedef struct
{
    uint16_t event_token;
    uint16_t push_token;
    uint32_t processed_bytes; /**< How many bytes have been processed in the current event. */
    mesh_flash_op_cb_t cb;
    struct
    {
        msq_t queue;
        uint8_t stages[FLASH_OP_STAGES];
        flash_operation_t elems[FLASH_OP_QUEUE_LEN];
    } flash_op_queue;
} flash_user_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static bool                m_suspended; /**< Suspend flag, preventing flash operations while set. */

static flash_user_t        m_users[MESH_FLASH_USERS];
static bearer_event_flag_t m_event_flag;
/** Constant "All operations" operation, used to signalize that all operations
 * have been completed for a user. */
static const flash_operation_t m_all_operations =
{
    .type = FLASH_OP_TYPE_ALL
}; /*lint !e785 Too few initializers for flash_operation_t. */
/*****************************************************************************
* Static functions
*****************************************************************************/
static bool write_as_much_as_possible(const flash_operation_t* p_write_op, timestamp_t available_time, uint32_t* p_bytes_written)
{
    uint32_t offset = *p_bytes_written;
    NRF_MESH_ASSERT(p_write_op->type == FLASH_OP_TYPE_WRITE);
    const uint32_t max_time = ((available_time < FLASH_OP_MAX_TIME_US) ? available_time : FLASH_OP_MAX_TIME_US);
    uint32_t bytes_to_write = WORD_SIZE * (max_time / FLASH_TIME_TO_WRITE_ONE_WORD_US);
    if (bytes_to_write > p_write_op->params.write.length - offset)
    {
        bytes_to_write = p_write_op->params.write.length - offset;
    }

    if (bytes_to_write == 0)
    {
        return false;
    }

    NRF_MESH_ASSERT(nrf_flash_write(&p_write_op->params.write.p_start_addr[offset / WORD_SIZE],
                    &p_write_op->params.write.p_data[offset / WORD_SIZE],
                    bytes_to_write) == NRF_SUCCESS);

    *p_bytes_written += bytes_to_write;
    return true;
}

static bool erase_as_much_as_possible(const flash_operation_t * p_erase_op, timestamp_t available_time, uint32_t* p_bytes_erased)
{
    uint32_t offset = *p_bytes_erased;
    NRF_MESH_ASSERT(p_erase_op->type == FLASH_OP_TYPE_ERASE);
    const uint32_t max_time = ((available_time < FLASH_OP_MAX_TIME_US) ? available_time : FLASH_OP_MAX_TIME_US);
    uint32_t bytes_to_erase = PAGE_SIZE * (max_time / FLASH_TIME_TO_ERASE_PAGE_US);
    if (bytes_to_erase > p_erase_op->params.erase.length - offset)
    {
        bytes_to_erase = p_erase_op->params.erase.length - offset;
    }

    if (bytes_to_erase == 0)
    {
        return false;
    }

    NRF_MESH_ASSERT(nrf_flash_erase(&p_erase_op->params.erase.p_start_addr[offset / WORD_SIZE],
                    bytes_to_erase) == NRF_SUCCESS);

    *p_bytes_erased += bytes_to_erase;
    return true;
}

/** Call the callbacks of all users for all processed events. */
static void send_end_events(void)
{
    for (mesh_flash_user_t i = (mesh_flash_user_t) 0; i < MESH_FLASH_USERS; i++)
    {
        uint32_t notified_events = 0;
        flash_operation_t * p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        while (p_op != NULL)
        {
            if (m_users[i].cb != NULL)
            {
                m_users[i].cb(i, p_op, m_users[i].event_token);
            }
            m_users[i].event_token++;
            msq_move(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
            notified_events++;
            p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        }
        /* When every item in the queue has been processed and notified, we
         * tell the user. */
        if (notified_events != 0 &&
            (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) == FLASH_OP_QUEUE_LEN) &&
            m_users[i].cb != NULL)
        {
            m_users[i].cb(i, &m_all_operations, 0);
        }
    }
}

static inline void end_event_schedule(void)
{
    bearer_event_flag_set(m_event_flag);
}

static bool execute_next_operation(flash_user_t * p_user, uint32_t available_time)
{
    bool operation_executed = false;
    flash_operation_t * p_op = msq_get(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
    if (p_op != NULL)
    {
        uint32_t operation_length = 0;
        switch (p_op->type)
        {
            case FLASH_OP_TYPE_WRITE:
                operation_executed = write_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
                operation_length = p_op->params.write.length;
                break;
            case FLASH_OP_TYPE_ERASE:
                operation_executed = erase_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
                operation_length = p_op->params.erase.length;
                break;
            default:
                NRF_MESH_ASSERT(false);
        }

        if (operation_length == p_user->processed_bytes)
        {
            msq_move(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
            p_user->processed_bytes = 0;
            end_event_schedule();
        }
    }
    return operation_executed;
}

static void init_flash_op_queue(flash_user_t * p_user)
{
    p_user->flash_op_queue.queue.elem_count = FLASH_OP_QUEUE_LEN;
    p_user->flash_op_queue.queue.elem_size = sizeof(flash_operation_t);
    p_user->flash_op_queue.queue.p_elem_array = p_user->flash_op_queue.elems;
    p_user->flash_op_queue.queue.stage_count = FLASH_OP_STAGES;
    p_user->flash_op_queue.queue.p_stages = p_user->flash_op_queue.stages;
    msq_init(&p_user->flash_op_queue.queue);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void mesh_flash_init(void)
{
    m_event_flag = bearer_event_flag_add(send_end_events);
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
}

void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb)
{
    NRF_MESH_ASSERT(user < MESH_FLASH_USERS);
    m_users[user].cb = cb;
}

uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token)
{
    NRF_MESH_ASSERT(user < MESH_FLASH_USERS);
    NRF_MESH_ASSERT(p_op != NULL);

    if (p_op->type == FLASH_OP_TYPE_WRITE)
    {
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_start_addr));
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_data));
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.length));
        NRF_MESH_ASSERT(p_op->params.write.length != 0);
    }
    else if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.p_start_addr));
        NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.length));
        NRF_MESH_ASSERT(p_op->params.erase.length != 0);
    }
    else
    {
        /* operation type must be WRITE or ERASE */
        NRF_MESH_ASSERT(false);
    }
    uint32_t was_masked;
    uint32_t status;
    _DISABLE_IRQS(was_masked);
    flash_operation_t * p_free_op = msq_get(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
    if (p_free_op == NULL)
    {
        status = NRF_ERROR_NO_MEM;
    }
    else
    {
        msq_move(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        memcpy(p_free_op, p_op, sizeof(flash_operation_t));
        status = NRF_SUCCESS;
        if (p_token != NULL)
        {
            *p_token = m_users[user].push_token;
        }
        m_users[user].push_token++;
    }
    _ENABLE_IRQS(was_masked);

    return status;
}

uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user)
{
    if (user < MESH_FLASH_USERS)
    {
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        uint32_t available_slots = msq_available(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        _ENABLE_IRQS(was_masked);

        return available_slots;
    }
    else
    {
        return 0;
    }
}

bool mesh_flash_in_progress(void)
{
    uint32_t was_masked;
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        _DISABLE_IRQS(was_masked);
        bool operations_in_progress = (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) != FLASH_OP_QUEUE_LEN);
        _ENABLE_IRQS(was_masked);

        if (operations_in_progress)
        {
            return true;
        }
    }
    return false;
}

void mesh_flash_op_execute(timestamp_t available_time)
{
    if (m_suspended || available_time < FLASH_PROCESS_TIME_OVERHEAD + FLASH_TIME_TO_WRITE_ONE_WORD_US)
    {
        return;
    }
    available_time -= FLASH_PROCESS_TIME_OVERHEAD;

    const uint32_t time_at_start = timer_now();
    uint32_t elapsed_time = 0;

    bool operation_executed;
    do
    {
        operation_executed = false;

        for (uint32_t i = 0;
             i < MESH_FLASH_USERS && (elapsed_time + FLASH_TIME_TO_WRITE_ONE_WORD_US <= available_time);
             i++)
        {
            if (execute_next_operation(&m_users[i], available_time - elapsed_time))
            {
                uint32_t time_now = timer_now();
                elapsed_time = TIMER_DIFF(time_now, time_at_start);
                operation_executed = true;
            }
        }
    } while (operation_executed);
}

void mesh_flash_set_suspended(bool suspend)
{
    static uint32_t suspend_count = 0;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    NRF_MESH_ASSERT(suspend || suspend_count > 0);
    if (suspend)
    {
        suspend_count++;
    }
    else
    {
        suspend_count--;
    }
    m_suspended = (suspend_count > 0);
    _ENABLE_IRQS(was_masked);
}

#ifdef UNIT_TEST
/**
 * @internal
 * Test-utility function to reset the state of the module between tests. Not
 * exposed in the header, as it should never be called when running on target.
 */
void mesh_flash_reset(void)
{
    m_event_flag = 0;
    m_suspended = false;
    memset(m_users, 0, sizeof(m_users));
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
}
#endif
