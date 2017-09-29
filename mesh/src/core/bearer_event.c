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

#include <stddef.h>

#include "bearer_event.h"

#include "toolchain.h"
#include "fifo.h"
#include "utils.h"
#include "nrf_mesh_assert.h"
#include "bitfield.h"
#include "nrf.h"
#include "nrf_soc.h"

#ifdef BEARER_EVENT_USE_SWI0
#define EVENT_IRQn          SWI0_IRQn
#define EVENT_IRQHandler    SWI0_IRQHandler
#else
#define EVENT_IRQn          QDEC_IRQn
#define EVENT_IRQHandler    QDEC_IRQHandler
#endif

/** Offset in SCB->ICSR register to match NVIC IRQ defines. */
#define NVIC_ICSR_OFFSET    (16)

/** Mask in SCB->ICSR for NVIC IRQ defines.
 *
 * @note For nRF51 (Cortex M0) http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihfaaha.html
 *       For nRF52 (Cortex M4) http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/Cihfaaha.html
 */
#if defined(NRF51) && !defined(NRF52)
#define NVIC_ICSR_MASK      (0x3f)
#elif !defined(NRF51) && defined(NRF52)
#define NVIC_ICSR_MASK      (0x01ff)
#endif

/*****************************************************************************
* Local type definitions
*****************************************************************************/
/** The different types of bearer events. */
typedef enum
{
    BEARER_EVENT_TYPE_TIMER,    /**< Timer event type */
    BEARER_EVENT_TYPE_GENERIC,  /**< Generic event type */
    BEARER_EVENT_TYPE_TIMER_SCHEDULER /**< Timer scheduler type */
} bearer_event_type_t;

/** Bearer event type which is queued for asynchronous processing. */
typedef struct
{
    bearer_event_type_t type;                   /**< Event type of this event. */
    union
    {
        struct
        {
            timer_callback_t callback;          /**< Callback function to call during processing of timer event. */
            timestamp_t timeout;                /**< Timestamp to give to the callback. */
        } timer;                                /**< Timer event parameters */
        struct
        {
            bearer_event_callback_t callback;   /**< Callback function to call during processing of generic event. */
            void* p_context;                    /**< Context pointer to give to callback. */
        } generic;                              /**< Generic event parameters */
        struct
        {
            timer_sch_callback_t callback;
            timestamp_t timeout;
            void * p_context;
        } timer_sch;
    } params;                                   /**< Parameters for async event */
} bearer_event_t;


/*****************************************************************************
* Static globals
*****************************************************************************/
/** FIFO structure for bearer event handler */
static fifo_t m_bearer_event_fifo;
/** FIFO buffer for bearer event handler */
static bearer_event_t m_bearer_event_fifo_buffer[BEARER_EVENT_FIFO_SIZE];
/** IRQ critical section mask */
static uint32_t m_critical;
/** Event flag field. */
static volatile uint32_t m_flags[BITFIELD_BLOCK_COUNT(BEARER_EVENT_FLAG_COUNT)];
/** Lookup table of flag event handlers. */
static bearer_event_flag_callback_t m_flag_event_callbacks[BEARER_EVENT_FLAG_COUNT];
/** Number of flags allocated. */
static uint32_t m_flag_count;
/*****************************************************************************
* System callback functions
*****************************************************************************/

/* Gets the current IRQ handler. */
static inline int get_current_irq(void)
{
#if HOST
    return Reset_IRQn; /* Fallback for other platforms. */
#else
    return ((SCB->ICSR - NVIC_ICSR_OFFSET) & NVIC_ICSR_MASK);
#endif
}

/* Function for calling a callback according to the callback type. */
static void call_callback(const bearer_event_t * p_evt)
{
    switch (p_evt->type)
    {
        case BEARER_EVENT_TYPE_TIMER:
            if (p_evt->params.timer.callback)
            {
                p_evt->params.timer.callback(p_evt->params.timer.timeout);
            }
            break;
        case BEARER_EVENT_TYPE_TIMER_SCHEDULER:
            if (p_evt->params.timer_sch.callback)
            {
                p_evt->params.timer_sch.callback(p_evt->params.timer_sch.timeout, p_evt->params.timer_sch.p_context);
            }
            break;
        case BEARER_EVENT_TYPE_GENERIC:
            if (p_evt->params.generic.callback)
            {
                p_evt->params.generic.callback(p_evt->params.generic.p_context);
            }
            break;
    }
}

/* Function for handling bearer events. */
static void handle_events(void)
{
    /* Handle flag events */
    for (uint32_t i = 0; i < m_flag_count; i++)
    {
        if (bitfield_get((uint32_t *) m_flags, i))
        {
            uint32_t was_masked;
            _DISABLE_IRQS(was_masked);
            bitfield_clear((uint32_t *) m_flags, i);
            _ENABLE_IRQS(was_masked);
            m_flag_event_callbacks[i]();
        }
    }
    /* Handle queued events */
    bearer_event_t evt;
    while (fifo_pop(&m_bearer_event_fifo, &evt) == NRF_SUCCESS)
    {
        call_callback(&evt);
    }
}

#if !HOST
/* IRQ handler for asynchronous processing */
void EVENT_IRQHandler(void)
{
    handle_events();
}
#endif

/*****************************************************************************
* Static functions
*****************************************************************************/
static void trigger_event_handler(void)
{
#if HOST
    if (m_critical == 0)
    {
        handle_events();
    }
#else
    NVIC_SetPendingIRQ(EVENT_IRQn);
#endif /* HOST */
}

/** Push a bearer event to the processing FIFO, and notify the IRQ. */
static uint32_t evt_push(const bearer_event_t* p_evt)
{
    if (fifo_push(&m_bearer_event_fifo, p_evt) == NRF_SUCCESS)
    {
        trigger_event_handler();
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}


/*****************************************************************************
* Interface functions
*****************************************************************************/
void bearer_event_init(void)
{
    m_bearer_event_fifo.elem_array = m_bearer_event_fifo_buffer;
    m_bearer_event_fifo.elem_size = sizeof(bearer_event_t);
    m_bearer_event_fifo.array_len = BEARER_EVENT_FIFO_SIZE;
    fifo_init(&m_bearer_event_fifo);

#if !HOST
    NVIC_SetPriority(EVENT_IRQn, BEARER_EVENT_IRQ_PRIORITY);
    NVIC_EnableIRQ(EVENT_IRQn);
#endif
}

uint32_t bearer_event_generic_post(bearer_event_callback_t callback, void* p_context)
{
    NRF_MESH_ASSERT(callback != NULL);
    bearer_event_t evt;
    evt.type = BEARER_EVENT_TYPE_GENERIC;
    evt.params.generic.callback = callback;
    evt.params.generic.p_context = p_context;

    return evt_push(&evt);
}

uint32_t bearer_event_timer_post(timer_callback_t callback, timestamp_t timestamp)
{
    NRF_MESH_ASSERT(callback != NULL);
    bearer_event_t evt;
    evt.type = BEARER_EVENT_TYPE_TIMER;
    evt.params.timer.callback = callback;
    evt.params.timer.timeout = timestamp;

    return evt_push(&evt);
}

uint32_t bearer_event_timer_sch_post(timer_sch_callback_t callback, timestamp_t timestamp, void * p_context)
{
    NRF_MESH_ASSERT(callback != NULL);
    bearer_event_t evt;
    evt.type = BEARER_EVENT_TYPE_TIMER_SCHEDULER;
    evt.params.timer_sch.callback = callback;
    evt.params.timer_sch.timeout = timestamp;
    evt.params.timer_sch.p_context = p_context;

    return evt_push(&evt);
}

void bearer_event_critical_section_begin(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (!m_critical++)
    {
#if !HOST
        NVIC_DisableIRQ(EVENT_IRQn);
#endif
    }

    NRF_MESH_ASSERT(m_critical != 0);
    _ENABLE_IRQS(was_masked);
}

void bearer_event_critical_section_end(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    NRF_MESH_ASSERT(m_critical != 0);

    if (!--m_critical)
    {
#if HOST
        trigger_event_handler();
#else
        NVIC_EnableIRQ(EVENT_IRQn);
#endif

    }
    _ENABLE_IRQS(was_masked);
}

bool bearer_event_in_progress(void)
{
    return (get_current_irq() == EVENT_IRQn);
}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback)
{
    NRF_MESH_ASSERT(callback != NULL);

    /* Check if we can still fit flags in the pool. */
    NRF_MESH_ASSERT(m_flag_count < BEARER_EVENT_FLAG_COUNT);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    uint32_t flag = m_flag_count++;
    m_flag_event_callbacks[flag] = callback;

    _ENABLE_IRQS(was_masked);

    return flag;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    NRF_MESH_ASSERT(flag < m_flag_count);
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    bitfield_set((uint32_t *) m_flags, flag);
    trigger_event_handler();
    _ENABLE_IRQS(was_masked);
}

