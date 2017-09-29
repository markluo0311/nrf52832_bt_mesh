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
#ifndef NRF_MESH_ASSERT_H__
#define NRF_MESH_ASSERT_H__

#ifdef UNIT_TEST
#include <setjmp.h>
#endif

#ifdef _lint
#include <stdlib.h>
#endif

#include "nrf_mesh.h"
#include "nrf.h"

/**
 * @defgroup MESH_ASSERT Internal assert triggering functions.
 * @ingroup MESH_CORE
 * Allows the framework to propagate errors that can't be recovered from.
 * @{
 */

/** Extern reference to assert handler, normally defined in nrf_mesh.c */
extern nrf_mesh_assertion_handler_t m_assertion_handler;

#if HOST
    #if defined(_lint)
        #define HARD_FAULT() abort()
        #define GET_PC(pc) pc = __current_pc();
    #elif defined(__GNUC__)
        #define HARD_FAULT() __builtin_trap()
        /** Get program counter on x86-32 using AT&T assembler syntax (GNU default). */
        #define GET_PC(pc) do {                         \
                __asm volatile (                        \
                    "movl $., %0\n\t"                   \
                    : "=r" (pc)                         \
                    );                                  \
            } while(0)
    #else
        #error "Compiler used is not supported."
    #endif /* defined(__GNUC__) */
#elif defined(_lint)
    #define HARD_FAULT() abort()
#elif defined(__CC_ARM) /* ARMCC for NRF */
    /** Produces a hardfault. */
    #define HARD_FAULT() __breakpoint(0)

    /** Get program counter (ARMCC) */
    #define GET_PC(__pc) do {                       \
            __pc = __current_pc();                  \
        } while (0)
#elif defined(__GNUC__) /* GCC for NRF*/
    #define HARD_FAULT() asm volatile (".inst 0xde00\n")
    /** Get program counter (GCC) */
    #define GET_PC(__pc) do {                       \
            __asm volatile (                        \
                "MOV %0, pc\n\t"                    \
                : "=r" (__pc)                       \
                );                                  \
        } while (0)
#else
    #error "Compiler used is not supported."
#endif /* HOST*/

/** Assert condition cond. Will trigger the assert handler if it evaluates to false. */
#ifdef UNIT_TEST
    extern bool mesh_assert_expect;
    extern jmp_buf assert_jump_buf;
    #define TEST_NRF_MESH_ASSERT_EXPECT(func) do { \
            mesh_assert_expect = true;             \
            if (!setjmp(assert_jump_buf))          \
            {                                      \
                (void) func;                       \
                mesh_assert_expect = false;        \
                TEST_FAIL();                       \
            }                                      \
        } while (0)

    #define NRF_MESH_ASSERT(cond)   if (!(cond))   \
     {                                             \
        uint32_t pc;                               \
        GET_PC(pc);                                \
        if(mesh_assert_expect)                     \
        {                                          \
            mesh_assert_expect = false;            \
            longjmp(assert_jump_buf,1);            \
        }                                          \
        else if (m_assertion_handler)              \
        {                                          \
            m_assertion_handler(pc);               \
        }                                          \
        else                                       \
        {                                          \
            HARD_FAULT();                          \
        }                                          \
     }
#elif _lint
    #define NRF_MESH_ASSERT(cond) do {if (!(cond)) {abort();}} while (0)
#else
    #define NRF_MESH_ASSERT(cond)   if (!(cond))                        \
    {                                                                   \
        uint32_t pc;                                                    \
        GET_PC(pc);                                                     \
        if (m_assertion_handler)                                        \
        {                                                               \
            m_assertion_handler(pc);                                    \
        }                                                               \
        else                                                            \
        {                                                               \
            HARD_FAULT();                                               \
        }                                                               \
    }
#endif

/** Check whether error code is valid */
#define NRF_MESH_ERROR_CHECK(err) NRF_MESH_ASSERT(err == NRF_SUCCESS)

/**  Static (i.e. compile time) assertion.*/
#define NRF_MESH_STATIC_ASSERT(cond) typedef char static_assert[(cond) ? 1 : -1]
/** @} */

#endif /* NRF_MESH_ASSERT_H__ */
