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

#ifndef MESH_LIST_H__
#define MESH_LIST_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <nrf_error.h>

#include "nrf_mesh_assert.h"

/**
 * @defgroup LINKED_LIST Generic linked list implementation.
 * @ingroup MESH_CORE
 * @{
 */

/**
 * Get the pointer to the start of the structure containing a linked list node.
 *
 * @param[in] type        Data type to be stored.
 * @param[in] p_member    Pointer to the linked list node member within the struct.
 * @param[in] member_name Name of the member in the struct.
 *
 * @return Pointer to the start of the \<type\> structure.
 */
#define LIST_CONTAINER(type, p_member, member_name)                     \
    ((type *) ((uint8_t *) p_member - offsetof(type, member_name)))

/**
 * Generic linked list node.
 */
typedef struct list_node
{
    /** Pointer to the next item in the list. */
    struct list_node * p_next;
} list_node_t;

/**
 * List node compare function callback.
 *
 * @param[in] p1 Pointer to first node.
 * @param[in] p2 Pointer to second node.
 *
 * @returns True if p1 is to be sorted before p2.
 */
typedef bool (*list_cmp_cb_t)(const list_node_t * p1, const list_node_t * p2);

/**
 * Insert a new item at a given point in a list.
 *
 * @param[in,out] p_node The node at which the new node is inserted after.
 * @param[in,out] p_new  The new node to be inserted.
 */
static inline void list_insert(list_node_t * p_node, list_node_t * p_new)
{
    NRF_MESH_ASSERT(p_node != NULL && p_new != NULL);

    if (p_node != p_new)
    {
        p_new->p_next  = p_node->p_next;
        p_node->p_next = p_new;
    }
}

/**
 * Add an item at the end of the linked list.
 *
 * @param[in,out] pp_head Pointer to the head pointer of the linked list.
 * @param[in,out] p_new   New item to be pushed to the list.
 */
static inline void list_add(list_node_t ** pp_head, list_node_t * p_new)
{
    NRF_MESH_ASSERT(pp_head != NULL && p_new != NULL);
    NRF_MESH_ASSERT(*pp_head != p_new);

    if (*pp_head == NULL)
    {
        *pp_head           = p_new;
        (*pp_head)->p_next = NULL;    /* sanitize */
    }
    else
    {
        list_node_t * p_node = *pp_head;
        while (p_node->p_next != NULL && p_node != p_new)
        {
            p_node = p_node->p_next;
        }

        list_insert(p_node, p_new);
    }
}

/**
 * Remove an item from the list.
 *
 * @param[in,out] pp_head Pointer to the head pointer of the list.
 * @param[in,out] p_node  Node to remove from the list.
 *
 * @retval NRF_SUCCESS         Successfully removed item.
 * @retval NRF_ERROR_NOT_FOUND Item not found in the list.
 */
static inline uint32_t list_remove(list_node_t ** pp_head, list_node_t * p_node)
{
    NRF_MESH_ASSERT(p_node != NULL && pp_head != NULL);

    if (*pp_head == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (p_node == *pp_head)
    {
        *pp_head       = p_node->p_next;
        p_node->p_next = NULL;

        return NRF_SUCCESS;
    }

    list_node_t * p_item = *pp_head;
    while (p_item->p_next != NULL && p_item->p_next != p_node)
    {
        p_item = p_item->p_next;
    }

    if (p_item == NULL || p_item->p_next != p_node)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    p_item->p_next = p_node->p_next;
    p_node->p_next = NULL;

    return NRF_SUCCESS;
}

/**
 * Get the size of a list.
 *
 * @param[in] p_head Pointer to the head of the list.
 *
 * @return size Size of the linked list. 0 if an invalid pointer is supplied.
 */
static inline uint32_t list_size_get(list_node_t * p_head)
{
    if (p_head == NULL)
    {
        return 0;
    }

    uint32_t i = 1;
    while (p_head->p_next != NULL)
    {
        p_head = p_head->p_next;
        ++i;
    }

    return i;
}

/**
 * Adds an item to the list based on sort criterion.
 *
 * @todo Check for double insertion?
 *
 * @param[in,out] pp_head Linked list head double pointer.
 * @param[in,out] p_new   Node to be added to the list.
 * @param[in]     cmp_cb  Comparison callback function.
 */
static inline void list_sorted_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb)
{
    NRF_MESH_ASSERT(pp_head != NULL && p_new != NULL);

    if (*pp_head == NULL)
    {
        *pp_head           = p_new;
        (*pp_head)->p_next = NULL;    /* sanitize */
    }
    else
    {
        if (cmp_cb(p_new, *pp_head))
        {
            p_new->p_next = *pp_head;
            *pp_head = p_new;
        }
        else
        {
            list_node_t * p_node = (*pp_head)->p_next;
            list_node_t * p_prev = *pp_head;
            while (p_node != NULL &&
                   cmp_cb(p_node, p_new))
            {
                p_prev = p_node;
                p_node = p_node->p_next;
            }

            list_insert(p_prev, p_new);
        }
    }
}

/** @} */
#endif
