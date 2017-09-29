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
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "bearer_event_mock.h"
#include "device_state_manager.h"
#include "device_state_manager_flash.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_keygen_mock.h"
#include "net_state_mock.h"
#include "flash_manager_mock.h"

/** Non-VLA version of a dsm flash entry */
typedef struct
{
    fm_header_t       header;
    dsm_flash_entry_t entry;
} dsm_entry_t;

nrf_mesh_assertion_handler_t m_assertion_handler;

static flash_manager_t * mp_flash_manager;

static flash_manager_page_t m_flash_area[2];
static dsm_entry_t * mp_multiple_expect_prev = NULL;
static fm_mem_listener_t * mp_mem_listener;
static uint32_t m_fm_mem_listener_register_expect;
static fm_state_t m_add_manager_result_state;
static struct
{
    bool verify_contents; /**< Verify contents of the flash data. Should be turned off if we're doing batch-operations */
    void * p_data;
    uint32_t data_length;
    uint16_t flash_group;
    uint16_t flash_handle; /**< Expected flash handle if flash group is 0 */
} m_expected_flash_data;

static uint32_t flash_manager_add_cb(flash_manager_t * p_manager, const flash_manager_config_t * p_config, int calls)
{
    mp_flash_manager = p_manager;
    TEST_ASSERT_EQUAL_PTR(&m_flash_area[0], p_config->p_area);
    TEST_ASSERT_EQUAL(1, p_config->page_count);
    TEST_ASSERT_NOT_NULL(p_config->write_complete_cb);
    TEST_ASSERT_NOT_NULL(p_config->invalidate_complete_cb);
    memcpy(&p_manager->config, p_config, sizeof(flash_manager_config_t));
    p_manager->internal.state = m_add_manager_result_state;
    return NRF_SUCCESS;
}

void nrf_mesh_assertion_handler(uint32_t pc)
{
    TEST_FAIL_MESSAGE("Mesh assert handler called!");
}

void setUp(void)
{
    m_assertion_handler = nrf_mesh_assertion_handler;
    m_fm_mem_listener_register_expect = 0;
    nrf_mesh_mock_Init();
    nrf_mesh_keygen_mock_Init();
    flash_manager_mock_Init();
    net_state_mock_Init();

    mp_flash_manager = NULL;
    m_add_manager_result_state = FM_STATE_READY;
    m_expected_flash_data.verify_contents = true;

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    flash_manager_recovery_page_get_IgnoreAndReturn(&m_flash_area[1]);
    flash_manager_add_StubWithCallback(flash_manager_add_cb);

    net_state_flash_area_get_ExpectAndReturn((void *)(PAGE_SIZE + (uint32_t)m_flash_area));
    dsm_init();
}

void tearDown(void)
{
    flash_manager_remove_IgnoreAndReturn(NRF_SUCCESS);
    dsm_clear();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_keygen_mock_Verify();
    nrf_mesh_keygen_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
}

static void flash_manager_mem_listener_register_cb(fm_mem_listener_t * p_listener, int calls)
{
    TEST_ASSERT_NOT_EQUAL(0, m_fm_mem_listener_register_expect);
    m_fm_mem_listener_register_expect--;
    mp_mem_listener = p_listener;
}


/* fail-version of the flash callback, so we can "fail" flashing */
static fm_entry_t * flash_manager_entry_alloc_fail_cb(flash_manager_t * p_manager,
                                                        fm_handle_t handle,
                                                        uint32_t data_length,
                                                        int calls)
{
    return NULL;
}


static uint32_t flash_manager_entry_invalidate_fail_cb(flash_manager_t * p_manager,
                                                         fm_handle_t handle,
                                                         int calls)
{
    return NRF_ERROR_NO_MEM;
}

fm_entry_t * flash_manager_entry_alloc_cb(flash_manager_t * p_manager,
                                       fm_handle_t       handle,
                                       uint32_t          data_length,
                                       int calls)
{
    TEST_ASSERT_NOT_NULL(p_manager);
    TEST_ASSERT_EQUAL_PTR(mp_flash_manager, p_manager);
    if (m_expected_flash_data.verify_contents)
    {
        TEST_ASSERT_EQUAL(m_expected_flash_data.data_length, data_length);
        if (m_expected_flash_data.flash_group == 0)
        {
            TEST_ASSERT_EQUAL_HEX16(m_expected_flash_data.flash_handle, handle);
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX16(m_expected_flash_data.flash_group, handle & DSM_FLASH_HANDLE_FILTER_MASK);
        }
    }

    fm_entry_t * p_data   = malloc(sizeof(fm_header_t) + ((data_length + 3) & 0xFFFFFFFC));
    TEST_ASSERT_NOT_NULL(p_data);
    p_data->header.len_words = (sizeof(fm_header_t) + data_length + 3) / 4;
    p_data->header.handle = handle;
    return p_data;
}

static void flash_manager_entry_commit_cb(const fm_entry_t * p_entry, int call_count)
{
    if (m_expected_flash_data.verify_contents)
    {
        TEST_ASSERT_NOT_NULL(m_expected_flash_data.p_data);
        TEST_ASSERT_NOT_EQUAL(0, m_expected_flash_data.data_length);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_flash_data.p_data,
                                    p_entry->data,
                                    m_expected_flash_data.data_length);
    }
    if (m_expected_flash_data.p_data)
    {
        free(m_expected_flash_data.p_data);
    }
    free((fm_entry_t*) p_entry);
    m_expected_flash_data.p_data = NULL;
}


static void flash_expect(void * p_expected_data, uint32_t data_length)
{
    TEST_ASSERT_EQUAL_MESSAGE(NULL, m_expected_flash_data.p_data, "The previous flash operation has not been finished yet.");
    m_expected_flash_data.data_length = data_length;

    m_expected_flash_data.p_data = malloc(data_length);
    memcpy(m_expected_flash_data.p_data, p_expected_data, data_length);

    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_cb);
    flash_manager_entry_commit_StubWithCallback(flash_manager_entry_commit_cb);
}

static void flash_expect_metainfo(void)
{
    dsm_flash_entry_metainfo_t data;
    data.max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX;
    data.max_addrs_virtual    = DSM_VIRTUAL_ADDR_MAX;
    data.max_subnets          = DSM_SUBNET_MAX;
    data.max_appkeys          = DSM_APP_MAX;
    data.max_devkeys          = DSM_DEVICE_MAX;
    m_expected_flash_data.flash_group = 0;
    m_expected_flash_data.flash_handle = DSM_FLASH_HANDLE_METAINFO;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_unicast(dsm_local_unicast_address_t * p_addr)
{
    dsm_flash_entry_addr_unicast_t data;
    memcpy(&data.addr, p_addr, sizeof(data.addr));
    m_expected_flash_data.flash_group  = 0;
    m_expected_flash_data.flash_handle = DSM_FLASH_HANDLE_UNICAST;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_addr_nonvirtual(uint16_t addr)
{
    dsm_flash_entry_addr_nonvirtual_t data;
    memcpy(&data.addr, &addr, sizeof(data.addr));
    m_expected_flash_data.flash_group  = DSM_FLASH_GROUP_ADDR_NONVIRTUAL;
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_ADDR_NONVIRTUAL;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_addr_virtual(const uint8_t * p_uuid)
{
    dsm_flash_entry_addr_virtual_t data;
    memcpy(data.uuid, p_uuid, sizeof(data.uuid));
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_ADDR_VIRTUAL;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_subnet(const uint8_t * p_key, uint16_t key_index)
{
    dsm_flash_entry_subnet_t data;
    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_SUBNETS;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_subnet_update(const uint8_t * p_key, uint16_t key_index, dsm_handle_t handle)
{
    dsm_flash_entry_subnet_t data;
    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    m_expected_flash_data.flash_group = 0;
    m_expected_flash_data.flash_handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, handle);
    flash_expect(&data, sizeof(data));
}

static void flash_expect_appkey(const uint8_t * p_key, uint16_t key_index, dsm_handle_t subnet_handle)
{
    dsm_flash_entry_appkey_t data;
    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    memcpy(&data.subnet_handle, &subnet_handle, sizeof(data.subnet_handle));
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_APPKEYS;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_appkey_update(const uint8_t * p_key, uint16_t key_index, dsm_handle_t subnet_handle, dsm_handle_t app_handle)
{
    dsm_flash_entry_appkey_t data;
    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    memcpy(&data.subnet_handle, &subnet_handle, sizeof(data.subnet_handle));
    m_expected_flash_data.flash_group = 0;
    m_expected_flash_data.flash_handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app_handle);
    flash_expect(&data, sizeof(data));
}

static void flash_expect_devkey(const uint8_t * p_key, uint16_t key_owner, dsm_handle_t subnet_handle)
{
    dsm_flash_entry_devkey_t data;
    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_owner, &key_owner, sizeof(data.key_owner));
    memcpy(&data.subnet_handle, &subnet_handle, sizeof(data.subnet_handle));
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_DEVKEYS;
    flash_expect(&data, sizeof(data));
}

static void flash_invalidate_expect(uint16_t flash_handle)
{
    flash_manager_entry_invalidate_ExpectAndReturn(NULL, flash_handle, NRF_SUCCESS);
    flash_manager_entry_invalidate_IgnoreArg_p_manager();
}

static void flash_get_multiple_expect_start(void)
{
    mp_multiple_expect_prev = NULL;
}

static void flash_get_multiple_expect(dsm_entry_t *        p_entries,
                                      uint32_t             entry_count)
{

    for (uint32_t i = 0; i < entry_count; i++)
    {
        flash_manager_entry_next_get_ExpectAndReturn(NULL,
                                                     NULL,
                                                     (fm_entry_t *) mp_multiple_expect_prev,
                                                     (fm_entry_t *) &p_entries[i]);
        flash_manager_entry_next_get_IgnoreArg_p_manager();
        mp_multiple_expect_prev = &p_entries[i];
    }
}

static void flash_get_multiple_expect_end(void)
{
    /* last call yields NULL */
    flash_manager_entry_next_get_ExpectAndReturn(NULL, NULL, (fm_entry_t *) mp_multiple_expect_prev, NULL);
    flash_manager_entry_next_get_IgnoreArg_p_manager();
}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_addresses(void)
{
    /* local unicast */
    dsm_local_unicast_address_t unicast_get;

    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, unicast_get.address_start);
    TEST_ASSERT_EQUAL(0, unicast_get.count);

    dsm_local_unicast_address_t unicast = { 0x0000, 1};
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0xF000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0x8000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0x0002;
    unicast.count = 0x8000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.count = 0x0000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, unicast_get.address_start);
    TEST_ASSERT_EQUAL(0, unicast_get.count);
    unicast.count = 0x0003;
    flash_expect_unicast(&unicast);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_local_unicast_addresses_set(&unicast));
    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(2, unicast_get.address_start);
    TEST_ASSERT_EQUAL(3, unicast_get.count);

    /**** add addresses ****/
    struct
    {
        dsm_handle_t handle;
        uint16_t raw_address;
        bool in_list;
        bool duplicate;
        uint32_t expected_status;
    } addrs[] =
    {
        /* init the handle with something the module is unlikely to use, to
         * ensure that the module always changes the handle if successful. */
        {0xABAB,  0x1234, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0xFFFF, true,  false, NRF_SUCCESS},            /* group (all) */
        {0xABAB,  0xF010, true,  false, NRF_SUCCESS},            /* group */
        {0xABAB,  0x0000, false, false, NRF_ERROR_INVALID_ADDR}, /* invalid */
        {0xABAB,  0x80AB, false, false, NRF_ERROR_INVALID_ADDR}, /* virtual */
        {0xABAB,  0x1234, true,  true,  NRF_SUCCESS},            /* duplicate */
        {0xABAB,  0x0001, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0002, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0003, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0004, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0005, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0006, false, false, NRF_ERROR_NO_MEM},       /* full */ //FIXME: As this is a unicast address, it should probably not generate this error?
        {0xABAB,  0x0007, false, false, NRF_ERROR_NO_MEM},       /* full */
    };
    for (uint32_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u] (0x%04x)", i, addrs[i].raw_address);
        if (addrs[i].in_list && !addrs[i].duplicate)
        {
            flash_expect_addr_nonvirtual(addrs[i].raw_address);
        }
        TEST_ASSERT_EQUAL_MESSAGE(addrs[i].expected_status, dsm_address_publish_add(addrs[i].raw_address, &addrs[i].handle), errormsg);
        if (addrs[i].expected_status == NRF_SUCCESS)
        {
            TEST_ASSERT_NOT_EQUAL_MESSAGE(DSM_HANDLE_INVALID, addrs[i].handle, errormsg);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(0xABAB, addrs[i].handle, errormsg); /* The handle must have changed */
        }
    }

    /* illegal params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_add(0x0F12, NULL));

    /**** Remove addresses ****/
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, addrs[6].handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[6].handle));
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, addrs[7].handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[7].handle));
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, addrs[8].handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[8].handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(addrs[6].handle)); /* already deleted */

    const dsm_handle_t deleted_handle = addrs[8].handle;

    nrf_mesh_address_t addr;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(addrs[6].handle, &addr)); /* get deleted handle */

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(0x8888)); /* out of bounds */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(DSM_HANDLE_INVALID));
    addrs[6].in_list = false;
    addrs[7].in_list = false;
    addrs[8].in_list = false;

    /* Addresses 6, 7 and 8 are now removed. */

    /* re-add previous handle 6. */
    flash_expect_addr_nonvirtual(addrs[6].raw_address);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(addrs[6].raw_address, &addrs[6].handle));
    TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, addrs[6].handle);
    addrs[6].in_list = true;

    /**** Get addresses ****/
    for (uint32_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u] (0x%04x)", i, addrs[i].raw_address);
        if (addrs[i].in_list)
        {
            TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, dsm_address_get(addrs[i].handle, &addr), errormsg);
            nrf_mesh_address_type_t expected_type;
            if ((addr.value & 0xC000) == 0xC000)
            {
                expected_type = NRF_MESH_ADDRESS_TYPE_GROUP;
            }
            else
            {
                expected_type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            }
            TEST_ASSERT_EQUAL_PTR(NULL, addr.p_virtual_uuid);

            //FIXME: This fails for address 0xFFFF, as that is ignored by the module
            TEST_ASSERT_EQUAL_MESSAGE(expected_type, addr.type, errormsg);
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(addrs[i].raw_address, addr.value, errormsg);

            /* Get the handle back */
            dsm_handle_t handle = 0xABAB;
            TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, dsm_address_handle_get(&addr, &handle), errormsg);
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(addrs[i].handle, handle, errormsg);
        }
        else if (addrs[i].handle != DSM_HANDLE_INVALID)
        {
            TEST_ASSERT_EQUAL_MESSAGE(NRF_ERROR_NOT_FOUND, dsm_address_get(addrs[i].handle, &addr), errormsg);
        }
    }
    /* Invalid params: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(DSM_HANDLE_INVALID, &addr));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(deleted_handle, &addr)); /* deleted handle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get(0, NULL));
    dsm_handle_t handle = 0xABAB;
    addr.value = 0;
    addr.type = NRF_MESH_ADDRESS_TYPE_INVALID;
    addr.p_virtual_uuid = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, dsm_address_handle_get(&addr, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_handle_get(NULL, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_handle_get(&addr, NULL));
    addr.value = 0x01239;
    addr.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_handle_get(&addr, &handle));

    /**** Virtual addresses ****/
    struct
    {
        uint8_t uuid[NRF_MESH_UUID_SIZE];
        dsm_handle_t handle;
        uint16_t generated_short_addr;
        uint32_t expected_status;
    } virtual_uuids[] =
    {
        {{0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8001, NRF_SUCCESS},
        {{1, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8003, NRF_SUCCESS},
        {{2, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8004, NRF_SUCCESS},
        {{3, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8005, NRF_SUCCESS},
        {{4, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8006, NRF_ERROR_NO_MEM}, /* full */
        {{5, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8007, NRF_ERROR_NO_MEM}, /* full */
    };
    nrf_mesh_keygen_mock_Verify();
    for (uint32_t i = 0; i < sizeof(virtual_uuids) / sizeof(virtual_uuids[0]); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u]", i);
        if (virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuids[i].uuid, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
            nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_uuids[i].generated_short_addr, 2);
            flash_expect_addr_virtual(virtual_uuids[i].uuid);
        }
        TEST_ASSERT_EQUAL_MESSAGE(virtual_uuids[i].expected_status, dsm_address_publish_virtual_add(virtual_uuids[i].uuid, &virtual_uuids[i].handle), errormsg);
        nrf_mesh_keygen_mock_Verify();
        if (virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            TEST_ASSERT_NOT_EQUAL_MESSAGE(DSM_HANDLE_INVALID, virtual_uuids[i].handle, errormsg);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(0xABAB, virtual_uuids[i].handle, errormsg); /* The handle must have changed. */
        }
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_virtual_add(NULL, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_virtual_add(virtual_uuids[0].uuid, NULL));
    nrf_mesh_keygen_mock_Verify();

    /* delete virtual addresses */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, virtual_uuids[2].handle - DSM_NONVIRTUAL_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(virtual_uuids[2].handle));
    /* re-add, as we have the space again */
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuids[2].uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_uuids[2].generated_short_addr, 2);
    flash_expect_addr_virtual(virtual_uuids[2].uuid);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_virtual_add(virtual_uuids[2].uuid, &virtual_uuids[2].handle));
    /* delete again */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, virtual_uuids[2].handle - DSM_NONVIRTUAL_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(virtual_uuids[2].handle));
    virtual_uuids[2].handle = DSM_HANDLE_INVALID; /* mark as deleted in our book keeping */

    /* get virtual addrs */
    for (uint32_t i = 0; i < sizeof(virtual_uuids) / sizeof(virtual_uuids[0]); i++)
    {
        if (virtual_uuids[i].handle != DSM_HANDLE_INVALID && virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get(virtual_uuids[i].handle, &addr));
            TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_VIRTUAL, addr.type);
            TEST_ASSERT_NOT_NULL(addr.p_virtual_uuid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(virtual_uuids[i].uuid, addr.p_virtual_uuid, NRF_MESH_UUID_SIZE);
            TEST_ASSERT_EQUAL_HEX16(virtual_uuids[i].generated_short_addr, addr.value);

            /* Get the handle back */
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_handle_get(&addr, &handle));
            TEST_ASSERT_EQUAL_HEX16(virtual_uuids[i].handle, handle);
        }
    }

    /* Test get all */
    dsm_handle_t addr_list[DSM_ADDR_MAX];
    memset(addr_list, 0x1B, sizeof(addr_list));
    uint32_t count = DSM_ADDR_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(addr_list, &count));

    /* Make sure it holds all our addresses, and nothing else: */
    uint32_t expected_count = 0;
    bool found[DSM_ADDR_MAX];
    memset(found, 0, sizeof(found));

    for (uint32_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++)
    {
        bool address_found = false;
        if (addrs[i].in_list && !addrs[i].duplicate)
        {
            expected_count++;
        }
        for (uint32_t j = 0; j < count; j++)
        {
            if (addr_list[j] == addrs[i].handle)
            {
                TEST_ASSERT_TRUE(addrs[i].in_list);

                /* Make sure we don't find multiple of the same handle in the output. */
                TEST_ASSERT_FALSE(address_found);
                /* mark the address in this list found. */
                found[j] = true;
                break;
            }
        }
    }

    for (uint32_t i = 0; i < sizeof(virtual_uuids) / sizeof(virtual_uuids[0]); i++)
    {
        bool expected_in_list = (virtual_uuids[i].handle != DSM_HANDLE_INVALID && virtual_uuids[i].expected_status == NRF_SUCCESS);
        for (uint32_t j = 0; j < count; j++)
        {
            if (addr_list[j] == virtual_uuids[i].handle)
            {
                TEST_ASSERT_TRUE(expected_in_list);
                /* Make sure we don't find multiple of the same handle in the
                 * output. */
                TEST_ASSERT_FALSE(found[j]);
                /* mark the address in this list found. */
                found[j] = true;
                break;
            }
        }
        if (expected_in_list)
        {
            expected_count++;
        }
    }
    /* Check that all the outputted handles were recognized as something
     * we've inputted ourselves. */
    for (uint32_t i = 0; i < count; i++)
    {
        TEST_ASSERT_TRUE(found[i]);
    }
    TEST_ASSERT_EQUAL(expected_count, count);

    dsm_handle_t addr_list2[DSM_ADDR_MAX];
    /* Run again with exact length, should have the same result. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(addr_list2, &count));
    TEST_ASSERT_EQUAL(expected_count, count);
    TEST_ASSERT_EQUAL_HEX16_ARRAY(addr_list, addr_list2, count);

    /* Can't fit the last entry */
    count--;

    dsm_handle_t addr_list3[DSM_ADDR_MAX];
    memset(addr_list3, 0x1B, sizeof(addr_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_address_get_all(addr_list3, &count));
    TEST_ASSERT_EQUAL_HEX16_ARRAY(addr_list, addr_list3, count); /* Should be the same except for the last (missing) entry. */
    TEST_ASSERT_EQUAL_HEX16(0x1B1B, addr_list3[count]); /* last entry should be untouched. */

    /* invalid params */
    count = 0;
    memset(addr_list3, 0x1B, sizeof(addr_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_address_get_all(addr_list3, &count));
    TEST_ASSERT_EQUAL_HEX16(0x1B1B, addr_list3[0]); /* even the first entry should be untouched. */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(NULL, &count));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(NULL, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(addr_list3, NULL));
}

void test_rx_addr(void)
{
    /* add some addresses */
    dsm_handle_t handles[3];

    flash_expect_addr_nonvirtual(0xF001);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xF001, &handles[0]));
    flash_expect_addr_nonvirtual(0xF002);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xF002, &handles[1]));
    uint8_t uuid[NRF_MESH_UUID_SIZE] = {};
    uint16_t virtual_short_addr = 0x8001;
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_short_addr, sizeof(virtual_short_addr));
    flash_expect_addr_virtual(uuid);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(uuid, &handles[2]));

    /* Add them all as rx addresses */
    dsm_local_unicast_address_t unicast = { 0x0001, 1};
    flash_expect_unicast(&unicast);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_local_unicast_addresses_set(&unicast));
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, dsm_local_unicast_addresses_set(&unicast)); /* add again, should generate an error */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(DSM_HANDLE_INVALID));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(0x8887)); /* never referenced before */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(0x8888)); /* never referenced before */

    /* Test getter */
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[1]));
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[2]));
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[0]));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(0x8887));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(0x8888));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(DSM_HANDLE_INVALID));

    /* Test core lookup of rx addresses */
    nrf_mesh_address_t addr;

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(0xF001, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_GROUP, addr.type);
    TEST_ASSERT_EQUAL_HEX16(0xF001, addr.value);

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(0xF002, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_GROUP, addr.type);
    TEST_ASSERT_EQUAL_HEX16(0xF002, addr.value);

    /* Remove the second address from the RX list */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, handles[1]));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(handles[1]));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(handles[1])); /* re-remove, shouldn't work */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(handles[1])); /* re-add the handle, shouldn't work */

    /* second address no longer available */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0xF002, &addr));

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(virtual_short_addr, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_VIRTUAL, addr.type);
    TEST_ASSERT_NOT_NULL(addr.p_virtual_uuid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(uuid, addr.p_virtual_uuid, NRF_MESH_UUID_SIZE);
    TEST_ASSERT_EQUAL_HEX16(virtual_short_addr, addr.value);

    /* invalid addresses */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x0002, &addr)); /* unicast */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0xF002, &addr)); /* group */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x8002, &addr)); /* virtual */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x0000, &addr)); /* invalid */

    //TODO: Test sublist overflow
}

void test_net(void)
{
    struct
    {
        bool in_the_list;
        uint16_t key_index;
        uint8_t nid;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } net[] =
    {
        {true,  0,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS},
        {true,  1,      0x11, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_SUCCESS},
        {true,  2,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS}, /* same key as first, different index (allowed) */
        {false, 0,      0x11, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, different key (not allowed) */
        {false, 0,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, same key (not allowed) */
        {false, 0xF000, 0x11, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_ERROR_INVALID_PARAM}, /* key index out of bounds */
        {true,  3,      0x22, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, 0xABCD, NRF_SUCCESS},
        {true,  4,      0x22, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_SUCCESS},
        {true,  5,      0x33, {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5}, 0xABCD, NRF_SUCCESS},
        {true,  6,      0x33, {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6}, 0xABCD, NRF_SUCCESS},
        {true,  7,      0x33, {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}, 0xABCD, NRF_SUCCESS},
        {false, 8,      0x22, {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, 0xABCD, NRF_ERROR_NO_MEM}, /* too many keys */
    };

/* temporary define, to get rid of common pattern */
#define NET_COUNT   (sizeof(net) / sizeof(net[0]))

    nrf_mesh_network_secmat_t net_secmat;
    net_secmat.nid = 0x51;
    memset(net_secmat.privacy_key, 0xAA, NRF_MESH_KEY_SIZE);
    memset(net_secmat.encryption_key, 0xBB, NRF_MESH_KEY_SIZE);

    nrf_mesh_beacon_secmat_t beacon_secmat;
    memset(beacon_secmat.net_id, 0xCC, NRF_MESH_NETID_SIZE);
    memset(beacon_secmat.key, 0xDD, NRF_MESH_KEY_SIZE);

    uint8_t identity_key[NRF_MESH_KEY_SIZE];
    memset(identity_key, 0xEE, NRF_MESH_KEY_SIZE);

    /**** Add ****/
    for (uint32_t i = 0; i < NET_COUNT; i++)
    {
        net_secmat.nid = net[i].nid;
        /* expected keygen calls */
        if (net[i].in_the_list)
        {
            nrf_mesh_keygen_network_secmat_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net_secmat, sizeof(net_secmat));

            nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&beacon_secmat, sizeof(beacon_secmat));

#if GATT_PROXY
            nrf_mesh_keygen_identitykey_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
            nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(identity_key, NRF_MESH_KEY_SIZE);
#endif
            flash_expect_subnet(net[i].key, net[i].key_index);
        }
        /* add the net */
        TEST_ASSERT_EQUAL(net[i].expected_status, dsm_subnet_add(net[i].key_index, net[i].key, &net[i].handle));
        if (net[i].in_the_list)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, net[i].expected_status);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, net[i].handle);
            TEST_ASSERT_NOT_EQUAL(0xABCD, net[i].handle); /* The handle must have changed */
        }
        else
        {
            TEST_ASSERT_NOT_EQUAL(NRF_SUCCESS, net[i].expected_status);
        }
    }
    /* invalid params */
    dsm_handle_t handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_add(9, NULL, &handle));
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_add(9, key, NULL));

    /**** Update ****/
    uint8_t update_indexes[] = {0, 1, 6, 9};
    for (uint32_t i = 0; i < sizeof(update_indexes); i++)
    {
        memset(net[update_indexes[i]].key, 0x44, NRF_MESH_KEY_SIZE);
        net[update_indexes[i]].nid = 0x44;
        net_secmat.nid = 0x44;

        /* expected keygen calls */
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(net[update_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net_secmat, sizeof(net_secmat));

        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(net[update_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&beacon_secmat, sizeof(beacon_secmat));

#if GATT_PROXY
        nrf_mesh_keygen_identitykey_ExpectAndReturn(net[update_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
        nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(identity_key, NRF_MESH_KEY_SIZE);
#endif

        flash_expect_subnet_update(net[update_indexes[i]].key,
                                   net[update_indexes[i]].key_index,
                                   net[update_indexes[i]].handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(net[update_indexes[i]].handle, net[update_indexes[i]].key));
    }
    nrf_mesh_keygen_mock_Verify();
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(DSM_HANDLE_INVALID, key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(0x8888, key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_update(net[0].handle, NULL));

    /**** Delete ****/
    uint8_t delete_indexes[] = {0, 8};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, net[delete_indexes[i]].handle));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_delete(net[delete_indexes[i]].handle));
        net[delete_indexes[i]].in_the_list = false;
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(net[delete_indexes[0]].handle));      /* already deleted */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(net[delete_indexes[0]].handle, key)); /* already deleted */

    /**** Re-add ****/
    /* since we've deleted a couple, we should be able to add some more again */
    uint8_t readd_indexes[] = {
        3, /* the one that was rejected for having the same index as net[0], which has been deleted */
        11, /* The one that was rejected because we were full */
    };
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        net_secmat.nid = net[readd_indexes[i]].nid;

        /* expected keygen calls */
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net_secmat, sizeof(net_secmat));

        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&beacon_secmat, sizeof(beacon_secmat));

#if GATT_PROXY
        nrf_mesh_keygen_identitykey_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
        nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(identity_key, NRF_MESH_KEY_SIZE);
#endif

        flash_expect_subnet(net[readd_indexes[i]].key, net[readd_indexes[i]].key_index);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(net[readd_indexes[i]].key_index, net[readd_indexes[i]].key, &net[readd_indexes[i]].handle));
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, net[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(0xABCD, net[readd_indexes[i]].handle); /* The handle must have changed */
        net[readd_indexes[i]].in_the_list = true;
    }

    /**** Get ****/
    uint32_t count = NET_COUNT;
    bool found_keys[NET_COUNT];
    memset(found_keys, 0, sizeof(found_keys));
    mesh_key_index_t key_indexes[NET_COUNT];
    memset(key_indexes, 0xAB, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_get_all(key_indexes, &count));

    uint32_t found_key_count = 0;
    for (uint32_t i = 0; i < count; i++)
    {
        for (uint32_t j = 0; j < NET_COUNT; j++)
        {
            if (net[j].key_index == key_indexes[i] && net[j].in_the_list)
            {
                TEST_ASSERT_FALSE(found_keys[j]);
                found_key_count++;

                found_keys[j] = true;
                /* don't break on found entry, want to check that we only find a single one */
            }
        }
    }
    /* Check that all keys that were expected to be found were found. */
    for (uint32_t j = 0; j < NET_COUNT; j++)
    {
        TEST_ASSERT_EQUAL(found_keys[j], net[j].in_the_list);
    }
    TEST_ASSERT_EQUAL(count, found_key_count);

    /* fetch again, with exact size: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_get_all(key_indexes, &count));
    /* fetch with too few spots */
    count--;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(found_key_count - 1, count);
    /* give 1, only the first one should be set */
    count = 1;
    memset(key_indexes, 0xAB, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_NOT_EQUAL(0xABAB, key_indexes[0]);
    for (uint32_t i = 1; i < NET_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_HEX16(0xABAB, key_indexes[i]);
    }
    /* give 0, nothing should be done. */
    count = 0;
    memset(key_indexes, 0x10, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(0, count);
    for (uint32_t i = 0; i < NET_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_HEX16(0x1010, key_indexes[i]);
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(NULL, &count));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(key_indexes, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(NULL, NULL));

    /**** Secmat Get ****/
    struct
    {
        uint8_t nid;
        uint32_t count;
    } nid_groups[] =
    {
        {0x11, 2},
        {0x22, 2},
        {0x33, 1},
        {0x44, 3},
    };
    const nrf_mesh_network_secmat_t * p_secmat = NULL;
    for (uint32_t i = 0; i < sizeof(nid_groups) / sizeof(nid_groups[0]); i++)
    {
        for (uint32_t j = 0; j < nid_groups[i].count; j++)
        {
            nrf_mesh_net_secmat_next_get(nid_groups[i].nid, &p_secmat);
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(net_secmat.privacy_key,    p_secmat->privacy_key,    NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(net_secmat.encryption_key, p_secmat->encryption_key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_HEX8(nid_groups[i].nid, p_secmat->nid);

            /* Check that the secmats match their handles */
            handle = dsm_subnet_handle_get(p_secmat);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, handle);
            /* search for this handle */
            bool found = false;
            for (uint32_t k = 0; k < NET_COUNT; k++)
            {
                if (net[k].in_the_list && net[k].handle == handle)
                {
                    found = true;
                    TEST_ASSERT_EQUAL(nid_groups[i].nid, net[k].nid);
                    break;
                }
            }
            TEST_ASSERT_TRUE(found);
        }
        nrf_mesh_net_secmat_next_get(nid_groups[i].nid, &p_secmat);
        TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, "Found more networks than expected");
    }

    nrf_mesh_net_secmat_next_get(0x55, &p_secmat); /* no such nid */
    TEST_ASSERT_EQUAL(NULL, p_secmat);
    nrf_mesh_network_secmat_t dummy_secmat = {};
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_subnet_handle_get(&dummy_secmat)); /* not in the list */

    /* Bind one app and one devkey to net[1] */
    uint8_t dummy_key[NRF_MESH_KEY_SIZE] = {};
    dsm_handle_t app_handle;
    nrf_mesh_keygen_aid_IgnoreAndReturn(NRF_SUCCESS);
    flash_expect_appkey(dummy_key, 0, net[1].handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(0, net[1].handle, dummy_key, &app_handle));
    dsm_handle_t devkey_handle;
    flash_expect_devkey(dummy_key, 0x0001, net[1].handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_add(0x0001, net[1].handle, dummy_key, &devkey_handle));

    /* Check if the devkey handle retrieval function works: */
    dsm_handle_t test_devkey_handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_handle_get(0x0001, &test_devkey_handle));
    TEST_ASSERT_EQUAL(test_devkey_handle, devkey_handle);

    /* Delete the app */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_delete(app_handle));
    /* Delete the devkey */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, devkey_handle - DSM_APP_MAX));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(devkey_handle));
    /* Delete the network and the keys. */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, net[1].handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_delete(net[1].handle));

    /* Check that the appkey and the devkey were actually deleted: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(app_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(devkey_handle));

#undef NET_COUNT
}

void test_app(void)
{
    struct
    {
        bool in_the_list;
        uint16_t key_index;
        dsm_handle_t net_handle;
        uint8_t aid;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } app[] =
    {
        {true,  0,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS},
        {true,  1,      DSM_HANDLE_INVALID, 0x01, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_SUCCESS},
        {true,  2,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS}, /* same key as first, different index (allowed) */
        {false, 0,      DSM_HANDLE_INVALID, 0x01, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, different key (not allowed) */
        {false, 0,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, same key (not allowed) */
        {false, 0xF000, DSM_HANDLE_INVALID, 0x01, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_ERROR_INVALID_PARAM}, /* key index out of bounds */
        {true,  3,      DSM_HANDLE_INVALID, 0x02, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, 0xABCD, NRF_SUCCESS},
        {true,  4,      DSM_HANDLE_INVALID, 0x02, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_SUCCESS},
        {true,  5,      DSM_HANDLE_INVALID, 0x03, {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5}, 0xABCD, NRF_SUCCESS},
        {true,  6,      DSM_HANDLE_INVALID, 0x03, {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6}, 0xABCD, NRF_SUCCESS},
        {true,  7,      DSM_HANDLE_INVALID, 0x03, {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}, 0xABCD, NRF_SUCCESS},
        {false, 8,      DSM_HANDLE_INVALID, 0x02, {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, 0xABCD, NRF_ERROR_NO_MEM}, /* too many keys */
    };

/* temporary define, to get rid of common pattern */
#define APP_COUNT   (sizeof(app) / sizeof(app[0]))
    /* Add two networks, and assign the apps to different ones */
    dsm_handle_t net_handles[2];
    uint8_t net_key[NRF_MESH_KEY_SIZE] = {};
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);

    flash_expect_subnet(net_key, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, net_key, &net_handles[0]));
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);
    flash_expect_subnet(net_key, 1);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(1, net_key, &net_handles[1]));

    for (uint32_t i = 0; i < APP_COUNT; i++)
    {
        app[i].net_handle = net_handles[i % 2];
    }

    /**** Add ****/
    for (uint32_t i = 0; i < APP_COUNT; i++)
    {
        /* expected keygen calls */
        if (app[i].in_the_list)
        {
            nrf_mesh_keygen_aid_ExpectAndReturn(app[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_aid_IgnoreArg_p_aid();
            nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, 1);
            flash_expect_appkey(app[i].key, app[i].key_index, app[i].net_handle);
        }

        /* add the app */
        TEST_ASSERT_EQUAL(app[i].expected_status, dsm_appkey_add(app[i].key_index, app[i].net_handle, app[i].key, &app[i].handle));
        if (app[i].in_the_list)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, app[i].expected_status);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, app[i].handle);
            TEST_ASSERT_NOT_EQUAL(0xABCD, app[i].handle); /* The handle must have changed */
        }
        else
        {
            TEST_ASSERT_NOT_EQUAL(NRF_SUCCESS, app[i].expected_status);
        }
    }
    /* invalid params */

    dsm_handle_t handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_add(9, net_handles[0], NULL, &handle));
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_add(9, net_handles[0], key, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_add(9, 0xABAB, key, &handle)); /* invalid nethandle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_add(4, 7, key, &handle)); /* unallocated nethandle */

    /**** Update ****/
    uint8_t update_indexes[] = {0, 1, 6, 9};
    for (uint32_t i = 0; i < sizeof(update_indexes); i++)
    {
        memset(app[update_indexes[i]].key, 0x44, NRF_MESH_KEY_SIZE);
        app[update_indexes[i]].aid = 0x04;

        /* expected keygen calls */
        nrf_mesh_keygen_aid_ExpectAndReturn(app[update_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[update_indexes[i]].aid, 1);
        flash_expect_appkey_update(app[update_indexes[i]].key,
                                   app[update_indexes[i]].key_index,
                                   app[update_indexes[i]].net_handle,
                                   app[update_indexes[i]].handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_update(app[update_indexes[i]].handle, app[update_indexes[i]].key));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(DSM_HANDLE_INVALID, key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(0x8888, key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_update(app[0].handle, NULL));

    /**** Delete ****/
    uint8_t delete_indexes[] = {0, 8};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[delete_indexes[i]].handle));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_delete(app[delete_indexes[i]].handle));
        app[delete_indexes[i]].in_the_list = false;
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(delete_indexes[0]));      /* already deleted */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(delete_indexes[0], key)); /* already deleted */

    /**** Re-add ****/
    /* since we've deleted a couple, we should be able to add some more again */
    uint8_t readd_indexes[] = {
        3, /* the one that was rejected for having the same index as app[0], which has been deleted */
        11, /* The one that was rejected because we were full */
    };
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        /* expected keygen calls */
        nrf_mesh_keygen_aid_ExpectAndReturn(app[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[readd_indexes[i]].aid, 1);

        flash_expect_appkey(app[readd_indexes[i]].key,
                            app[readd_indexes[i]].key_index,
                            app[readd_indexes[i]].net_handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(app[readd_indexes[i]].key_index, app[readd_indexes[i]].net_handle, app[readd_indexes[i]].key, &app[readd_indexes[i]].handle));
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, app[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(0xABCD, app[readd_indexes[i]].handle); /* The handle must have changed */
        app[readd_indexes[i]].in_the_list = true;
    }

    /**** Get ****/
    uint32_t keys_in_storage = 0;

    for (uint32_t net = 0; net < 2; net++)
    {
        uint32_t count = APP_COUNT;
        bool found_keys[APP_COUNT];
        memset(found_keys, 0, sizeof(found_keys));
        mesh_key_index_t key_indexes[APP_COUNT];
        memset(key_indexes, 0xAB, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_get_all(net_handles[net], key_indexes, &count));

        uint32_t found_key_count = 0;
        for (uint32_t i = 0; i < count; i++)
        {
            for (uint32_t j = 0; j < APP_COUNT; j++)
            {
                if (app[j].key_index == key_indexes[i] && app[j].in_the_list)
                {
                    TEST_ASSERT_FALSE(found_keys[j]);
                    found_key_count++;

                    found_keys[j] = true;
                    /* don't break on found entry, want to check that we only find a single one */
                }
            }
        }
        /* Check that all keys that were expected to be found were found. */
        for (uint32_t j = 0; j < APP_COUNT; j++)
        {
            char errormsg[128];
            sprintf(errormsg, "(%d [0x%04x])", j, app[j].key_index);
            TEST_ASSERT_EQUAL_MESSAGE(found_keys[j], (app[j].in_the_list && app[j].net_handle == net_handles[net]), errormsg);
        }
        TEST_ASSERT_EQUAL(count, found_key_count);

        /* fetch again, with exact size: */
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        /* fetch with too few spots */
        count--;
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[1], key_indexes, &count));
        TEST_ASSERT_EQUAL(found_key_count - 1, count);
        /* give 1, only the first one should be set */
        count = 1;
        memset(key_indexes, 0xAB, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        TEST_ASSERT_EQUAL(1, count);
        TEST_ASSERT_NOT_EQUAL(0xABAB, key_indexes[0]);
        for (uint32_t i = 1; i < APP_COUNT; i++)
        {
            TEST_ASSERT_EQUAL_HEX16(0xABAB, key_indexes[i]);
        }
        /* give 0, nothing should be done. */
        count = 0;
        memset(key_indexes, 0x10, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        TEST_ASSERT_EQUAL(0, count);
        for (uint32_t i = 0; i < APP_COUNT; i++)
        {
            TEST_ASSERT_EQUAL_HEX16(0x1010, key_indexes[i]);
        }
        keys_in_storage += found_key_count;

        /* invalid params */
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_get_all(0x8888, key_indexes, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_get_all(DSM_HANDLE_INVALID, key_indexes, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], NULL, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], key_indexes, NULL));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], NULL, NULL));
    }

    /**** Secmat Get ****/
    struct
    {
        uint8_t network_index;
        uint8_t aid;
        uint32_t count;
    } aid_groups[] =
    {
        {0, 0x01, 1},
        {0, 0x02, 0},
        {0, 0x03, 1},
        {0, 0x04, 1},
        {1, 0x01, 1},
        {1, 0x02, 2},
        {1, 0x03, 0},
        {1, 0x04, 2},
    };
    const nrf_mesh_network_secmat_t * p_net_secmats[2] = {NULL, NULL};
    for (uint32_t net = 0; net < 2; net++)
    {
        /* Find the right network secmat (not really a part of the test,
         * just necessary for getting the input parameters in
         * app_secmat_next_get())*/
        nrf_mesh_secmat_t secmat;
        for (uint32_t j = 0; j < APP_COUNT; j++)
        {
            if (app[j].in_the_list && app[j].net_handle == net_handles[net])
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(app[j].handle, &secmat));
                TEST_ASSERT_NOT_NULL(secmat.p_net);
                p_net_secmats[net] = secmat.p_net;
                break;
            }
        }
    }
    const nrf_mesh_application_secmat_t * p_secmat = NULL;
    uint32_t represented_aids = 0;
    for (uint32_t i = 0; i < sizeof(aid_groups) / sizeof(aid_groups[0]); i++)
    {
        represented_aids += aid_groups[i].count;
        for (uint32_t net = 0; net < 2; net++)
        {
            if (aid_groups[i].network_index != net)
            {
                continue;
            }
            p_secmat = NULL;
            for (uint32_t j = 0; j < aid_groups[i].count; j++)
            {
                nrf_mesh_app_secmat_next_get(p_net_secmats[net], aid_groups[i].aid, &p_secmat);
                TEST_ASSERT_NOT_NULL(p_secmat);
                TEST_ASSERT_EQUAL_HEX8(aid_groups[i].aid, p_secmat->aid);

                /* Check that the secmats match their handles */
                handle = dsm_appkey_handle_get(p_secmat);
                TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, handle);
                /* search for this handle */
                bool found = false;
                for (uint32_t k = 0; k < APP_COUNT; k++)
                {
                    if (app[k].in_the_list && app[k].handle == handle)
                    {
                        found = true;
                        TEST_ASSERT_EQUAL(aid_groups[i].aid, app[k].aid);
                        break;
                    }
                }
                TEST_ASSERT_TRUE(found);
            }
            nrf_mesh_app_secmat_next_get(p_net_secmats[net], aid_groups[i].aid, &p_secmat);
            TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, "Found more applications than expected");
        }
    }
    /* Check that we've covered all aid's in storage */
    TEST_ASSERT_EQUAL(keys_in_storage, represented_aids);

    /* Illegal params */
    nrf_mesh_app_secmat_next_get(p_net_secmats[0], 0x55, &p_secmat); /* no such aid */
    TEST_ASSERT_EQUAL(NULL, p_secmat);
    nrf_mesh_application_secmat_t dummy_secmat = {};
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_appkey_handle_get(&dummy_secmat)); /* not in the list */
    nrf_mesh_network_secmat_t dummy_net_secmat = {};
    nrf_mesh_app_secmat_next_get(&dummy_net_secmat, aid_groups[0].aid, &p_secmat); /* net secmat not in the list */
    TEST_ASSERT_EQUAL_PTR(NULL, p_secmat);
    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_app_secmat_next_get(NULL, aid_groups[0].aid, &p_secmat));
    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_app_secmat_next_get(p_net_secmats[0], aid_groups[0].aid, NULL));
#undef APP_COUNT
}

void test_devkey(void)
{
    dsm_handle_t subnet_handle;
    uint8_t dummy_key[NRF_MESH_KEY_SIZE] = {};
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);
    flash_expect_subnet(dummy_key, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, dummy_key, &subnet_handle));

    struct
    {
        bool in_the_list;
        bool duplicate;
        uint16_t owner;
        dsm_handle_t net_handle;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } devkey[] =
    {
        {true,  false, 0x1000, subnet_handle, {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}, 0xABCD, NRF_SUCCESS},
        {true,  false, 0x1001, subnet_handle, {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}, 0xABCD, NRF_SUCCESS},
        {true,  false, 0x1002, subnet_handle, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_SUCCESS},
        {false, false, 0x1009, 0xabcd,        {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_ERROR_NOT_FOUND}, /* Non-existing network handle */
        {true,  false, 0x1003, subnet_handle, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_SUCCESS}, /* duplicate key (okay) */
        {false, false, 0x0000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, false, 0x8000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, false, 0xC000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, true,  0x1002, subnet_handle, {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* duplicate address */
        {false, false, 0x1004, subnet_handle, {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66}, 0xABCD, NRF_ERROR_NO_MEM}, /* Full */
    };
#define DEVKEYS     (sizeof(devkey) / sizeof(devkey[0]))

    /* Add devkeys */
    for (uint32_t i = 0; i < DEVKEYS; i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        if (devkey[i].in_the_list && !devkey[i].duplicate)
        {
            flash_expect_devkey(devkey[i].key, devkey[i].owner, devkey[i].net_handle);
        }
        TEST_ASSERT_EQUAL_MESSAGE(devkey[i].expected_status, dsm_devkey_add(devkey[i].owner, devkey[i].net_handle, devkey[i].key, &devkey[i].handle), err_msg);
        if (devkey[i].in_the_list)
        {
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, devkey[i].handle);
        }
    }
    /* invalid params */
    dsm_handle_t dummy_handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, dsm_devkey_add(NRF_MESH_ADDR_UNASSIGNED, subnet_handle, dummy_key, &dummy_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_devkey_add(0x1234, subnet_handle, NULL, &dummy_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_devkey_add(0x1235, subnet_handle, dummy_key, NULL));

    /* delete some */
    uint8_t delete_indexes[] = {0, 1};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, devkey[delete_indexes[i]].handle - DSM_APP_MAX));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(devkey[delete_indexes[i]].handle));
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(devkey[delete_indexes[i]].handle)); /* already removed */
        devkey[delete_indexes[i]].in_the_list = false;
    }
    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(0xABCD));

    /* Re-add, as we have space now */
    uint8_t readd_indexes[] = {9};
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        flash_expect_devkey(devkey[readd_indexes[i]].key,
                            devkey[readd_indexes[i]].owner,
                            devkey[readd_indexes[i]].net_handle);
        TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS,
                dsm_devkey_add(devkey[readd_indexes[i]].owner,
                    devkey[readd_indexes[i]].net_handle,
                    devkey[readd_indexes[i]].key,
                    &devkey[readd_indexes[i]].handle), err_msg);
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, devkey[readd_indexes[i]].handle);
        devkey[readd_indexes[i]].in_the_list = true;
    }

    /* Get rx secmats for all devkeys */
    for (uint32_t i = 0; i < DEVKEYS; i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        const nrf_mesh_application_secmat_t * p_secmat = NULL;
        nrf_mesh_devkey_secmat_get(devkey[i].owner, &p_secmat);
        if (devkey[i].in_the_list)
        {
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(devkey[i].key, p_secmat->key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_TRUE(p_secmat->is_device_key);
            TEST_ASSERT_EQUAL(0, p_secmat->aid);
        }
        else if (devkey[i].duplicate)
        {
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_TRUE(p_secmat->is_device_key);
            TEST_ASSERT_EQUAL(0, p_secmat->aid);
        }
        else
        {
            TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, err_msg);
        }
    }

#undef DEVKEYS
}

void test_secmat(void)
{
    nrf_mesh_secmat_t secmat;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(0, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(0x8888, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, &secmat));

    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    for (uint32_t i = 0; i < sizeof(net) / sizeof(net[0]); i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));
        flash_expect_subnet(key, i);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i, key, &net[i].handle));
    }
    /* Add some dummy apps */
    struct
    {
        dsm_handle_t handle;
        uint8_t aid;
    } app[6];
    for (uint32_t i = 0; i < sizeof(app) / sizeof(app[0]); i++)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, sizeof(app[i].aid));
        flash_expect_appkey(key, i, net[i / 2].handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, net[i / 2].handle, key, &app[i].handle));
    }
    /* Add some dummy device keys */
    struct
    {
        uint16_t owner;
        dsm_handle_t handle;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } dev[4];
    for (uint32_t i = 0; i < sizeof(dev) / sizeof(dev[0]); i++)
    {
        dev[i].owner = i + 0x1000;
        memset(dev[i].key, i, NRF_MESH_KEY_SIZE);
        flash_expect_devkey(dev[i].key, dev[i].owner, net[0].handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_add(dev[i].owner, net[0].handle, dev[i].key, &dev[i].handle));
    }

    /* Get tx secmats for apps */
    for (uint32_t i = 0; i < sizeof(app) / sizeof(app[0]); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(app[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_NOT_NULL(secmat.p_app);

        TEST_ASSERT_EQUAL_MEMORY(&net[i / 2].secmat, secmat.p_net, sizeof(nrf_mesh_network_secmat_t));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL(app[i].aid, secmat.p_app->aid);
        TEST_ASSERT_EQUAL(false, secmat.p_app->is_device_key);
    }
    /* Get tx secmats for devkeys */
    for (uint32_t i = 0; i < sizeof(dev) / sizeof(dev[0]); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(dev[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_NOT_NULL(secmat.p_app);

        TEST_ASSERT_EQUAL_MEMORY(&net[0].secmat, secmat.p_net, sizeof(nrf_mesh_network_secmat_t));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(dev[i].key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL(0, secmat.p_app->aid);
        TEST_ASSERT_EQUAL(true, secmat.p_app->is_device_key);
    }
    /* Delete a devkey, ensure we can't get a secmat for it anymore. */
    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[0].handle - DSM_APP_MAX));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(dev[0].handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(dev[0].handle, &secmat));

    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(0x8888, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_tx_secmat_get(0, NULL));

    /* get beacon secmats */
    const nrf_mesh_beacon_secmat_t * p_beacon_secmat = NULL;
    for (uint32_t i = 0; i < sizeof(net) / sizeof(net[0]); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_beacon_secmat_get(net[i].handle, &p_beacon_secmat));
        TEST_ASSERT_EQUAL_MEMORY(&net[i].beacon_secmat, p_beacon_secmat, sizeof(nrf_mesh_beacon_secmat_t));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_beacon_secmat_get(sizeof(net) / sizeof(net[0]), &p_beacon_secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_beacon_secmat_get(0x8888, &p_beacon_secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_beacon_secmat_get(0, NULL));

}

void test_beacon_info_get(void)
{
    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};

    // ONLY ADD TWO FIRST NETWORKS, WE'RE SAVING THE THIRD FOR LATER!
    for (uint32_t i = 0; i < 2; i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));
        flash_expect_subnet(key, i + 10);
        // Make sure we don't add net key index 0 yet, as that's the primary network
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i + 10, key, &net[i].handle));
    }
    /* Fetch without filters: */
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_interval_seconds);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_timestamp);
    /* Manipulate the tx info, to ensure it'll remain the same across multiple gets */
    p_beacon_info->p_tx_info->rx_count = 89;
    p_beacon_info->p_tx_info->tx_interval_seconds = 123;
    p_beacon_info->p_tx_info->tx_timestamp = 456;

    /* Get the first one again, should yield the same thing! */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(89, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(123, p_beacon_info->p_tx_info->tx_interval_seconds);
    TEST_ASSERT_EQUAL(456, p_beacon_info->p_tx_info->tx_timestamp);

    /* Get the next in the list */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_interval_seconds);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_timestamp);

    /* No more networks, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Now add the primary network, with a duplicate network ID. */
    memset(net[2].secmat.privacy_key, 2, NRF_MESH_KEY_SIZE);
    memset(net[2].secmat.encryption_key, 2 + 0x10, NRF_MESH_KEY_SIZE);
    memset(net[2].beacon_secmat.key, 2 + 0x20, NRF_MESH_KEY_SIZE);
    memset(net[2].beacon_secmat.net_id, 0x30, NRF_MESH_NETID_SIZE); // same net ID as the first network
    net[2].secmat.nid = 2;
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[2].secmat, sizeof(net[2].secmat));
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[2].beacon_secmat, sizeof(net[2].beacon_secmat));
    flash_expect_subnet(key, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, key, &net[2].handle)); // Network index 0 is the primary network

    /* Run the getters again, this time, only the primary network should have iv_update_permitted set. */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_FALSE(p_beacon_info->iv_update_permitted); // Not a primary network
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_FALSE(p_beacon_info->iv_update_permitted); // Not a primary network
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[2].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // The primary network!

    /* No more networks, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get with NetID filters: */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    /* Since the last network has the same network ID as the first, we should
     * get that one on the next iteration (but skip the one with a different
     * NetID) */
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[2].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));

    /* No more networks with a matching NetID, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get the unique Net ID */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(net[1].beacon_secmat.net_id, &p_beacon_info);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));

    /* No more networks with a matching NetID, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(net[1].beacon_secmat.net_id, &p_beacon_info);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get non-existing NetID, no results. */
    uint8_t dummy_net_id[NRF_MESH_NETID_SIZE];
    memset(dummy_net_id, 0xFE, NRF_MESH_NETID_SIZE);
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(dummy_net_id, &p_beacon_info);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_beacon_info_next_get(NULL, NULL));
}

void test_address_subcount_regular(void)
{
    uint16_t count = 0xffff;
    dsm_handle_t address_handle;
    flash_expect_addr_nonvirtual(0xc442);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xc442, &address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xc442, &address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(2, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_count_get(address_handle, &count));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(address_handle));
}

void test_address_subcount_virtual(void)
{
    const uint8_t virtual_uuid[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    uint16_t virtual_address = 0xabcd; /* This is not the actual address corresponding to the above UUID */
    dsm_handle_t address_handle;
    uint16_t count = 0xffff;

    nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&virtual_address);
    flash_expect_addr_virtual(virtual_uuid);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(virtual_uuid, &address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add_handle(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(2, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    flash_invalidate_expect(DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, address_handle - DSM_NONVIRTUAL_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_count_get(address_handle, &count));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(address_handle));
}

void test_getters(void)
{
    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    for (uint32_t i = 0; i < sizeof(net) / sizeof(net[0]); i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));
        flash_expect_subnet(key, i);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i, key, &net[i].handle));

        uint16_t index = 0xffff;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_handle_to_netkey_index(net[i].handle, &index));
        TEST_ASSERT_EQUAL(i, index);
    }
    /* Add some dummy apps */
    struct
    {
        dsm_handle_t handle;
        uint8_t aid;
    } app[6];
    for (uint32_t i = 0; i < sizeof(app) / sizeof(app[0]); i++)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, sizeof(app[i].aid));
        flash_expect_appkey(key, i, net[i / 2].handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, net[i / 2].handle, key, &app[i].handle));
    }

    for (uint32_t i = 0; i < sizeof(net) / sizeof(net[0]); i++)
    {
        TEST_ASSERT_EQUAL(net[i].handle, dsm_net_key_index_to_subnet_handle(i));
    }
    for (uint32_t i = 0; i < sizeof(app) / sizeof(app[0]); i++)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(app[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_EQUAL(net[i/2].handle, dsm_subnet_handle_get(secmat.p_net));
        TEST_ASSERT_NOT_NULL(secmat.p_app);
        TEST_ASSERT_EQUAL(app[i].handle, dsm_appkey_handle_get(secmat.p_app));

        TEST_ASSERT_EQUAL(app[i].handle, dsm_app_key_index_to_appkey_handle(i));

        uint16_t index = 0xffff;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_handle_to_appkey_index(i, &index));
        TEST_ASSERT_EQUAL(i, index);
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_appkey_handle_get(NULL));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_subnet_handle_get(NULL));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_net_key_index_to_subnet_handle(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_app_key_index_to_appkey_handle(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_handle_to_appkey_index(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_handle_to_netkey_index(0, NULL));

    uint16_t testvar;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_handle_to_appkey_index(DSM_HANDLE_INVALID, &testvar));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_handle_to_netkey_index(DSM_HANDLE_INVALID, &testvar));
}

void test_key_refresh(void)
{
    TEST_IGNORE_MESSAGE("Not properly implemented yet?");
}

/** Helper macro for flash load testing */
#define FLASH_ENTRY_GET_EXPECT(HANDLE, RETVAL)                                          \
    do                                                                                  \
    {                                                                                   \
        flash_manager_entry_get_ExpectAndReturn(NULL, HANDLE, (fm_entry_t *) (RETVAL)); \
        flash_manager_entry_get_IgnoreArg_p_manager();                                  \
    } while (0)




void test_flash_load(void)
{
    flash_manager_add_StubWithCallback(NULL); /* want custom checking on this */
    FLASH_ENTRY_GET_EXPECT(DSM_FLASH_HANDLE_METAINFO, NULL);
    TEST_ASSERT_FALSE(dsm_flash_config_load());

    /* Gradually add more entries until we get an acceptable flash config */

    dsm_entry_t metainfo;

    metainfo.entry.metainfo.max_subnets = DSM_SUBNET_MAX - 1; /* invalid! */
    metainfo.entry.metainfo.max_appkeys = DSM_APP_MAX;
    metainfo.entry.metainfo.max_devkeys = DSM_DEVICE_MAX;
    metainfo.entry.metainfo.max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX;
    metainfo.entry.metainfo.max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX;

    /* Fail resetting the flash manager */
    FLASH_ENTRY_GET_EXPECT(DSM_FLASH_HANDLE_METAINFO, &metainfo);
    flash_manager_remove_IgnoreAndReturn(NRF_ERROR_NO_MEM);
    flash_manager_mem_listener_register_StubWithCallback(flash_manager_mem_listener_register_cb);
    m_fm_mem_listener_register_expect = 1;
    TEST_ASSERT_FALSE(dsm_flash_config_load());
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    TEST_ASSERT_NOT_NULL(mp_mem_listener);
    /* recover from the failure to reset */
    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_SUCCESS);
    mp_mem_listener->callback(mp_mem_listener->p_args); /* should call remove */
    flash_manager_mock_Verify();
    /* fail adding */
    net_state_flash_area_get_ExpectAndReturn((void *)(PAGE_SIZE + (uint32_t)m_flash_area));
    flash_manager_add_ExpectAndReturn(mp_flash_manager,
                                      &mp_flash_manager->config,
                                      NRF_ERROR_NO_MEM);
    m_fm_mem_listener_register_expect = 1;
    mp_flash_manager->config.remove_complete_cb(mp_flash_manager);
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

    /* notify about more memory, and succeed all calls this time. */
    net_state_flash_area_get_ExpectAndReturn((void *)(PAGE_SIZE + (uint32_t)m_flash_area));
    flash_manager_add_ExpectAndReturn(mp_flash_manager,
                                      &mp_flash_manager->config,
                                      NRF_SUCCESS);
    mp_flash_manager->internal.state = FM_STATE_BUILDING;
    flash_expect_metainfo();
    mp_mem_listener->callback(mp_mem_listener->p_args);


    /* Run again, but this time, succeed resetting. */
    FLASH_ENTRY_GET_EXPECT(DSM_FLASH_HANDLE_METAINFO, &metainfo);
    flash_manager_remove_IgnoreAndReturn(NRF_SUCCESS);
    TEST_ASSERT_FALSE(dsm_flash_config_load());
    flash_manager_mock_Verify();

    net_state_flash_area_get_ExpectAndReturn((void *)(PAGE_SIZE + (uint32_t)m_flash_area));
    flash_manager_add_ExpectAndReturn(mp_flash_manager,
                                      &mp_flash_manager->config,
                                      NRF_SUCCESS);
    flash_expect_metainfo();
    mp_flash_manager->config.remove_complete_cb(mp_flash_manager);

    metainfo.entry.metainfo.max_subnets = DSM_SUBNET_MAX; /* valid */

    FLASH_ENTRY_GET_EXPECT(DSM_FLASH_HANDLE_METAINFO, &metainfo);
    flash_get_multiple_expect_start();
    flash_get_multiple_expect_end(); /* No other entries found */
    TEST_ASSERT_FALSE(dsm_flash_config_load()); /* Still false, as we couldn't find a unicast address */

    /* Set up a bunch of each type of stored entry, verify that we do the required encryption for each. */
#define ENTRY_COUNT 4

    /* Adding a unicast address will make the loader return true, as the device has been provisioned. */
    dsm_entry_t unicast;
    unicast.header.handle = DSM_FLASH_HANDLE_UNICAST;
    unicast.header.len_words = 2;
    unicast.entry.addr_unicast.addr.address_start = 0x0001;
    unicast.entry.addr_unicast.addr.count         = 0x0001;

    dsm_entry_t addr_nonvirtual[ENTRY_COUNT];
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        fm_header_t header = {.handle =
                                  DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, i),
                              .len_words = (sizeof(dsm_flash_entry_addr_nonvirtual_t) + 3) / 4 + 1};
        memcpy(&addr_nonvirtual[i].header, &header, sizeof(header));
        addr_nonvirtual[i].entry.addr_nonvirtual.addr = i;
    };

    dsm_entry_t addr_virtual[ENTRY_COUNT];
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        fm_header_t header = {.handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, i),
                              .len_words = (sizeof(dsm_flash_entry_addr_virtual_t) + 3) / 4 + 1};
        memcpy(&addr_virtual[i].header, &header, sizeof(header));
        memset(addr_virtual[i].entry.addr_virtual.uuid, i, NRF_MESH_UUID_SIZE);

        nrf_mesh_keygen_virtual_address_ExpectAndReturn(addr_virtual[i].entry.addr_virtual.uuid,
                                                        NULL,
                                                        NRF_SUCCESS);
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    };

    dsm_entry_t subnets[ENTRY_COUNT];
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        fm_header_t header = {.handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, i),
                              .len_words    = (sizeof(dsm_flash_entry_subnet_t) + 3) / 4 + 1};
        memcpy(&subnets[i].header, &header, sizeof(header));
        memset(subnets[i].entry.subnet.key, i, NRF_MESH_KEY_SIZE);
        subnets[i].entry.subnet.key_index = i + 0x100;

        nrf_mesh_keygen_network_secmat_ExpectAndReturn(subnets[i].entry.subnet.key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();

        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(subnets[i].entry.subnet.key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();

#if GATT_PROXY
        nrf_mesh_keygen_identitykey_ExpectAndReturn(subnets[i].entry.subnet.key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
#endif
    };

    dsm_entry_t appkeys[ENTRY_COUNT];
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        fm_header_t header = {.handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, i),
                              .len_words    = (sizeof(dsm_flash_entry_appkey_t) + 3) / 4 + 1};
        memcpy(&appkeys[i].header, &header, sizeof(header));
        memset(appkeys[i].entry.appkey.key, i, NRF_MESH_KEY_SIZE);
        appkeys[i].entry.appkey.key_index = i + 0x200;
        appkeys[i].entry.appkey.subnet_handle = i;

        nrf_mesh_keygen_aid_ExpectAndReturn(appkeys[i].entry.appkey.key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
    };

    dsm_entry_t devkeys[ENTRY_COUNT];
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        fm_header_t header = {.handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, i),
                              .len_words    = (sizeof(dsm_flash_entry_devkey_t) + 3) / 4 + 1};
        memcpy(&devkeys[i].header, &header, sizeof(header));
        memset(devkeys[i].entry.devkey.key, i, NRF_MESH_KEY_SIZE);
        devkeys[i].entry.devkey.key_owner     = i + 0x100;
        devkeys[i].entry.devkey.subnet_handle = i;
    };


    /* Load them all */
    FLASH_ENTRY_GET_EXPECT(DSM_FLASH_HANDLE_METAINFO, &metainfo);
    flash_get_multiple_expect_start();
    flash_get_multiple_expect(&unicast, 1);
    flash_get_multiple_expect(addr_nonvirtual, ENTRY_COUNT);
    flash_get_multiple_expect(addr_virtual, ENTRY_COUNT);
    flash_get_multiple_expect(subnets, ENTRY_COUNT);
    flash_get_multiple_expect(appkeys, ENTRY_COUNT);
    flash_get_multiple_expect(devkeys, ENTRY_COUNT);
    flash_get_multiple_expect_end();
    TEST_ASSERT_TRUE(dsm_flash_config_load());

    /* Check that they're all present */
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(i + DSM_APP_MAX, &secmat));
        TEST_ASSERT_EQUAL(true, secmat.p_app->is_device_key);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(devkeys[i].entry.devkey.key,
                                     secmat.p_app->key,
                                     NRF_MESH_KEY_SIZE);
    }
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        mesh_key_index_t key_index;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_handle_to_netkey_index(i, &key_index));
        TEST_ASSERT_EQUAL(i + 0x100, key_index);
    }
    for (uint32_t i = 0; i < ENTRY_COUNT; ++i)
    {
        mesh_key_index_t key_index;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_handle_to_appkey_index(i, &key_index));
        TEST_ASSERT_EQUAL(i + 0x200, key_index);
    }
}
#undef FLASH_ENTRY_GET_EXPECT

void test_flash_insufficient_resources(void)
{
    /* setup the flash functions to fail, so we have to retry flashing later */
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_fail_cb);
    flash_manager_entry_invalidate_StubWithCallback(flash_manager_entry_invalidate_fail_cb);

    flash_manager_mem_listener_register_StubWithCallback(flash_manager_mem_listener_register_cb);

    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    for (uint32_t i = 0; i < sizeof(net) / sizeof(net[0]); i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));

        m_fm_mem_listener_register_expect = 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i, key, &net[i].handle));
        TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

        uint16_t index = 0xffff;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_handle_to_netkey_index(net[i].handle, &index));
        TEST_ASSERT_EQUAL(i, index);
    }
    /* Add some dummy apps */
    struct
    {
        dsm_handle_t handle;
        uint8_t aid;
    } app[6];
    for (uint32_t i = 0; i < sizeof(app) / sizeof(app[0]); i++)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, sizeof(app[i].aid));
        m_fm_mem_listener_register_expect = 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, net[i / 2].handle, key, &app[i].handle));
        TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    }
    /* Add some dummy devkeys */
    struct
    {
        dsm_handle_t handle;
    } dev[4];
    for (uint32_t i = 0; i < sizeof(dev) / sizeof(dev[0]); i++)
    {
        m_fm_mem_listener_register_expect = 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_add(i + 0x1000, net[i / 2].handle, key, &dev[i].handle));
        TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    }

    /* Add some dummy nonvirtual addresses */
    struct
    {
        dsm_handle_t handle;
    } addr_nonvirtual[4];
    for (uint32_t i = 0; i < sizeof(addr_nonvirtual) / sizeof(addr_nonvirtual[0]); i++)
    {
        m_fm_mem_listener_register_expect = 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                          dsm_address_publish_add(i + 0xF000, &addr_nonvirtual[i].handle));
        TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    }
    /* Add some dummy virtual addresses */
    struct
    {
        dsm_handle_t handle;
        uint8_t uuid[NRF_MESH_UUID_SIZE];
    } addr_virtual[4];
    for (uint32_t i = 0; i < sizeof(addr_virtual) / sizeof(addr_virtual[0]); i++)
    {
        nrf_mesh_keygen_virtual_address_ExpectAndReturn(addr_virtual[i].uuid,
                                                        NULL,
                                                        NRF_SUCCESS); /*lint !e603 addr_virtual[i].uuid is intentionally not initialized*/
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        m_fm_mem_listener_register_expect = 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                          dsm_address_publish_virtual_add(addr_virtual[i].uuid,
                                                          &addr_virtual[i].handle));
        TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    }

    /*** We now have some entries of each type that hasn't been flashed yet. ***/
    TEST_ASSERT_TRUE(dsm_has_unflashed_data());

    /* Now start flashing the entries. */

    /* Switch to the flash stubs that actually do something */
    m_expected_flash_data.verify_contents = false;
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_cb);
    flash_manager_entry_commit_StubWithCallback(flash_manager_entry_commit_cb);

    mp_mem_listener->callback(mp_mem_listener->p_args);
    TEST_ASSERT_FALSE(dsm_has_unflashed_data());


    /******* Delete some entries *******/
    m_fm_mem_listener_register_expect = 1;
    /* setup the flash functions to fail, so we have to retry flashing later */
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_fail_cb);
    flash_manager_entry_invalidate_StubWithCallback(NULL);

    /* Delete the first of each entry type. Only the first call will attempt (and fail) flashing. */
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[0].handle - DSM_APP_MAX),
        NRF_ERROR_NO_MEM);
    m_fm_mem_listener_register_expect = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(dev[0].handle));
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    /* call the others, all of which will avoid calling flash, as the module is waiting for memory */

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[0].handle),
        NRF_ERROR_NO_MEM);
    m_fm_mem_listener_register_expect = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_delete(app[0].handle));

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[1].handle),
        NRF_ERROR_NO_MEM);
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[1].handle - DSM_APP_MAX),
        NRF_ERROR_NO_MEM);
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, net[0].handle),
        NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);
    m_fm_mem_listener_register_expect = 3;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_delete(net[0].handle)); /* This'll also delete dev[1] and app[1], as they're bound to it */
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, addr_nonvirtual[0].handle),
        NRF_ERROR_NO_MEM);
    m_fm_mem_listener_register_expect = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addr_nonvirtual[0].handle));
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, addr_virtual[0].handle - DSM_NONVIRTUAL_ADDR_MAX),
        NRF_ERROR_NO_MEM);
    m_fm_mem_listener_register_expect = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addr_virtual[0].handle));
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

    TEST_ASSERT_TRUE(dsm_has_unflashed_data());
    flash_manager_mock_Verify();

    /* Run async flashing, to attempt recovering. Fail about half of them. Need to do it in entry type order */
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_NONVIRTUAL, addr_nonvirtual[0].handle),
        NRF_SUCCESS);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_ADDR_VIRTUAL, addr_virtual[0].handle - DSM_NONVIRTUAL_ADDR_MAX),
        NRF_SUCCESS);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, net[0].handle),
        NRF_SUCCESS);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[0].handle),
        NRF_SUCCESS);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[1].handle),
        NRF_ERROR_NO_MEM);
    /*
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[0].handle - DSM_APP_MAX),
        NRF_ERROR_NO_MEM);
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[1].handle - DSM_APP_MAX),
        NRF_ERROR_NO_MEM);
    */

    /* Now do the flashing, and reorder the callback, as some operations failed: */
    m_fm_mem_listener_register_expect = 1;
    mp_mem_listener->callback(mp_mem_listener->p_args);
    flash_manager_mock_Verify();
    TEST_ASSERT_EQUAL(0, m_fm_mem_listener_register_expect);

    TEST_ASSERT_TRUE(dsm_has_unflashed_data());

    /* expect the entries we couldn't finish, and return success this time. */

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_APPKEYS, app[1].handle),
        NRF_SUCCESS);

    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[0].handle - DSM_APP_MAX),
        NRF_SUCCESS);
    flash_manager_entry_invalidate_ExpectAndReturn(
        mp_flash_manager,
        DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_DEVKEYS, dev[1].handle - DSM_APP_MAX),
        NRF_SUCCESS);
    /* Flash the last one, the listener shouldn't register this time. */
    mp_mem_listener->callback(mp_mem_listener->p_args);
    TEST_ASSERT_FALSE(dsm_has_unflashed_data());
}
