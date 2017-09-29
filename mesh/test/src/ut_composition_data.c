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
#include <string.h>

#include "access_config_mock.h"
#include "composition_data.h"

#define SIG_MODELS {0x0000, ACCESS_COMPANY_ID_NONE},    \
                   {0x8000, ACCESS_COMPANY_ID_NONE},    \
                   {0x0001, ACCESS_COMPANY_ID_NONE},    \
                   {0x1000, ACCESS_COMPANY_ID_NONE},    \
                   {0x1003, ACCESS_COMPANY_ID_NONE}
#define SIG_MODELS_COUNT 5
#define VENDOR_MODELS {0x002A, 0x003F}
#define VENDOR_MODELS_COUNT 2
#define ELEMENT_LOCATION 0x0100 /* Front */

const access_model_id_t models[SIG_MODELS_COUNT +
                               VENDOR_MODELS_COUNT] = {SIG_MODELS, VENDOR_MODELS};

const uint8_t composition_data[] = {0x0C, 0x00, /* CID (Company identifier) */
                                    0x1A, 0x00, /* PID (Product identifier) */
                                    0x01, 0x00, /* VID (Product version identifier) */
                                    0x08, 0x00, /* CRPL (Minimum replay protection list entries) */
                                    0x03, 0x00, /* Features (bitfield indicating device features) */
                                    0x00, 0x01, /* Location */
                                    0x05,       /* SIG models count */
                                    0x02,       /* Vendor models count */
                                    0x00, 0x00, /* SIG model 0x0000 */
                                    0x00, 0x80, /* SIG model 0x8000 */
                                    0x01, 0x00, /* SIG model 0x0001 */
                                    0x00, 0x10, /* SIG model 0x1000 */
                                    0x03, 0x10, /* SIG model 0x1003 */
                                    0x3F, 0x00, /* Vendor model 0x003f */
                                    0x2A, 0x00  /* Vendor model 0x002a */
    };

static uint32_t model_id_get_cb(access_model_handle_t handle,
                                       access_model_id_t * p_model_id,
                                       int num_calls)
{
    p_model_id->model_id = models[handle].model_id;
    p_model_id->company_id = models[handle].company_id;
    return NRF_SUCCESS;
}

static uint32_t element_models_get_cb(uint16_t element_index,
                                      access_model_handle_t * p_handles,
                                      uint16_t * p_size,
                                      int num_calls)
{
    TEST_ASSERT_EQUAL(0, element_index);
    TEST_ASSERT_EQUAL(0, num_calls);
    TEST_ASSERT(p_size != NULL);
    TEST_ASSERT(SIG_MODELS_COUNT + VENDOR_MODELS_COUNT <= *p_size);
    *p_size = 0;
    for (uint32_t i = 0; i < SIG_MODELS_COUNT+VENDOR_MODELS_COUNT; ++i)
    {
        (*p_size)++;
        p_handles[i] = i;
    }
    return NRF_SUCCESS;
}

void setUp(void)
{
    access_config_mock_Init();
}

void tearDown(void)
{
}

void test_composition_data(void)
{
    uint8_t data[CONFIG_COMPOSITION_DATA_SIZE];
    uint8_t sig_models_count = SIG_MODELS_COUNT;
    uint8_t vendor_models_count = VENDOR_MODELS_COUNT;
    uint16_t location = ELEMENT_LOCATION;

    /* Test with an invalid size: */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, config_composition_data_get(data, 0));

    access_element_sig_model_count_get_ExpectAndReturn(0, NULL, ACCESS_STATUS_SUCCESS);
    access_element_sig_model_count_get_IgnoreArg_p_sig_model_count();
    access_element_sig_model_count_get_ReturnThruPtr_p_sig_model_count(&sig_models_count);

    access_element_vendor_model_count_get_ExpectAndReturn(0, NULL, ACCESS_STATUS_SUCCESS);
    access_element_vendor_model_count_get_IgnoreArg_p_vendor_model_count();
    access_element_vendor_model_count_get_ReturnThruPtr_p_vendor_model_count(&vendor_models_count);

    access_element_location_get_ExpectAndReturn(0, NULL, ACCESS_STATUS_SUCCESS);
    access_element_location_get_IgnoreArg_p_location();
    access_element_location_get_ReturnThruPtr_p_location(&location);

    access_model_id_get_StubWithCallback(model_id_get_cb);
    access_element_models_get_StubWithCallback(element_models_get_cb);

    /* Get the composition data: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_composition_data_get(data, sizeof(data)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(composition_data, data, sizeof(composition_data));
}

