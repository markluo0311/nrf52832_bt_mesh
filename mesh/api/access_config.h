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

#ifndef ACCESS_CONFIG_H__
#define ACCESS_CONFIG_H__

#include <stdint.h>
#include "access.h"
#include "access_publish.h"
#include "access_status.h"
#include "device_state_manager.h"

/**
 * @defgroup ACCESS_CONFIG Access layer configuration API
 * @ingroup MESH_API_GROUP_ACCESS
 *
 * @warning This API is intended to be used by the Configuration Server model to configure a node
 * and is considered _internal_.
 *
 * @{
 */

/** Invalid element index. */
#define ACCESS_ELEMENT_INDEX_INVALID (0xFFFF)

/**
 * Recover access layer configuration from flash.
 *
 * @warning All models have to be added before this function is called.
 *
 * @returns @c true if a valid state was successfully restored from flash.
 */
bool access_flash_config_load(void);

/**
 * Store the current state of access layer - information related to element and model configuration -
 * in non volatile memory.
 */
void access_flash_config_store(void);

/**
 * Sets the default TTL for the node.
 * @param ttl The new value to use as the default TTL for message being sent from this node.
 */
void access_default_ttl_set(uint8_t ttl);

/**
 * Gets the default TTL for the node.
 * @return Returns the default TTL value used to send messages from this node.
 */
uint8_t access_default_ttl_get(void);

/**
 * Changes the publish address for the given model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  address_handle           Address to set as the current publish address.
 *
 * @retval     NRF_SUCCESS              Successfully set the publish address.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid address handle.
 */
uint32_t access_model_publish_address_set(access_model_handle_t handle,
                                          dsm_handle_t address_handle);

/**
 * Gets the current publish address for the given model.
 *
 * @note The address handle return may be @ref DSM_HANDLE_INVALID if the publish address for the
 * model isn't set.
 *
 * @param[in]  handle           Access model handle.
 * @param[out] p_address_handle Pointer to store the current publish address handle.
 *
 * @retval NRF_SUCCESS             Successfully returned the address handle.
 * @retval NRF_ERROR_NULL          Null pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND     Access handle invalid.
 */
uint32_t access_model_publish_address_get(access_model_handle_t handle,
                                          dsm_handle_t * p_address_handle);

/**
 * Sets the publish period for the given model.
 *
 * @note       The publish period is calculated according to <pre>period = resolution *
 *             step_number</pre> Thus, setting the @c step_number to @c 0, will disable periodic
 *             publishing.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  resolution               Resolution of each step. Most not be larger than @ref
 *                                      ACCESS_PUBLISH_RESOLUTION_MAX.
 * @param[in]  step_number              Number of steps. Must not be larger than @ref
 *                                      ACCESS_PUBLISH_PERIOD_STEP_MAX.
 *
 * @retval     NRF_SUCCSES              Successfully set the publish period.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  Publish step and/or resolution out of range.
 * @retval     NRF_ERROR_NOT_SUPPORTED  Periodic publishing not supported for this model.
 */
uint32_t access_model_publish_period_set(access_model_handle_t handle,
                                         access_publish_resolution_t resolution,
                                         uint8_t step_number);

/**
 * Gets the publish period for the given model.
 *
 * @param[in]  handle        Access model handle.
 * @param[out] p_resolution  Pointer to store the resolution.
 * @param[out] p_step_number Pointer to store the step number.
 *
 * @retval NRF_SUCCESS             Successfully stored the publish period.
 * @retval NRF_ERROR_NULL          NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND     Access handle invalid.
 */
uint32_t access_model_publish_period_get(access_model_handle_t handle,
                                         access_publish_resolution_t * p_resolution,
                                         uint8_t * p_step_number);

/**
 * Adds a subscription to a model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  address_handle           Address to add to the model's subscription list.
 *
 * @retval     NRF_SUCCESS              Successfully added the subscription.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_NOT_SUPPORTED  Subscriptions not supported for this model.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid address handle.
 */
uint32_t access_model_subscription_add(access_model_handle_t handle, dsm_handle_t address_handle);

/**
 * Remove a subscription from a model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  address_handle           Address to remove from the model's subscription list.
 *
 * @retval     NRF_SUCCESS              Successfully removed the subscription.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_NOT_SUPPORTED  Subscriptions not supported for this model.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid address handle.
 */
uint32_t access_model_subscription_remove(access_model_handle_t handle, dsm_handle_t address_handle);

/**
 * Gets the address handles for the subscription addresses bound to a model.
 *
 * @param[in]     handle            Access model handle.
 * @param[out]    p_address_handles Pointer to array for storing address handles.
 * @param[in,out] p_count           Pointer to number of available handles in p_address_handles.
 *                                  The number of written handles is returned
 *
 * @retval NRF_SUCCESS              Successfully stored the address handles.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_NOT_SUPPORTED  Subscriptions are not supported by this model.
 * @retval NRF_ERROR_INVALID_LENGTH Size of p_address_handles too small for storing all the bound keys.
 */
uint32_t access_model_subscriptions_get(access_model_handle_t handle,
                                        dsm_handle_t * p_address_handles,
                                        uint16_t * p_count);

/**
 * Binds an application key to a model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  appkey_handle            Application handle to add to the model.
 *
 * @retval     NRF_SUCCESS              Successfully added the application key.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid application key handle.
 */
uint32_t access_model_application_bind(access_model_handle_t handle, dsm_handle_t appkey_handle);

/**
 * Unbind an application key from a model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  appkey_handle            Application handle to remove from the model.
 *
 * @retval     NRF_SUCCESS              Successfully removed the application key.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid application key handle.
 */
uint32_t access_model_application_unbind(access_model_handle_t handle, dsm_handle_t appkey_handle);

/**
 * Gets the applications bound to a model.
 *
 * @param[in]     handle           Access model handle.
 * @param[out]    p_appkey_handles Pointer to array where the DSM handles will be stored.
 * @param[in,out] p_count          Pointer to number of handles available in p_appkey_handles.
 *                                 The number of written handles will be returned in p_count.
 *
 * @retval NRF_SUCCESS              Successfully stored the application handles.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid access model handle.
 * @retval NRF_ERROR_INVALID_LENGTH Size of p_appkey_handles too small for storing all the bound keys.
 */
uint32_t access_model_applications_get(access_model_handle_t handle,
                                       dsm_handle_t * p_appkey_handles,
                                       uint16_t * p_count);

/**
 * Sets the application key to be used when publishing for the given model.
 *
 * @note       To "unbind" the application key, set appkey_handle to @ref DSM_HANDLE_INVALID.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  appkey_handle            Application handle to bind with model.
 *
 * @retval     NRF_SUCCESS              Successfully set the application key used for publishing.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  Invalid application key handle.
 */
uint32_t access_model_publish_application_set(access_model_handle_t handle,
                                              dsm_handle_t appkey_handle);

/**
 * Gets the application key used when publishing for the given model.
 *
 * @param[in]  handle          Access model handle.
 * @param[out] p_appkey_handle Pointer to store the application handle.
 *
 * @retval NRF_SUCCESS             Successfully stored the application key used for publishing.
 * @retval NRF_ERROR_NULL          NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND     Access handle invalid.
 */
uint32_t access_model_publish_application_get(access_model_handle_t handle,
                                              dsm_handle_t * p_appkey_handle);

/**
 * Sets the default publication TTL value for the given model.
 *
 * @param[in]  handle                   Access model handle.
 * @param[in]  ttl                      New default TTL value.
 *
 * @retval     NRF_SUCCESS              Successfully set the default publication TTL value.
 * @retval     NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval     NRF_ERROR_INVALID_PARAM  TTL value has to be less than @ref NRF_MESH_TTL_MAX.
 */
uint32_t access_model_publish_ttl_set(access_model_handle_t handle, uint8_t ttl);

/**
 * Gets the default publication TTL value for the given model.
 *
 * @param[in]  handle Access model handle.
 * @param[out] p_ttl  Pointer to store the default TTL value.
 *
 * @retval NRF_SUCCESS             Successfully stored the default publication TTL value.
 * @retval NRF_ERROR_NULL          NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND     Access handle invalid.
 */
uint32_t access_model_publish_ttl_get(access_model_handle_t handle, uint8_t * p_ttl);

/**
 * Sets the location descriptor for an element.
 *
 * @note       See the list of Bluetooth SIG location descriptors:
 *             https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors
 *
 * @param[in]  element_index        Element index.
 * @param[in]  location             Location descriptor.
 *
 * @retval     NRF_SUCCESS          Successfully set location descriptor.
 * @retval     NRF_ERROR_NOT_FOUND  Invalid element index.
 */
uint32_t access_element_location_set(uint16_t element_index, uint16_t location);

/**
 * Gets the location descriptor for an element.
 *
 * @note See the list of Bluetooth SIG location descriptors:
 *       https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors
 *
 * @param[in]  element_index Element index.
 * @param[out] p_location    Pointer to store location descriptor.
 *
 * @retval NRF_SUCCESS         Successfully got the location descriptor.
 * @retval NRF_ERROR_NULL      NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t access_element_location_get(uint16_t element_index, uint16_t * p_location);

/**
 * Gets the number of Bluetooth SIG models for an element.
 *
 * @param[in]  element_index     Element index.
 * @param[out] p_sig_model_count Pointer to store number of SIG models.
 *
 * @retval NRF_SUCCESS         Successfully got the number of SIG models.
 * @retval NRF_ERROR_NULL      NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t access_element_sig_model_count_get(uint16_t element_index, uint8_t * p_sig_model_count);

/**
 * Gets the number of vendor specific models for an element.
 *
 * @param[in]  element_index        Element index.
 * @param[out] p_vendor_model_count Pointer to store number of vendor specific models.
 *
 * @retval NRF_SUCCESS         Successfully got the number of vendor specific models.
 * @retval NRF_ERROR_NULL      NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t access_element_vendor_model_count_get(uint16_t element_index, uint8_t * p_vendor_model_count);

/**
 * Gets the model ID of for the given model.
 *
 * @param[in]  handle     Access model handle.
 * @param[out] p_model_id Pointer to store the model ID.
 *
 * @retval NRF_SUCCESS             Successfully got the model ID.
 * @retval NRF_ERROR_NULL          NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND     Invalid access handle.
 */
uint32_t access_model_id_get(access_model_handle_t handle, access_model_id_t * p_model_id);

/**
 * Gets the access handle for the given model instance based on element index and model ID.
 *
 * @param[in]  element_index Index of the element to search.
 * @param[in]  model_id      Model ID.
 * @param[out] p_handle      Pointer to write the corresponding handle.
 *
 * @retval NRF_SUCCESS         Successfully got the model ID.
 * @retval NRF_ERROR_NULL      NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 * @retval NRF_ERROR_NOT_FOUND Couldn't find a model handle for the given ID.
 */
uint32_t access_handle_get(uint16_t element_index, access_model_id_t model_id, access_model_handle_t * p_handle);

/**
 * Gets the array of handles corresponding to an element.
 *
 * @param[in]     element_index Element index.
 * @param[out]    p_models      Model handle array
 * @param[in,out] p_count       Pointer to number of handles available in p_models.
 *                              The number of written handles will be returned in p_count.
 *
 * @retval NRF_SUCCESS              Successfully stored all model handles for this element index.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_LENGTH The input array was too small for the number of handles.
 */
uint32_t access_element_models_get(uint16_t element_index, access_model_handle_t * p_models, uint16_t * p_count);

/**
 * Gets the generic argument pointer bound to the given model.
 *
 * @param[in]  handle  Access model handle.
 * @param[out] pp_args Double pointer to return the generic argument pointer.
 *
 * @retval NRF_SUCCESS         Successfully returned the generic argument pointer.
 * @retval NRF_ERROR_NULL      NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND Access handle invalid.
 */
uint32_t access_model_p_args_get(access_model_handle_t handle, void ** pp_args);

/** @} */
#endif  /* ACCESS_CONFIG_H__ */
