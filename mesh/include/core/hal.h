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

#ifndef HAL_H__
#define HAL_H__

#include <stdint.h>
#include "nrf.h"

/**
 * @defgroup HAL Hardware Abstraction Layer
 * @ingroup MESH_CORE
 * @{
 */

#if NRF51
/** Gets nRF51 bootloader address. */
#define BOOTLOADERADDR() (NRF_UICR->BOOTLOADERADDR)
/** nRF51 flash page size. */
#define PAGE_SIZE        (0x400)
/** nRF51 code RAM start. */
#define CODE_RAM_START   (0x20000000)
/** nRF51 data RAM start. */
#define DATA_RAM_START   (0x20000000)
/** First address outside the data RAM */
#define DEVICE_DATA_RAM_END_GET() (DATA_RAM_START + (NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK))
/** First address outside the code RAM */
#define DEVICE_CODE_RAM_END_GET() (CODE_RAM_START + (NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK))
/** First address outside the device flash */
#define DEVICE_FLASH_END_GET()    (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE)
#elif NRF52
/** Gets the nRF52 bootloader address. */
#define BOOTLOADERADDR() (NRF_UICR->NRFFW[0])
/** nRF52 flash page size. */
#define PAGE_SIZE        (0x1000)
/** nRF52 code RAM start address. */
#define CODE_RAM_START   (0x800000)
/** nRF52 data RAM start address. */
#define DATA_RAM_START   (0x20000000)
/** First address outside the data RAM */
#define DEVICE_DATA_RAM_END_GET() (DATA_RAM_START + (1024 * NRF_FICR->INFO.RAM))
/** First address outside the code RAM */
#define DEVICE_CODE_RAM_END_GET() (CODE_RAM_START + (1024 * NRF_FICR->INFO.RAM))
/** First address outside the device flash */
#define DEVICE_FLASH_END_GET()    (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE)
#else
#if HOST
#define PAGE_SIZE      (0x400)
#define CODE_RAM_START (0)
#define DATA_RAM_START (0)
#define DEVICE_DATA_RAM_END_GET()   (0xFFFFFFFF)
#define DEVICE_CODE_RAM_END_GET()   (0xFFFFFFFF)
#define DEVICE_FLASH_END_GET()      (NRF_MESH_ASSERT(false))
#else
#error "Unsupported hardware platform"
#endif  /* HOST */
#endif  /* NRF51 || NRF52 */

/**
 * Clear the reset reason register, set the retention register, and reset the
 * device. All volatile memory will be lost. This function will never return.
 *
 * @param[in] gpregret_value Value to set the retention register to.
 */
void hal_device_reset(uint8_t gpregret_value);

/** @} */

#endif /* HAL_H__ */

