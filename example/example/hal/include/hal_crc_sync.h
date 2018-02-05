/**
 * \file
 *
 * \brief CRC Cyclic Redundancy Check(Sync) functionality declaration.
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef HAL_CRC_SYNC_H_INCLUDED
#define HAL_CRC_SYNC_H_INCLUDED

#include <hpl_crc_sync.h>
#include <utils_assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup doc_driver_hal_crc_sync
 *
 *@{
 */

/**
 * \brief CRC descriptor
 */
struct crc_sync_descriptor {
	struct _crc_sync_device dev; /*!< CRC HPL device descriptor */
};

/**
 * \brief Initialize CRC.
 *
 * This function initializes the given CRC descriptor.
 * It checks if the given hardware is not initialized and if the given hardware
 * is permitted to be initialized.
 *
 * \param[out] descr A CRC descriptor to initialize
 * \param[in] hw The pointer to hardware instance
 *
 * \return Initialization status.
 */
int32_t crc_sync_init(struct crc_sync_descriptor *const descr, void *const hw);

/**
 * \brief Deinitialize CRC.
 *
 * This function deinitializes the given CRC descriptor.
 * It checks if the given hardware is initialized and if the given hardware is
 * permitted to be deinitialized.
 *
 * \param[in] descr A CRC descriptor to deinitialize
 *
 * \return De-initialization status.
 */
int32_t crc_sync_deinit(struct crc_sync_descriptor *const descr);

/**
 * \brief Enable CRC
 *
 * This function enables CRC by the given CRC descriptor.
 *
 * \param[in] descr A CRC descriptor to enable
 *
 * \return Enabling status.
 */
int32_t crc_sync_enable(struct crc_sync_descriptor *const descr);

/**
 * \brief Disable CRC
 *
 * This function disables CRC by the given CRC descriptor.
 *
 * \param[in] descr A CRC descriptor to disable
 *
 * \return Disabling status.
 */
int32_t crc_sync_disable(struct crc_sync_descriptor *const descr);

/**
 * \brief Calculate CRC32 value of the buffer
 *
 * This function calculates the standard CRC-32 (IEEE 802.3).
 *
 * \param[in] data Pointer to the input data buffer
 * \param[in] len Length of the input data buffer
 * \param[in,out] pcrc Pointer to the CRC value
 *
 * \return Calculated result.
 */
int32_t crc_sync_crc32(struct crc_sync_descriptor *const descr, uint32_t *const data, const uint32_t len,
                       uint32_t *pcrc);

/**
 * \brief Calculate the CRC16 value of the buffer
 *
 * This function calculates CRC-16 (CCITT).
 *
 * \param[in] data Pointer to the input data buffer
 * \param[in] len Length of the input data buffer
 * \param[in,out] pcrc Pointer to the CRC value
 *
 * \return Calculated result.
 */
int32_t crc_sync_crc16(struct crc_sync_descriptor *const descr, uint16_t *const data, const uint32_t len,
                       uint16_t *pcrc);

/**
 * \brief Retrieve the current driver version
 *
 * \return Current driver version.
 */
uint32_t crc_sync_get_version(void);
/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* HAL_CRC_SYNC_H_INCLUDED */
