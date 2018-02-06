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

#ifndef HPL_CRC_SYNC_H_INCLUDED
#define HPL_CRC_SYNC_H_INCLUDED

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup hpl__crc__sync CRC Sync Driver
 *
 * \section crc_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

/**
 * \brief CRC Device
 */
struct _crc_sync_device {
	void *hw; /*!< Hardware module instance handler */
};

/**
 * \brief Initialize CRC.
 *
 * \param[in] device The pointer to device instance
 * \param[in] hw The pointer to hardware instance
 *
 * \return Initialization status.
 * \retval -1 Passed parameters were invalid or the CRC is already initialized
 * \retval 0 The initialization is completed successfully
 */
int32_t _crc_sync_init(struct _crc_sync_device *const device, void *const hw);

/**
 * \brief Deinitialize CRC.
 *
 * \param[in] device The pointer to device instance
 *
 * \return De-initialization status.
 * \retval -1 Passed parameters were invalid or the CRC is not initialized
 * \retval 0 The de-initialization is completed successfully
 */
int32_t _crc_sync_deinit(struct _crc_sync_device *const device);

/**
 * \brief Enable CRC
 *
 * \param[in] device The pointer to device instance
 *
 * \return Enabling status.
 * \retval -1 Passed parameters were invalid
 * \retval 0 The enable procedure is completed successfully
 */
int32_t _crc_sync_enable(struct _crc_sync_device *const device);

/**
 * \brief Disable CRC
 *
 * \param[in] device The pointer to device instance
 *
 * \return Disabling status.
 * \retval -1 Passed parameters were invalid
 * \retval 0 The disable procedure is completed successfully
 */
int32_t _crc_sync_disable(struct _crc_sync_device *const device);

/**
 * \brief Calculate CRC value of the buffer
 *
 * \param[in] device The pointer to device instance
 * \param[in] data Pointer to the input data buffer
 * \param[in] len Length of the input data buffer
 * \param[in,out] pcrc Pointer to the CRC value
 *
 * \return Calculate result.
 * \retval -1 Passed parameters were invalid or hardware error occurs.
 * \retval 0 The CRC calculate success.
 */
int32_t _crc_sync_crc32(struct _crc_sync_device *const device, uint32_t *const data, const uint32_t len,
                        uint32_t *pcrc);

/**
 * \brief Compute CRC16 value of the buffer
 *
 * This function calculate CRC-16 (CCITT)
 *
 * \param[in] device The pointer to device instance
 * \param[in] data Pointer to the input data buffer
 * \param[in] len Length of the input data buffer
 * \param[in,out] pcrc Pointer to the CRC value
 *
 * \return Calculate result.
 * \retval -1 Passed parameters were invalid or hardware error occurs.
 * \retval 0 The CRC calculate success.
 */
int32_t _crc_sync_crc16(struct _crc_sync_device *const device, uint16_t *const data, const uint32_t len,
                        uint16_t *pcrc);

/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* HPL_CRC_SYNC_H_INCLUDED */
