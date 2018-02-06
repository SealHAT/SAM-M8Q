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

#include <hal_crc_sync.h>

#define DRIVER_VERSION 0x00000001u

/**
 * \brief Initialize CRC.
 */
int32_t crc_sync_init(struct crc_sync_descriptor *const descr, void *const hw)
{
	ASSERT(descr && hw);

	return _crc_sync_init(&descr->dev, hw);
}

/**
 * \brief Deinitialize CRC.
 */
int32_t crc_sync_deinit(struct crc_sync_descriptor *const descr)
{
	ASSERT(descr);

	return _crc_sync_deinit(&descr->dev);
}

/**
 * \brief Enable CRC
 */
int32_t crc_sync_enable(struct crc_sync_descriptor *const descr)
{
	ASSERT(descr);

	return _crc_sync_enable(&descr->dev);
}

/**
 * \brief Disable CRC
 */
int32_t crc_sync_disable(struct crc_sync_descriptor *const descr)
{
	ASSERT(descr);

	return _crc_sync_disable(&descr->dev);
}
/**
 * \brief Calculate CRC32 value of the buffer
 */
int32_t crc_sync_crc32(struct crc_sync_descriptor *const descr, uint32_t *const data, const uint32_t len,
                       uint32_t *pcrc)
{
	ASSERT(descr && data && len && pcrc);

	return _crc_sync_crc32(&descr->dev, data, len, pcrc);
}

/**
 * \brief Calculate CRC16 value of the buffer
 */
int32_t crc_sync_crc16(struct crc_sync_descriptor *const descr, uint16_t *const data, const uint32_t len,
                       uint16_t *pcrc)
{
	ASSERT(descr && data && len && pcrc);

	return _crc_sync_crc16(&descr->dev, data, len, pcrc);
}

/**
 * \brief Retrieve the current driver version
 */
uint32_t crc_sync_get_version(void)
{
	return DRIVER_VERSION;
}
