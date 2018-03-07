/**
 * \file
 *
 * \brief Device Service Unit
 *
 * Copyright (C) 2015-2016 Atmel Corporation. All rights reserved.
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

#include <compiler.h>
#include <hpl_pac.h>
#include <hpl_crc_sync.h>
#include <utils_assert.h>
#include <err_codes.h>

/**
 * \brief Initialize CRC.
 */
int32_t _crc_sync_init(struct _crc_sync_device *const device, void *const hw)
{
	device->hw = hw;

	return ERR_NONE;
}

/**
 * \brief De-initialize CRC.
 */
int32_t _crc_sync_deinit(struct _crc_sync_device *const device)
{
	device->hw = NULL;

	return ERR_NONE;
}

/**
 * \brief Enable CRC
 */
int32_t _crc_sync_enable(struct _crc_sync_device *const device)
{
	(void)device;

	return ERR_NONE;
}

/**
 * \brief Disable CRC
 */
int32_t _crc_sync_disable(struct _crc_sync_device *const device)
{
	(void)device;

	return ERR_NONE;
}

/**
 * \brief Calculate CRC value of the buffer
 */
int32_t _crc_sync_crc32(struct _crc_sync_device *const device, uint32_t *const data, const uint32_t len, uint32_t *pcrc)
{
	int32_t  rc = ERR_NONE;
	uint32_t workaround_val;

	if (((uint32_t)data) & 0x00000003) {
		/* Address must be align with 4 bytes, refer to datasheet */
		return ERR_INVALID_ARG;
	}
	/* Workaround for DSU CRC32 computation is not functional */
	workaround_val = *((uint32_t *)0x41007058UL);
	(*((unsigned int *)0x41007058UL)) &= 0xFFFCFFFFUL;

	CRITICAL_SECTION_ENTER()
	/* Disable write-protected by PAC1->DSU before write DSU registers */
	_periph_unlock(device->hw);

	hri_dsu_write_ADDR_reg(device->hw, (uint32_t)data);
	hri_dsu_write_LENGTH_LENGTH_bf(device->hw, len);
	hri_dsu_write_DATA_reg(device->hw, *pcrc);
	hri_dsu_write_CTRL_reg(device->hw, DSU_CTRL_CRC);

	while (hri_dsu_get_STATUSA_DONE_bit(device->hw) == 0) {
	}

	if (hri_dsu_get_STATUSA_BERR_bit(device->hw)) {
		hri_dsu_clear_STATUSA_BERR_bit(device->hw);
		hri_dsu_clear_STATUSA_DONE_bit(device->hw);
		rc = ERR_IO;
	} else {
		*pcrc = (uint32_t)hri_dsu_read_DATA_reg(device->hw);
	}
	hri_dsu_clear_STATUSA_DONE_bit(device->hw);

	/* Restore write-protected of PAC->DSU */
	_periph_lock(device->hw);

	CRITICAL_SECTION_LEAVE()
	(*((unsigned int *)0x41007058UL)) = workaround_val;

	return rc;
}

/**
 * \brief Compute CRC16 value of the buffer
 */
int32_t _crc_sync_crc16(struct _crc_sync_device *const device, uint16_t *const data, const uint32_t len, uint16_t *pcrc)
{
	(void)device, (void)data, (void)len, (void)pcrc;
	return ERR_UNSUPPORTED_OP;
}
