/**
 * \file
 *
 * \brief SAM Peripheral Access Controller
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
#include <compiler.h>
#include <utils_assert.h>
#include <hpl_pac.h>

#define CONF_PAC0_PERIPH_MIN_Pos 0
#define CONF_PAC0_PERIPH_MAX_Pos 5
#define CONF_PAC1_PERIPH_MIN_Pos 0
#define CONF_PAC1_PERIPH_MAX_Pos 5
#define CONF_PAC2_PERIPH_MIN_Pos 0
#define CONF_PAC2_PERIPH_MAX_Pos 19
static int32_t _pac_get_pac1_instance(const void *const module)
{
	if ((uint32_t)module == (uint32_t)DSU) {
		return 1;
	} else if ((uint32_t)module == (uint32_t)NVMCTRL) {
		return 2;
	} else if ((uint32_t)module == (uint32_t)PORT) {
		return 3;
	}
#if defined(DMAC)
	else if ((uint32_t)module == (uint32_t)DMAC) {
		return 4;
	}
#endif
#if defined(USB)
	else if ((uint32_t)module == (uint32_t)USB) {
		return 5;
	}
#endif
#if defined(MTB)
	else if ((uint32_t)module == (uint32_t)MTB) {
		return 6;
	}
#endif
	return ERR_INVALID_DATA;
}

/**
 * \brief Enable write protect for the given hardware module
 */
int32_t _periph_lock(const void *const module)
{
	ASSERT(module);
	ASSERT((((uint32_t)module) > (uint32_t)PAC0));

	int32_t timeout;
	int32_t peripheral = (((uint32_t)module & 0x0000ff00) >> 10) - 1;

	if ((uint32_t)module < (uint32_t)PAC1) {
		if ((peripheral >= CONF_PAC0_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC0_PERIPH_MAX_Pos)) {
			timeout = 1000;
			hri_pac_set_WP_WP_bf(PAC0, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC0, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	} else if (((uint32_t)module) < ((uint32_t)PAC2)) {
		peripheral = _pac_get_pac1_instance(module);
		if (peripheral != ERR_INVALID_DATA) {
			peripheral -= 1;
			timeout = 1000;
			hri_pac_set_WP_WP_bf(PAC1, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC1, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	} else {
		if ((peripheral >= CONF_PAC2_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC2_PERIPH_MAX_Pos)) {
			timeout = 1000;
			hri_pac_set_WP_WP_bf(PAC2, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC2, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	}

	if (timeout < 0) {
		return ERR_TIMEOUT;
	}

	return ERR_NONE;
}

/**
 * \brief Disable write protect for the given hardware module
 */
int32_t _periph_unlock(const void *const module)
{
	ASSERT(module);
	ASSERT((((uint32_t)module) > (uint32_t)PAC0));

	int32_t timeout;

	int32_t peripheral = (((uint32_t)module & 0x0000ff00) >> 10) - 1;

	if ((uint32_t)module < (uint32_t)PAC1) {
		if ((peripheral >= CONF_PAC0_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC0_PERIPH_MAX_Pos)) {
			timeout = 1000;
			hri_pac_clear_WP_WP_bf(PAC0, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC0, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	} else if (((uint32_t)module) < ((uint32_t)PAC2)) {
		peripheral = _pac_get_pac1_instance(module);
		if (peripheral != ERR_INVALID_DATA) {
			peripheral -= 1;
			timeout = 1000;
			hri_pac_clear_WP_WP_bf(PAC1, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC1, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	} else {
		if ((peripheral >= CONF_PAC2_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC2_PERIPH_MAX_Pos)) {
			timeout = 1000;
			hri_pac_clear_WP_WP_bf(PAC2, 1 << peripheral);
			while (hri_pac_get_WP_WP_bf(PAC2, 1 << peripheral) && timeout--) {
			}
		} else {
			return ERR_INVALID_DATA;
		}
	}

	if (timeout < 0) {
		return ERR_TIMEOUT;
	}

	return ERR_NONE;
}

/**
 * \brief Get write protect for the given hardware module
 */
int32_t _periph_get_lock_state(const void *const module, bool *const state)
{
	ASSERT(module);
	ASSERT((((uint32_t)module) > (uint32_t)PAC0));

	int32_t peripheral = (((uint32_t)module & 0x0000ff00) >> 10) - 1;

	if ((uint32_t)module < (uint32_t)PAC1) {
		if ((peripheral >= CONF_PAC0_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC0_PERIPH_MAX_Pos)) {
			*state = hri_pac_get_WP_WP_bf(PAC0, 1 << peripheral);
		} else {
			return ERR_INVALID_DATA;
		}
	} else if (((uint32_t)module) < ((uint32_t)PAC2)) {
		peripheral = _pac_get_pac1_instance(module);
		if (peripheral != ERR_INVALID_DATA) {
			peripheral -= 1;
			*state = hri_pac_get_WP_WP_bf(PAC1, 1 << peripheral);
		} else {
			return ERR_INVALID_DATA;
		}
	} else {
		if ((peripheral >= CONF_PAC2_PERIPH_MIN_Pos) && (peripheral <= CONF_PAC2_PERIPH_MAX_Pos)) {
			*state = hri_pac_get_WP_WP_bf(PAC2, 1 << peripheral);
		} else {
			return ERR_INVALID_DATA;
		}
	}
	return ERR_NONE;
}
