/**
 * \file
 *
 * \brief PAC related functionality declaration.
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
#ifndef _HPL_PAC_H_INCLUDED
#define _HPL_PAC_H_INCLUDED

#include <compiler.h>
#include "hpl_irq.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Enable write protect for the given hardware module
 *
 * This function enables write protect for the given hardware module.
 * For an overview of available PAC and hardware modules see datasheet.
 *
 * \param[in] module A hardware module to enable write protect for
 */
int32_t _periph_lock(const void *const module);

/**
 * \brief Disable write protect for the given hardware module
 *
 * This function disables write protect for the given hardware module.
 * For an overview of available PAC and hardware modules see datasheet.
 *
 * \param[in] module A hardware module to disable clock for
 */
int32_t _periph_unlock(const void *const module);

/**
 * \brief Get write protect state for the given hardware module
 *
 * This function get write protect state for the given hardware module.
 * For an overview of available PAC and hardware modules see datasheet.
 *
 * \param[in] module A hardware module to disable clock for
 * \param[out] state The pointer to write protect state for specified module
 */
int32_t _periph_get_lock_state(const void *const module, bool *const state);

#ifdef __cplusplus
}
#endif

#endif /* _HPL_PAC_H_INCLUDED */
