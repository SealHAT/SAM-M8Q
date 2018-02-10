
/**
 * \file
 *
 * \brief RTC related functionality implementation.
 *
 * Copyright (C) 2017 Atmel Corporation. All rights reserved.
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

#include "rtc_lite.h"

/**
 * \brief Initialize RTC interface
 */
int8_t time_date_init()
{

	hri_rtcmode2_wait_for_sync(RTC);
	if (hri_rtcmode2_get_CTRL_ENABLE_bit(RTC)) {
		hri_rtcmode2_clear_CTRL_ENABLE_bit(RTC);
		hri_rtcmode2_wait_for_sync(RTC);
	}
	hri_rtcmode2_set_CTRL_SWRST_bit(RTC);
	hri_rtcmode2_wait_for_sync(RTC);

	hri_rtcmode2_write_CTRL_reg(RTC,
	                            10 << RTC_MODE2_CTRL_PRESCALER_Pos     /* Setting: 10 */
	                                | 0 << RTC_MODE2_CTRL_MATCHCLR_Pos /* Clear on Match: disabled */
	                                | 0 << RTC_MODE2_CTRL_CLKREP_Pos   /* Clock Representation: disabled */
	                                | 0x2 << RTC_MODE2_CTRL_MODE_Pos); /* Operating Mode: 0x2 */

	hri_rtcmode2_write_READREQ_reg(RTC,
	                               0 << RTC_READREQ_RREQ_Pos          /*  Read Request: disabled */
	                                   | 1 << RTC_READREQ_RCONT_Pos); /*  Read Request: enabled */

	// hri_rtcmode2_write_DBGCTRL_reg(RTC,0); /* Run in debug: 0 */

	hri_rtcmode2_write_FREQCORR_reg(RTC,
	                                0 << RTC_FREQCORR_SIGN_Pos /* Setting: disabled */
	                                    | 0x0);                /* Correction Value: 0x0 */

	// hri_rtcmode2_write_CLOCK_reg(RTC,0x0 << RTC_MODE2_CLOCK_YEAR_Pos /* Year: 0x0 */
	//		 | 0x0 << RTC_MODE2_CLOCK_MONTH_Pos /* Month: 0x0 */
	//		 | 0x0 << RTC_MODE2_CLOCK_DAY_Pos /* Day: 0x0 */
	//		 | 0x0 << RTC_MODE2_CLOCK_HOUR_Pos /* Hour: 0x0 */
	//		 | 0x0 << RTC_MODE2_CLOCK_MINUTE_Pos /* Minute: 0x0 */
	//		 | 0x0 << RTC_MODE2_CLOCK_SECOND_Pos); /* Second: 0x0 */

	// hri_rtcmode2_write_ALARM_reg(RTC, 0 ,0x0 << RTC_MODE2_ALARM_YEAR_Pos /* Year: 0x0 */
	//		 | 0x0 << RTC_MODE2_ALARM_MONTH_Pos /* Month: 0x0 */
	//		 | 0x0 << RTC_MODE2_ALARM_DAY_Pos /* Day: 0x0 */
	//		 | 0x0 << RTC_MODE2_ALARM_HOUR_Pos /* Hour: 0x0 */
	//		 | 0x0 << RTC_MODE2_ALARM_MINUTE_Pos /* Minute: 0x0 */
	//		 | 0x0 << RTC_MODE2_ALARM_SECOND_Pos); /* Second: 0x0 */

	// hri_rtcmode2_write_MASK_reg(RTC, 0 ,0); /* Setting: 0 */

	// hri_rtcmode2_write_EVCTRL_reg(RTC,0 << RTC_MODE2_EVCTRL_OVFEO_Pos /* Overflow Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO0_Pos /* Periodic Interval 0 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO1_Pos /* Periodic Interval 1 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO2_Pos /* Periodic Interval 2 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO3_Pos /* Periodic Interval 3 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO4_Pos /* Periodic Interval 4 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO5_Pos /* Periodic Interval 5 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO6_Pos /* Periodic Interval 6 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_PEREO7_Pos /* Periodic Interval 7 Event Output Enable: disabled */
	//		 | 0 << RTC_MODE2_EVCTRL_ALARMEO0_Pos); /* Alarm 0 Event Output Enable: disabled */

	// hri_rtcmode2_write_INTEN_reg(RTC,0 << RTC_MODE2_INTENSET_ALARM_Pos /* Alarm 0 Interrupt Enable: disabled */
	//		 | 0 << RTC_MODE2_INTENSET_SYNCRDY_Pos /* Synchronization Ready Interrupt Enable: disabled */
	//		 | 0 << RTC_MODE2_INTENSET_OVF_Pos); /* Overflow Interrupt enable: disabled */

	hri_rtcmode2_write_CTRL_ENABLE_bit(RTC, 1 << RTC_MODE2_CTRL_ENABLE_Pos); /* Enable: enabled */

	return 0;
}
