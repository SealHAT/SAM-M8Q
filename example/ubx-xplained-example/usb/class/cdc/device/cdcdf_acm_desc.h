/**
 * \file
 *
 * \brief USB Device Stack CDC ACM Function Descriptor Setting.
 *
 * Copyright (C) 2015 - 2017 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef USBDF_CDC_ACM_DESC_H_
#define USBDF_CDC_ACM_DESC_H_

#include "usb_protocol.h"
#include "usb_protocol_cdc.h"
#include "usbd_config.h"

#define CDCD_ACM_DEV_DESC                                                                                              \
	USB_DEV_DESC_BYTES(CONF_USB_CDCD_ACM_BCDUSB,                                                                       \
	                   0x02,                                                                                           \
	                   0x00,                                                                                           \
	                   0x00,                                                                                           \
	                   CONF_USB_CDCD_ACM_BMAXPKSZ0,                                                                    \
	                   CONF_USB_CDCD_ACM_IDVENDER,                                                                     \
	                   CONF_USB_CDCD_ACM_IDPRODUCT,                                                                    \
	                   CONF_USB_CDCD_ACM_BCDDEVICE,                                                                    \
	                   CONF_USB_CDCD_ACM_IMANUFACT,                                                                    \
	                   CONF_USB_CDCD_ACM_IPRODUCT,                                                                     \
	                   CONF_USB_CDCD_ACM_ISERIALNUM,                                                                   \
	                   CONF_USB_CDCD_ACM_BNUMCONFIG)

#define CDCD_ACM_DEV_QUAL_DESC                                                                                         \
	USB_DEV_QUAL_DESC_BYTES(                                                                                           \
	    CONF_USB_CDCD_ACM_BCDUSB, 0x02, 0x00, 0x00, CONF_USB_CDCD_ACM_BMAXPKSZ0, CONF_USB_CDCD_ACM_BNUMCONFIG)

#define CDCD_ACM_CFG_DESC                                                                                              \
	USB_CONFIG_DESC_BYTES(67,                                                                                          \
	                      2,                                                                                           \
	                      CONF_USB_CDCD_ACM_BCONFIGVAL,                                                                \
	                      CONF_USB_CDCD_ACM_ICONFIG,                                                                   \
	                      CONF_USB_CDCD_ACM_BMATTRI,                                                                   \
	                      CONF_USB_CDCD_ACM_BMAXPOWER)

#define CDCD_ACM_OTH_SPD_CFG_DESC                                                                                      \
	USB_OTH_SPD_CFG_DESC_BYTES(67,                                                                                     \
	                           2,                                                                                      \
	                           CONF_USB_CDCD_ACM_BCONFIGVAL,                                                           \
	                           CONF_USB_CDCD_ACM_ICONFIG,                                                              \
	                           CONF_USB_CDCD_ACM_BMATTRI,                                                              \
	                           CONF_USB_CDCD_ACM_BMAXPOWER)

#define CDCD_ACM_COMM_IFACE_DESCES                                                                                     \
	USB_IFACE_DESC_BYTES(                                                                                              \
	    CONF_USB_CDCD_ACM_COMM_BIFCNUM, CONF_USB_CDCD_ACM_COMM_BALTSET, 1, 0x2, 0x2, 0x0, CONF_USB_CDCD_ACM_COMM_IIFC) \
	,                                                                                                                  \
	    USB_CDC_HDR_DESC_BYTES(0x1001), USB_CDC_CALL_MGMT_DESC_BYTES(0x01, 0x00), USB_CDC_ACM_DESC_BYTES(0x02),        \
	    USB_CDC_UNION_DESC_BYTES(CONF_USB_CDCD_ACM_COMM_BIFCNUM, 0x01),                                                \
	    USB_ENDP_DESC_BYTES(CONF_USB_CDCD_ACM_COMM_INT_EPADDR,                                                         \
	                        3,                                                                                         \
	                        CONF_USB_CDCD_ACM_COMM_INT_MAXPKSZ,                                                        \
	                        CONF_USB_CDCD_ACM_COMM_INT_INTERVAL)

#define CDCD_ACM_DATA_IFACE_DESCES                                                                                     \
	USB_IFACE_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BIFCNUM,                                                               \
	                     CONF_USB_CDCD_ACM_DATA_BALTSET,                                                               \
	                     2,                                                                                            \
	                     0x0A,                                                                                         \
	                     0x0,                                                                                          \
	                     0x0,                                                                                          \
	                     CONF_USB_CDCD_ACM_DATA_IIFC)                                                                  \
	,                                                                                                                  \
	    USB_ENDP_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BULKOUT_EPADDR, 2, CONF_USB_CDCD_ACM_DATA_BULKOUT_MAXPKSZ, 0),      \
	    USB_ENDP_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BULKIN_EPADDR, 2, CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ, 0)

#define CDCD_ACM_DATA_IFACE_DESCES_HS                                                                                  \
	USB_IFACE_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BIFCNUM,                                                               \
	                     CONF_USB_CDCD_ACM_DATA_BALTSET,                                                               \
	                     2,                                                                                            \
	                     0x0A,                                                                                         \
	                     0x0,                                                                                          \
	                     0x0,                                                                                          \
	                     CONF_USB_CDCD_ACM_DATA_IIFC)                                                                  \
	,                                                                                                                  \
	    USB_ENDP_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BULKOUT_EPADDR, 2, CONF_USB_CDCD_ACM_DATA_BULKOUT_MAXPKSZ_HS, 0),   \
	    USB_ENDP_DESC_BYTES(CONF_USB_CDCD_ACM_DATA_BULKIN_EPADDR, 2, CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS, 0)

#define CDCD_ACM_STR_DESCES                                                                                            \
	CONF_USB_CDCD_ACM_LANGID_DESC                                                                                      \
	CONF_USB_CDCD_ACM_IMANUFACT_STR_DESC                                                                               \
	CONF_USB_CDCD_ACM_IPRODUCT_STR_DESC                                                                                \
	CONF_USB_CDCD_ACM_ISERIALNUM_STR_DESC                                                                              \
	CONF_USB_CDCD_ACM_ICONFIG_STR_DESC

/** USB Device descriptors and configuration descriptors */
#define CDCD_ACM_DESCES_LS_FS                                                                                          \
	CDCD_ACM_DEV_DESC, CDCD_ACM_CFG_DESC, CDCD_ACM_COMM_IFACE_DESCES, CDCD_ACM_DATA_IFACE_DESCES, CDCD_ACM_STR_DESCES

#define CDCD_ACM_HS_DESCES_LS_FS                                                                                       \
	CDCD_ACM_DEV_DESC, CDCD_ACM_DEV_QUAL_DESC, CDCD_ACM_CFG_DESC, CDCD_ACM_COMM_IFACE_DESCES,                          \
	    CDCD_ACM_DATA_IFACE_DESCES, CDCD_ACM_OTH_SPD_CFG_DESC, CDCD_ACM_COMM_IFACE_DESCES,                             \
	    CDCD_ACM_DATA_IFACE_DESCES_HS, CDCD_ACM_STR_DESCES

#define CDCD_ACM_HS_DESCES_HS                                                                                          \
	CDCD_ACM_CFG_DESC, CDCD_ACM_COMM_IFACE_DESCES, CDCD_ACM_DATA_IFACE_DESCES_HS, CDCD_ACM_OTH_SPD_CFG_DESC,           \
	    CDCD_ACM_COMM_IFACE_DESCES, CDCD_ACM_DATA_IFACE_DESCES

#endif /* USBDF_CDC_ACM_DESC_H_ */
