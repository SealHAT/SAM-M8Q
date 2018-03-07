/*
 * sealHAT_usb.h
 * 
 * Based on the atmel provided usb_start files but modified to
 * give an interactive serial terminal instead of an echo function.
 *
 * Created: 18-Feb-18 15:51:42
 *  Author: Ethan
 */ 


#ifndef SEALHAT_USB_H_
#define SEALHAT_USB_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

/**
 * \brief Initialize CDC-ACM USB
 */
void usb_initialize(void);

/**
 * \brief start the USB echo
 */
void usb_start(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* SEALHAT_USB_H_ */