/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#ifndef USB_DEVICE_MAIN_H
#define USB_DEVICE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

/**
 * \brief Initialize USB
 */
void usb_init(void);

/**
 * \brief Returns true if the USB is configured (done with enumeration and not asleep)
 */
bool usb_configured(void);

/**
 * \brief Returns true if DTR signal is high
 */
bool usb_dtr(void);

/**
 * \brief Returns true if RTS signal is high
 */
bool usb_rts(void);

/**
 * \brief flush the TX buffer and send it's contents to the USB host. This functions blocks.
 */
void usb_flush(void);

/**
 * \brief Initialize USB
 */
void usb_put(uint8_t outChar);

/**
 * \brief Initialize USB
 *
 * \return less than 0 is an error return from usb_d_ep_transfer()
 */
int usb_get(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_DEVICE_MAIN_H
