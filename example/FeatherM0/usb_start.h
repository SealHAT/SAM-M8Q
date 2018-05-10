/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#ifndef USB_DEVICE_MAIN_H
#define USB_DEVICE_MAIN_H

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef enum {
	USB_Detached   = -1,  /* device is unplugged and VBUS is inactive*/
	USB_Attached   = 0,   /* device is attached to the bus but not powered ( ex. host detected over current, also used to indicate unattached state) */
	USB_Powered    = 1,   /* device is connected to a host, but enumeration has not begun. VBUS is active and device can draw up to 100mA. */
	USB_Default    = 2,   /* device's USB bus has been reset by the host and is waiting for the host to begin the enumeration process */
	USB_Addressed  = 3,   /* device has been addressed by the USB Host, but is not configured. */
	USB_Configured = 4,   /* device has been enumerated by the host and is ready for USB communications to begin */
	USB_Suspended  = 0x10 /* USB bus has been suspended by the host, and the device should power down to a minimal power level until the bus is resumed. */
} USB_State_t;

/************************ UTILITY FUNCTIONS *************************************/
/**
 * @brief Initialize USB and allocate resources
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
int32_t usb_start(void);

/// For Atmel START Compatibility
static inline void usb_init(void) { usb_start(); }

/**
 * @brief Turn off USB and deallocate resources
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
int32_t usb_stop(void);

/**
 * @brief Get the current state of the USB
 *
 * @returns the USB state as defined by the enum USB_State_t
 */
USB_State_t usb_state(void);

/**
 * @brief Returns true if DTR signal is high.
 * do not trust the transitions on this function, it tends to oscillate before settling :(
 */
bool usb_dtr(void);

/**
 * @brief Returns true if RTS signal is high.
 * This does not appear to work (lower level CDC API does not ever change the RTS status)
 */
bool usb_rts(void);

/**
 * @brief halts the current transfer
 */
void usb_haltTraffic(void);

/************************ TRANSMITTING DATA *************************************/
/**
 * @brief Send a buffer of data to the USB host
 *
 * Send a buffer of data over USB. The data buffer can be of any length but will be split
 * into several packets by the underlying USB stack and sent chunks dictated by the USB endpoint
 * size.
 * 
 * @param outData [IN] buffer of data to output over USB
 * @param BUFFER_SIZE [IN] size of buffer to transfer
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
int32_t usb_write(void* outData, uint32_t BUFFER_SIZE);

/**
 * @brief send a single character to the USB host
 *
 * Buffers a single character to be sent to the USB host. Once the buffer is full
 * it will be sent to the host. To send immediatly use the usb_flushTx() function.
 *
 * @param outChar [IN] the byte of data to send to the host
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
static inline int32_t usb_put(uint8_t outChar) { return usb_write(&outChar, 1); }

/**
 * @brief flush the TX buffer and send it's contents to the USB host.
 *
 * This function currently does nothing and will always return ERR_NONE.
 * IN transactions are not buffered, this function is kept in only for compatibility.
 *
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
static inline int32_t usb_flushTx(void) { return ERR_NONE; }

/************************ RECEIVING DATA ****************************************/
/**
 * @brief checks if there is any data available to read
 * @return The number of characters waiting to be read
 */
int32_t usb_available();

/**
 * @brief receive a buffer of data from the USB host
 * Receive a buffer of data over USB. Input data buffer can be of any size. 
 *
 * @param receiveBuffer [IN] buffer to store incoming USB data
 * @param BUFFER_SIZE [IN] size of receiving buffer
 * @returns the number of bytes received into the buffer
 */
int32_t usb_read(void* receiveBuffer, uint32_t BUFFER_SIZE);

/**
 * @brief receive a single character from the USB host
 *
 * @return less than 0 is an error return from usb_d_ep_transfer()
 */
int32_t usb_get(void);

/**
 * @brief flush the RX buffer and send it's contents.
 * @returns 0 if successful, or negative if error (as listed in err_codes.h)
 */
int32_t usb_flushRx(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_DEVICE_MAIN_H
