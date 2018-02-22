/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_sync.h>
#include <hal_crc_sync.h>

#include <hal_i2c_m_sync.h>
#include <hal_spi_m_sync.h>

#include <hal_delay.h>
#include <rtc_lite.h>

#include "hal_usb_device.h"

extern struct adc_sync_descriptor analog_in;
extern struct crc_sync_descriptor hash_chk;

extern struct i2c_m_sync_desc       wire;
extern struct spi_m_sync_descriptor spi_dev;

void analog_in_PORT_init(void);
void analog_in_CLOCK_init(void);
void analog_in_init(void);

void wire_CLOCK_init(void);
void wire_init(void);
void wire_PORT_init(void);

void spi_dev_PORT_init(void);
void spi_dev_CLOCK_init(void);
void spi_dev_init(void);

void delay_driver_init(void);

void   time_date_CLOCK_init(void);
int8_t time_date_init(void);

void USB_DEVICE_INSTANCE_CLOCK_init(void);
void USB_DEVICE_INSTANCE_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
