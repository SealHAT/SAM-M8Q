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

extern struct adc_sync_descriptor ADC;
extern struct crc_sync_descriptor CRC;

extern struct i2c_m_sync_desc       I2C;
extern struct spi_m_sync_descriptor SPI;

void ADC_PORT_init(void);
void ADC_CLOCK_init(void);
void ADC_init(void);

void I2C_CLOCK_init(void);
void I2C_init(void);
void I2C_PORT_init(void);

void SPI_PORT_init(void);
void SPI_CLOCK_init(void);
void SPI_init(void);

void delay_driver_init(void);

void   CALENDAR_CLOCK_init(void);
int8_t CALENDAR_init(void);

void USB_CLOCK_init(void);
void USB_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
