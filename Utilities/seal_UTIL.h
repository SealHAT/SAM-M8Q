/*
 * SerialPrint.h
 *
 * Created: 3/7/2018 2:39:48 PM
 *  Author: eslatter
 */


#ifndef SEALUTIL_H_
#define SEALUTIL_H_

#include "driver_init.h"
#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_usb_device.h"

typedef enum {
    I2C_STATE_UNKNOWN = 0x00,
    I2C_STATE_IDLE    = 0x01,
    I2C_STATE_OWNER   = 0x02,
    I2C_STATE_BUSY    = 0x03
} I2C_STATE_t;

/**
 * This function unblocks an I2C bus where the slave is holding
 * data low because out an unexpected restart or bus error where
 * all the clock pulses were not detected. It will first check to
 * see if the line is actually help low and if it is toggle the
 * clock 16 times to complete whatever stalled transaction is present.
 *
 * @param SDA [IN] pin for the I2C devices data line
 * @param SCL [IN] pin for the I2C devices clock line
 */
void i2c_unblock_bus(const uint8_t SDA_PIN, const uint8_t SCL_PIN);


/**
 * This function sets up the L21s low power setting by:
 *    - disable brown out detector
 *    - select buck converter as main voltage reg
 *    - Set buck to low power mode efficiency
 *    - Apply the saml21 errata 15264: CPU freezes in standby if buck disabled
 *    - set performance level to PL0 @ 12MHz
 */
void set_lowPower_mode(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* SEALUTIL_H_ */
