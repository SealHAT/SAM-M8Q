/**
 * GPS interface
 *
 * Author: Anthony
 * Due Date: 14 Feb 2018   <3
 */

#ifndef GPSINTERFACE_H_
#define GPSINTERFACE_H_

#include "driver_init.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ubxproto/ubx.h"
#include "gps_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_FIFOSIZE	(208)

#define GPS_INVALID_LAT	(-1)
#define GPS_INVALID_LON	(-1)
/* power saving defines */
#define GPS_SEARCH_DIV  (2)     /* fraction of update period time to retry acquisitions     */
#define GPS_SEARCH_MUL  (2)     /* multiplier of update period time to retry acquisitions   */
#define GPS_SEARCH_MAX  (10)    /* maximum time to spend in acquisition state               */

/* i2c defines */
#define GPS_I2C_TIMEOUT	(10)
#define GPS_CFG_TIMEOUT (10)
#define GPS_MAX_MESSAGE (102)

#define M8Q_TXR_CNT		(GPS_FIFOSIZE >> 1)
#define M8Q_TXR_PIO		(6)	/* The pin to use for TxReady						*/
#define M8Q_TXR_POL		(1)	/* TxReady polarity 0 - High-active, 1 - Low-active	*/

#define M8Q_REG_R(ADDR)			((uint8_t)(ADDR << 1) | 0x1)
#define M8Q_REG_W(ADDR)			((uint8_t)(ADDR << 1) | 0x0)

typedef enum {
	M8Q_SLAVE_ADDR		= 0x42,
	M8Q_BYTES_HI_ADDR	= 0xFD,
	M8Q_BYTES_LO_ADDR	= 0xFE,
	M8Q_DATA_ADDR		= 0xFF
} M8Q_DDC_ADDR;

typedef enum {
	GPS_SUCCESS = 0x00,
	GPS_FAILURE = 0x01,
	GPS_INVALID = 0x02,
	GPS_NORXMSG = 0x04,
	GPS_NOTXMSG = 0x08
} GPS_ERROR;

/**
 * gps_init_i2c
 *
 * Initializes and starts the GPS module with the default sampling
 * and messaging rates
 *
 * @param i2c device descriptor from AtmelStart configuration
 * @return 1 if successful, 0 if initialization fails
 */
GPS_ERROR gps_init_i2c(struct i2c_m_sync_desc* const I2C_DESC);

/**
 * gps_reconfig
 *
 *  checks the device RAM for saved settings and ensures they are
 *  loaded. reconfigures the device from scratch if the settings
 *  are not available
 */
GPS_ERROR gps_reconfig();

/**
 * gps_disable_nmea
 *
 * Disables the default NMEA messages on all ports
 *
 * @return 0 if successful, 1 if initialization fails
 */
uint8_t gps_disable_nmea();

/**
 * gps_init_msgs
 *
 * Initializes the minimal periodic messages (position) at
 *	a default rate.
 *
 * @return 0 if successful, 1 if initialization fails
 */
uint8_t gps_init_msgs();

/**
 * gps_write_i2c
 *
 * Sends a message to the I2C slave. M8Q does not support individual
 *	addressing on writes, so only the data is needed.
 *
 * @param data to send over i2c not including address
 * @param size of the data in bytes to send
 * @return 1 if successful, 0 if i2c transmission times out
 */
uint8_t gps_write_i2c(const uint8_t *DATA, const uint16_t SIZE);

/**
 * gps_read_i2c
 *
 * Sends a message to from I2C slave. This uses the "current"
 *  read address scheme where the data register is start and
 *  next address for all transmissions. This allows for reading
 *  without sending a new address, but disallows choosing an
 *  address to read.
 *
 * @param data received from the device is stored here
 * @param size of the data in bytes to send
 * @return 1 if successful, 0 if i2c transmission times out
 */
uint8_t gps_read_i2c(uint8_t *data, const uint16_t SIZE);

/**
 * gps_read_i2c_poll
 *
 * Sends a message to from I2C slave. This uses the "current"
 *  read address scheme where the data register is start and
 *  next address for all transmissions. This allows for reading
 *  without sending a new address, but disallows choosing an
 *  address to read.
 * Poll version checks for 0xff and discards, continuing to
 *	read until valid data is presented.
 *
 * @param data received from the device is stored here
 * @param size of the data in bytes to send
 * @return 1 if successful, 0 if i2c transmission times out
 */
uint8_t gps_read_i2c_poll(uint8_t *data, const uint16_t SIZE);

/**
 * gps_read_i2c_search
 *
 * Sends a message to from I2C slave. This uses the "current"
 *  read address scheme where the data register is start and
 *  next address for all transmissions. This allows for reading
 *  without sending a new address, but disallows choosing an
 *  address to read.
 * Search version discards all messages that do not match the
 *  header information in *data and returns the first message
 *  that does.
 *
 * @param data received from the device is stored here
 * @param size of the data in bytes to send
 * @return 1 if successful, 0 if i2c transmission times out
 */
uint8_t gps_read_i2c_search(uint8_t *data, const uint16_t SIZE);

uint8_t gps_read_i2c_clear();

/**
 * gps_getfix
 *
 * Polls the GPS module for a single fix and the minimum recommended
 * positional data
 *
 * @param fix reference to a gps location structure
 * @return 0 if successful, integer status code otherwise
 */
/*uint8_t gps_getfix(location_t *fix, UBXNAV_PVT *soln);*/

/**
 * gps_gettime
 *
 * Polls the GPS module for a satellite provided time sample
 *
 * @param time reference to a time/date structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_gettime(utc_time_t *time);

/**
 * gps_setrate
 *
 * Configure the GPS module to sample at a given rate
 *
 * @param period sampling rate of GPS in ms
 * @return true if successful, false if sampling rate not set
 */
GPS_ERROR gps_setrate(const uint32_t period);

/**
 * gps_sleep
 *
 * Disable the GPS module by putting it in low-power (sleep) mode
 *
 * @return true if successful, false if device fails to sleep
 */
GPS_ERROR gps_sleep();

/**
 * gps_wake
 *
 * Enable the GPS module by waking it from low-power (sleep) mode,
 * it will resume sampling according to last configuration
 *
 * @return true if successful, false if device fails to sleep
 */
GPS_ERROR gps_wake();

/**
 * gps_cfgpsmoo/gps_cfgpsmoo_18
 *
 * Configures the ON/OFF power-saving mode of the UBX GPS device
 *  by setting the minimum state variables to allow for periodic GPS
 *  acquisition without tracking. The 18 version does the same, but
 *  is only available for protocol v18 and is not well documented.
 *
 * @param period the period of time in ms to attempt to obtain a fix
 * @returns if an error occurred during configuration
 */
GPS_ERROR gps_cfgpsmoo(uint32_t period);
GPS_ERROR gps_cfgpsmoo_magic(uint32_t period);
GPS_ERROR gps_cfgpsmoo_18(uint32_t period);

/**
 * gps_enablepsm
 *  enables the power saving mode as configured in gps_cfgpsmoo. This
 *  needs to be called after configuring to start power saving mode
 */
GPS_ERROR gps_enablepsm();


/**
 * gps_checkconfig
 *  verifies that the current configuration is the desired one set by
 *  gps_cfgprts. will attempt to load saved configurations and check.
 */
GPS_ERROR gps_checkconfig();

/**
 * gps_savecfg
 *
 *  saves all of the current GPS configuration settings
 *
 *  @returns Messaging success, 0 if successful
 */
GPS_ERROR gps_savecfg(const uint16_t MASK);

/**
 * gps_savecfg
 *
 *  loads all of the current GPS configuration settings
 *
 *  @returns Messaging success, 0 if successful
 */
GPS_ERROR gps_loadcfg(const uint16_t MASK);

/**
 * gps_clear
 *
 *  clears all of the current GPS configuration settings
 *
 *  @returns Messaging success, 0 if successful
 */
GPS_ERROR gps_clearcfg(const uint16_t MASK);

/**
 * gps_verifyprt
 *
 *  checks the port settings to ensure they have been configured
 *  as requested by gps_cfgprt. necessary as tx_rdy settings are
 *  not included in ACK/NAK response
 *
 *  @returns 0 if the configuration matches
 */
GPS_ERROR gps_verifyprt();

/**
 * gps_readfifo
 *  extracts the entire ublox fifo from the device to a local array
 *  @returns 0 if successful
 */
uint8_t gps_readfifo();

/**
 * gps_parsefifo
 *  extracts the message contents from the local copy of the FIFO
 *  requires that the fifo has been saved with gps_readfifo
 *  currently supports messages:
 *      NAV_PVT, NAV_UTC
 *  @param log pointer to the array of records for storing on a device
 *  @param LOG_SIZE size of record array
 *  @returns 0 if successful
 */
uint8_t gps_parsefifo(gps_log_t *log, const uint16_t LOG_SIZE);

/**
 * gps_cfgprt
 *  configures the ublox communication ports to the settings specified
 *  by MSG. When used with gps_verifyprt, MSG should configure tx_rdy
 *  settings on the DDC port
 *
 *  @param MSG struct of port settings to configure
 *  @returns 0 if successful
 */
uint8_t gps_cfgprt(const UBXMsg MSG);

/**
 * gps_cfgprts
 *  concatenates each port communication message into one message to
 *  reduce blocking time
 */
GPS_ERROR gps_cfgprts();

/**
 * gps_checkfifo
 *  poll the ddc port for the number of messages currently in the UBX fifo
 *
 *  @returns number of bytes in fifo
 */
int32_t gps_checkfifo();

/**
 * gps_ack
 *  performs a UBX ack/nak check, to be called after CFG messages when needed
 *
 *  @returns 0 if ACK
 */
GPS_ERROR gps_ack();

GPS_ERROR gps_nap(uint32_t ms);

GPS_ERROR gps_init_channels();
#ifdef __cplusplus
}
#endif

#endif	/* GPSINTERFACE_H_ */
