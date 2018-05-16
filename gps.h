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
#include "ubx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_BUFFSIZE	(256)
#define UBX_FFTCNT		(32)
#define GPS_FIFOSIZE	(2048)
#define M8Q_TXR_CNT		(GPS_FIFOSIZE >> 1)
#define M8Q_TXR_PIO		(6)	/* The pin to use for TxReady						*/
#define M8Q_TXR_POL		(1)	/* TxReady polarity 0 - High-active, 1 - Low-active	*/

/* i2c defines */
#define I2C_TIMEOUT	(4)
#define CFG_TIMEOUT (4)

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

extern uint8_t GPS_FIFO[GPS_FIFOSIZE];

/**
 * utc_time_t enum
 *
 * This type represents the UTC time data as given by the GPS device
 */
typedef struct utc_time_t {
    uint8_t        year;             /**< 0 .. 257 (1980 == 0)         */
    uint8_t        month;            /**< 1 .. 12                      */
    uint8_t        day;              /**< 1 .. 31                      */
    uint8_t        hour;             /**< 0 .. 23                      */
    uint8_t        minute;           /**< 0 .. 59                      */
    uint8_t        second;           /**< 0 .. 60                      */ 
    uint16_t       millis;           /**< 0 .. 999                     */
} utc_time_t;

/**
 * utc_time_t enum
 *
 * This type represents A location message, parsed from the NMEA data
 * Received from the gps module
 */
typedef struct location_t {
    utc_time_t     time;             /**< UTC date/time                */
    uint16_t       mask;             /**<                              */
    uint8_t        correction;       /**< GPS/UTC offset               */
    uint8_t        type;             /**< fix type                     */
    int32_t        latitude;         /**< (WGS84) degrees, 1e7         */
    int32_t        longitude;        /**< (WGS84) degrees, 1e7         */
    int32_t        altitude;         /**< (MSL) m, 1e3                 */
    int32_t        separation;       /**< (WGS84) = (MSL) + separation */
    uint32_t       speed;            /**< m/s, 1e3                     */
    uint32_t       course;           /**< degrees, 1e5                 */
    int32_t        climb;            /**< m/s, 1e3                     */
    uint32_t       ehpe;             /**< m, 1e3                       */
    uint32_t       evpe;             /**< m, 1e3                       */
    uint8_t        quality;          /**< fix quality                  */
    uint8_t        numsv;            /**< fix numsv                    */
    uint16_t       pdop;             /**< 1e2                          */
    uint16_t       hdop;             /**< 1e2                          */
    uint16_t       vdop;             /**< 1e2                          */
    uint32_t       ticks;            /**< system tick                  */
} location_t;

// typedef struct pvtsoln_t {
//     
// };

/**
 * GPS_PROFILE enum
 *
 * Each represents a predefined configuration scheme for the CAM8 
 * gps module, implementation left to GNSS device header
 */
typedef enum
{
    /* TODO: Verify accuracy/current of schemes */
    GPS_PSMOO1H,    /**< 1hr rate, 7cm accuracy, 246 uA avg current    */
    GPS_PSMOO30S    /**< 30s rate, 7cm accuracy, 26 mA avg current     */
    /* TODO: Add profiles as they are tested */
} GPS_PROFILE;

/**
 * gps_init_spi
 *
 * Initializes and starts the GPS module with the default sampling 
 * and messaging rates
 * 
 * @param spi device descriptor from AtmelStart configuration
 * @return true if successful, false if initialization fails
 */
uint8_t gps_init_spi(struct spi_m_sync_descriptor *spi_desc);

/**
 * gps_init_i2c
 *
 * Initializes and starts the GPS module with the default sampling 
 * and messaging rates
 * 
 * @param i2c device descriptor from AtmelStart configuration
 * @return 1 if successful, 0 if initialization fails
 */
uint8_t gps_init_i2c(struct i2c_m_sync_desc* const I2C_DESC);

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
 * gps_read_i2c_pol
 *
 * Sends a message to from I2C slave. This uses the "current" 
 *  read address scheme where the data register is start and 
 *  next address for all transmissions. This allows for reading
 *  without sending a new address, but disallows choosing an
 *  address to read. 
 * Pol version checks for 0xff and discards, continuing to 
 *	read until valid data is presented.
 * 
 * @param data received from the device is stored here
 * @param size of the data in bytes to send
 * @return 1 if successful, 0 if i2c transmission times out
 */
uint8_t gps_read_i2c_poll(uint8_t *data, const uint16_t SIZE);

uint8_t gps_read_i2c_search(uint8_t *data, const uint16_t SIZE);

/**
 * gps_getfix
 *
 * Polls the GPS module for a single fix and the minimum recommended 
 * positional data
 *
 * @param fix reference to a gps location structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_getfix(location_t *fix, UBXNAV_PVT *soln);

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
 * @param period sampling rate of gps in seconds
 * @return true if succesful, false if sampling rate not set
 */
bool gps_setrate(const uint32_t period);

/**
 * gps_sleep
 *
 * Disable the GPS module by putting it in low-power (sleep) mode
 *
 * @return true if successful, false if device fails to sleep
 */
bool gps_sleep();

/**
 * gps_wake
 *
 * Enable the GPS module by waking it from low-power (sleep) mode, 
 * it will resume sampling according to last configuration
 *
 * @return true if successful, false if device fails to sleep
 */
bool gps_wake();

/**
 * gps_setprofile
 *
 * Configure the GPS module with a predefined sampling/power 
 * scheme
 *
 * @param profile preconfigured sampling profile
 * @returns true if successful, false if profile not set
 */
bool gps_setprofile(const GPS_PROFILE profile);


//TODO    encapsulate these helper functions
uint8_t gps_selftest();
uint8_t cfgUBXoverSPI(uint8_t ffCnt);
//uint8_t cfgPSMOO(uint8_t period);

/************************************************************************/
/* gps_readfifo															*/
/************************************************************************/
uint8_t gps_readfifo();
uint8_t gps_cfgprt(const UBXMsg MSG);
  
#ifdef __cplusplus
}
#endif

#endif	/* GPSINTERFACE_H_ */
