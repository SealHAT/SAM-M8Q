/**
 * CAM-M8 GNSS module interface
 *
 * This is the header to include for using the ublox CAM-M8 GPS module
 * it exposes the gps api for main code use and the types needed.
 *
 * Author: Ethan Slattery
 * Due Date: 6 Feb 2018
 */
#ifndef UBXTYPES_H_
#define UBXTYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include "minmea.h"
#include "ubx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *	This type represents the UTC time data as given by the GPS device
 */
typedef struct utc_time_t {
    uint8_t        year;             /* 0 .. 255 (1980 == 0)         */
    uint8_t        month;            /* 1 .. 12                      */
    uint8_t        day;              /* 1 .. 31                      */
    uint8_t        hour;             /* 0 .. 23                      */
    uint8_t        minute;           /* 0 .. 59                      */
    uint8_t        second;           /* 0 .. 60                      */ 
    uint16_t       millis;           /* 0 .. 999                     */
} utc_time_t;

/**
 * This type represents A location message, parsed from the NMEA data
 * Received from the gps module
 */
typedef struct location_t {
    utc_time_t     time;             /* UTC date/time                */
    uint16_t       mask;             /*                              */
    uint8_t        correction;       /* GPS/UTC offset               */
    uint8_t        type;             /* fix type                     */
    int32_t        latitude;         /* (WGS84) degrees, 1e7         */
    int32_t        longitude;        /* (WGS84) degrees, 1e7         */
    int32_t        altitude;         /* (MSL) m, 1e3                 */
    int32_t        separation;       /* (WGS84) = (MSL) + separation */
    uint32_t       speed;            /* m/s, 1e3                     */
    uint32_t       course;           /* degrees, 1e5                 */
    int32_t        climb;            /* m/s, 1e3                     */
    uint32_t       ehpe;             /* m, 1e3                       */
    uint32_t       evpe;             /* m, 1e3                       */
    uint8_t        quality;          /* fix quality                  */
    uint8_t        numsv;            /* fix numsv                    */
    uint16_t       pdop;             /* 1e2                          */
    uint16_t       hdop;             /* 1e2                          */
    uint16_t       vdop;             /* 1e2                          */
    uint32_t       ticks;            /* system tick                  */
} location_t;

/**
 * camm8_init
 *
 * Checks to see if argument 0 is a builtin command and then executes
 * the command if it is.
 *
 * args    - the arguments passed from the main shell program
 * RETURNS - true if successful, false if initialization fails
 */
bool camm8_init();

/**
 * camm8_get
 *
 * Checks to see if argument 0 is a builtin command and then executes
 * the command if it is.
 *
 * data    - buffer of data to send
 * count   - number of bytes in the buffer. must be equal to or smaller than buffer size.
 * RETURNS - Pointer to a cstring of the current prompt text, NULL if error.
 */
uint32_t camm8_get(const uint8_t *data, const uint32_t count);

/**
 * camm8_set_periodic
 *
 * RETURNS - true if successful, false if initialization fails
 */
bool camm8_set_periodic();


/**
 * camm8_sleep
 *
 * RETURNS - true if successful, false if initialization fails
 */
bool camm8_sleep();


/**
 * camm8_wakeup
 *
 *
 * RETURNS - true if successful, false if initialization fails
 */
bool camm8_wakeup();

#ifdef __cplusplus
}
#endif

#endif	/* UBXTYPES_H_ */