/**
 * GPS interface
 *
 * Author: Anthony
 * Due Date: 14 Feb 2018   <3
 */

#ifndef UBXTYPES_H_
#define UBXTYPES_H_

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
 * gps_init
 *
 * Initializes and starts the GPS module with the default sampling and
 * messaging rates
 *
 * @return true if successful, false if initialization fails
 */
bool gps_init();

/**
 * gps_getfix
 *
 * Polls the GPS module for a single fix and the minimum recommended 
 * positional data
 *
 * @param fix reference to a gps fix data structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_getfix(GpsData *fix );

/**
 * gps_gettime
 *
 * Polls the GPS module for a satellite provided time sample
 *
 * @param time reference to a time/date structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_gettime(Time *time);

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
 * Enable the GPS module by waking it from low-power (sleep) mode, it will 
 * resume sampling according to last configuration
 *
 * @return true if successful, false if device fails to sleep
 */
bool gps_sleep();

/**
 * gps_setprofile
 *
 * Configure the GPS module with a predefined sampling/power scheme
 *
 * @param profile preconfigured sampling profile
 * @returns true if successful, false if profile not set
 */
bool gps_setprofile(const GpsProfile profile);
  
#ifdef __cplusplus
}
#endif

#endif	/* UBXTYPES_H_ */
