/*
 * gps_types.h
 *
 * Created: 09-Jun-18 23:34:41
 *  Author: Ethan
 */


#ifndef GPS_TYPES_H_
#define GPS_TYPES_H_

#define GPS_VERBOSE_LOG (1)

/**
 * utc_time_t enum
 *
 * This type represents the UTC time data as given by the GPS device
 */
typedef struct __attribute__((__packed__)) {
    uint16_t    year;             /**< 0 .. 257 (1980 == 0)         */
    uint8_t     month;            /**< 1 .. 12                      */
    uint8_t     day;              /**< 1 .. 31                      */
    uint8_t     hour;             /**< 0 .. 23                      */
    uint8_t     minute;           /**< 0 .. 59                      */
    uint8_t     second;           /**< 0 .. 60                      */
    int32_t		nano;			  /**< 0 .. 999                     */
} utc_time_t;


typedef struct __attribute__((__packed__)) {
	bool        fixOk;
    int32_t     lon;	/* int32_t */
    int32_t     lat;	/* int32_t */
#if (GPS_VERBOSE_LOG == 1)
    uint8_t     fixType;
    uint32_t     hAcc;
    uint32_t     vAcc;
#endif
} min_pvt_t;

typedef struct __attribute__((__packed__)) {
	min_pvt_t   position;
    utc_time_t  time;             /**< UTC date/time                */
} gps_log_t;

#endif /* GPS_TYPES_H_ */