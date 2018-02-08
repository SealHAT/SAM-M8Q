/* UBX variable type definitions go here */

#ifndef NMEA_H_
#define NMEA_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NMEA_MAX_LENGTH     (82)

/* GPS TALKER - FIRST TWO CHARACTERS OF THE ADDRESS FIELD */
#define START_CHAR          "$"
#define TALKER_GPS          "GP"
#define TALKER_GLONASS      "GL"
#define TALKER_GALILEO      "GA"
#define TALKER_BEIDOU       "GB"
#define TALKER_COMBO        "GN"
#define TALKER_UBX          "PUBX"      /* proprietary UBX message, not NMEA*/

/* SENTENCE FORMAT CODE - LAST THREE CHARACTERS OF ADDRESS FIELD (page 110) */
#define MSG_DATUM           "DTM"
#define MSG_POLL_GB         "GBQ"
#define MSG_FAULT_DETECT    "GBS"
#define MSG_POSITION        "GGA"
#define MSG_LATLONTIME      "GLL"
#define MSG_POLL_GL         "GLQ"
#define MSG_POLL_GN         "GNQ"
#define MSG_FIX_DATA        "GNS"
#define MSG_POLL_GP         "GPQ"
#define MSG_RANGE_RESIDUALS "GRS"
#define MSG_DOP             "GSA"
#define MSG_ERROR_STATS     "GST"
#define MSG_SAT_VIEW        "GSV"
#define MSG_MIN_DATA        "RMC"
#define MSG_TEXT            "TXT"
#define MSG_WATER_DIST      "VLW"
#define MSG_COURSE_OVR_GND  "VTG"
#define MSG_TIME_DATE       "ZDA"

/* Enum for nmea message numbers in CFG-MSG (pg 126) */
typedef enum {
    NMEA_DTM    = 0x0A,
    NMEA_GBQ    = 0x44,
    NMEA_GBS    = 0x09,
    NMEA_GGA    = 0x00,
    NMEA_GLL    = 0x01,
    NMEA_GLQ    = 0x43,
    NMEA_GNQ    = 0x42,
    NMEA_GNS    = 0x0D,
    NMEA_GPQ    = 0x40,
    NMEA_GRS    = 0x06,
    NMEA_GSA    = 0x02,
    NMEA_GST    = 0x07,
    NMEA_GSV    = 0x03,
    NMEA_RMC    = 0x04,
    NMEA_TxT    = 0x41,
    NMEA_VLW    = 0x0F,
    NMEA_VTG    = 0x05,
    NMEA_ZDA    = 0x08,
    UBX_ID      = 0xF0     /* ID for CFG-MSG is this plus the above */ 
} NMEA_MESSAGE;

/* Enum for UBX nmea message numbers (pg 126) */
typedef enum {
    UBX_CONFIG   = 0x41,
    UBX_POSITION = 0x00,
    UBX_RATE     = 0x40,
    UBX_SVSTATUS = 0x03,
    UBX_TIME     = 0x04,
    UBX_ID       = 0xF1     /* ID for CFG-MSG is this plus the above */ 
} UBX_MESSAGE;

/**
 * nmea_checksum
 *
 * Performs the NMEA checksum  calculation on the buffer and returns the calculated
 * checksum to the calling function. The sentance does not need to start with '$' but it
 * MUST end with a '*' to indicate where the checksum operation should stop.
 *
 * sentence - The buffer to run the checksum on
 * RETURNS - the checksum as a 8-bit integer
 */
uint8_t nmea_checksum(const char *sentence);

#ifdef __cplusplus
}
#endif

#endif  /* NMEA_H_ */