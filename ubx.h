/* UBX variable type definitions go here */
#include <stdint.h>

#ifndef UBXTYPES_H_
#define UBXTYPES_H_

#define UBXMSG_HEADER (0xB562)

/* UBX message class identifiers */
typedef enum
{
    UBX_NAV = 0x01, /* Navigation results messages */
    UBX_RXM = 0x02, /* Receiver manager messages */
    UBX_INF = 0x04, /* Information messages */
    UBX_ACK = 0x05, /* ACK/NAK meassages */
    UBX_CFG = 0x06, /* Configuration input messages */
    UBX_UPD = 0x09, /* Firmware update messages */
    UBX_MON = 0x0A, /* Monitoring messages */
    UBX_AID = 0x0B, /* AssistNow aiding messages */
    UBX_TIM = 0x0D, /* Timing messages */
    UBX_ESF = 0x10, /* External sensor fusion messages */
    UBX_MGA = 0x13, /* Multiple GNSS assistance messages */
    UBX_LOG = 0x21, /* Logging messages */
    UBX_SEC = 0x27, /* Security feature messages */
    UBX_HNR = 0x28  /* High rate navigation results messages*/
} UBX_CLASS;

typedef struct ubx_header 
{
	uint16_t    header;
	UBX_CLASS   msgclass;
	uint8_t     id;
	uint16_t    length;
} ubx_header;


/**
 * checksum
 *
 * Performs the UBX checksum on the buffer and returns the CK_A and CK_B values
 * by pointer to the calling function
 *
 * BUF     - The buffer to run the checksum on, this should be the message buffer not
 *           includeing the sync characters (CLASS through the end of PAYLOAD)
 * SIZE    - The size of the buffer to perform checksum on 
 * ck_a    - (out) pointer to the first byte of checksum 
 * ck_b    - (out) pointer Second byte of checksum 
 * RETURNS - true if successful, false if initialization fails
 */
void checksum(const uint8_t const * BUF, const uint8_t SIZE, uint8_t* ck_a, uint8_t* ck_b);


/**
 * getclass
 *
 * Parses a UBX message to determine the UBX message class.
 *
 * BUF     - The buffer to run the checksum on, this should be the message buffer not
 *           includeing the sync characters (CLASS through the end of PAYLOAD)
 * SIZE    - The size of the buffer to perform checksum on 
 * RETURNS - The enumerated class group of the ubx message
 */
UBX_CLASS getclass(const uint8_t const * BUF, const uint8_t SIZE);

#endif	/* UBXTYPES_H_ */
