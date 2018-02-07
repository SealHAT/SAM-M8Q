/* UBX variable type definitions go here */

#ifndef UBXTYPES_H_
#define UBXTYPES_H_

typedef struct ubx_header {
	uint16_t header;
	uint8_t  class;
	uint8_t  id;
	uint16_t length;
} ubx_header;

/**
 * checksum
 *
 * Performs the UBX checksum on the buffer and returns the CK_A and CK_B values
 * by pointer to the calling function
 *
 * buf     - The buffer to run the checksum on, this should be the message buffer not
 *           includeing the sync characters (CLASS through the end of PAYLOAD)
 * ck_a    - (out) pointer to the first byte of checksum 
 * ck_b    - (out) pointer Second byte of checksum 
 * RETURNS - true if successful, false if initialization fails
 */
void checksum(const uint8_t const * buf, uint8_t* ck_a, uint8_t* ck_b);

#endif	/* UBXTYPES_H_ */