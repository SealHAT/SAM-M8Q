#include "ubx.h"

/* Message Acknowledged, fixed-length, output, p143 */
static const ubx_header ACK_ACK = {
    .header     = UBXMSG_HEADER,
    .msgclass   = UBX_ACK,
    .id         = 0x01,
    .length     = 0x0002
};

/* Message Not-Acknowledged, fixed-length, output, p143 */
static const ubx_header ACK_NAK = {
    .header     = UBXMSG_HEADER,
    .msgclass   = UBX_ACK,
    .id         = 0x00,
    .length     = 0x0002
};

/* Antenna Control Settings, fixed-length, get/set, p154 */
static const ubx_header CFG_ANT = {
    .header     = UBXMSG_HEADER,
    .msgclass   = UBX_CFG,
    .id         = 0x13,
    .length     = 0x0004
};

void checksum(const uint8_t const * BUF, const uint8_t SIZE, uint8_t* ck_a, uint8_t* ck_b)
{
	int i;		/* loop counter */

	/* Reset checksums and calculate new ones*/
	ck_a = 0;
	ck_b = 0;
	for(i = 0; i < SIZE; i++) {
		*ck_a = *ck_a + BUF[i];
		*ck_b = *ck_b + *ck_a;
	}
}

