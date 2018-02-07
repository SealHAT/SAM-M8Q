#include "ubx.h"

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

