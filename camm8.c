#include "camm8.h"

typedef struct ubx_header {
	uint16_t header;
	uint8_t  class;
	uint8_t  id;
	uint16_t length;
} ubx_header;

typedef struct ubx_message {
	ubx_header header;
	uint8_t payload[8]
	uint8_t CK_A;
	uint8_t CK_B;
}

static const struct 