#include <atmel_start.h>
#include <string.h>
#include "gps.h"

int main(void)
{
	uint8_t gps_test;
	location_t gps_fix;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	gps_init(&SPI_0);
	
	/* Do a Poll message to get info/check if GPS is ready */
	gps_test = gps_selftest();

	while (1)
	{
		delay_ms(1000);
		gps_getfix(&gps_fix);
	}
}
