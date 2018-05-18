#include <atmel_start.h>
#include "usb_start.h"
#include "Utilities/seal_UTIL.h"
#include "gps.h"

static gps_log_t log[20];

int main(void)
{
	uint8_t	 count = 0;
	uint16_t blink = 5000;
	
	/* Initializes MCU, drivers and middleware */
	i2c_unblock_bus(SDA,SCL);
	atmel_start_init();
	gpio_set_pin_level(TX_RDY, true);

	/* set up the device for DDC (i2c) communication */
    if (GPS_SUCCESS == gps_init_i2c(&I2C_DEV)) {
        blink = 1000;
		gpio_set_pin_level(LED_BUILTIN, gps_selftest());
		
    } else {
		blink = 100;
		gpio_set_pin_level(LED_BUILTIN, false);
	}

	for (;;)
	{
		if (gpio_get_pin_level(TX_RDY) == false) {
			gps_readfifo();
			gpio_toggle_pin_level(LED_BUILTIN);
			count = gps_parsefifo(GPS_FIFO, log, 20);
			gpio_toggle_pin_level(LED_BUILTIN);
		}
	}
}
