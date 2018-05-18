#include <atmel_start.h>
#include "usb_start.h"
#include "Utilities/seal_UTIL.h"
#include "gps.h"

static uint8_t		log_count = 0;
static gps_log_t	log[GPS_LOGSIZE];

int main(void)
{	
	/* Initializes MCU, drivers and middleware */
	i2c_unblock_bus(SDA,SCL);
	atmel_start_init();
	gpio_set_pin_level(TX_RDY, true);
	gpio_set_pin_level(DBG, false);

	/* set up the device for DDC (i2c) communication */
    if (GPS_SUCCESS == gps_init_i2c(&I2C_DEV)) {
		gpio_set_pin_level(DBG, gps_selftest());
    } else {
		/* TODO - need error state */
		for(;;){ gpio_toggle_pin_level(LED_BUILTIN);delay_ms(50); }
	}
	
	// TODO - wait/sleep for some time to allow for a fix

	for (;;)
	{
		/* wait for TX_READY signal to go active low */
		if (gpio_get_pin_level(TX_RDY) == false &&
			gps_readfifo()) /* and copy the FIFO data */
		{ 			
			/* if successful, parse the data into the log */
			log_count = gps_parsefifo(GPS_FIFO, log, 20);
		}
	}
}
