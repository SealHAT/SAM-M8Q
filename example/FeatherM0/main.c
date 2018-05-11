#include <atmel_start.h>
#include "usb_start.h"
#include "Utilities/seal_UTIL.h"
#include "gps.h"

int main(void)
{
	uint16_t blink = 5000;
	/* Initializes MCU, drivers and middleware */
	//i2c_unblock_bus(SDA,SCL);
	atmel_start_init();
	
	/* turn on led for 5 seconds to indicate startup */
	gpio_set_pin_level(LED_BUILTIN, true);
	delay_ms(blink);

	/* set up the device for DDC (i2c) communication */
    if (gps_init_i2c(&I2C_DEV)) {
        blink = 1000;
    } else {
		blink = 100;
	}

	for (;;)
	{
		gpio_toggle_pin_level(LED_BUILTIN);
		delay_ms(blink);
	}
}
