#include <atmel_start.h>
#include "usb_start.h"
#include "Utilities/seal_UTIL.h"
#include "gps.h"

int main(void)
{
	uint16_t blink = 5000;
	/* Initializes MCU, drivers and middleware */
	i2c_unblock_bus(SDA,SCL);
	atmel_start_init();
	
	/* turn on led for 5 seconds to indicate startup */
	gpio_set_pin_level(LED_BUILTIN, true);
//	delay_ms(blink);
	gpio_set_pin_level(LED_BUILTIN, false);

	/* set up the device for DDC (i2c) communication */
    if (gps_init_i2c(&I2C_DEV)) {
        blink = 1000;
		gpio_set_pin_level(LED_BUILTIN, true);
    } else {
		blink = 100;
		gpio_set_pin_level(LED_BUILTIN, false);
	}

	for (;;)
	{
		if (gpio_get_pin_level(TX_RDY) == false) {
			gps_readfifo();
			gpio_toggle_pin_level(LED_BUILTIN);
			delay_ms(100);
			gpio_toggle_pin_level(LED_BUILTIN);
		}
	}
}
