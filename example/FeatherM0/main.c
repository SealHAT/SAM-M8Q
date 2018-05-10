#include <atmel_start.h>
#include "usb_start.h"
#include "gps.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    if (gps_init_i2c(&GPS_I2C)) {
        gpio_toggle_pin_level(LED_BUILTIN);
    }

    for (;;)
    {
    }
}
