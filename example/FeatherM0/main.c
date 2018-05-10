#include <atmel_start.h>
#include "usb_start.h"
//#include "Utilities/seal_UTIL.h"
//#include "gps.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	//i2c_unblock_bus(SDA,SCL);
	atmel_start_init();

//     if (gps_init_i2c(&GPS_I2C)) {
//         gpio_toggle_pin_level(LED_BUILTIN);
//     }

    for (;;)
    {
		gpio_toggle_pin_level(LED_BUILTIN);
		delay_ms(300);
    }
}
