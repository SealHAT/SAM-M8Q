#include <atmel_start.h>
#include "usb_start.h"

int main(void)
{
	int letter;
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	while (1) {
		/* Turn on LED if the DTR signal is set (serial terminal open on host) */
		gpio_set_pin_level(LED_BUILTIN, usb_dtr());
		
		/* If the host has a terminal open then try to read, and if we read send it back */
		if(usb_dtr()) {
			letter = usb_get();
			if(letter > 0) {
				usb_put(letter);
				usb_flush();
			}
		}

	}
}
