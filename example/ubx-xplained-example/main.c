#include <atmel_start.h>
#include <string.h>
#include "gps.h"

int main(void)
{
	uint8_t gps_test;
    int32_t lat_h;
    int32_t lon_h;
    char usb_out[23];

	location_t gps_fix;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	gps_init(&SPI_0);
	
	/* Do a Poll message to get info/check if GPS is ready */
	gps_test = gps_selftest();
    gpio_set_pin_direction(LED_DEBUG, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_DEBUG, true);
    gpio_set_pin_function(LED_DEBUG,GPIO_PIN_FUNCTION_OFF);

	while (1)
	{
		delay_ms(100);

		gps_test = gps_getfix(&gps_fix);
        
        if( usb_dtr() && gps_test == 1)
        //if( gps_test == 1)
        {
            gpio_set_pin_level(LED_DEBUG,false);
            snprintf(usb_out, 26 ,"%ld.%07d, %ld.%07d\n",
                gps_fix.latitude / 10000000,
                abs(gps_fix.latitude % 10000000),
                gps_fix.longitude / 10000000,
                abs(gps_fix.longitude % 10000000));      
            usb_write((uint8_t*)usb_out,strlen(usb_out));
            //usb_put('\n');
        }
        gpio_set_pin_level(LED_DEBUG, true);
	}
}
