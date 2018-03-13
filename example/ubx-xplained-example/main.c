#include <atmel_start.h>
#include <string.h>
#include "gps.h"

void gps_usb_writefix(location_t *fix);

int main(void)
{
	uint8_t gps_test;
	location_t gps_fix;

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	gps_init(&SPI_0);
    gpio_set_pin_direction(LED_DEBUG, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_DEBUG, true);
    gpio_set_pin_function(LED_DEBUG,GPIO_PIN_FUNCTION_OFF);
	
	/* Do a Poll message to get info/check if GPS is ready */
	gps_test = gps_selftest();

	while (1)
	{
		delay_ms(100);
		gps_test = gps_getfix(&gps_fix);
        
        if( usb_dtr() && gps_test == 1)
        {
            gpio_set_pin_level(LED_DEBUG,false);
            gps_usb_writefix(&gps_fix);
        }
        gpio_set_pin_level(LED_DEBUG, true);
	}
}

void gps_usb_writefix(location_t *fix)
{
    char    usb_out[26];
    uint8_t usb_result;

    /* print latitude and longitude as signed floats */
    snprintf(usb_out, 26 ,"%ld.%07d, %ld.%07d\n",
        fix.latitude / 10000000,
        abs(gps_fix.latitude % 10000000),
        fix.longitude / 10000000,
        abs(gps_fix.longitude % 10000000));
        
    /* write to usb */
    do 
    {
        usb_result = usb_write((uint8_t*)usb_out, strlen(usb_out));
    } while (usb_result != 0);

}